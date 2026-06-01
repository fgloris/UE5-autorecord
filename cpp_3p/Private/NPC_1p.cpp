#include "NPC_1p.h"

#include "Components/CapsuleComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "NavigationSystem.h"
#include "NPCMovementRecorder.h"

ANPC_1p::ANPC_1p()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;

    ExploreActionDuration = 1.0f;

    CameraYawStepDegrees = 10.0f;
    CameraPitchStepDegrees = 10.0f;
    MaxCameraPitchOffsetActionCount = 1;
    CameraPitchHoldToleranceDegrees = 0.1f;

    bDebugDrawExploreCandidates = false;

    CameraBoomLength = 0.0f;
    FirstPersonCameraRelativeLocation = FVector(0.0f, 0.0f, 60.0f);
    InPlacePaceInputScale = 0.12f;
    InPlacePaceCyclesPerAction = 1.0f;
    bRestoreLocationAfterInPlacePacing = true;

    if (CameraBoom)
    {
        CameraBoom->TargetArmLength = CameraBoomLength;
        CameraBoom->bDoCollisionTest = false;
        CameraBoom->SetRelativeLocation(FirstPersonCameraRelativeLocation);
    }
}

void ANPC_1p::BeginPlay()
{
    Super::BeginPlay();

    if (CameraBoom)
    {
        CameraBoom->TargetArmLength = CameraBoomLength;
        CameraBoom->bDoCollisionTest = false;
        CameraBoom->SetRelativeLocation(FirstPersonCameraRelativeLocation);
        CameraBoom->SetWorldRotation(FRotator(CameraBoomPitch, GetActorRotation().Yaw, 0.0f));
    }

    SyncActorYawToCameraYaw();
}

void ANPC_1p::ExecuteNextStep(float DeltaTime)
{
    if (!GetWorld())
    {
        return;
    }

    if (!bIsExecutingExploreAction)
    {
        StartExploreAction();
    }

    const bool bWasExecutingThisFrame = bIsExecutingExploreAction;
    ExecuteExploreAction(DeltaTime);

    if (MovementRecorder && MovementRecorder->bIsRecording)
    {
        MovementRecorder->RecordFrameFromNPC(this, CurrentPath, CurrentPathIndex, DeltaTime, bWasExecutingThisFrame);
    }
}

void ANPC_1p::ClearExploreMoveTarget()
{
    bIsExecutingExploreAction = false;
    CurrentExploreMoveTarget = FVector::ZeroVector;
    StartExploreActorLocation = FVector::ZeroVector;
    CurrentExploreMoveAction = ENPC1PExploreMoveAction::Idle;
    CurrentExploreActionElapsed = 0.0f;
    bHasDesiredCameraWorldRotation = false;
    CurrentExploreCameraAction = ENPC1PExploreCameraAction::None;
    StartCameraWorldRotation = FRotator::ZeroRotator;
    DesiredCameraWorldRotation = FRotator::ZeroRotator;
}

void ANPC_1p::GetCurrentRecorderControlSignals(int32& OutWS, int32& OutAD, int32& OutLR, int32& OutUD) const
{
    OutWS = CurrentRecorderWS;
    OutAD = CurrentRecorderAD;
    OutLR = CurrentRecorderLR;
    OutUD = CurrentRecorderUD;
}

void ANPC_1p::StartExploreAction()
{
    UpdateVisitedStatsAtCurrentPosition();

    TArray<FExploreMoveCandidate> Candidates;
    BuildLegalActionCandidates(Candidates);

    if (Candidates.Num() <= 0)
    {
        const FString ErrorMessage = FString::Printf(TEXT("NPC_1p[%s] no legal movement candidate"), *GetActorLabel());
        UE_LOG(LogTemp, Error, TEXT("%s"), *ErrorMessage);
        if (GEngine)
        {
            GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, ErrorMessage);
        }
        return;
    }

    const int32 PickedIndex = SampleRandomCandidate(Candidates);
    if (!Candidates.IsValidIndex(PickedIndex))
    {
        return;
    }

    const FExploreMoveCandidate& Picked = Candidates[PickedIndex];

    bIsExecutingExploreAction = true;
    CurrentExploreMoveAction = Picked.Action;
    CurrentExploreMoveTarget = Picked.LandingActorLocation;
    StartExploreActorLocation = GetActorLocation();
    CurrentExploreActionElapsed = 0.0f;
    GetMoveActionSignals(Picked.Action, CurrentRecorderWS, CurrentRecorderAD);

    const USpringArmComponent* CameraBoomComp = GetCameraBoom();
    StartCameraWorldRotation = CameraBoomComp ? CameraBoomComp->GetComponentRotation() : FRotator(CameraBoomPitch, GetActorRotation().Yaw, 0.0f);
    CurrentExploreCameraAction = ChooseRandomCameraAction(StartCameraWorldRotation, DesiredCameraWorldRotation);
    GetCameraActionSignals(CurrentExploreCameraAction, CurrentRecorderLR, CurrentRecorderUD);
    bHasDesiredCameraWorldRotation = CurrentExploreCameraAction != ENPC1PExploreCameraAction::None;
}

void ANPC_1p::ExecuteExploreAction(float DeltaTime)
{
    if (!bIsExecutingExploreAction)
    {
        return;
    }

    const float Duration = FMath::Max(ExploreActionDuration, KINDA_SMALL_NUMBER);
    const float PreviousElapsed = CurrentExploreActionElapsed;
    const float RemainingTime = FMath::Max(Duration - PreviousElapsed, 0.0f);
    const float EffectiveDeltaTime = FMath::Clamp(DeltaTime, 0.0f, RemainingTime);
    CurrentExploreActionElapsed = FMath::Min(PreviousElapsed + EffectiveDeltaTime, Duration);
    const float Alpha = FMath::Clamp(CurrentExploreActionElapsed / Duration, 0.0f, 1.0f);

    UCharacterMovementComponent* MoveComp = GetCharacterMovement();
    if (!MoveComp)
    {
        ClearExploreMoveTarget();
        return;
    }

    USpringArmComponent* CameraBoomComp = GetCameraBoom();
    if (CameraBoomComp && bHasDesiredCameraWorldRotation)
    {
        CameraBoomComp->SetWorldRotation(FQuat::Slerp(StartCameraWorldRotation.Quaternion(), DesiredCameraWorldRotation.Quaternion(), Alpha).Rotator());
    }

    SyncActorYawToCameraYaw();

    const float InputScaleByFrame = (DeltaTime > KINDA_SMALL_NUMBER) ? (EffectiveDeltaTime / DeltaTime) : 0.0f;

    if (CurrentExploreMoveAction == ENPC1PExploreMoveAction::W)
    {
        MoveComp->SetMovementMode(MOVE_Walking);
        AddMovementInput(GetActorForwardVector().GetSafeNormal2D(), InputScaleByFrame);
    }
    else if (CurrentExploreCameraAction != ENPC1PExploreCameraAction::None && InPlacePaceInputScale > KINDA_SMALL_NUMBER)
    {
        MoveComp->SetMovementMode(MOVE_Walking);
        const float PacePhase = Alpha * 2.0f * PI * FMath::Max(InPlacePaceCyclesPerAction, 0.5f);
        const float PaceScale = FMath::Sin(PacePhase) * InPlacePaceInputScale * InputScaleByFrame;
        AddMovementInput(GetActorForwardVector().GetSafeNormal2D(), PaceScale);
    }

    if (CurrentExploreActionElapsed >= Duration)
    {
        if (CameraBoomComp && bHasDesiredCameraWorldRotation)
        {
            CameraBoomComp->SetWorldRotation(DesiredCameraWorldRotation);
        }

        SyncActorYawToCameraYaw();

        if (CurrentExploreMoveAction == ENPC1PExploreMoveAction::Idle &&
            CurrentExploreCameraAction != ENPC1PExploreCameraAction::None &&
            bRestoreLocationAfterInPlacePacing)
        {
            SetActorLocation(StartExploreActorLocation, false);
        }

        const FVector ReachedLocation = GetActorLocation();
        ClearExploreMoveTarget();
        OnExploreMoveTargetReached(ReachedLocation);
    }
}

void ANPC_1p::BuildLegalActionCandidates(TArray<FExploreMoveCandidate>& OutCandidates) const
{
    OutCandidates.Reset();

    const ENPC1PExploreMoveAction AllActions[2] =
    {
        ENPC1PExploreMoveAction::Idle,
        ENPC1PExploreMoveAction::W
    };

    for (ENPC1PExploreMoveAction Action : AllActions)
    {
        FExploreMoveCandidate Candidate;
        if (TryBuildActionCandidate(Action, Candidate))
        {
            OutCandidates.Add(Candidate);
        }
    }
}

bool ANPC_1p::TryBuildActionCandidate(ENPC1PExploreMoveAction Action, FExploreMoveCandidate& OutCandidate) const
{
    if (Action == ENPC1PExploreMoveAction::Idle)
    {
        const FVector ActorLocation = GetActorLocation();
        const UCapsuleComponent* Capsule = GetCapsuleComponent();
        const float CapsuleHalfHeight = Capsule ? Capsule->GetScaledCapsuleHalfHeight() : 0.0f;

        OutCandidate.Action = Action;
        OutCandidate.WorldDirection = FVector::ZeroVector;
        OutCandidate.LandingActorLocation = ActorLocation;
        OutCandidate.LandingFootLocation = ActorLocation - FVector(0.0f, 0.0f, CapsuleHalfHeight);
        return true;
    }

    FVector DesiredWorldDirection = FVector::ZeroVector;
    if (!GetWorldDirectionForAction(Action, DesiredWorldDirection))
    {
        return false;
    }

    if (!IsLandingValidForDirection(DesiredWorldDirection, OutCandidate))
    {
        return false;
    }

    OutCandidate.Action = Action;
    return true;
}

bool ANPC_1p::GetWorldDirectionForAction(ENPC1PExploreMoveAction Action, FVector& OutDirection) const
{
    if (Action != ENPC1PExploreMoveAction::W)
    {
        OutDirection = FVector::ZeroVector;
        return false;
    }

    const USpringArmComponent* CameraBoomComp = GetCameraBoom();
    if (!CameraBoomComp)
    {
        return false;
    }

    FVector CamForward = CameraBoomComp->GetForwardVector();
    CamForward.Z = 0.0f;
    CamForward = CamForward.GetSafeNormal();
    if (CamForward.IsNearlyZero())
    {
        CamForward = GetActorForwardVector().GetSafeNormal2D();
    }

    OutDirection = CamForward;
    return !OutDirection.IsNearlyZero();
}

void ANPC_1p::GetMoveActionSignals(ENPC1PExploreMoveAction Action, int32& OutWS, int32& OutAD) const
{
    OutWS = 0;
    OutAD = 0;

    if (Action == ENPC1PExploreMoveAction::W)
    {
        OutWS = 1;
    }
}

int32 ANPC_1p::SampleRandomCandidate(const TArray<FExploreMoveCandidate>& Candidates) const
{
    if (Candidates.Num() <= 0)
    {
        return INDEX_NONE;
    }

    return FMath::RandRange(0, Candidates.Num() - 1);
}

bool ANPC_1p::IsLandingValidForDirection(const FVector& DesiredWorldDirection, FExploreMoveCandidate& OutCandidate) const
{
    UWorld* World = GetWorld();
    if (!World)
    {
        return false;
    }

    UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
    const UCharacterMovementComponent* MoveComp = GetCharacterMovement();
    const UCapsuleComponent* Capsule = GetCapsuleComponent();
    if (!NavSys || !MoveComp || !Capsule)
    {
        return false;
    }

    FVector Dir2D = DesiredWorldDirection;
    Dir2D.Z = 0.0f;
    Dir2D = Dir2D.GetSafeNormal();
    if (Dir2D.IsNearlyZero())
    {
        return false;
    }

    const float CapsuleHalfHeight = Capsule->GetScaledCapsuleHalfHeight();
    const float ProbeStepDistance = MoveComp->MaxWalkSpeed * FMath::Max(ExploreActionDuration, 0.0f);
    if (ProbeStepDistance <= KINDA_SMALL_NUMBER)
    {
        return false;
    }

    const FVector StartActorLocation = GetActorLocation();
    const FVector StartFootLocation = StartActorLocation - FVector(0.0f, 0.0f, CapsuleHalfHeight);
    const FVector DesiredFootLocation = StartFootLocation + Dir2D * ProbeStepDistance;

    FNavLocation ProjectedLocation;
    const FVector QueryExtent(GridSize * 0.5f, GridSize * 0.5f, 200.0f);
    if (!NavSys->ProjectPointToNavigation(DesiredFootLocation, ProjectedLocation, QueryExtent))
    {
        return false;
    }

    const FVector LandingFootLocation = ProjectedLocation.Location;

    if (FVector::Dist2D(StartFootLocation, LandingFootLocation) < 10.0f)
    {
        return false;
    }

    if (!IsLocationValidForNPC(LandingFootLocation))
    {
        return false;
    }

    const FVector LandingActorLocation = LandingFootLocation + FVector(0.0f, 0.0f, CapsuleHalfHeight);
    // Keep the same tuning as the third-person explorer: landing capsule/NavMesh checks are authoritative.
    // Enable this only if the third-person NPC also enables its swept path check.
    // if (!IsMovePathCollisionFree(StartActorLocation, LandingActorLocation))
    // {
    //     return false;
    // }

    OutCandidate.WorldDirection = Dir2D;
    OutCandidate.LandingFootLocation = LandingFootLocation;
    OutCandidate.LandingActorLocation = LandingActorLocation;

    if (bDebugDrawExploreCandidates)
    {
        const float DrawDuration = FMath::Max(ExploreActionDuration, KINDA_SMALL_NUMBER);
        DrawDebugSphere(World, LandingActorLocation, 12.0f, 8, FColor::Green, false, DrawDuration);
        DrawDebugLine(World, StartActorLocation, LandingActorLocation, FColor::Green, false, DrawDuration, 0, 1.5f);
    }

    return true;
}

bool ANPC_1p::IsMovePathCollisionFree(const FVector& StartActorLocation, const FVector& EndActorLocation) const
{
    UWorld* World = GetWorld();
    const UCapsuleComponent* Capsule = GetCapsuleComponent();
    if (!World || !Capsule)
    {
        return false;
    }

    const float CapsuleRadius = Capsule->GetScaledCapsuleRadius() + 10.0f;
    const float CapsuleHalfHeight = Capsule->GetScaledCapsuleHalfHeight();
    const FCollisionShape CapsuleShape = FCollisionShape::MakeCapsule(CapsuleRadius, CapsuleHalfHeight);

    FCollisionQueryParams Params(SCENE_QUERY_STAT(NPC1pCapsuleOverlap), false, this);
    Params.AddIgnoredActor(this);

    const FVector Delta = EndActorLocation - StartActorLocation;
    const float TravelDist = Delta.Size2D();
    const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(TravelDist / FMath::Max(25.0f, GridSize * 0.25f)));

    for (int32 StepIdx = 1; StepIdx <= NumSteps; ++StepIdx)
    {
        const float Alpha = static_cast<float>(StepIdx) / static_cast<float>(NumSteps);
        const FVector SampleActorLocation = FMath::Lerp(StartActorLocation, EndActorLocation, Alpha);

        const bool bHasOverlap = World->OverlapAnyTestByChannel(
            SampleActorLocation,
            FQuat::Identity,
            ECollisionChannel::ECC_Pawn,
            CapsuleShape,
            Params);

        if (bHasOverlap)
        {
            return false;
        }
    }

    return true;
}

ENPC1PExploreCameraAction ANPC_1p::ChooseRandomCameraAction(const FRotator& CurrentCameraRotation, FRotator& OutDesiredRotation)
{
    const float CameraPitchCenter = CameraBoomPitch;
    const float CurrentPitch = FMath::Clamp(CurrentCameraRotation.Pitch, -30.0f, 15.0f);
    const float CurrentPitchOffset = CurrentPitch - CameraPitchCenter;

    UpdatePitchOffsetHoldState(CurrentPitchOffset);

    const int32 LRSignal = FMath::RandRange(0, 2);
    int32 UDSignal = FMath::RandRange(0, 2);

    if (FMath::Abs(CurrentPitchOffset) > CameraPitchHoldToleranceDegrees &&
        SameNonZeroCameraPitchOffsetActionCount > MaxCameraPitchOffsetActionCount)
    {
        UDSignal = (CurrentPitch > CameraPitchCenter) ? 1 : 2;
    }

    float DesiredPitch = CurrentPitch;
    if (UDSignal == 1)
    {
        DesiredPitch -= CameraPitchStepDegrees;
    }
    else if (UDSignal == 2)
    {
        DesiredPitch += CameraPitchStepDegrees;
    }
    DesiredPitch = FMath::Clamp(DesiredPitch, -30.0f, 15.0f);

    int32 EffectiveUDSignal = 0;
    if (DesiredPitch < CurrentPitch - KINDA_SMALL_NUMBER)
    {
        EffectiveUDSignal = 1;
    }
    else if (DesiredPitch > CurrentPitch + KINDA_SMALL_NUMBER)
    {
        EffectiveUDSignal = 2;
    }

    float DesiredYaw = CurrentCameraRotation.Yaw;
    if (LRSignal == 1)
    {
        DesiredYaw -= CameraYawStepDegrees;
    }
    else if (LRSignal == 2)
    {
        DesiredYaw += CameraYawStepDegrees;
    }

    OutDesiredRotation = FRotator(DesiredPitch, DesiredYaw, CurrentCameraRotation.Roll);
    return MakeCameraAction(LRSignal, EffectiveUDSignal);
}

ENPC1PExploreCameraAction ANPC_1p::MakeCameraAction(int32 LRSignal, int32 UDSignal) const
{
    if (LRSignal == 1 && UDSignal == 1)
    {
        return ENPC1PExploreCameraAction::LU;
    }
    if (LRSignal == 1 && UDSignal == 2)
    {
        return ENPC1PExploreCameraAction::LD;
    }
    if (LRSignal == 2 && UDSignal == 1)
    {
        return ENPC1PExploreCameraAction::RU;
    }
    if (LRSignal == 2 && UDSignal == 2)
    {
        return ENPC1PExploreCameraAction::RD;
    }
    if (LRSignal == 1)
    {
        return ENPC1PExploreCameraAction::L;
    }
    if (LRSignal == 2)
    {
        return ENPC1PExploreCameraAction::R;
    }
    if (UDSignal == 1)
    {
        return ENPC1PExploreCameraAction::U;
    }
    if (UDSignal == 2)
    {
        return ENPC1PExploreCameraAction::D;
    }
    return ENPC1PExploreCameraAction::None;
}

void ANPC_1p::GetCameraActionSignals(ENPC1PExploreCameraAction Action, int32& OutLR, int32& OutUD) const
{
    OutLR = 0;
    OutUD = 0;

    switch (Action)
    {
    case ENPC1PExploreCameraAction::L:
        OutLR = 1;
        break;
    case ENPC1PExploreCameraAction::R:
        OutLR = 2;
        break;
    case ENPC1PExploreCameraAction::U:
        OutUD = 1;
        break;
    case ENPC1PExploreCameraAction::D:
        OutUD = 2;
        break;
    case ENPC1PExploreCameraAction::LU:
        OutLR = 1;
        OutUD = 1;
        break;
    case ENPC1PExploreCameraAction::LD:
        OutLR = 1;
        OutUD = 2;
        break;
    case ENPC1PExploreCameraAction::RU:
        OutLR = 2;
        OutUD = 1;
        break;
    case ENPC1PExploreCameraAction::RD:
        OutLR = 2;
        OutUD = 2;
        break;
    default:
        break;
    }
}

void ANPC_1p::UpdatePitchOffsetHoldState(float CurrentPitchOffset)
{
    if (FMath::Abs(CurrentPitchOffset) <= CameraPitchHoldToleranceDegrees)
    {
        LastNonZeroCameraPitchOffset = 0.0f;
        SameNonZeroCameraPitchOffsetActionCount = 0;
        return;
    }

    if (FMath::Abs(CurrentPitchOffset - LastNonZeroCameraPitchOffset) <= CameraPitchHoldToleranceDegrees)
    {
        ++SameNonZeroCameraPitchOffsetActionCount;
    }
    else
    {
        LastNonZeroCameraPitchOffset = CurrentPitchOffset;
        SameNonZeroCameraPitchOffsetActionCount = 1;
    }
}

void ANPC_1p::SyncActorYawToCameraYaw()
{
    const USpringArmComponent* CameraBoomComp = GetCameraBoom();
    if (!CameraBoomComp)
    {
        return;
    }

    FRotator DesiredActorRot = GetActorRotation();
    DesiredActorRot.Pitch = 0.0f;
    DesiredActorRot.Roll = 0.0f;
    DesiredActorRot.Yaw = CameraBoomComp->GetComponentRotation().Yaw;
    SetActorRotation(DesiredActorRot);
}
