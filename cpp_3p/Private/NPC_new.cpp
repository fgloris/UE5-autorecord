#include "NPC_new.h"

#include "Components/CapsuleComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "NavigationSystem.h"
#include "NPCMovementRecorder.h"

ANPC_new::ANPC_new()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;

    ExploreActionDuration = 1.0f;

    CameraYawStepDegrees = 30.0f;
    CameraPitchStepDegrees = 15.0f;
    MinCameraPitchDegrees = -15.0f;
    MaxCameraPitchOffsetActionCount = 4;
    CameraPitchHoldToleranceDegrees = 1.0f;

    bUpdateVisitedBeforeSampling = true;
    bDebugDrawExploreCandidates = false;
}

void ANPC_new::BeginPlay()
{
    Super::BeginPlay();
}

void ANPC_new::ExecuteNextStep(float DeltaTime)
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

void ANPC_new::ClearExploreMoveTarget()
{
    bIsExecutingExploreAction = false;
    CurrentExploreMoveTarget = FVector::ZeroVector;
    CurrentExploreMoveDirection = FVector::ZeroVector;
    CurrentExploreActionElapsed = 0.0f;
    bHasDesiredCameraWorldRotation = false;
    CurrentExploreCameraAction = ENPCExploreCameraAction::None;
    StartCameraWorldRotation = FRotator::ZeroRotator;
    DesiredCameraWorldRotation = FRotator::ZeroRotator;
}

void ANPC_new::GetCurrentRecorderControlSignals(int32& OutWS, int32& OutAD, int32& OutLR, int32& OutUD) const
{
    OutWS = CurrentRecorderWS;
    OutAD = CurrentRecorderAD;
    OutLR = CurrentRecorderLR;
    OutUD = CurrentRecorderUD;
}

void ANPC_new::StartExploreAction()
{
    if (bUpdateVisitedBeforeSampling)
    {
        UpdateVisitedStatsAtCurrentPosition();
    }

    TArray<FExploreMoveCandidate> Candidates;
    BuildLegalActionCandidates(Candidates);

    if (Candidates.Num() <= 0)
    {
        if (bDebugDrawExploreCandidates)
        {
            UE_LOG(LogTemp, Warning, TEXT("NPC_new[%s] no legal action this tick"), *GetActorLabel());
        }
        return;
    }

    const FExploreMoveCandidate& Picked = Candidates[FMath::RandRange(0, Candidates.Num() - 1)];
    bIsExecutingExploreAction = true;
    CurrentExploreMoveDirection = Picked.WorldDirection.GetSafeNormal2D();
    CurrentExploreMoveTarget = Picked.LandingActorLocation;
    CurrentExploreActionElapsed = 0.0f;
    GetMoveActionSignals(Picked.Action, CurrentRecorderWS, CurrentRecorderAD);

    const USpringArmComponent* CameraBoomComp = GetCameraBoom();
    StartCameraWorldRotation = CameraBoomComp ? CameraBoomComp->GetComponentRotation() : FRotator(CameraBoomPitch, 0.0f, 0.0f);
    CurrentExploreCameraAction = ChooseRandomCameraAction(StartCameraWorldRotation, DesiredCameraWorldRotation);
    GetCameraActionSignals(CurrentExploreCameraAction, CurrentRecorderLR, CurrentRecorderUD);
    bHasDesiredCameraWorldRotation = CurrentExploreCameraAction != ENPCExploreCameraAction::None;
}

void ANPC_new::ExecuteExploreAction(float DeltaTime)
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
    if (!MoveComp || CurrentExploreMoveDirection.IsNearlyZero())
    {
        ClearExploreMoveTarget();
        return;
    }

    MoveComp->SetMovementMode(MOVE_Walking);
    const float InputScale = (DeltaTime > KINDA_SMALL_NUMBER) ? (EffectiveDeltaTime / DeltaTime) : 0.0f;
    AddMovementInput(CurrentExploreMoveDirection, InputScale);

    FRotator DesiredActorRot = CurrentExploreMoveDirection.Rotation();
    DesiredActorRot.Pitch = 0.0f;
    DesiredActorRot.Roll = 0.0f;
    SetActorRotation(FQuat::Slerp(GetActorRotation().Quaternion(), DesiredActorRot.Quaternion(), Alpha).Rotator());

    USpringArmComponent* CameraBoomComp = GetCameraBoom();
    if (CameraBoomComp && bHasDesiredCameraWorldRotation)
    {
        CameraBoomComp->SetWorldRotation(FQuat::Slerp(StartCameraWorldRotation.Quaternion(), DesiredCameraWorldRotation.Quaternion(), Alpha).Rotator());
    }

    if (CurrentExploreActionElapsed >= Duration)
    {
        const FVector ReachedLocation = GetActorLocation();
        if (CameraBoomComp && bHasDesiredCameraWorldRotation)
        {
            CameraBoomComp->SetWorldRotation(DesiredCameraWorldRotation);
        }

        ClearExploreMoveTarget();
        UpdateVisitedStatsAtCurrentPosition();
        OnExploreMoveTargetReached(ReachedLocation);
    }
}

void ANPC_new::BuildLegalActionCandidates(TArray<FExploreMoveCandidate>& OutCandidates) const
{
    OutCandidates.Reset();

    const ENPCExploreMoveAction AllActions[8] =
    {
        ENPCExploreMoveAction::W,
        ENPCExploreMoveAction::A,
        ENPCExploreMoveAction::S,
        ENPCExploreMoveAction::D,
        ENPCExploreMoveAction::WA,
        ENPCExploreMoveAction::WD,
        ENPCExploreMoveAction::SA,
        ENPCExploreMoveAction::SD
    };

    for (ENPCExploreMoveAction Action : AllActions)
    {
        FExploreMoveCandidate Candidate;
        if (TryBuildActionCandidate(Action, Candidate))
        {
            OutCandidates.Add(Candidate);
        }
    }
}

bool ANPC_new::TryBuildActionCandidate(ENPCExploreMoveAction Action, FExploreMoveCandidate& OutCandidate) const
{
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

bool ANPC_new::GetWorldDirectionForAction(ENPCExploreMoveAction Action, FVector& OutDirection) const
{
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

    FVector CamRight = CameraBoomComp->GetRightVector();
    CamRight.Z = 0.0f;
    CamRight = CamRight.GetSafeNormal();
    if (CamRight.IsNearlyZero())
    {
        CamRight = GetActorRightVector().GetSafeNormal2D();
    }

    switch (Action)
    {
    case ENPCExploreMoveAction::W:
        OutDirection = CamForward;
        break;
    case ENPCExploreMoveAction::A:
        OutDirection = -CamRight;
        break;
    case ENPCExploreMoveAction::S:
        OutDirection = -CamForward;
        break;
    case ENPCExploreMoveAction::D:
        OutDirection = CamRight;
        break;
    case ENPCExploreMoveAction::WA:
        OutDirection = (CamForward - CamRight).GetSafeNormal();
        break;
    case ENPCExploreMoveAction::WD:
        OutDirection = (CamForward + CamRight).GetSafeNormal();
        break;
    case ENPCExploreMoveAction::SA:
        OutDirection = (-CamForward - CamRight).GetSafeNormal();
        break;
    case ENPCExploreMoveAction::SD:
        OutDirection = (-CamForward + CamRight).GetSafeNormal();
        break;
    default:
        OutDirection = FVector::ZeroVector;
        break;
    }

    return !OutDirection.IsNearlyZero();
}

void ANPC_new::GetMoveActionSignals(ENPCExploreMoveAction Action, int32& OutWS, int32& OutAD) const
{
    OutWS = 0;
    OutAD = 0;

    switch (Action)
    {
    case ENPCExploreMoveAction::W:
        OutWS = 1;
        break;
    case ENPCExploreMoveAction::S:
        OutWS = 2;
        break;
    case ENPCExploreMoveAction::A:
        OutAD = 2;
        break;
    case ENPCExploreMoveAction::D:
        OutAD = 1;
        break;
    case ENPCExploreMoveAction::WA:
        OutWS = 1;
        OutAD = 2;
        break;
    case ENPCExploreMoveAction::WD:
        OutWS = 1;
        OutAD = 1;
        break;
    case ENPCExploreMoveAction::SA:
        OutWS = 2;
        OutAD = 2;
        break;
    case ENPCExploreMoveAction::SD:
        OutWS = 2;
        OutAD = 1;
        break;
    default:
        break;
    }
}

bool ANPC_new::IsLandingValidForDirection(const FVector& DesiredWorldDirection, FExploreMoveCandidate& OutCandidate) const
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

    if (FMath::Abs(LandingFootLocation.Z - StartFootLocation.Z) > MaxStepHeight)
    {
        return false;
    }

    if (!IsLocationValidForNPC(LandingFootLocation))
    {
        return false;
    }

    const FVector LandingActorLocation = LandingFootLocation + FVector(0.0f, 0.0f, CapsuleHalfHeight);
    if (!IsMovePathCollisionFree(StartActorLocation, LandingActorLocation))
    {
        return false;
    }

    OutCandidate.WorldDirection = Dir2D;
    OutCandidate.LandingFootLocation = LandingFootLocation;
    OutCandidate.LandingActorLocation = LandingActorLocation;

    if (bDebugDrawExploreCandidates)
    {
        DrawDebugSphere(World, LandingActorLocation, 12.0f, 8, FColor::Green, false, 0.2f);
        DrawDebugLine(World, StartActorLocation, LandingActorLocation, FColor::Green, false, 0.2f, 0, 1.5f);
    }

    return true;
}

bool ANPC_new::IsMovePathCollisionFree(const FVector& StartActorLocation, const FVector& EndActorLocation) const
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

    FCollisionQueryParams Params(SCENE_QUERY_STAT(NPCNewCapsuleOverlap), false, this);
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

ENPCExploreCameraAction ANPC_new::ChooseRandomCameraAction(const FRotator& CurrentCameraRotation, FRotator& OutDesiredRotation)
{
    const float MaxPitchOffset = FMath::Abs(MinCameraPitchDegrees);
    const float CurrentPitchOffset = FMath::Clamp(
        CurrentCameraRotation.Pitch - CameraBoomPitch,
        -MaxPitchOffset,
        MaxPitchOffset);

    UpdatePitchOffsetHoldState(CurrentPitchOffset);

    const int32 LRSignal = FMath::RandRange(0, 2);
    int32 UDSignal = FMath::RandRange(0, 2);

    if (FMath::Abs(CurrentPitchOffset) > CameraPitchHoldToleranceDegrees &&
        SameNonZeroCameraPitchOffsetActionCount > MaxCameraPitchOffsetActionCount)
    {
        UDSignal = (CurrentPitchOffset > 0.0f) ? 2 : 1;
    }

    float DesiredPitchOffset = CurrentPitchOffset;
    if (UDSignal == 1)
    {
        DesiredPitchOffset += CameraPitchStepDegrees;
    }
    else if (UDSignal == 2)
    {
        DesiredPitchOffset -= CameraPitchStepDegrees;
    }
    DesiredPitchOffset = FMath::Clamp(DesiredPitchOffset, -MaxPitchOffset, MaxPitchOffset);

    int32 EffectiveUDSignal = 0;
    if (DesiredPitchOffset > CurrentPitchOffset + KINDA_SMALL_NUMBER)
    {
        EffectiveUDSignal = 1;
    }
    else if (DesiredPitchOffset < CurrentPitchOffset - KINDA_SMALL_NUMBER)
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

    OutDesiredRotation = FRotator(CameraBoomPitch + DesiredPitchOffset, DesiredYaw, CurrentCameraRotation.Roll);
    return MakeCameraAction(LRSignal, EffectiveUDSignal);
}

ENPCExploreCameraAction ANPC_new::MakeCameraAction(int32 LRSignal, int32 UDSignal) const
{
    if (LRSignal == 1 && UDSignal == 1)
    {
        return ENPCExploreCameraAction::LU;
    }
    if (LRSignal == 1 && UDSignal == 2)
    {
        return ENPCExploreCameraAction::LD;
    }
    if (LRSignal == 2 && UDSignal == 1)
    {
        return ENPCExploreCameraAction::RU;
    }
    if (LRSignal == 2 && UDSignal == 2)
    {
        return ENPCExploreCameraAction::RD;
    }
    if (LRSignal == 1)
    {
        return ENPCExploreCameraAction::L;
    }
    if (LRSignal == 2)
    {
        return ENPCExploreCameraAction::R;
    }
    if (UDSignal == 1)
    {
        return ENPCExploreCameraAction::U;
    }
    if (UDSignal == 2)
    {
        return ENPCExploreCameraAction::D;
    }
    return ENPCExploreCameraAction::None;
}

void ANPC_new::GetCameraActionSignals(ENPCExploreCameraAction Action, int32& OutLR, int32& OutUD) const
{
    OutLR = 0;
    OutUD = 0;

    switch (Action)
    {
    case ENPCExploreCameraAction::L:
        OutLR = 1;
        break;
    case ENPCExploreCameraAction::R:
        OutLR = 2;
        break;
    case ENPCExploreCameraAction::U:
        OutUD = 1;
        break;
    case ENPCExploreCameraAction::D:
        OutUD = 2;
        break;
    case ENPCExploreCameraAction::LU:
        OutLR = 1;
        OutUD = 1;
        break;
    case ENPCExploreCameraAction::LD:
        OutLR = 1;
        OutUD = 2;
        break;
    case ENPCExploreCameraAction::RU:
        OutLR = 2;
        OutUD = 1;
        break;
    case ENPCExploreCameraAction::RD:
        OutLR = 2;
        OutUD = 2;
        break;
    default:
        break;
    }
}

void ANPC_new::UpdatePitchOffsetHoldState(float CurrentPitchOffset)
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
