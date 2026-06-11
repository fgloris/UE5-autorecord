#include "NPC_1p.h"

#include "Components/CapsuleComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "NavigationSystem.h"
#include "NPCMovementRecorder.h"
#include "NPC1PCharacterMovementComponent.h"

ANPC_1p::ANPC_1p(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer.SetDefaultSubobjectClass<UNPC1PCharacterMovementComponent>(ACharacter::CharacterMovementComponentName))
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;

    ExploreActionDuration = 1.0f;

    CameraYawStepDegrees = 15.0f;
    CameraPitchStepDegrees = 10.0f;
    MaxCameraPitchOffsetActionCount = 1;
    CameraPitchHoldToleranceDegrees = 0.1f;

    bDebugDrawExploreCandidates = false;
    VisitedSoftmaxTemperature = 0.01f;

    CameraBoomLength = 0.0f;
    FirstPersonCameraRelativeLocation = FVector(0.0f, 0.0f, 60.0f);
    TurnInPlaceYawSpeedDegrees = 15.0f;
    TurnYawToleranceDegrees = 1.0f;
    // Fake animation-facing velocity used only during idle camera / turn-in-place.
    // 24 = previous 120 / 5, so in-place pacing animation is much slower.
    InPlacePaceAnimSpeed = 8.0f;

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
        CameraBoom->SetAbsolute(false, false, false);
        SetCameraBoomYawRelativePitchWorld(CameraBoom, FRotator(CameraBoomPitch, GetActorRotation().Yaw, 0.0f));
    }
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
    EndCodeOnlyInPlacePace(true);

    bIsExecutingExploreAction = false;
    CurrentExploreMoveTarget = FVector::ZeroVector;
    StartExploreActorLocation = FVector::ZeroVector;
    StartExploreFootLocation = FVector::ZeroVector;
    CurrentExploreMoveAction = ENPC1PExploreMoveAction::Idle;
    CurrentExplorePhase = ENPC1PExplorePhase::None;
    CurrentExploreActionElapsed = 0.0f;
    bWalkCameraActionStarted = false;
    CurrentTurnCameraAction = ENPC1PExploreCameraAction::None;
    bHasDesiredCameraWorldRotation = false;
    CurrentExploreCameraAction = ENPC1PExploreCameraAction::None;
    StartCameraYawRelativePitchWorld = FRotator::ZeroRotator;
    DesiredCameraYawRelativePitchWorld = FRotator::ZeroRotator;
    SetRecorderSignals(0, 0, 0, 0);
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

    bool bHasLegalMoveCandidate = false;
    for (const FExploreMoveCandidate& Candidate : Candidates)
    {
        if (Candidate.Action != ENPC1PExploreMoveAction::Idle)
        {
            bHasLegalMoveCandidate = true;
            break;
        }
    }

    bool bUseForcedCandidate = false;
    FExploreMoveCandidate ForcedCandidate;
    if (!bHasLegalMoveCandidate)
    {
        const ENPC1PExploreMoveAction OppositeAction = GetOppositeMoveAction(LastNonIdleExploreMoveAction);
        if (OppositeAction != ENPC1PExploreMoveAction::Idle)
        {
            ForcedCandidate.Action = OppositeAction;
            if (GetWorldDirectionForAction(OppositeAction, ForcedCandidate.WorldDirection))
            {
                ForcedCandidate.LandingActorLocation = GetActorLocation();
                ForcedCandidate.LandingFootLocation = GetActorLocation();
                bUseForcedCandidate = true;
            }
        }
    }

    FExploreMoveCandidate Picked;
    if (bUseForcedCandidate)
    {
        Picked = ForcedCandidate;
    }
    else
    {
        const int32 PickedIndex = SampleCandidateByVisitedSoftmax(Candidates);
        if (!Candidates.IsValidIndex(PickedIndex))
        {
            return;
        }
        Picked = Candidates[PickedIndex];
    }

    bIsExecutingExploreAction = true;
    CurrentExploreMoveAction = Picked.Action;
    CurrentExploreMoveTarget = Picked.LandingActorLocation;
    StartExploreActorLocation = GetActorLocation();
    const UCapsuleComponent* Capsule = GetCapsuleComponent();
    const float CapsuleHalfHeight = Capsule ? Capsule->GetScaledCapsuleHalfHeight() : 0.0f;
    StartExploreFootLocation = StartExploreActorLocation - FVector(0.0f, 0.0f, CapsuleHalfHeight);
    CurrentExploreActionElapsed = 0.0f;
    bWalkCameraActionStarted = false;

    if (Picked.Action == ENPC1PExploreMoveAction::Idle)
    {
        CurrentExplorePhase = ENPC1PExplorePhase::IdleCamera;
        const USpringArmComponent* CameraBoomComp = GetCameraBoom();
        StartCameraYawRelativePitchWorld = CameraBoomComp ? GetCameraBoomYawRelativePitchWorld(CameraBoomComp) : FRotator(CameraBoomPitch, GetActorRotation().Yaw, 0.0f);
        CurrentExploreCameraAction = ChooseRandomCameraAction(StartCameraYawRelativePitchWorld, DesiredCameraYawRelativePitchWorld);
        GetCameraActionSignals(CurrentExploreCameraAction, CurrentRecorderLR, CurrentRecorderUD);
        CurrentRecorderWS = 0;
        CurrentRecorderAD = 0;
        bHasDesiredCameraWorldRotation = CurrentExploreCameraAction != ENPC1PExploreCameraAction::None;
        if (CurrentExploreCameraAction != ENPC1PExploreCameraAction::None)
        {
            BeginCodeOnlyInPlacePace();
        }
    }
    else
    {
        CurrentExplorePhase = ENPC1PExplorePhase::TurnToMoveDirection;
        if (Picked.Action != ENPC1PExploreMoveAction::Idle)
        {
            LastNonIdleExploreMoveAction = Picked.Action;
        }

        StartTurnActorRotation = GetActorRotation();
        CurrentTurnCameraAction = GetTurnCameraActionForMoveAction(Picked.Action);
        DesiredTurnActorRotation = StartTurnActorRotation;
        DesiredTurnActorRotation.Yaw = StartTurnActorRotation.Yaw + GetTurnYawOffsetDegreesForMoveAction(Picked.Action, CurrentTurnCameraAction);
        DesiredTurnActorRotation.Pitch = 0.0f;
        DesiredTurnActorRotation.Roll = 0.0f;
        bHasDesiredCameraWorldRotation = false;
        CurrentExploreCameraAction = ENPC1PExploreCameraAction::None;
        DesiredCameraYawRelativePitchWorld = FRotator::ZeroRotator;

        int32 TurnLR = 0;
        int32 TurnUD = 0;
        GetCameraActionSignals(CurrentTurnCameraAction, TurnLR, TurnUD);
        SetRecorderSignals(0, 0, TurnLR, 0);

        const float YawDelta = FMath::FindDeltaAngleDegrees(StartTurnActorRotation.Yaw, DesiredTurnActorRotation.Yaw);
        if (FMath::Abs(YawDelta) <= TurnYawToleranceDegrees)
        {
            EndCodeOnlyInPlacePace(true);
            SetActorRotation(DesiredTurnActorRotation);
            BeginWalkCameraAction();
            CurrentExplorePhase = ENPC1PExplorePhase::WalkForward;
            CurrentExploreActionElapsed = 0.0f;
        }
        else
        {
            BeginCodeOnlyInPlacePace();
        }
    }
}

void ANPC_1p::ExecuteExploreAction(float DeltaTime)
{
    if (!bIsExecutingExploreAction)
    {
        return;
    }

    const bool bIsIdleAction = CurrentExploreMoveAction == ENPC1PExploreMoveAction::Idle;
    const float WalkDuration = FMath::Max(ExploreActionDuration, KINDA_SMALL_NUMBER);

    UCharacterMovementComponent* MoveComp = GetCharacterMovement();
    if (!MoveComp)
    {
        ClearExploreMoveTarget();
        return;
    }

    USpringArmComponent* CameraBoomComp = GetCameraBoom();

    if (bIsIdleAction)
    {
        const float RemainingTime = FMath::Max(WalkDuration - CurrentExploreActionElapsed, 0.0f);
        const float EffectiveDeltaTime = FMath::Clamp(DeltaTime, 0.0f, RemainingTime);
        CurrentExploreActionElapsed = FMath::Min(CurrentExploreActionElapsed + EffectiveDeltaTime, WalkDuration);
        const float Alpha = FMath::Clamp(CurrentExploreActionElapsed / WalkDuration, 0.0f, 1.0f);

        if (CameraBoomComp && bHasDesiredCameraWorldRotation)
        {
            const FRotator MixedCameraRotation = FQuat::Slerp(StartCameraYawRelativePitchWorld.Quaternion(), DesiredCameraYawRelativePitchWorld.Quaternion(), Alpha).Rotator();
            SetCameraBoomYawRelativePitchWorld(CameraBoomComp, MixedCameraRotation);
        }

        if (CurrentExploreCameraAction != ENPC1PExploreCameraAction::None)
        {
            UpdateCodeOnlyInPlacePace();
        }
        else
        {
            EndCodeOnlyInPlacePace(true);
        }

        if (CurrentExploreActionElapsed >= WalkDuration)
        {
            if (CameraBoomComp && bHasDesiredCameraWorldRotation)
            {
                SetCameraBoomYawRelativePitchWorld(CameraBoomComp, DesiredCameraYawRelativePitchWorld);
            }


            const FVector ReachedLocation = GetActorLocation();
            ClearExploreMoveTarget();
            OnExploreMoveTargetReached(ReachedLocation);
        }
        return;
    }

    if (CurrentExplorePhase == ENPC1PExplorePhase::TurnToMoveDirection)
    {
        UpdateCodeOnlyInPlacePace();
        const float CurrentYaw = GetActorRotation().Yaw;
        const float TargetYaw = DesiredTurnActorRotation.Yaw;
        const float YawDeltaBefore = FMath::FindDeltaAngleDegrees(CurrentYaw, TargetYaw);
        const float MaxYawStep = FMath::Max(TurnInPlaceYawSpeedDegrees, 1.0f) * FMath::Max(DeltaTime, 0.0f);

        int32 TurnLR = 0;
        int32 TurnUD = 0;
        GetCameraActionSignals(CurrentTurnCameraAction, TurnLR, TurnUD);
        SetRecorderSignals(0, 0, FMath::Abs(YawDeltaBefore) > TurnYawToleranceDegrees ? TurnLR : 0, 0);

        float NewYaw = CurrentYaw;
        if (CurrentTurnCameraAction == ENPC1PExploreCameraAction::L)
        {
            NewYaw = CurrentYaw - MaxYawStep;
            if (FMath::Abs(FMath::FindDeltaAngleDegrees(NewYaw, TargetYaw)) > FMath::Abs(YawDeltaBefore))
            {
                NewYaw = TargetYaw;
            }
        }
        else if (CurrentTurnCameraAction == ENPC1PExploreCameraAction::R)
        {
            NewYaw = CurrentYaw + MaxYawStep;
            if (FMath::Abs(FMath::FindDeltaAngleDegrees(NewYaw, TargetYaw)) > FMath::Abs(YawDeltaBefore))
            {
                NewYaw = TargetYaw;
            }
        }
        else
        {
            NewYaw = TargetYaw;
        }

        SetActorRotation(FRotator(0.0f, NewYaw, 0.0f));

        const float YawDeltaRemaining = FMath::FindDeltaAngleDegrees(GetActorRotation().Yaw, TargetYaw);

        // Turning has no fixed duration now, so drive the stepping pose by real time.
        // The custom movement component exposes fake Velocity/Acceleration to AnimBP
        // while PhysWalking is intercepted so the Actor never translates.
        CurrentExploreActionElapsed += DeltaTime;
        UpdateCodeOnlyInPlacePace();

        if (FMath::Abs(YawDeltaBefore) <= TurnYawToleranceDegrees || FMath::Abs(YawDeltaRemaining) <= TurnYawToleranceDegrees)
        {
            EndCodeOnlyInPlacePace(true);
            SetActorRotation(DesiredTurnActorRotation);
            BeginWalkCameraAction();
            CurrentExplorePhase = ENPC1PExplorePhase::WalkForward;
            CurrentExploreActionElapsed = 0.0f;
        }
        return;
    }

    if (CurrentExplorePhase != ENPC1PExplorePhase::WalkForward)
    {
        EndCodeOnlyInPlacePace(true);
        BeginWalkCameraAction();
        CurrentExplorePhase = ENPC1PExplorePhase::WalkForward;
        CurrentExploreActionElapsed = 0.0f;
    }

    const float RemainingTime = FMath::Max(WalkDuration - CurrentExploreActionElapsed, 0.0f);
    const float EffectiveDeltaTime = FMath::Clamp(DeltaTime, 0.0f, RemainingTime);
    CurrentExploreActionElapsed = FMath::Min(CurrentExploreActionElapsed + EffectiveDeltaTime, WalkDuration);
    const float WalkAlpha = FMath::Clamp(CurrentExploreActionElapsed / WalkDuration, 0.0f, 1.0f);
    const float InputScaleByFrame = (DeltaTime > KINDA_SMALL_NUMBER) ? (EffectiveDeltaTime / DeltaTime) : 0.0f;

    if (CameraBoomComp && bHasDesiredCameraWorldRotation)
    {
        const FRotator MixedCameraRotation = FQuat::Slerp(StartCameraYawRelativePitchWorld.Quaternion(), DesiredCameraYawRelativePitchWorld.Quaternion(), WalkAlpha).Rotator();
        SetCameraBoomYawRelativePitchWorld(CameraBoomComp, MixedCameraRotation);
    }

    EndCodeOnlyInPlacePace(true);
    MoveComp->SetMovementMode(MOVE_Walking);
    AddMovementInput(GetActorForwardVector().GetSafeNormal2D(), InputScaleByFrame);

    if (CurrentExploreActionElapsed >= WalkDuration)
    {
        if (CameraBoomComp && bHasDesiredCameraWorldRotation)
        {
            SetCameraBoomYawRelativePitchWorld(CameraBoomComp, DesiredCameraYawRelativePitchWorld);
        }

        const FVector ReachedLocation = GetActorLocation();
        ClearExploreMoveTarget();
        OnExploreMoveTargetReached(ReachedLocation);
    }
}

void ANPC_1p::BuildLegalActionCandidates(TArray<FExploreMoveCandidate>& OutCandidates) const
{
    OutCandidates.Reset();

    const ENPC1PExploreMoveAction AllActions[9] =
    {
        ENPC1PExploreMoveAction::Idle,
        ENPC1PExploreMoveAction::W,
        ENPC1PExploreMoveAction::A,
        ENPC1PExploreMoveAction::S,
        ENPC1PExploreMoveAction::D,
        ENPC1PExploreMoveAction::WA,
        ENPC1PExploreMoveAction::WD,
        ENPC1PExploreMoveAction::SA,
        ENPC1PExploreMoveAction::SD
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
        OutCandidate.VisitedScore = GetVisitedScoreAtLocation(OutCandidate.LandingFootLocation);
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
    FVector ActorForward = GetActorForwardVector().GetSafeNormal2D();
    if (ActorForward.IsNearlyZero())
    {
        ActorForward = FVector::ForwardVector;
    }

    FVector ActorRight = GetActorRightVector().GetSafeNormal2D();
    if (ActorRight.IsNearlyZero())
    {
        ActorRight = FVector::RightVector;
    }

    switch (Action)
    {
    case ENPC1PExploreMoveAction::W:
        OutDirection = ActorForward;
        break;
    case ENPC1PExploreMoveAction::A:
        OutDirection = -ActorRight;
        break;
    case ENPC1PExploreMoveAction::S:
        OutDirection = -ActorForward;
        break;
    case ENPC1PExploreMoveAction::D:
        OutDirection = ActorRight;
        break;
    case ENPC1PExploreMoveAction::WA:
        OutDirection = (ActorForward - ActorRight).GetSafeNormal();
        break;
    case ENPC1PExploreMoveAction::WD:
        OutDirection = (ActorForward + ActorRight).GetSafeNormal();
        break;
    case ENPC1PExploreMoveAction::SA:
        OutDirection = (-ActorForward - ActorRight).GetSafeNormal();
        break;
    case ENPC1PExploreMoveAction::SD:
        OutDirection = (-ActorForward + ActorRight).GetSafeNormal();
        break;
    default:
        OutDirection = FVector::ZeroVector;
        break;
    }

    return !OutDirection.IsNearlyZero();
}

void ANPC_1p::GetMoveActionSignals(ENPC1PExploreMoveAction Action, int32& OutWS, int32& OutAD) const
{
    // First-person data should not record strafing/backward intent.
    // The 8-direction sample is only used to pick a target direction; the NPC turns first,
    // then walks forward with W once aligned.
    OutWS = (Action == ENPC1PExploreMoveAction::Idle) ? 0 : 1;
    OutAD = 0;
}

ENPC1PExploreMoveAction ANPC_1p::GetOppositeMoveAction(ENPC1PExploreMoveAction Action) const
{
    switch (Action)
    {
    case ENPC1PExploreMoveAction::W:
        return ENPC1PExploreMoveAction::S;
    case ENPC1PExploreMoveAction::S:
        return ENPC1PExploreMoveAction::W;
    case ENPC1PExploreMoveAction::A:
        return ENPC1PExploreMoveAction::D;
    case ENPC1PExploreMoveAction::D:
        return ENPC1PExploreMoveAction::A;
    case ENPC1PExploreMoveAction::WA:
        return ENPC1PExploreMoveAction::SD;
    case ENPC1PExploreMoveAction::WD:
        return ENPC1PExploreMoveAction::SA;
    case ENPC1PExploreMoveAction::SA:
        return ENPC1PExploreMoveAction::WD;
    case ENPC1PExploreMoveAction::SD:
        return ENPC1PExploreMoveAction::WA;
    default:
        return ENPC1PExploreMoveAction::Idle;
    }
}

ENPC1PExploreCameraAction ANPC_1p::GetTurnCameraActionForMoveAction(ENPC1PExploreMoveAction Action) const
{
    switch (Action)
    {
    case ENPC1PExploreMoveAction::A:
    case ENPC1PExploreMoveAction::WA:
    case ENPC1PExploreMoveAction::SA:
        return ENPC1PExploreCameraAction::L;
    case ENPC1PExploreMoveAction::D:
    case ENPC1PExploreMoveAction::WD:
    case ENPC1PExploreMoveAction::SD:
        return ENPC1PExploreCameraAction::R;
    case ENPC1PExploreMoveAction::S:
        return FMath::RandBool() ? ENPC1PExploreCameraAction::L : ENPC1PExploreCameraAction::R;
    default:
        return ENPC1PExploreCameraAction::None;
    }
}

float ANPC_1p::GetTurnYawOffsetDegreesForMoveAction(ENPC1PExploreMoveAction Action, ENPC1PExploreCameraAction TurnAction) const
{
    switch (Action)
    {
    case ENPC1PExploreMoveAction::W:
        return 0.0f;
    case ENPC1PExploreMoveAction::WA:
        return -45.0f;
    case ENPC1PExploreMoveAction::A:
        return -90.0f;
    case ENPC1PExploreMoveAction::SA:
        return -135.0f;
    case ENPC1PExploreMoveAction::S:
        return (TurnAction == ENPC1PExploreCameraAction::L) ? -180.0f : 180.0f;
    case ENPC1PExploreMoveAction::SD:
        return 135.0f;
    case ENPC1PExploreMoveAction::D:
        return 90.0f;
    case ENPC1PExploreMoveAction::WD:
        return 45.0f;
    default:
        return 0.0f;
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

    OutCandidate.WorldDirection = Dir2D;
    OutCandidate.LandingFootLocation = LandingFootLocation;
    OutCandidate.LandingActorLocation = LandingActorLocation;
    OutCandidate.VisitedScore = GetVisitedScoreAtLocation(LandingFootLocation);

    if (bDebugDrawExploreCandidates)
    {
        const float ColorScalar = FMath::Clamp(1.0f / (1.0f + OutCandidate.VisitedScore), 0.0f, 1.0f);
        const FColor Color = FColor::MakeRedToGreenColorFromScalar(ColorScalar);
        const float DrawDuration = FMath::Max(ExploreActionDuration + 180.0f / FMath::Max(TurnInPlaceYawSpeedDegrees, 1.0f), KINDA_SMALL_NUMBER);
        DrawDebugSphere(World, LandingActorLocation, 12.0f, 8, Color, false, DrawDuration);
        DrawDebugLine(World, StartActorLocation, LandingActorLocation, Color, false, DrawDuration, 0, 1.5f);
    }

    return true;
}

float ANPC_1p::GetVisitedScoreAtLocation(const FVector& WorldLocation) const
{
    const int32 GridX = FMath::RoundToInt(WorldLocation.X / GridSize);
    const int32 GridY = FMath::RoundToInt(WorldLocation.Y / GridSize);
    const FString GridKey = FString::Printf(TEXT("%d_%d"), GridX, GridY);

    if (const float* FoundScore = Visited.Find(GridKey))
    {
        return *FoundScore;
    }

    return 0.0f;
}

int32 ANPC_1p::SampleCandidateByVisitedSoftmax(const TArray<FExploreMoveCandidate>& Candidates) const
{
    if (Candidates.Num() <= 0)
    {
        return INDEX_NONE;
    }

    constexpr float VisitedScoreEpsilon = 0.01f;

    TArray<float> Preferences;
    Preferences.Reserve(Candidates.Num());

    const float Temperature = FMath::Max(VisitedSoftmaxTemperature, KINDA_SMALL_NUMBER);
    float MaxPreference = TNumericLimits<float>::Lowest();
    for (const FExploreMoveCandidate& Candidate : Candidates)
    {
        const float Preference = (1.0f / FMath::Max(Candidate.VisitedScore + VisitedScoreEpsilon, VisitedScoreEpsilon)) / Temperature;
        Preferences.Add(Preference);
        MaxPreference = FMath::Max(MaxPreference, Preference);
    }

    float TotalWeight = 0.0f;
    TArray<float> Weights;
    Weights.Reserve(Candidates.Num());
    for (float Preference : Preferences)
    {
        const float Weight = FMath::Exp(Preference - MaxPreference);
        Weights.Add(Weight);
        TotalWeight += Weight;
    }

    if (TotalWeight <= KINDA_SMALL_NUMBER)
    {
        return FMath::RandRange(0, Candidates.Num() - 1);
    }

    float RandomValue = FMath::FRandRange(0.0f, TotalWeight);
    for (int32 Index = 0; Index < Weights.Num(); ++Index)
    {
        RandomValue -= Weights[Index];
        if (RandomValue <= 0.0f)
        {
            return Index;
        }
    }

    return Candidates.Num() - 1;
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

FRotator ANPC_1p::GetCameraBoomYawRelativePitchWorld(const USpringArmComponent* CameraBoomComp) const
{
    if (!CameraBoomComp)
    {
        return FRotator(CameraBoomPitch, GetActorRotation().Yaw, 0.0f);
    }

    const float WorldPitch = CameraBoomComp->GetComponentRotation().Pitch;
    const float ActorWorldYaw = GetActorRotation().Yaw;
    return FRotator(WorldPitch, ActorWorldYaw, 0.0f);
}

void ANPC_1p::SetCameraBoomYawRelativePitchWorld(USpringArmComponent* CameraBoomComp, const FRotator& MixedCameraRotation)
{
    if (!CameraBoomComp)
    {
        return;
    }

    CameraBoomComp->SetAbsolute(false, false, false);

    FRotator ActorRotation = GetActorRotation();
    ActorRotation.Yaw = MixedCameraRotation.Yaw;
    ActorRotation.Pitch = 0.0f;
    ActorRotation.Roll = 0.0f;
    SetActorRotation(ActorRotation);

    CameraBoomComp->SetRelativeRotation(FRotator(0.0f, 0.0f, 0.0f));
    CameraBoomComp->SetWorldRotation(FRotator(MixedCameraRotation.Pitch, GetActorRotation().Yaw, 0.0f));
}

void ANPC_1p::BeginWalkCameraAction()
{
    if (bWalkCameraActionStarted)
    {
        return;
    }

    bWalkCameraActionStarted = true;
    const USpringArmComponent* CameraBoomComp = GetCameraBoom();
    StartCameraYawRelativePitchWorld = CameraBoomComp ? GetCameraBoomYawRelativePitchWorld(CameraBoomComp) : FRotator(CameraBoomPitch, GetActorRotation().Yaw, 0.0f);
    CurrentExploreCameraAction = ChooseRandomCameraAction(StartCameraYawRelativePitchWorld, DesiredCameraYawRelativePitchWorld);
    GetCameraActionSignals(CurrentExploreCameraAction, CurrentRecorderLR, CurrentRecorderUD);
    CurrentRecorderWS = 1;
    CurrentRecorderAD = 0;
    bHasDesiredCameraWorldRotation = CurrentExploreCameraAction != ENPC1PExploreCameraAction::None;
}


UNPC1PCharacterMovementComponent* ANPC_1p::GetNPC1PMovementComponent() const
{
    return Cast<UNPC1PCharacterMovementComponent>(GetCharacterMovement());
}

void ANPC_1p::BeginCodeOnlyInPlacePace()
{
    if (InPlacePaceAnimSpeed <= KINDA_SMALL_NUMBER)
    {
        EndCodeOnlyInPlacePace(true);
        return;
    }

    UNPC1PCharacterMovementComponent* MoveComp = GetNPC1PMovementComponent();
    if (!MoveComp)
    {
        return;
    }

    FVector FakeVelocity = GetActorForwardVector().GetSafeNormal2D() * InPlacePaceAnimSpeed;
    if (FakeVelocity.IsNearlyZero())
    {
        FakeVelocity = FVector::ForwardVector * InPlacePaceAnimSpeed;
    }

    MoveComp->StartInPlacePace(FakeVelocity);

    // Some AnimBPs also check movement input / acceleration in addition to velocity.
    // Our custom movement component consumes this without translating the Actor.
    AddMovementInput(FakeVelocity.GetSafeNormal2D(), 1.0f, true);
}

void ANPC_1p::UpdateCodeOnlyInPlacePace()
{
    if (InPlacePaceAnimSpeed <= KINDA_SMALL_NUMBER)
    {
        EndCodeOnlyInPlacePace(true);
        return;
    }

    UNPC1PCharacterMovementComponent* MoveComp = GetNPC1PMovementComponent();
    if (!MoveComp)
    {
        return;
    }

    FVector FakeVelocity = GetActorForwardVector().GetSafeNormal2D() * InPlacePaceAnimSpeed;
    if (FakeVelocity.IsNearlyZero())
    {
        FakeVelocity = FVector::ForwardVector * InPlacePaceAnimSpeed;
    }

    if (!MoveComp->IsInPlacePaceMode())
    {
        MoveComp->StartInPlacePace(FakeVelocity);
    }
    else
    {
        MoveComp->UpdateInPlacePaceVelocity(FakeVelocity);
    }

    // Some AnimBPs also check movement input / acceleration in addition to velocity.
    // Our custom movement component consumes this without translating the Actor.
    AddMovementInput(FakeVelocity.GetSafeNormal2D(), 1.0f, true);
}

void ANPC_1p::EndCodeOnlyInPlacePace(bool bRestoreWalking)
{
    if (UNPC1PCharacterMovementComponent* MoveComp = GetNPC1PMovementComponent())
    {
        if (MoveComp->IsInPlacePaceMode())
        {
            MoveComp->StopInPlacePace(bRestoreWalking);
        }
    }
}

void ANPC_1p::SetRecorderSignals(int32 WS, int32 AD, int32 LR, int32 UD)
{
    CurrentRecorderWS = WS;
    CurrentRecorderAD = AD;
    CurrentRecorderLR = LR;
    CurrentRecorderUD = UD;
}

