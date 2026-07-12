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
    CameraPitchHoldToleranceDegrees = 0.1f;

    bDebugDrawExploreCandidates = false;
    VisitedSoftmaxTemperature = 0.01f;

    CameraBoomLength = 0.0f;
    FirstPersonCameraRelativeLocation = FVector(0.0f, 0.0f, 60.0f);
    TurnYawToleranceDegrees = 1.0f;
    // Fake animation-facing velocity used only during idle camera / turn-in-place.
    // 24 = previous 120 / 5, so in-place pacing animation is much slower.
    InPlacePaceAnimSpeed = 8.0f;

    CurrentDesiredPitch = CameraBoomPitch;

    if (CameraBoom)
    {
        CameraBoom->TargetArmLength = CameraBoomLength;
        CameraBoom->bDoCollisionTest = false;
        CameraBoom->SetRelativeLocation(FirstPersonCameraRelativeLocation);
    }

    SocialTurnMinInterval = 10.0f;
    SocialTurnMaxInterval = 25.0f;
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
    }

    CurrentDesiredYaw = GetActorRotation().Yaw;
    CurrentDesiredPitch = CameraBoomPitch;
    EnterPitchState(ENPC1PPitchState::Center);
}

void ANPC_1p::ExecuteNextStep(float DeltaTime)
{
    if (!GetWorld())
    {
        return;
    }

    // Count down social turn cooldown in real time.
    SocialTurnCooldownRemaining -= DeltaTime;

    if (!bIsExecutingExploreAction)
    {
        if (SocialTurnCooldownRemaining <= 0.0f)
        {
            StartSocialTurn();
        }

        if (!bIsExecutingExploreAction)
        {
            StartExploreAction();
        }
    }

    bWasExecutingThisFrameForRecording = bIsExecutingExploreAction;
    ExecuteExploreAction(DeltaTime);
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
    DesiredCameraYawRelativePitchWorld = FRotator::ZeroRotator;
    CurrentDesiredYaw = GetActorRotation().Yaw;
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
        const FRotator CurrentCameraRot(CameraBoomPitch, GetActorRotation().Yaw, 0.0f);
        CurrentExploreCameraAction = ChooseRandomCameraAction(CurrentCameraRot, DesiredCameraYawRelativePitchWorld);
        CurrentDesiredYaw = DesiredCameraYawRelativePitchWorld.Yaw;
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

    if (CurrentExplorePhase == ENPC1PExplorePhase::SocialTurnToPeer)
    {
        // Live-track the target position each frame (other NPCs are moving).
        FVector ToTarget = FVector::ZeroVector;
        if (SocialTurnTargetNPC && IsValid(SocialTurnTargetNPC))
        {
            ToTarget = (SocialTurnTargetNPC->GetActorLocation() - GetActorLocation()).GetSafeNormal2D();
        }

        // Centralize yaw control — ApplyDesiredRotation reads CurrentDesiredYaw and interpolates atomically.
        const float TargetYaw = !ToTarget.IsNearlyZero() ? ToTarget.Rotation().Yaw : DesiredTurnActorRotation.Yaw;
        CurrentDesiredYaw = TargetYaw;

        UpdateCodeOnlyInPlacePace();

        // Completion check uses the current actor yaw (set by ApplyDesiredRotation last frame).
        const float YawDelta = FMath::FindDeltaAngleDegrees(GetActorRotation().Yaw, CurrentDesiredYaw);
        if (FMath::Abs(YawDelta) <= TurnYawToleranceDegrees)
        {
            AActor* TargetNPC = SocialTurnTargetNPC;
            ClearExploreMoveTarget();
            OnSocialTurnExecuted(TargetNPC);
            // Start next explore action immediately instead of waiting for next frame.
            StartExploreAction();

            if (GEngine && TargetNPC)
            {
                GEngine->AddOnScreenDebugMessage(
                    INDEX_NONE,
                    3.0f,
                    FColor::Green,
                    FString::Printf(TEXT("NPC_1p[%s] Social Turn DONE -> %s"), *GetActorLabel(), *TargetNPC->GetActorLabel()));
            }
        }
        return;
    }

    if (bIsIdleAction)
    {
        const float RemainingTime = FMath::Max(WalkDuration - CurrentExploreActionElapsed, 0.0f);
        const float EffectiveDeltaTime = FMath::Clamp(DeltaTime, 0.0f, RemainingTime);
        CurrentExploreActionElapsed = FMath::Min(CurrentExploreActionElapsed + EffectiveDeltaTime, WalkDuration);

        // Camera yaw/pitch are interpolated atomically by ApplyDesiredRotation via CurrentDesiredYaw/CurrentDesiredPitch.

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
            const FVector ReachedLocation = GetActorLocation();
            ClearExploreMoveTarget();
            OnExploreMoveTargetReached(ReachedLocation);
        }
        return;
    }

    if (CurrentExplorePhase == ENPC1PExplorePhase::TurnToMoveDirection)
    {
        UpdateCodeOnlyInPlacePace();

        // Centralize yaw control — ApplyDesiredRotation reads CurrentDesiredYaw and interpolates atomically.
        CurrentDesiredYaw = DesiredTurnActorRotation.Yaw;

        const float CurrentYaw = GetActorRotation().Yaw;
        const float YawDeltaBefore = FMath::FindDeltaAngleDegrees(CurrentYaw, CurrentDesiredYaw);

        int32 TurnLR = 0;
        int32 TurnUD = 0;
        GetCameraActionSignals(CurrentTurnCameraAction, TurnLR, TurnUD);
        SetRecorderSignals(0, 0, FMath::Abs(YawDeltaBefore) > TurnYawToleranceDegrees ? TurnLR : 0, 0);

        CurrentExploreActionElapsed += DeltaTime;
        UpdateCodeOnlyInPlacePace();

        // Completion check — actor yaw moves toward CurrentDesiredYaw via ApplyDesiredRotation.
        const float YawDeltaRemaining = FMath::FindDeltaAngleDegrees(GetActorRotation().Yaw, CurrentDesiredYaw);
        if (FMath::Abs(YawDeltaBefore) <= TurnYawToleranceDegrees || FMath::Abs(YawDeltaRemaining) <= TurnYawToleranceDegrees)
        {
            EndCodeOnlyInPlacePace(true);
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
    const float InputScaleByFrame = (DeltaTime > KINDA_SMALL_NUMBER) ? (EffectiveDeltaTime / DeltaTime) : 0.0f;

    // Camera yaw/pitch are interpolated atomically by ApplyDesiredRotation via CurrentDesiredYaw/CurrentDesiredPitch.

    EndCodeOnlyInPlacePace(true);
    MoveComp->SetMovementMode(MOVE_Walking);
    AddMovementInput(GetActorForwardVector().GetSafeNormal2D(), InputScaleByFrame);

    if (CurrentExploreActionElapsed >= WalkDuration)
    {
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
        const float DrawDuration = FMath::Max(ExploreActionDuration + 180.0f / FMath::Max(YawAngularSpeed, 1.0f), KINDA_SMALL_NUMBER);
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

ENPC1PExploreCameraAction ANPC_1p::ChooseRandomCameraAction(const FRotator& CurrentCameraRotation, FRotator& OutDesiredRotation)
{
    // Pitch is managed independently by UpdateIndependentPitch().
    // This function only samples yaw (L/R). Pitch is always preserved.
    const int32 LRSignal = FMath::RandRange(0, 2);
    const int32 UDSignal = 0;

    float DesiredYaw = CurrentCameraRotation.Yaw;
    if (LRSignal == 1)
    {
        DesiredYaw -= CameraYawStepDegrees;
    }
    else if (LRSignal == 2)
    {
        DesiredYaw += CameraYawStepDegrees;
    }

    OutDesiredRotation = FRotator(CurrentCameraRotation.Pitch, DesiredYaw, CurrentCameraRotation.Roll);
    return MakeCameraAction(LRSignal, UDSignal);
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
    OutUD = 0; // pitch UD is now owned by UpdateIndependentPitch()

    switch (Action)
    {
    case ENPC1PExploreCameraAction::L:
        OutLR = 1;
        break;
    case ENPC1PExploreCameraAction::R:
        OutLR = 2;
        break;
    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// Unified rotation application: atomically applies both yaw (actor) and pitch (camera boom).
// Call AFTER ExecuteNextStep and UpdateIndependentPitch each frame from Blueprint.
// ---------------------------------------------------------------------------

void ANPC_1p::ApplyDesiredRotation(float DeltaTime)
{
    USpringArmComponent* CameraBoomComp = GetCameraBoom();
    if (!CameraBoomComp)
    {
        return;
    }

    // --- Yaw: interpolate actor toward CurrentDesiredYaw at YawAngularSpeed ---
    const float CurrentYaw = GetActorRotation().Yaw;
    const float RawYawDelta = FMath::FindDeltaAngleDegrees(CurrentYaw, CurrentDesiredYaw);
    float NewYaw = CurrentYaw;
    if (FMath::Abs(RawYawDelta) > TurnYawToleranceDegrees)
    {
        const float MaxYawStep = FMath::Max(YawAngularSpeed, 1.0f) * FMath::Max(DeltaTime, 0.0f);
        NewYaw = CurrentYaw + FMath::Clamp(RawYawDelta, -MaxYawStep, MaxYawStep);
        // Prevent overshoot
        if (FMath::Abs(FMath::FindDeltaAngleDegrees(NewYaw, CurrentDesiredYaw)) > FMath::Abs(RawYawDelta))
        {
            NewYaw = CurrentDesiredYaw;
        }
    }

    // --- Pitch: interpolate camera boom toward CurrentDesiredPitch at PitchAngularSpeed ---
    const float CurrentPitch = CameraBoomComp->GetComponentRotation().Pitch;
    const float PitchDelta = CurrentDesiredPitch - CurrentPitch;
    const float MaxPitchStep = FMath::Max(PitchAngularSpeed, 1.0f) * FMath::Max(DeltaTime, 0.0f);
    float NewPitch = CurrentPitch + FMath::Clamp(PitchDelta, -MaxPitchStep, MaxPitchStep);
    if (FMath::Abs(NewPitch - CurrentDesiredPitch) > FMath::Abs(PitchDelta))
    {
        NewPitch = CurrentDesiredPitch;
    }

    // --- Atomic apply: actor yaw + camera boom pitch-only relative rotation ---
    // SetWorldRotation is NOT used: the mandatory quaternion decomposition relative
    // to the parent (whose yaw was just set) introduced frame-to-frame pitch
    // inconsistency. Writing RelativeRotation directly is unambiguous.
    SetActorRotation(FRotator(0.0f, NewYaw, 0.0f));
    CameraBoomComp->SetRelativeRotation(FRotator(NewPitch, 0.0f, 0.0f));

    // --- Recorder UD signal based on pitch interpolation progress ---
    const bool bPitchReached = FMath::Abs(NewPitch - CurrentDesiredPitch) <= CameraPitchHoldToleranceDegrees;
    if (bPitchReached)
    {
        CurrentRecorderUD = 0;
    }
    else
    {
        CurrentRecorderUD = (CurrentDesiredPitch > NewPitch) ? 1 : 2;
    }
}

void ANPC_1p::RecordCurrentFrame(float DeltaTime)
{
    if (MovementRecorder && MovementRecorder->bIsRecording)
    {
        MovementRecorder->RecordFrameFromNPC(this, CurrentPath, CurrentPathIndex, DeltaTime, bWasExecutingThisFrameForRecording);
    }

    if (GEngine)
    {
        const USpringArmComponent* CamBoom = GetCameraBoom();
        const float CurrentPitch = CamBoom ? CamBoom->GetComponentRotation().Pitch : 0.0f;
        GEngine->AddOnScreenDebugMessage(
            INDEX_NONE,
            0.0f,
            FColor::Cyan,
            FString::Printf(TEXT("NPC_1p[%s] CameraPitch = %.2f"), *GetActorLabel(), CurrentPitch));
    }
}

void ANPC_1p::BeginWalkCameraAction()
{
    if (bWalkCameraActionStarted)
    {
        return;
    }

    bWalkCameraActionStarted = true;
    const FRotator CurrentCameraRot(CameraBoomPitch, GetActorRotation().Yaw, 0.0f);
    CurrentExploreCameraAction = ChooseRandomCameraAction(CurrentCameraRot, DesiredCameraYawRelativePitchWorld);
    CurrentDesiredYaw = DesiredCameraYawRelativePitchWorld.Yaw;
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

// ---------------------------------------------------------------------------
// Independent Pitch State Machine
// ---------------------------------------------------------------------------

void ANPC_1p::EnterPitchState(ENPC1PPitchState NewState)
{
    CurrentPitchState = NewState;
    PitchStateElapsed = 0.0f;

    if (NewState == ENPC1PPitchState::Center)
    {
        CurrentDesiredPitch = CameraBoomPitch;
        CurrentPitchStateDuration = FMath::FRandRange(PitchStateMinCenterDuration, PitchStateMaxCenterDuration);
    }
    else
    {
        CurrentDesiredPitch = FMath::FRandRange(PitchMinAngle, PitchMaxAngle);
        CurrentPitchStateDuration = FMath::FRandRange(0.1f, PitchStateMaxNonCenterDuration);
    }
}

void ANPC_1p::UpdateIndependentPitch(float DeltaTime)
{
    USpringArmComponent* CameraBoomComp = GetCameraBoom();
    if (!CameraBoomComp)
    {
        return;
    }

    // State machine only — rotation interpolation and application are handled by ApplyDesiredRotation.
    PitchStateElapsed += DeltaTime;

    // Check if the pitch target has been reached (component rotation set by ApplyDesiredRotation last frame).
    const bool bPitchReached = FMath::Abs(CameraBoomComp->GetComponentRotation().Pitch - CurrentDesiredPitch) <= CameraPitchHoldToleranceDegrees;

    // Transition if target reached and duration elapsed.
    if (bPitchReached && PitchStateElapsed >= CurrentPitchStateDuration)
    {
        if (CurrentPitchState == ENPC1PPitchState::Center)
        {
            EnterPitchState(ENPC1PPitchState::Away);
        }
        else
        {
            EnterPitchState(ENPC1PPitchState::Center);
        }
    }
}

// ---------------------------------------------------------------------------
// Social Turn System (framerate-based smooth rotation via state machine)
// ---------------------------------------------------------------------------

void ANPC_1p::StartSocialTurn()
{
    // Reset cooldown first — even on early-out, don't spin-loop every frame.
    SocialTurnCooldownRemaining = FMath::FRandRange(SocialTurnMinInterval, SocialTurnMaxInterval);

    AActor* Nearest = FindNearestSameTypeNPC();
    if (!Nearest)
    {
        return;
    }

    UpdateVisitedStatsAtCurrentPosition();

    const FVector ToNearest = (Nearest->GetActorLocation() - GetActorLocation()).GetSafeNormal2D();
    if (ToNearest.IsNearlyZero())
    {
        return;
    }

    SocialTurnTargetNPC = Nearest;

    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(
            INDEX_NONE,
            3.0f,
            FColor::Yellow,
            FString::Printf(TEXT("NPC_1p[%s] Social Turn -> facing %s started"), *GetActorLabel(), *Nearest->GetActorLabel()));
    }

    bIsExecutingExploreAction = true;
    CurrentExploreMoveAction = ENPC1PExploreMoveAction::Idle;
    CurrentExploreMoveTarget = Nearest->GetActorLocation();
    CurrentExplorePhase = ENPC1PExplorePhase::SocialTurnToPeer;

    StartTurnActorRotation = GetActorRotation();
    DesiredTurnActorRotation = StartTurnActorRotation;
    DesiredTurnActorRotation.Yaw = ToNearest.Rotation().Yaw;
    CurrentDesiredYaw = DesiredTurnActorRotation.Yaw;

    bHasDesiredCameraWorldRotation = false;
    CurrentExploreCameraAction = ENPC1PExploreCameraAction::None;
    SetRecorderSignals(0, 0, 0, 0);

    // SocialTurnToPeer handles the completion check; ApplyDesiredRotation handles the interpolation.
    BeginCodeOnlyInPlacePace();
}

AActor* ANPC_1p::FindNearestSameTypeNPC() const
{
    AActor* Nearest = nullptr;
    float NearestDistSq = FLT_MAX;

    for (AActor* Candidate : SameTypeNPCList)
    {
        if (!IsValid(Candidate) || Candidate == this)
        {
            continue;
        }

        const float DistSq = FVector::DistSquared2D(GetActorLocation(), Candidate->GetActorLocation());
        if (DistSq < NearestDistSq)
        {
            NearestDistSq = DistSq;
            Nearest = Candidate;
        }
    }

    return Nearest;
}

