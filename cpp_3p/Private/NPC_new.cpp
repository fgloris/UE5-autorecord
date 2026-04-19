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

    ProbeStepDistance = 100.0f;
    ExploreActionDuration = 0.5f;
    MoveInputScale = 1.0f;

    CameraYawStepDegrees = 15.0f;
    CameraPitchStepDegrees = 15.0f;
    MinCameraPitchDegrees = -30.0f;
    MaxCameraPitchDegrees = 30.0f;

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
    StartCameraWorldRotation = FRotator::ZeroRotator;
    DesiredCameraWorldRotation = FRotator::ZeroRotator;
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

    const USpringArmComponent* CameraBoomComp = GetCameraBoom();
    StartCameraWorldRotation = CameraBoomComp ? CameraBoomComp->GetComponentRotation() : FRotator(CameraBoomPitch, 0.0f, 0.0f);
    bHasDesiredCameraWorldRotation = ChooseRandomCameraAction(DesiredCameraWorldRotation);
    if (!bHasDesiredCameraWorldRotation)
    {
        DesiredCameraWorldRotation = StartCameraWorldRotation;
    }
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
    const float InputScale = (DeltaTime > KINDA_SMALL_NUMBER) ? (MoveInputScale * EffectiveDeltaTime / DeltaTime) : 0.0f;
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

bool ANPC_new::IsLandingValidForDirection(const FVector& DesiredWorldDirection, FExploreMoveCandidate& OutCandidate) const
{
    UWorld* World = GetWorld();
    if (!World)
    {
        return false;
    }

    UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
    const UCapsuleComponent* Capsule = GetCapsuleComponent();
    if (!NavSys || !Capsule)
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

bool ANPC_new::ChooseRandomCameraAction(FRotator& OutDesiredRotation) const
{
    const USpringArmComponent* CameraBoomComp = GetCameraBoom();
    if (!CameraBoomComp)
    {
        return false;
    }

    const FRotator CurrentRot = CameraBoomComp->GetComponentRotation();
    TArray<FRotator> CandidateRotations;
    CandidateRotations.Reserve(4);

    CandidateRotations.Add(FRotator(CurrentRot.Pitch, CurrentRot.Yaw - CameraYawStepDegrees, CurrentRot.Roll));
    CandidateRotations.Add(FRotator(CurrentRot.Pitch, CurrentRot.Yaw + CameraYawStepDegrees, CurrentRot.Roll));
    CandidateRotations.Add(FRotator(FMath::Clamp(CurrentRot.Pitch + CameraPitchStepDegrees, MinCameraPitchDegrees, MaxCameraPitchDegrees), CurrentRot.Yaw, CurrentRot.Roll));
    CandidateRotations.Add(FRotator(FMath::Clamp(CurrentRot.Pitch - CameraPitchStepDegrees, MinCameraPitchDegrees, MaxCameraPitchDegrees), CurrentRot.Yaw, CurrentRot.Roll));

    if (CandidateRotations.Num() <= 0)
    {
        return false;
    }

    OutDesiredRotation = CandidateRotations[FMath::RandRange(0, CandidateRotations.Num() - 1)];
    return true;
}
