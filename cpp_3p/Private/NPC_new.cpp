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
    MaxCameraPitchOffsetActionCount = 2;
    CameraPitchHoldToleranceDegrees = 1.0f;

    bUpdateVisitedBeforeSampling = true;
    bDebugDrawExploreCandidates = false;
    VisitedSoftmaxTemperature = 25.0f;
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
    CurrentExploreMoveAction = ENPCExploreMoveAction::Idle;
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
        const FString ErrorMessage = FString::Printf(TEXT("NPC_new[%s] no legal movement candidate"), *GetActorLabel());
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
        if (Candidate.Action != ENPCExploreMoveAction::Idle)
        {
            bHasLegalMoveCandidate = true;
            break;
        }
    }
    if (!bHasLegalMoveCandidate)
    {
        const FString ErrorMessage = FString::Printf(TEXT("NPC_new[%s] no legal movement candidate; using Idle"), *GetActorLabel());
        UE_LOG(LogTemp, Error, TEXT("%s"), *ErrorMessage);
        if (GEngine)
        {
            GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, ErrorMessage);
        }
    }

    const int32 PickedIndex = SampleCandidateByVisitedSoftmax(Candidates);
    if (!Candidates.IsValidIndex(PickedIndex))
    {
        return;
    }

    const FExploreMoveCandidate& Picked = Candidates[PickedIndex];
    bIsExecutingExploreAction = true;
    CurrentExploreMoveAction = Picked.Action;
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

    if (CurrentExploreMoveAction != ENPCExploreMoveAction::Idle)
    {
        FVector FrameMoveDirection = FVector::ZeroVector;
        if (!GetWorldDirectionForAction(CurrentExploreMoveAction, FrameMoveDirection))
        {
            ClearExploreMoveTarget();
            return;
        }
        FrameMoveDirection = FrameMoveDirection.GetSafeNormal2D();

        MoveComp->SetMovementMode(MOVE_Walking);
        const float InputScale = (DeltaTime > KINDA_SMALL_NUMBER) ? (EffectiveDeltaTime / DeltaTime) : 0.0f;
        AddMovementInput(FrameMoveDirection, InputScale);

        FRotator DesiredActorRot = FrameMoveDirection.Rotation();
        DesiredActorRot.Pitch = 0.0f;
        DesiredActorRot.Roll = 0.0f;
        SetActorRotation(FQuat::Slerp(GetActorRotation().Quaternion(), DesiredActorRot.Quaternion(), Alpha).Rotator());
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

    const ENPCExploreMoveAction AllActions[9] =
    {
        ENPCExploreMoveAction::Idle,
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
    if (Action == ENPCExploreMoveAction::Idle)
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
        OutAD = 1;
        break;
    case ENPCExploreMoveAction::D:
        OutAD = 2;
        break;
    case ENPCExploreMoveAction::WA:
        OutWS = 1;
        OutAD = 1;
        break;
    case ENPCExploreMoveAction::WD:
        OutWS = 1;
        OutAD = 2;
        break;
    case ENPCExploreMoveAction::SA:
        OutWS = 2;
        OutAD = 1;
        break;
    case ENPCExploreMoveAction::SD:
        OutWS = 2;
        OutAD = 2;
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

    // NavMesh 投影检查
    const FVector QueryExtent(GridSize * 0.5f, GridSize * 0.5f, 200.0f);
    if (!NavSys->ProjectPointToNavigation(DesiredFootLocation, ProjectedLocation, QueryExtent))
    {
        return false;
    }

    const FVector LandingFootLocation = ProjectedLocation.Location;

    // 落点不能太接近起点
    if (FVector::Dist2D(StartFootLocation, LandingFootLocation) < 10.0f)
    {
        return false;
    }

    // // 高度差限制
    // if (FMath::Abs(LandingFootLocation.Z - StartFootLocation.Z) > MaxStepHeight)
    // {
    //     return false;
    // }

    // 落点胶囊碰撞检查
    if (!IsLocationValidForNPC(LandingFootLocation))
    {
        return false;
    }

    // 路径碰撞检查
    const FVector LandingActorLocation = LandingFootLocation + FVector(0.0f, 0.0f, CapsuleHalfHeight);
    //if (!IsMovePathCollisionFree(StartActorLocation, LandingActorLocation))
    //{
    //    return false;
    //}

    OutCandidate.WorldDirection = Dir2D;
    OutCandidate.LandingFootLocation = LandingFootLocation;
    OutCandidate.LandingActorLocation = LandingActorLocation;
    OutCandidate.VisitedScore = GetVisitedScoreAtLocation(LandingFootLocation);

    if (bDebugDrawExploreCandidates)
    {
        const float ColorScalar = FMath::Clamp(1.0f / (1.0f + OutCandidate.VisitedScore), 0.0f, 1.0f);
        const FColor Color = FColor::MakeRedToGreenColorFromScalar(ColorScalar);
        const float DrawDuration = FMath::Max(ExploreActionDuration, KINDA_SMALL_NUMBER);
        DrawDebugSphere(World, LandingActorLocation, 12.0f, 8, Color, false, DrawDuration);
        DrawDebugLine(World, StartActorLocation, LandingActorLocation, Color, false, DrawDuration, 0, 1.5f);
    }

    return true;
}

float ANPC_new::GetVisitedScoreAtLocation(const FVector& WorldLocation) const
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

int32 ANPC_new::SampleCandidateByVisitedSoftmax(const TArray<FExploreMoveCandidate>& Candidates) const
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
    const float CameraPitchCenter = -15.0f;
    const float CurrentPitch = FMath::Clamp(CurrentCameraRotation.Pitch, -45.0f, 15.0f);
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
