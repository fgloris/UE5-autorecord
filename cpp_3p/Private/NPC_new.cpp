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
	ExplorationBias = 1.0f;
	MoveAcceptanceRadius = 20.0f;
	MoveInputScale = 1.0f;
	ActorYawInterpSpeed = 10.0f;

	CameraYawStepDegrees = 15.0f;
	CameraPitchStepDegrees = 8.0f;
	MinCameraPitchDegrees = -50.0f;
	MaxCameraPitchDegrees = 10.0f;
	CameraCollisionProbeRadius = 15.0f;
	CameraRotationInterpSpeed = 6.0f;

	bUpdateVisitedBeforeSampling = true;
	bDebugDrawExploreCandidates = false;
	bRequireCameraCollisionFreeForMoveCandidate = false;
}

void ANPC_new::BeginPlay()
{
	Super::BeginPlay();

	if (ProbeStepDistance <= KINDA_SMALL_NUMBER)
	{
		ProbeStepDistance = GridSize;
	}
}

void ANPC_new::ExecuteNextStep(float DeltaTime)
{
	if (!GetWorld())
	{
		return;
	}

	if (!bHasActiveExploreMoveTarget)
	{
		TryStartNewExploreMove();
	}

	UpdateMoveFollowing(DeltaTime);
	UpdateCameraSmoothing(DeltaTime);

	if (MovementRecorder && MovementRecorder->bIsRecording)
	{
		MovementRecorder->RecordFrameFromNPC(this, CurrentPath, CurrentPathIndex, DeltaTime, bHasActiveExploreMoveTarget);
	}
}

void ANPC_new::ClearExploreMoveTarget()
{
	bHasActiveExploreMoveTarget = false;
	CurrentExploreMoveTarget = FVector::ZeroVector;
	CurrentExploreMoveNavFootLocation = FVector::ZeroVector;
}

void ANPC_new::TryStartNewExploreMove()
{
	if (bUpdateVisitedBeforeSampling)
	{
		UpdateVisitedStatsAtCurrentPosition();
	}

	TArray<FExploreMoveCandidate> Candidates;
	BuildReachableMoveCandidates(Candidates);

	const int32 PickedIndex = SampleWeightedCandidateIndex(Candidates);
	if (!Candidates.IsValidIndex(PickedIndex))
	{
		if (bDebugDrawExploreCandidates)
		{
			UE_LOG(LogTemp, Warning, TEXT("NPC_new[%s] no valid move candidate this tick"), *GetActorLabel());
		}
		return;
	}

	const FExploreMoveCandidate& Picked = Candidates[PickedIndex];
	bHasActiveExploreMoveTarget = true;
	CurrentExploreMoveTarget = Picked.ActorTargetLocation;
	CurrentExploreMoveNavFootLocation = Picked.NavFootLocation;

	FRotator ChosenCameraRot;
	if (ChooseRandomCameraAction(ChosenCameraRot))
	{
		bHasDesiredCameraWorldRotation = true;
		DesiredCameraWorldRotation = ChosenCameraRot;
	}

	if (bDebugDrawExploreCandidates)
	{
		DrawDebugSphere(GetWorld(), CurrentExploreMoveTarget, 18.0f, 12, FColor::Cyan, false, 0.5f);
		DrawDebugDirectionalArrow(
			GetWorld(),
			GetActorLocation(),
			CurrentExploreMoveTarget,
			35.0f,
			FColor::Cyan,
			false,
			0.5f,
			0,
			2.5f);
	}
}

void ANPC_new::UpdateMoveFollowing(float DeltaTime)
{
	if (!bHasActiveExploreMoveTarget)
	{
		return;
	}

	UCharacterMovementComponent* MoveComp = GetCharacterMovement();
	if (!MoveComp)
	{
		ClearExploreMoveTarget();
		return;
	}

	FVector ToTarget = CurrentExploreMoveTarget - GetActorLocation();
	ToTarget.Z = 0.0f;
	const float Dist2D = ToTarget.Size();
	if (Dist2D <= MoveAcceptanceRadius)
	{
		const FVector ReachedLocation = CurrentExploreMoveTarget;
		ClearExploreMoveTarget();
		UpdateVisitedStatsAtCurrentPosition();
		OnExploreMoveTargetReached(ReachedLocation);
		return;
	}

	const FVector MoveDir = ToTarget.GetSafeNormal();
	if (MoveDir.IsNearlyZero())
	{
		return;
	}

	MoveComp->SetMovementMode(MOVE_Walking);
	MoveComp->AddInputVector(MoveDir * MoveInputScale, true);

	const FRotator CurrentRot = GetActorRotation();
	FRotator DesiredActorRot = MoveDir.Rotation();
	DesiredActorRot.Pitch = 0.0f;
	DesiredActorRot.Roll = 0.0f;
	const FRotator SmoothedActorRot = FMath::RInterpTo(CurrentRot, DesiredActorRot, DeltaTime, ActorYawInterpSpeed);
	SetActorRotation(SmoothedActorRot);
}

void ANPC_new::UpdateCameraSmoothing(float DeltaTime)
{
	USpringArmComponent* CameraBoomComp = GetCameraBoom();
	if (!CameraBoomComp || !bHasDesiredCameraWorldRotation)
	{
		return;
	}

	const FRotator CurrentRot = CameraBoomComp->GetComponentRotation();
	const FRotator NewRot = FMath::RInterpTo(CurrentRot, DesiredCameraWorldRotation, DeltaTime, CameraRotationInterpSpeed);
	CameraBoomComp->SetWorldRotation(NewRot);

	if (NewRot.Equals(DesiredCameraWorldRotation, 0.75f))
	{
		CameraBoomComp->SetWorldRotation(DesiredCameraWorldRotation);
		bHasDesiredCameraWorldRotation = false;
	}
}

void ANPC_new::BuildReachableMoveCandidates(TArray<FExploreMoveCandidate>& OutCandidates) const
{
	OutCandidates.Reset();

	const USpringArmComponent* CameraBoomComp = GetCameraBoom();
	if (!CameraBoomComp)
	{
		return;
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

	TArray<FVector> DesiredDirections;
	DesiredDirections.Reserve(8);
	DesiredDirections.Add(CamForward);
	DesiredDirections.Add(-CamForward);
	DesiredDirections.Add(CamRight);
	DesiredDirections.Add(-CamRight);
	DesiredDirections.Add((CamForward + CamRight).GetSafeNormal());
	DesiredDirections.Add((CamForward - CamRight).GetSafeNormal());
	DesiredDirections.Add((-CamForward + CamRight).GetSafeNormal());
	DesiredDirections.Add((-CamForward - CamRight).GetSafeNormal());

	for (const FVector& Dir : DesiredDirections)
	{
		FExploreMoveCandidate Candidate;
		if (TryBuildMoveCandidate(Dir, Candidate))
		{
			OutCandidates.Add(Candidate);
		}
	}
}

bool ANPC_new::TryBuildMoveCandidate(const FVector& DesiredWorldDirection, FExploreMoveCandidate& OutCandidate) const
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

	const FVector NavFootLocation = ProjectedLocation.Location;
	if (FVector::Dist2D(StartFootLocation, NavFootLocation) < 10.0f)
	{
		return false;
	}

	if (FMath::Abs(NavFootLocation.Z - StartFootLocation.Z) > MaxStepHeight)
	{
		return false;
	}

	if (!IsLocationValidForNPC(NavFootLocation))
	{
		return false;
	}

	const FVector ActorTargetLocation = NavFootLocation + FVector(0.0f, 0.0f, CapsuleHalfHeight);
	if (!IsMovePathCollisionFree(StartActorLocation, ActorTargetLocation))
	{
		return false;
	}

	if (bRequireCameraCollisionFreeForMoveCandidate)
	{
		const FRotator CurrentCameraWorldRot = GetCameraBoom() ? GetCameraBoom()->GetComponentRotation() : FRotator(CameraBoomPitch, 0.0f, 0.0f);
		if (!IsCameraPoseCollisionFree(ActorTargetLocation, CurrentCameraWorldRot))
		{
			return false;
		}
	}

	OutCandidate.WorldDirection = (ActorTargetLocation - StartActorLocation).GetSafeNormal2D();
	OutCandidate.NavFootLocation = NavFootLocation;
	OutCandidate.ActorTargetLocation = ActorTargetLocation;
	OutCandidate.VisitValue = GetVisitedValueAtLocation(NavFootLocation);
	OutCandidate.Weight = FMath::Exp(-OutCandidate.VisitValue * ExplorationBias);

	if (bDebugDrawExploreCandidates)
	{
		const FColor Color = FColor::MakeRedToGreenColorFromScalar(FMath::Clamp(1.0f / (1.0f + OutCandidate.VisitValue), 0.0f, 1.0f));
		DrawDebugSphere(GetWorld(), ActorTargetLocation, 12.0f, 8, Color, false, 0.2f);
		DrawDebugLine(GetWorld(), StartActorLocation, ActorTargetLocation, Color, false, 0.2f, 0, 1.5f);
	}

	return true;
}

float ANPC_new::GetVisitedValueAtLocation(const FVector& WorldLocation) const
{
	const int32 GridX = FMath::RoundToInt(WorldLocation.X / GridSize);
	const int32 GridY = FMath::RoundToInt(WorldLocation.Y / GridSize);
	const FString GridKey = FString::Printf(TEXT("%d_%d"), GridX, GridY);
	if (const float* FoundValue = Visited.Find(GridKey))
	{
		return *FoundValue;
	}
	return 0.0f;
}

int32 ANPC_new::SampleWeightedCandidateIndex(const TArray<FExploreMoveCandidate>& Candidates) const
{
	if (Candidates.Num() <= 0)
	{
		return INDEX_NONE;
	}

	float TotalWeight = 0.0f;
	for (const FExploreMoveCandidate& Candidate : Candidates)
	{
		TotalWeight += FMath::Max(Candidate.Weight, 0.0001f);
	}

	if (TotalWeight <= KINDA_SMALL_NUMBER)
	{
		return FMath::RandRange(0, Candidates.Num() - 1);
	}

	float RandomValue = FMath::FRandRange(0.0f, TotalWeight);
	for (int32 Index = 0; Index < Candidates.Num(); ++Index)
	{
		RandomValue -= FMath::Max(Candidates[Index].Weight, 0.0001f);
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

	const float CapsuleRadius = Capsule->GetScaledCapsuleRadius();
	const float CapsuleHalfHeight = Capsule->GetScaledCapsuleHalfHeight();
	const FCollisionShape CapsuleShape = FCollisionShape::MakeCapsule(CapsuleRadius, CapsuleHalfHeight);

	FCollisionQueryParams Params(SCENE_QUERY_STAT(NPCNewMoveSweep), false, this);
	Params.AddIgnoredActor(this);

	FHitResult HitResult;
	const bool bHitStatic = World->SweepSingleByChannel(
		HitResult,
		StartActorLocation,
		EndActorLocation,
		FQuat::Identity,
		ECC_WorldStatic,
		CapsuleShape,
		Params);
	if (bHitStatic)
	{
		return false;
	}

	const bool bHitDynamic = World->SweepSingleByChannel(
		HitResult,
		StartActorLocation,
		EndActorLocation,
		FQuat::Identity,
		ECC_WorldDynamic,
		CapsuleShape,
		Params);
	if (bHitDynamic)
	{
		return false;
	}

	return true;
}

bool ANPC_new::IsCameraPoseCollisionFree(const FVector& ActorLocation, const FRotator& DesiredCameraWorldRotation) const
{
	const USpringArmComponent* CameraBoomComp = GetCameraBoom();
	UWorld* World = GetWorld();
	if (!World || !CameraBoomComp)
	{
		return false;
	}

	const FVector BoomOrigin = ActorLocation + CameraBoomComp->GetRelativeLocation();
	const FVector CameraEnd = BoomOrigin - DesiredCameraWorldRotation.Vector() * CameraBoomComp->TargetArmLength;
	const FCollisionShape ProbeShape = FCollisionShape::MakeSphere(CameraCollisionProbeRadius);

	FCollisionQueryParams Params(SCENE_QUERY_STAT(NPCNewCameraSweep), false, this);
	Params.AddIgnoredActor(this);

	FHitResult HitResult;
	const bool bHitStatic = World->SweepSingleByChannel(
		HitResult,
		BoomOrigin,
		CameraEnd,
		FQuat::Identity,
		ECC_WorldStatic,
		ProbeShape,
		Params);
	if (bHitStatic)
	{
		return false;
	}

	const bool bHitDynamic = World->SweepSingleByChannel(
		HitResult,
		BoomOrigin,
		CameraEnd,
		FQuat::Identity,
		ECC_WorldDynamic,
		ProbeShape,
		Params);
	if (bHitDynamic)
	{
		return false;
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

	TArray<int32> ValidIndices;
	ValidIndices.Reserve(CandidateRotations.Num());
	for (int32 Index = 0; Index < CandidateRotations.Num(); ++Index)
	{
		if (IsCameraPoseCollisionFree(GetActorLocation(), CandidateRotations[Index]))
		{
			ValidIndices.Add(Index);
		}
	}

	if (ValidIndices.Num() <= 0)
	{
		return false;
	}

	const int32 SelectedValidIndex = ValidIndices[FMath::RandRange(0, ValidIndices.Num() - 1)];
	OutDesiredRotation = CandidateRotations[SelectedValidIndex];
	return true;
}
