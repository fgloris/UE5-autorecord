#include "NPC_new.hpp"

#include "Camera/CameraComponent.h"
#include "Components/CapsuleComponent.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/SpringArmComponent.h"
#include "NavigationSystem.h"
#include "Engine/World.h"
#include "NPCMovementRecorder.h"

ANPC_new::ANPC_new()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;

	ProbeStepDistance = 100.0f;
	ExplorationBias = 1.0f;
	CameraYawStepDegrees = 15.0f;
	CameraPitchStepDegrees = 8.0f;
	MinCameraPitchDegrees = -50.0f;
	MaxCameraPitchDegrees = 10.0f;
	CameraCollisionProbeRadius = 15.0f;
	bUpdateVisitedBeforeSampling = true;
	bDebugDrawExploreCandidates = false;
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

	if (bUpdateVisitedBeforeSampling)
	{
		UpdateVisitedStatsAtCurrentPosition();
	}

	TArray<FExploreMoveCandidate> Candidates;
	BuildReachableMoveCandidates(Candidates);

	const int32 PickedIndex = SampleWeightedCandidateIndex(Candidates);
	if (Candidates.IsValidIndex(PickedIndex))
	{
		const FExploreMoveCandidate& Picked = Candidates[PickedIndex];
		AddMovementInput(Picked.WorldDirection, 1.0f);

		if (bDebugDrawExploreCandidates)
		{
			DrawDebugDirectionalArrow(
				GetWorld(),
				GetActorLocation(),
				GetActorLocation() + Picked.WorldDirection * 120.0f,
				30.0f,
				FColor::Cyan,
				false,
				0.15f,
				0,
				2.5f);
		}
	}

	TryApplyRandomCameraAction();

	if (MovementRecorder && MovementRecorder->bIsRecording)
	{
		MovementRecorder->RecordFrameFromNPC(this, CurrentPath, CurrentPathIndex, DeltaTime, Candidates.IsValidIndex(PickedIndex));
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
	if (!NavSys)
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

	const FVector StartLocation = GetActorLocation();
	const FVector DesiredLocation = StartLocation + Dir2D * ProbeStepDistance;

	FNavLocation ProjectedLocation;
	const FVector QueryExtent(GridSize * 0.5f, GridSize * 0.5f, 200.0f);
	if (!NavSys->ProjectPointToNavigation(DesiredLocation, ProjectedLocation, QueryExtent))
	{
		return false;
	}

	const FVector TargetLocation = ProjectedLocation.Location;
	if (FVector::Dist2D(StartLocation, TargetLocation) < 10.0f)
	{
		return false;
	}

	if (FMath::Abs(TargetLocation.Z - StartLocation.Z) > MaxStepHeight)
	{
		return false;
	}

	if (!IsLocationValidForNPC(TargetLocation))
	{
		return false;
	}

	if (!IsMovePathCollisionFree(StartLocation, TargetLocation))
	{
		return false;
	}

	const FRotator CurrentCameraWorldRot = GetCameraBoom() ? GetCameraBoom()->GetComponentRotation() : FRotator(CameraBoomPitch, 0.0f, 0.0f);
	if (!IsCameraPoseCollisionFree(TargetLocation, CurrentCameraWorldRot))
	{
		return false;
	}

	OutCandidate.WorldDirection = (TargetLocation - StartLocation).GetSafeNormal2D();
	OutCandidate.TargetLocation = TargetLocation;
	OutCandidate.VisitValue = GetVisitedValueAtLocation(TargetLocation);
	OutCandidate.Weight = FMath::Exp(-OutCandidate.VisitValue * ExplorationBias);
		
	if (bDebugDrawExploreCandidates)
	{
		const FColor Color = FColor::MakeRedToGreenColorFromScalar(FMath::Clamp(1.0f / (1.0f + OutCandidate.VisitValue), 0.0f, 1.0f));
		DrawDebugSphere(GetWorld(), TargetLocation + FVector(0.0f, 0.0f, 25.0f), 10.0f, 8, Color, false, 0.15f);
		DrawDebugLine(GetWorld(), StartLocation + FVector(0.0f, 0.0f, 5.0f), TargetLocation + FVector(0.0f, 0.0f, 5.0f), Color, false, 0.15f, 0, 1.5f);
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

bool ANPC_new::IsMovePathCollisionFree(const FVector& StartLocation, const FVector& EndLocation) const
{
	if (!GetWorld())
	{
		return false;
	}

	const UCapsuleComponent* Capsule = GetCapsuleComponent();
	if (!Capsule)
	{
		return false;
	}

	const float CapsuleRadius = Capsule->GetScaledCapsuleRadius() + 10.0f;
	const float CapsuleHalfHeight = Capsule->GetScaledCapsuleHalfHeight();
	const FVector CapsuleCenter = FVector(EndLocation.X, EndLocation.Y, 10.0f + EndLocation.Z + CapsuleHalfHeight);
	const FCollisionShape CapsuleShape = FCollisionShape::MakeCapsule(CapsuleRadius, CapsuleHalfHeight);

	FCollisionQueryParams Params;
	Params.AddIgnoredActor(this);

	const TArray<ECollisionChannel> Channels = {
		ECollisionChannel::ECC_Pawn,
	};

	for (const ECollisionChannel Channel : Channels)
	{
		const bool bHasOverlap = GetWorld()->OverlapAnyTestByChannel(
			CapsuleCenter,
			FQuat::Identity,
			Channel,
			CapsuleShape,
			Params);

		if (bHasOverlap)
		{
			if (bDebugDrawExploreCandidates)
			{
				DrawDebugCapsule(GetWorld(), CapsuleCenter, CapsuleHalfHeight, CapsuleRadius, FQuat::Identity, FColor::Red, false, 0.15f);
			}
			return false;
		}
	}

	if (bDebugDrawExploreCandidates)
	{
		DrawDebugCapsule(GetWorld(), CapsuleCenter, CapsuleHalfHeight, CapsuleRadius, FQuat::Identity, FColor::Green, false, 0.15f);
	}

	return true;
}

bool ANPC_new::IsCameraPoseCollisionFree(const FVector& ActorLocation, const FRotator& DesiredCameraWorldRotation) const
{
	const USpringArmComponent* CameraBoomComp = GetCameraBoom();
	if (!GetWorld() || !CameraBoomComp)
	{
		return false;
	}

	const FVector CameraStart = ActorLocation + FVector(0.0f, 0.0f, 50.0f);
	const FVector CameraPos = CameraStart - DesiredCameraWorldRotation.Vector() * CameraBoomComp->TargetArmLength;
	const float CameraRadius = CameraCollisionProbeRadius > 0.0f ? CameraCollisionProbeRadius : 20.0f;

	FCollisionQueryParams Params;
	Params.AddIgnoredActor(this);

	const bool bHasOverlap = GetWorld()->OverlapAnyTestByChannel(
		CameraPos,
		FQuat::Identity,
		ECollisionChannel::ECC_Camera,
		FCollisionShape::MakeSphere(CameraRadius),
		Params);

	FHitResult HitResult;
	const bool bHit = GetWorld()->LineTraceSingleByChannel(
		HitResult,
		CameraStart,
		CameraPos,
		ECollisionChannel::ECC_Camera,
		Params);

	const bool bBlocked = bHasOverlap || (bHit && HitResult.Distance < CameraBoomComp->TargetArmLength);

	if (bDebugDrawExploreCandidates)
	{
		DrawDebugLine(GetWorld(), CameraStart, CameraPos, bBlocked ? FColor::Red : FColor::Blue, false, 0.15f, 0, 1.0f);
		DrawDebugSphere(GetWorld(), CameraPos, CameraRadius, 10, bBlocked ? FColor::Red : FColor::Blue, false, 0.15f);
	}

	return !bBlocked;
}

bool ANPC_new::TryApplyRandomCameraAction()
{
	USpringArmComponent* CameraBoomComp = GetCameraBoom();
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
	CameraBoomComp->SetWorldRotation(CandidateRotations[SelectedValidIndex]);
	return true;
}
