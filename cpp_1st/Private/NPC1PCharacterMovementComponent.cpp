#include "NPC1PCharacterMovementComponent.h"

#include "GameFramework/Character.h"
#include "GameFramework/Pawn.h"

UNPC1PCharacterMovementComponent::UNPC1PCharacterMovementComponent()
{
}

FVector UNPC1PCharacterMovementComponent::BuildFakeAccelerationFromVelocity(const FVector& InFakeVelocity) const
{
	FVector Direction = InFakeVelocity.GetSafeNormal2D();
	if (Direction.IsNearlyZero())
	{
		Direction = FVector::ForwardVector;
	}

	const float AccelSize = FMath::Max(MaxAcceleration, 2048.0f);
	return Direction * AccelSize;
}

void UNPC1PCharacterMovementComponent::ApplyInPlaceAnimKinematics()
{
	Velocity = FakeInPlaceVelocity;
	Acceleration = FakeInPlaceAcceleration;

	// Make both the movement component and the updated primitive report a non-zero velocity.
	// Most AnimBPs use TryGetPawnOwner()->GetVelocity(), which for Character reads this component.
	UpdateComponentVelocity();

	if (UpdatedComponent)
	{
		UpdatedComponent->ComponentVelocity = FakeInPlaceVelocity;
	}
}

void UNPC1PCharacterMovementComponent::ClearPendingInputVectors()
{
	if (APawn* OwningPawn = Cast<APawn>(GetOwner()))
	{
		OwningPawn->ConsumeMovementInputVector();
	}
}

void UNPC1PCharacterMovementComponent::StartInPlacePace(const FVector& InFakeVelocity)
{
	bInPlacePaceMode = true;
	FakeInPlaceVelocity = InFakeVelocity;
	FakeInPlaceAcceleration = BuildFakeAccelerationFromVelocity(FakeInPlaceVelocity);

	// Present as Walking instead of MOVE_Custom because many AnimBPs gate locomotion on
	// MovementMode == Walking / IsMovingOnGround in addition to Speed.
	if (MovementMode != MOVE_Walking)
	{
		SetMovementMode(MOVE_Walking);
	}

	ApplyInPlaceAnimKinematics();
}

void UNPC1PCharacterMovementComponent::UpdateInPlacePaceVelocity(const FVector& InFakeVelocity)
{
	FakeInPlaceVelocity = InFakeVelocity;
	FakeInPlaceAcceleration = BuildFakeAccelerationFromVelocity(FakeInPlaceVelocity);

	if (!bInPlacePaceMode)
	{
		StartInPlacePace(FakeInPlaceVelocity);
		return;
	}

	if (MovementMode != MOVE_Walking)
	{
		SetMovementMode(MOVE_Walking);
	}

	ApplyInPlaceAnimKinematics();
}

void UNPC1PCharacterMovementComponent::StopInPlacePace(bool bRestoreWalking)
{
	if (!bInPlacePaceMode)
	{
		FakeInPlaceVelocity = FVector::ZeroVector;
		FakeInPlaceAcceleration = FVector::ZeroVector;
		return;
	}

	bInPlacePaceMode = false;
	FakeInPlaceVelocity = FVector::ZeroVector;
	FakeInPlaceAcceleration = FVector::ZeroVector;
	Velocity = FVector::ZeroVector;
	Acceleration = FVector::ZeroVector;
	UpdateComponentVelocity();
	ClearPendingInputVectors();

	if (bRestoreWalking)
	{
		SetMovementMode(MOVE_Walking);
	}
	else
	{
		SetMovementMode(MOVE_None);
	}
}

void UNPC1PCharacterMovementComponent::PhysWalking(float DeltaTime, int32 Iterations)
{
	if (bInPlacePaceMode)
	{
		// Do not call Super::PhysWalking here. Super would integrate Velocity and move the capsule.
		// We only refresh animation-facing kinematics and leave UpdatedComponent transform unchanged.
		ApplyInPlaceAnimKinematics();
		ClearPendingInputVectors();
		return;
	}

	Super::PhysWalking(DeltaTime, Iterations);
}

void UNPC1PCharacterMovementComponent::PhysCustom(float DeltaTime, int32 Iterations)
{
	if (CustomMovementMode == static_cast<uint8>(ENPC1PCustomMovementMode::InPlacePace))
	{
		// Backward-compatible path if someone manually switches to the old custom mode.
		bInPlacePaceMode = true;
		ApplyInPlaceAnimKinematics();
		return;
	}

	Super::PhysCustom(DeltaTime, Iterations);
}

void UNPC1PCharacterMovementComponent::OnMovementModeChanged(EMovementMode PreviousMovementMode, uint8 PreviousCustomMode)
{
	Super::OnMovementModeChanged(PreviousMovementMode, PreviousCustomMode);

	// If external code pushes us out of walking/custom while in-place pacing, stop cleanly.
	if (bInPlacePaceMode && MovementMode != MOVE_Walking && MovementMode != MOVE_Custom)
	{
		bInPlacePaceMode = false;
		FakeInPlaceVelocity = FVector::ZeroVector;
		FakeInPlaceAcceleration = FVector::ZeroVector;
	}
}
