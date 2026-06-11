#pragma once

#include "CoreMinimal.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "NPC1PCharacterMovementComponent.generated.h"

UENUM(BlueprintType)
enum class ENPC1PCustomMovementMode : uint8
{
	None = 0,
	InPlacePace = 1 UMETA(DisplayName = "In Place Pace")
};

/**
 * First-person NPC movement component.
 *
 * Important design choice:
 * The first attempt used MOVE_Custom. That can expose non-zero Velocity, but many
 * AnimBPs also require MovementMode == Walking and CurrentAcceleration != 0.
 *
 * This component therefore uses a "walking-presented" in-place mode:
 * - MovementMode remains MOVE_Walking, so ordinary locomotion AnimBPs still pass
 *   their "is on ground / should move" tests.
 * - Velocity and Acceleration are kept non-zero for animation.
 * - PhysWalking is overridden while the in-place mode is active, so no real
 *   translation is applied to the Actor. There is no SetActorLocation snap-back.
 */
UCLASS()
class CPP_1ST_API UNPC1PCharacterMovementComponent : public UCharacterMovementComponent
{
	GENERATED_BODY()

public:
	UNPC1PCharacterMovementComponent();

	UFUNCTION(BlueprintCallable, Category = "Movement|First Person")
	void StartInPlacePace(const FVector& InFakeVelocity);

	UFUNCTION(BlueprintCallable, Category = "Movement|First Person")
	void UpdateInPlacePaceVelocity(const FVector& InFakeVelocity);

	UFUNCTION(BlueprintCallable, Category = "Movement|First Person")
	void StopInPlacePace(bool bRestoreWalking = true);

	UFUNCTION(BlueprintPure, Category = "Movement|First Person")
	bool IsInPlacePaceMode() const { return bInPlacePaceMode; }

protected:
	virtual void PhysWalking(float DeltaTime, int32 Iterations) override;
	virtual void PhysCustom(float DeltaTime, int32 Iterations) override;
	virtual void OnMovementModeChanged(EMovementMode PreviousMovementMode, uint8 PreviousCustomMode) override;

private:
	void ApplyInPlaceAnimKinematics();
	FVector BuildFakeAccelerationFromVelocity(const FVector& InFakeVelocity) const;
	void ClearPendingInputVectors();

private:
	bool bInPlacePaceMode = false;
	FVector FakeInPlaceVelocity = FVector::ZeroVector;
	FVector FakeInPlaceAcceleration = FVector::ZeroVector;
};
