#pragma once

#include "CoreMinimal.h"
#include "NPC.h"
#include "NPC_1p.generated.h"

class UNavigationSystemV1;
class UCharacterMovementComponent;
class UNPC1PCharacterMovementComponent;

UENUM()
enum class ENPC1PExploreMoveAction : uint8
{
	Idle,
	W,
	S,
	A,
	D,
	WA,
	WD,
	SA,
	SD
};

UENUM()
enum class ENPC1PExplorePhase : uint8
{
	None,
	IdleCamera,
	TurnToMoveDirection,
	WalkForward
};

UENUM()
enum class ENPC1PExploreCameraAction : uint8
{
	None,
	L,
	R,
	U,
	D,
	LU,
	LD,
	RU,
	RD
};

UCLASS()
class CPP_1ST_API ANPC_1p : public ANPC
{
	GENERATED_BODY()

public:
	ANPC_1p(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get());

	UFUNCTION(BlueprintCallable, Category = "Explore")
	void ExecuteNextStep(float DeltaTime);

	UFUNCTION(BlueprintCallable, Category = "Explore")
	bool HasActiveExploreMoveTarget() const { return bIsExecutingExploreAction; }

	UFUNCTION(BlueprintCallable, Category = "Explore")
	FVector GetCurrentExploreMoveTarget() const { return CurrentExploreMoveTarget; }

	UFUNCTION(BlueprintCallable, Category = "Explore")
	void ClearExploreMoveTarget();

	void GetCurrentRecorderControlSignals(int32& OutWS, int32& OutAD, int32& OutLR, int32& OutUD) const;

	UFUNCTION(BlueprintImplementableEvent, Category = "Explore")
	void OnExploreMoveTargetReached(const FVector& ReachedLocation);

protected:
	virtual void BeginPlay() override;

	struct FExploreMoveCandidate
	{
		ENPC1PExploreMoveAction Action = ENPC1PExploreMoveAction::Idle;
		FVector WorldDirection = FVector::ZeroVector;
		FVector LandingFootLocation = FVector::ZeroVector;
		FVector LandingActorLocation = FVector::ZeroVector;
		float VisitedScore = 0.0f;
	};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Explore", meta = (ClampMin = "0.01", UIMin = "0.01"))
	float ExploreActionDuration;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Explore")
	float CameraYawStepDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Explore")
	float CameraPitchStepDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Explore")
	int32 MaxCameraPitchOffsetActionCount;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Explore")
	float CameraPitchHoldToleranceDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Explore")
	bool bDebugDrawExploreCandidates;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Explore", meta = (ClampMin = "0.01", UIMin = "0.01"))
	float VisitedSoftmaxTemperature;

	/** Camera location for first-person sampling. Collision test stays off; only the inherited capsule/NavMesh movement checks are used. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|First Person")
	FVector FirstPersonCameraRelativeLocation;

	/** Constant yaw speed used while rotating in place toward the sampled movement direction. ExploreActionDuration only controls the later walking phase. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Explore|First Person", meta = (ClampMin = "1.0", UIMin = "1.0"))
	float TurnInPlaceYawSpeedDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Explore|First Person", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float TurnYawToleranceDegrees;

	/** Fake XY speed exposed through CharacterMovement->Velocity while PhysWalking blocks actual translation.
	 *  Set to 24 by default, which is one fifth of the previous 120 in-place animation speed.
	 *  Values <= 0 disable code-only in-place pacing animation.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Explore|First Person", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float InPlacePaceAnimSpeed;


private:
	void StartExploreAction();
	void ExecuteExploreAction(float DeltaTime);

	void BuildLegalActionCandidates(TArray<FExploreMoveCandidate>& OutCandidates) const;
	bool TryBuildActionCandidate(ENPC1PExploreMoveAction Action, FExploreMoveCandidate& OutCandidate) const;
	bool IsLandingValidForDirection(const FVector& DesiredWorldDirection, FExploreMoveCandidate& OutCandidate) const;
	bool GetWorldDirectionForAction(ENPC1PExploreMoveAction Action, FVector& OutDirection) const;
	void GetMoveActionSignals(ENPC1PExploreMoveAction Action, int32& OutWS, int32& OutAD) const;
	ENPC1PExploreMoveAction GetOppositeMoveAction(ENPC1PExploreMoveAction Action) const;
	ENPC1PExploreCameraAction GetTurnCameraActionForMoveAction(ENPC1PExploreMoveAction Action) const;
	float GetTurnYawOffsetDegreesForMoveAction(ENPC1PExploreMoveAction Action, ENPC1PExploreCameraAction TurnAction) const;
	float GetVisitedScoreAtLocation(const FVector& WorldLocation) const;
	int32 SampleCandidateByVisitedSoftmax(const TArray<FExploreMoveCandidate>& Candidates) const;
	int32 SampleRandomCandidate(const TArray<FExploreMoveCandidate>& Candidates) const;
	bool IsMovePathCollisionFree(const FVector& StartActorLocation, const FVector& EndActorLocation) const;

	ENPC1PExploreCameraAction ChooseRandomCameraAction(const FRotator& CurrentCameraRotation, FRotator& OutDesiredRotation);
	ENPC1PExploreCameraAction MakeCameraAction(int32 LRSignal, int32 UDSignal) const;
	void GetCameraActionSignals(ENPC1PExploreCameraAction Action, int32& OutLR, int32& OutUD) const;
	void UpdatePitchOffsetHoldState(float CurrentPitchOffset);
	void SetCameraBoomYawRelativePitchWorld(USpringArmComponent* CameraBoomComp, const FRotator& MixedCameraRotation);
	FRotator GetCameraBoomYawRelativePitchWorld(const USpringArmComponent* CameraBoomComp) const;
	void BeginWalkCameraAction();
	void BeginCodeOnlyInPlacePace();
	void UpdateCodeOnlyInPlacePace();
	void EndCodeOnlyInPlacePace(bool bRestoreWalking = true);
	UNPC1PCharacterMovementComponent* GetNPC1PMovementComponent() const;
	void SetRecorderSignals(int32 WS, int32 AD, int32 LR, int32 UD);

private:
	bool bIsExecutingExploreAction = false;
	FVector CurrentExploreMoveTarget = FVector::ZeroVector;
	FVector StartExploreActorLocation = FVector::ZeroVector;
	FVector StartExploreFootLocation = FVector::ZeroVector;

	ENPC1PExploreMoveAction CurrentExploreMoveAction = ENPC1PExploreMoveAction::Idle;
	ENPC1PExploreMoveAction LastNonIdleExploreMoveAction = ENPC1PExploreMoveAction::Idle;
	ENPC1PExplorePhase CurrentExplorePhase = ENPC1PExplorePhase::None;
	float CurrentExploreActionElapsed = 0.0f;
	FRotator StartTurnActorRotation = FRotator::ZeroRotator;
	FRotator DesiredTurnActorRotation = FRotator::ZeroRotator;
	bool bWalkCameraActionStarted = false;
	ENPC1PExploreCameraAction CurrentTurnCameraAction = ENPC1PExploreCameraAction::None;

	bool bHasDesiredCameraWorldRotation = false;
	ENPC1PExploreCameraAction CurrentExploreCameraAction = ENPC1PExploreCameraAction::None;
	FRotator StartCameraYawRelativePitchWorld = FRotator::ZeroRotator;
	FRotator DesiredCameraYawRelativePitchWorld = FRotator::ZeroRotator;

	int32 CurrentRecorderWS = 0;
	int32 CurrentRecorderAD = 0;
	int32 CurrentRecorderLR = 0;
	int32 CurrentRecorderUD = 0;

	float LastNonZeroCameraPitchOffset = 0.0f;
	int32 SameNonZeroCameraPitchOffsetActionCount = 0;
};
