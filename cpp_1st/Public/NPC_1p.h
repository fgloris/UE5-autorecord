#pragma once

#include "CoreMinimal.h"
#include "NPC.h"
#include "NPC_1p.generated.h"

class UNavigationSystemV1;
class UCharacterMovementComponent;
class UNPC1PCharacterMovementComponent;

UENUM()
enum class ENPC1PPitchState : uint8
{
	Center,
	Away
};

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
	WalkForward,
	SocialTurnToPeer
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

	/** Called after smoothly turning to face another same-type NPC. */
	UFUNCTION(BlueprintImplementableEvent, Category = "Explore|Social")
	void OnSocialTurnExecuted(AActor* TargetNPC);

	/** Set the list of same-type NPCs to consider for social turns. Call with GetAllActorsOfClass output. */
	UFUNCTION(BlueprintCallable, Category = "Explore|Social")
	void SetSameTypeNPCList(const TArray<AActor*>& InList) { SameTypeNPCList = InList; }

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
	float YawAngularSpeed = 15.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Explore|First Person", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float TurnYawToleranceDegrees;

	/** Fake XY speed exposed through CharacterMovement->Velocity while PhysWalking blocks actual translation.
	 *  Set to 24 by default, which is one fifth of the previous 120 in-place animation speed.
	 *  Values <= 0 disable code-only in-place pacing animation.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Explore|First Person", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float InPlacePaceAnimSpeed;

	/** Independent pitch state machine.
	 *  Call from Blueprint every frame (e.g. on a Timeline or Event Tick).
	 *  Completely decoupled from ExecuteNextStep — pitch changes at its own rhythm. */
	UFUNCTION(BlueprintCallable, Category = "Camera|Pitch")
	void UpdateIndependentPitch(float DeltaTime);

	/** Angular velocity for pitch interpolation (degrees/sec). Mirrors YawAngularSpeed. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Pitch", meta = (ClampMin = "1.0", UIMin = "1.0"))
	float PitchAngularSpeed = 15.0f;

	/** Hard limit: pitch will never stay in a non-Center state longer than this (seconds). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Pitch", meta = (ClampMin = "0.1", UIMin = "0.1"))
	float PitchStateMaxNonCenterDuration = 2.0f;

	/** Minimum random wait in Center state before pitching away (seconds). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Pitch", meta = (ClampMin = "0.1", UIMin = "0.1"))
	float PitchStateMinCenterDuration = 2.0f;

	/** Maximum random wait in Center state before pitching away (seconds). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Pitch", meta = (ClampMin = "0.5", UIMin = "0.5"))
	float PitchStateMaxCenterDuration = 8.0f;

	/** Minimum absolute pitch angle when in Away state (degrees). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Pitch", meta = (ClampMin = "-90.0", UIMin = "-90.0"))
	float PitchMinAngle = -30.0f;

	/** Maximum absolute pitch angle when in Away state (degrees). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Pitch", meta = (ClampMin = "-90.0", UIMin = "-90.0"))
	float PitchMaxAngle = 30.0f;

	/** Minimum random interval between social turns (seconds). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Explore|Social", meta = (ClampMin = "1.0", UIMin = "1.0"))
	float SocialTurnMinInterval;

	/** Maximum random interval between social turns (seconds). Average(min,max) should give >= 3 turns/minute. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Explore|Social", meta = (ClampMin = "1.0", UIMin = "1.0"))
	float SocialTurnMaxInterval;

private:
	void StartExploreAction();
	void ExecuteExploreAction(float DeltaTime);

	void BuildLegalActionCandidates(TArray<FExploreMoveCandidate>& OutCandidates) const;
	bool TryBuildActionCandidate(ENPC1PExploreMoveAction Action, FExploreMoveCandidate& OutCandidate) const;
	bool IsLandingValidForDirection(const FVector& DesiredWorldDirection, FExploreMoveCandidate& OutCandidate) const;
	bool GetWorldDirectionForAction(ENPC1PExploreMoveAction Action, FVector& OutDirection) const;
	ENPC1PExploreMoveAction GetOppositeMoveAction(ENPC1PExploreMoveAction Action) const;
	ENPC1PExploreCameraAction GetTurnCameraActionForMoveAction(ENPC1PExploreMoveAction Action) const;
	float GetTurnYawOffsetDegreesForMoveAction(ENPC1PExploreMoveAction Action, ENPC1PExploreCameraAction TurnAction) const;
	float GetVisitedScoreAtLocation(const FVector& WorldLocation) const;
	int32 SampleCandidateByVisitedSoftmax(const TArray<FExploreMoveCandidate>& Candidates) const;
	ENPC1PExploreCameraAction ChooseRandomCameraAction(const FRotator& CurrentCameraRotation, FRotator& OutDesiredRotation);
	ENPC1PExploreCameraAction MakeCameraAction(int32 LRSignal, int32 UDSignal) const;
	void GetCameraActionSignals(ENPC1PExploreCameraAction Action, int32& OutLR, int32& OutUD) const;
	void SetCameraBoomYawRelativePitchWorld(USpringArmComponent* CameraBoomComp, const FRotator& MixedCameraRotation);
	FRotator GetCameraBoomYawRelativePitchWorld(const USpringArmComponent* CameraBoomComp) const;
	void BeginWalkCameraAction();
	void BeginCodeOnlyInPlacePace();
	void UpdateCodeOnlyInPlacePace();
	void EndCodeOnlyInPlacePace(bool bRestoreWalking = true);
	UNPC1PCharacterMovementComponent* GetNPC1PMovementComponent() const;
	void SetRecorderSignals(int32 WS, int32 AD, int32 LR, int32 UD);

	/** --- Independent pitch state machine --- */
	void EnterPitchState(ENPC1PPitchState NewState);

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

	/** --- Independent pitch state machine members --- */
	ENPC1PPitchState CurrentPitchState = ENPC1PPitchState::Center;
	float PitchStateElapsed = 0.0f;
	float CurrentPitchStateDuration = 0.0f;
	float CurrentDesiredPitch = 0.0f;

	/** --- Social turn system --- */
	void StartSocialTurn();
	AActor* FindNearestSameTypeNPC() const;

	UPROPERTY()
	TArray<AActor*> SameTypeNPCList;

	UPROPERTY()
	AActor* SocialTurnTargetNPC = nullptr;

	float SocialTurnCooldownRemaining = 0.0f;
};
