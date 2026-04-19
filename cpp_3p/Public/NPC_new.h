#pragma once

#include "CoreMinimal.h"
#include "NPC.h"
#include "NPC_new.generated.h"

class UNavigationSystemV1;
class UCharacterMovementComponent;

UENUM()
enum class ENPCExploreMoveAction : uint8
{
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
enum class ENPCExploreCameraAction : uint8
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
class CPP_3P_API ANPC_new : public ANPC
{
	GENERATED_BODY()

public:
	ANPC_new();

	UFUNCTION(BlueprintCallable, Category = "Navigation|Explore")
	void ExecuteNextStep(float DeltaTime);

	UFUNCTION(BlueprintCallable, Category = "Navigation|Explore")
	bool HasActiveExploreMoveTarget() const { return bIsExecutingExploreAction; }

	UFUNCTION(BlueprintCallable, Category = "Navigation|Explore")
	FVector GetCurrentExploreMoveTarget() const { return CurrentExploreMoveTarget; }

	UFUNCTION(BlueprintCallable, Category = "Navigation|Explore")
	void ClearExploreMoveTarget();

	void GetCurrentRecorderControlSignals(int32& OutWS, int32& OutAD, int32& OutLR, int32& OutUD) const;

	UFUNCTION(BlueprintImplementableEvent, Category = "Navigation|Explore")
	void OnExploreMoveTargetReached(const FVector& ReachedLocation);

protected:
	virtual void BeginPlay() override;

	struct FExploreMoveCandidate
	{
		ENPCExploreMoveAction Action = ENPCExploreMoveAction::W;
		FVector WorldDirection = FVector::ZeroVector;
		FVector LandingFootLocation = FVector::ZeroVector;
		FVector LandingActorLocation = FVector::ZeroVector;
		float VisitedScore = 0.0f;
	};

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore", meta = (ClampMin = "0.01", UIMin = "0.01"))
	float ExploreActionDuration;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float CameraYawStepDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float CameraPitchStepDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float MinCameraPitchDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	int32 MaxCameraPitchOffsetActionCount;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float CameraPitchHoldToleranceDegrees;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore")
	bool bUpdateVisitedBeforeSampling;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore")
	bool bDebugDrawExploreCandidates;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore", meta = (ClampMin = "0.01", UIMin = "0.01"))
	float VisitedSoftmaxTemperature;

private:
	void StartExploreAction();
	void ExecuteExploreAction(float DeltaTime);

	void BuildLegalActionCandidates(TArray<FExploreMoveCandidate>& OutCandidates) const;
	bool TryBuildActionCandidate(ENPCExploreMoveAction Action, FExploreMoveCandidate& OutCandidate) const;
	bool IsLandingValidForDirection(const FVector& DesiredWorldDirection, FExploreMoveCandidate& OutCandidate) const;
	bool GetWorldDirectionForAction(ENPCExploreMoveAction Action, FVector& OutDirection) const;
	void GetMoveActionSignals(ENPCExploreMoveAction Action, int32& OutWS, int32& OutAD) const;
	float GetVisitedScoreAtLocation(const FVector& WorldLocation) const;
	int32 SampleCandidateByVisitedSoftmax(const TArray<FExploreMoveCandidate>& Candidates) const;

	bool IsMovePathCollisionFree(const FVector& StartActorLocation, const FVector& EndActorLocation) const;
	ENPCExploreCameraAction ChooseRandomCameraAction(const FRotator& CurrentCameraRotation, FRotator& OutDesiredRotation);
	ENPCExploreCameraAction MakeCameraAction(int32 LRSignal, int32 UDSignal) const;
	void GetCameraActionSignals(ENPCExploreCameraAction Action, int32& OutLR, int32& OutUD) const;
	void UpdatePitchOffsetHoldState(float CurrentPitchOffset);

private:
	bool bIsExecutingExploreAction = false;
	FVector CurrentExploreMoveTarget = FVector::ZeroVector;

	ENPCExploreMoveAction CurrentExploreMoveAction = ENPCExploreMoveAction::W;
	float CurrentExploreActionElapsed = 0.0f;

	bool bHasDesiredCameraWorldRotation = false;
	ENPCExploreCameraAction CurrentExploreCameraAction = ENPCExploreCameraAction::None;
	FRotator StartCameraWorldRotation = FRotator::ZeroRotator;
	FRotator DesiredCameraWorldRotation = FRotator::ZeroRotator;

	int32 CurrentRecorderWS = 0;
	int32 CurrentRecorderAD = 0;
	int32 CurrentRecorderLR = 0;
	int32 CurrentRecorderUD = 0;

	float LastNonZeroCameraPitchOffset = 0.0f;
	int32 SameNonZeroCameraPitchOffsetActionCount = 0;
};
