#pragma once

#include "CoreMinimal.h"
#include "NPC.h"
#include "NPC_new.generated.h"

class UNavigationSystemV1;

UCLASS()
class CPP_3P_API ANPC_new : public ANPC
{
	GENERATED_BODY()

public:
	ANPC_new();

	/**
	 * 在 Event Tick 中调用：
	 * - 先在 8 个相机局部移动动作中，按“低探索度优先”的分布随机采样一个可达动作
	 * - 再在相机 LRUD 四个动作中随机采样一个可用动作
	 * - 同时保证人物与相机不撞墙
	 */
	UFUNCTION(BlueprintCallable, Category = "Navigation|Explore")
	void ExecuteNextStep(float DeltaTime);

protected:
	virtual void BeginPlay() override;

	struct FExploreMoveCandidate
	{
		FVector WorldDirection = FVector::ZeroVector;
		FVector TargetLocation = FVector::ZeroVector;
		float VisitValue = 0.0f;
		float Weight = 0.0f;
	};

	/** 单步探测距离；只用于候选状态评估，不是实际瞬移距离 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore")
	float ProbeStepDistance;

	/** 越大越偏向未探索区域；权重 = exp(-Visited * ExplorationBias) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore", meta = (ClampMin = "0.01", UIMin = "0.01"))
	float ExplorationBias;

	/** 摄像机每次左右动作的偏航步长（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float CameraYawStepDegrees;

	/** 摄像机每次上下动作的俯仰步长（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float CameraPitchStepDegrees;

	/** 摄像机俯仰最小值（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float MinCameraPitchDegrees;

	/** 摄像机俯仰最大值（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float MaxCameraPitchDegrees;

	/** 相机碰撞检测球半径 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Explore")
	float CameraCollisionProbeRadius;

	/** 是否在每次执行前，把当前位置记入探索统计 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore")
	bool bUpdateVisitedBeforeSampling;

	/** 是否绘制候选点与相机碰撞调试信息 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Explore")
	bool bDebugDrawExploreCandidates;

private:
	void BuildReachableMoveCandidates(TArray<FExploreMoveCandidate>& OutCandidates) const;
	bool TryBuildMoveCandidate(const FVector& DesiredWorldDirection, FExploreMoveCandidate& OutCandidate) const;
	float GetVisitedValueAtLocation(const FVector& WorldLocation) const;
	int32 SampleWeightedCandidateIndex(const TArray<FExploreMoveCandidate>& Candidates) const;

	bool IsMovePathCollisionFree(const FVector& StartLocation, const FVector& EndLocation) const;
	bool IsCameraPoseCollisionFree(const FVector& ActorLocation, const FRotator& DesiredCameraWorldRotation) const;
	bool TryApplyRandomCameraAction();
};
