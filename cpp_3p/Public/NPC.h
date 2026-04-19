// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "NPC.generated.h"

class USpringArmComponent;
class UNPCMovementRecorder;

/**
 * 自定义A*导航算法的状态结构
 * 状态空间：(x, y, z, θ) 其中 θ 是摄像机偏航角
 *
 * 用途：表示A*算法中的一个节点，包含位置、角度和成本信息
 */
USTRUCT(BlueprintType)
struct FNPCNavigationState
{
	GENERATED_BODY()

	/** 世界空间位置坐标 (x, y, z)，z轴高度由NavMesh动态提供 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation State")
	FVector Position;

	/** 摄像机相对于角色的水平偏航角（弧度，范围 [0, 2π)） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation State")
	float CameraYawAngle;

	/** A*算法的g值：从起点到当前状态的实际成本 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation State")
	float GCost;

	/** A*算法的h值：从当前状态到目标的估计成本（启发式） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation State")
	float HCost;

	/** A*算法的f值：总成本 = g + h，用于选择最优节点 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation State")
	float FCost;

	/** 父节点索引，用于路径重建（使用索引避免TArray扩容导致的指针失效） */
	int32 ParentIndex;

	/** 构造函数：初始化所有成员为默认值 */
	FNPCNavigationState()
		: Position(FVector::ZeroVector)
		, CameraYawAngle(0.0f)
		, GCost(0.0f)
		, HCost(0.0f)
		, FCost(0.0f)
		, ParentIndex(-1)  // -1表示无父节点（起点）
	{
	}

	/** 计算总成本 f = g + h */
	void CalculateFCost()
	{
		FCost = GCost + HCost;
	}

	/** 转换为FVector4格式 (x,y,z,θ) 用于哈希和比较 */
	FVector4 ToVector4() const
	{
		return FVector4(Position.X, Position.Y, Position.Z, CameraYawAngle);
	}

	/** 状态相等性比较（用于判断是否为相同状态）
	 * @param Other 要比较的状态
	 * @param PositionTolerance 位置容差（cm），默认75cm（1.5个网格）
	 * @param AngleTolerance 角度容差（弧度），默认22.5度（PI/8）
	 * @return 如果位置和角度都在容差范围内则返回true
	 */
	bool IsSameState(const FNPCNavigationState& Other, float PositionTolerance = 75.0f, float AngleTolerance = PI / 8.0f) const
	{
		// 只计算XY平面的距离（忽略Z轴高度差）
		FVector2D Pos2D(Position.X, Position.Y);
		FVector2D OtherPos2D(Other.Position.X, Other.Position.Y);
		float Distance = FVector2D::Distance(Pos2D, OtherPos2D);

		// 计算角度差异（处理环绕，例如359°和1°只差2°）
		float AngleDiff = FMath::Abs(CameraYawAngle - Other.CameraYawAngle);
		if (AngleDiff > PI)
			AngleDiff = 2.0f * PI - AngleDiff;

		return Distance < PositionTolerance && AngleDiff < AngleTolerance;
	}

	bool IsSameStateFinal(const FNPCNavigationState& Other, float PositionTolerance = 75.0f) const
	{
		// 只计算XY平面的距离（忽略Z轴高度差）
		FVector2D Pos2D(Position.X, Position.Y);
		FVector2D OtherPos2D(Other.Position.X, Other.Position.Y);
		float Distance = FVector2D::Distance(Pos2D, OtherPos2D);
		return Distance < PositionTolerance;
	}
};

/**
 * NPC角色基类：只保留 NPC_new 仍然需要的基础能力
 *
 * 核心功能：
 * - 弹簧臂相机基座
 * - 基础碰撞站立检测
 * - visited 访问统计
 * - 录制组件
 */
UCLASS()
class CPP_3P_API ANPC : public ACharacter
{
	GENERATED_BODY()

public:
	ANPC();

protected:
	virtual void BeginPlay() override;

	/** 摄像机弹簧臂：将摄像机定位在角色后方 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Camera, meta = (AllowPrivateAccess = "true"))
	USpringArmComponent* CameraBoom;

public:
	/** 网格单元大小（cm） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Base")
	float GridSize;

	/** 相邻节点之间允许的最大高度差（cm） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Base", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float MaxStepHeight;

	/** 当前路径（保留给录制组件复用） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Navigation|Base")
	TArray<FNPCNavigationState> CurrentPath;

	/** 当前路径点索引（保留给录制组件复用） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Navigation|Base")
	int32 CurrentPathIndex;

	/** 摄像机弹簧臂长度 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
	float CameraBoomLength;

	/** 摄像机俯仰角（度） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
	float CameraBoomPitch;

	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	/** 获取摄像机弹簧臂组件 */
	FORCEINLINE USpringArmComponent* GetCameraBoom() const { return CameraBoom; }

	/** 公共接口：检查位置是否适合NPC站立（仅胶囊体碰撞检测） */
	UFUNCTION(BlueprintCallable, Category = "Navigation|Collision")
	bool IsLocationValidForNPC(const FVector& Position) const;

	/** ==================== NPC录制系统 ==================== */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Recording", meta = (AllowPrivateAccess = "true"))
	UNPCMovementRecorder* MovementRecorder;

	UFUNCTION(BlueprintCallable, Category = "Recording")
	void StartMovementRecording(bool bClearOldData = true);

	UFUNCTION(BlueprintCallable, Category = "Recording")
	bool SaveMovementRecording(const FString& SavePath = TEXT(""));

	UFUNCTION(BlueprintCallable, Category = "Recording")
	UNPCMovementRecorder* GetMovementRecorder() const { return MovementRecorder; }

	/** ==================== 粗网格访问统计 ==================== */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Visit Stats")
	float KernelSigma;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Navigation|Visit Stats")
	float KernelInfluenceRadius;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Navigation|Visit Stats")
	TMap<FString, float> Visited;

	UFUNCTION(BlueprintCallable, Category = "Navigation|Visit Stats")
	void UpdateVisitedStats(const TArray<FNPCNavigationState>& Path);

	UFUNCTION(BlueprintCallable, Category = "Navigation|Visit Stats")
	void UpdateVisitedStatsAtCurrentPosition();

	UFUNCTION(BlueprintCallable, Category = "Navigation|Visit Stats")
	void VisualizeVisitedInRadius(float Radius = 1000.0f);
};
