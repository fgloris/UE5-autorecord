// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "Delegates/DelegateCombinations.h"
#include "NPC.generated.h"

class USpringArmComponent;
class UCameraComponent;
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
 * NPC角色类：具备自定义A*导航算法的第三人称角色
 *
 * 核心功能：
 * - 弹簧臂 + 第三人称摄像机系统
 * - 自定义A*路径规划算法（状态空间：x,y,z,θ）
 * - 摄像机碰撞避免
 * - 自动路径跟随执行
 */
UCLASS()
class CPP_3P_API ANPC : public ACharacter
{
	GENERATED_BODY()

public:
	// 构造函数：设置角色的默认属性值
	ANPC();

protected:
	// 游戏开始时调用
	virtual void BeginPlay() override;

	/** ==================== 摄像机系统 ==================== */

	/** 摄像机弹簧臂：将摄像机定位在角色后方，支持碰撞检测 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Camera, meta = (AllowPrivateAccess = "true"))
	class USpringArmComponent* CameraBoom;

	/** 摄像机碰撞可视化球体：显示摄像机碰撞检测区域（编辑器中始终可见） */
	// UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Camera, meta = (AllowPrivateAccess = "true"))
	// class UStaticMeshComponent* CameraCollisionVisualizer;

	/** ==================== A*导航算法配置 ==================== */

	/** 导航网格单元大小（cm），默认100cm，影响路径精度和性能 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|AStar")
	float GridSize;

	/** 摄像机角度离散化步长（弧度），默认PI/8（22.5度），影响角度探索精度 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|AStar")
	float CameraAngleStep;

	/** A*搜索最大距离（cm），默认5000cm（50米），限制搜索范围以提升性能 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|AStar", meta = (ClampMin = "100.0", UIMin = "100.0"))
	float MaxDistance;

	/**
	 * 相邻节点之间允许的最大高度差（cm），默认50cm，超过此值视为不可通过
	 * 子类可以通过覆盖GetMaxStepHeight()来动态调整此值
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|AStar", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float MaxStepHeight;

	/** ==================== 异步寻路系统 ==================== */

	/**
	 * 异步A*路径查找（不阻塞主线程，委托给Manager串行执行）
	 * @param StartPos 起点位置
	 * @param StartAngle 起点摄像机角度（弧度）
	 * @param GoalPos 终点位置
	 * @param PathfindingManager 寻路管理器（必须提供）
	 */
	UFUNCTION(BlueprintCallable, Category = "Navigation|AStar")
	void FindPathAStarAsync(const FVector& StartPos, float StartAngle, const FVector& GoalPos, ANPCPathfindingManager* PathfindingManager);

	/**
	 * 到达目标点委托
	 * 参数：
	 *   - NPC: 到达目标点的 NPC 指针
	 *   - CompletedPath: 完成的路径
	 */
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnDestinationReached, ANPC*, NPC, const TArray<FNPCNavigationState>&, CompletedPath);

	/** 到达目标点事件，可在蓝图中绑定 */
	UPROPERTY(BlueprintAssignable, Category = "Navigation|Follow")
	FOnDestinationReached OnDestinationReached;

public:
	/** 是否正在执行异步寻路 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Navigation|AStar")
	bool bIsPathfindingInProgress;

	/** ==================== 路径跟随系统 ==================== */

	/** 当前正在跟随的路径 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Navigation|Follow")
	TArray<FNPCNavigationState> CurrentPath;

	/** 当前路径点索引，指向CurrentPath数组中的下一个目标点 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Navigation|Follow")
	int32 CurrentPathIndex;

	/** 是否已经广播过到达目标点事件（用于防止重复广播） */
	bool bHasBroadcastDestinationReached;

	/** NPC移动速度（单位/秒），默认300 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Follow")
	float MovementSpeed;

	/** 摄像机旋转插值速度，默认5.0，影响角度平滑度 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Follow")
	float CameraRotationSpeed;

	/** ==================== 摄像机系统设置 ==================== */

	/** 摄像机弹簧臂长度，默认200cm */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
	float CameraBoomLength;

	/** 摄像机俯仰角（度），默认15度（向下倾斜） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
	float CameraBoomPitch;

	/** 是否启用摄像机碰撞检测 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
	bool bEnableCameraCollision;

	/** ==================== 调试工具 ==================== */

	/** 是否绘制路径调试可视化（绿线=已完成，红线=未完成） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Debug")
	bool bDebugDrawPath;

	/** 是否绘制摄像机碰撞检测调试信息 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Debug")
	bool bDebugDrawCameraCollision;

	/**
	 * 绘制导航调试信息（路径线、路径点、摄像机方向）
	 * 需要在Tick中调用或在蓝图事件中调用
	 */
	UFUNCTION(BlueprintCallable, Category = "Navigation|Debug")
	void DrawDebugNavigation();

public:
	// 每帧调用，处理路径跟随逻辑
	// virtual void Tick(float DeltaTime) override;

	// 绑定输入到功能（当前为空，可扩展）
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	/** ==================== 摄像机组件访问器 ==================== */

	/** 获取摄像机弹簧臂组件 */
	FORCEINLINE class USpringArmComponent* GetCameraBoom() const { return CameraBoom; }

	/**
	 * 获取当前摄像机的偏航角（Yaw）
	 * @return 摄像机的偏航角度（弧度，范围 [0, 2π)）
	 */
	UFUNCTION(BlueprintCallable, Category = "Camera")
	float GetCameraYaw() const;

	UFUNCTION(BlueprintCallable, Category = "Navigation|Execute")
	void ExecuteCurrentPath(float DeltaTime, ANPCPathfindingManager* PathfindingManager = nullptr);

	/** ==================== 路径跟随控制函数 ==================== */

	/**
	 * 开始路径跟随
	 * @param Path 要跟随的路径数组（由Manager的寻路系统生成）
	 * @param PathfindingManager 路径管理器指针（用于注册路径）
	 */
	UFUNCTION(BlueprintCallable, Category = "Navigation|Follow")
	void StartPathFollowing(const TArray<FNPCNavigationState>& Path, ANPCPathfindingManager* PathfindingManager = nullptr);

	/**
	 * 停止路径跟随，清空当前路径
	 * @param PathfindingManager 路径管理器指针（用于移除注册）
	 */
	UFUNCTION(BlueprintCallable, Category = "Navigation|Follow")
	void StopPathFollowing(ANPCPathfindingManager* PathfindingManager = nullptr);

	/**
	 * 检查是否已到达目的地
	 * @return 如果已到达最后一个路径点则返回true
	 */
	UFUNCTION(BlueprintCallable, Category = "Navigation|Follow")
	bool HasReachedDestination() const;

	/** ==================== NPC录制系统 ==================== */

	/** NPC移动记录器组件 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Recording", meta = (AllowPrivateAccess = "true"))
	class UNPCMovementRecorder* MovementRecorder;

	/**
	 * 开始记录NPC移动
	 * @param bClearOldData 是否清空之前的记录数据，默认true
	 */
	UFUNCTION(BlueprintCallable, Category = "Recording")
	void StartMovementRecording(bool bClearOldData = true);

	/**
	 * 保存NPC移动记录到JSON文件
	 * @param SavePath 自定义保存路径，为空时使用默认路径
	 * @return 是否保存成功
	 */
	UFUNCTION(BlueprintCallable, Category = "Recording")
	bool SaveMovementRecording(const FString& SavePath = TEXT(""));

	/**
	 * 获取记录器组件（用于高级配置）
	 * @return 记录器组件指针
	 */
	UFUNCTION(BlueprintCallable, Category = "Recording")
	UNPCMovementRecorder* GetMovementRecorder() const { return MovementRecorder; }


	/**
	 * 公共接口：检查位置是否适合NPC站立（仅胶囊体碰撞检测）
	 * @param Position 要检查的位置
	 * @return 如果位置适合站立则返回true
	 */
	UFUNCTION(BlueprintCallable, Category = "Navigation|Collision")
	bool IsLocationValidForNPC(const FVector& Position) const;

	/** ==================== 粗网格访问统计 ==================== */

	/** 正态分布核标准差（cm），默认等于GridSize */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|Visit Stats")
	float KernelSigma;

	/** 核函数影响半径（cm），根据KernelSigma和阈值0.1预计算 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Navigation|Visit Stats")
	float KernelInfluenceRadius;

	/** 网格访问统计：Key为网格坐标"X_Y"，Value为访问权重（浮点数） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Navigation|Visit Stats")
	TMap<FString, float> Visited;

	/**
	 * 根据路径更新访问统计
	 * 遍历路径上的所有点，增加对应网格的访问次数
	 */
	UFUNCTION(BlueprintCallable, Category = "Navigation|Visit Stats")
	void UpdateVisitedStats(const TArray<FNPCNavigationState>& Path);

	/**
	 * 根据NPC当前位置更新访问统计（简化版）
	 * 使用正态分布核函数为当前位置周围的网格增加访问权重
	 */
	UFUNCTION(BlueprintCallable, Category = "Navigation|Visit Stats")
	void UpdateVisitedStatsAtCurrentPosition();

	/**
	 * 可视化NPC周围半径内所有点的visited状况
	 * 颜色说明：
	 *   - 绿色：未访问（count=0）
	 *   - 黄色：访问1次
	 *   - 橙色：访问2次
	 *   - 红色：访问3次及以上
	 * @param Radius 可视化半径（cm）
	 */
	UFUNCTION(BlueprintCallable, Category = "Navigation|Visit Stats")
	void VisualizeVisitedInRadius(float Radius = 1000.0f);
};
