// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Delegates/DelegateCombinations.h"
#include "Misc/SpinLock.h"
#include "NPC.h"
#include "NPCPathfindingHelpers.h"
#include "NPCPathfindingManager.generated.h"

class ANPC;
class USpringArmComponent;
class UCameraComponent;
class UNavigationSystemV1;
struct FNPCNavigationState;
class FMinHeap;
class FStateSet;
class ANavMeshBoundsVolume;

/**
 * NPC路径数据结构：存储单个NPC的完整路径和当前进度
 */
USTRUCT(BlueprintType)
struct FNPCPathData
{
	GENERATED_BODY()

	/** 此路径所属的NPC（使用弱引用避免循环依赖和悬空指针） */
	UPROPERTY()
	TWeakObjectPtr<ANPC> NPC;

	/** 从起点到终点的完整路径 */
	UPROPERTY()
	TArray<FNPCNavigationState> Path;

	/** 路径中的当前索引（未到达的点从此索引开始） */
	UPROPERTY()
	int32 CurrentPathIndex;

	/** 构造函数：初始化默认值 */
	FNPCPathData()
		: NPC(nullptr)
		, CurrentPathIndex(0)
	{
	}

	/**
	 * 检查路径数据是否有效
	 * @return 如果NPC有效且路径非空则返回true
	 */
	bool IsValid() const
	{
		return NPC.IsValid() && Path.Num() > 0;
	}

	/**
	 * 获取未到达路径点的数量
	 * @return 从CurrentPathIndex到路径末尾的点数量
	 */
	int32 GetUnreachedPointCount() const
	{
		if (!IsValid() || CurrentPathIndex >= Path.Num())
			return 0;
		return Path.Num() - CurrentPathIndex;
	}
};

/**
 * 寻路任务状态
 */
enum class EPathfindingTaskStatus
{
	NotStarted,      // 任务未开始
	InProgress,      // 任务进行中
	Completed,       // 任务完成
	Failed           // 任务失败
};

/**
 * 寻路任务结构：包含一次寻路请求的所有信息
 */
USTRUCT(BlueprintType)
struct FPathfindingTask
{
	GENERATED_BODY()
	/** 请求寻路的NPC */
	TWeakObjectPtr<ANPC> RequestingNPC;

	/** 起点位置 */
	FVector StartPos;

	/** 起点摄像机角度（弧度） */
	float StartAngle;

	/** 终点位置 */
	FVector GoalPos;

	/** 任务ID（用于追踪） */
	int32 TaskId;

	/** 任务状态 */
	EPathfindingTaskStatus Status;

	/** 算法内部状态 */
	FMinHeap OpenSet;
	FStateSet OpenSetLookup;
	FStateSet ClosedSet;
	TArray<FNPCNavigationState> AllStates;

	/** 目标状态缓存 */
	FNPCNavigationState GoalState;

	/** 预设的不可达点 */
	TArray<FVector> InaccessiblePoints;

	/** 当前迭代次数 */
	int32 IterationCount;

	/** 累计执行时间(毫秒) */
	float AccumulatedTime;

	/** 找到的路径(任务完成时有效) */
	TArray<FNPCNavigationState> FoundPath;

	/** 默认构造函数（USTRUCT要求） */
	FPathfindingTask()
		: RequestingNPC(nullptr)
		, StartPos(FVector::ZeroVector)
		, StartAngle(0.0f)
		, GoalPos(FVector::ZeroVector)
		, TaskId(0)
		, Status(EPathfindingTaskStatus::NotStarted)
		, IterationCount(0)
		, AccumulatedTime(0.0f)
	{
	}

	/** 参数化构造函数 */
	FPathfindingTask(ANPC* NPC, const FVector& InStartPos, float InStartAngle, const FVector& InGoalPos, int32 InTaskId)
		: RequestingNPC(NPC)
		, StartPos(InStartPos)
		, StartAngle(InStartAngle)
		, GoalPos(InGoalPos)
		, TaskId(InTaskId)
		, Status(EPathfindingTaskStatus::NotStarted)
		, IterationCount(0)
		, AccumulatedTime(0.0f)
	{
	}
};

/**
 * 寻路任务结果结构
 */
USTRUCT(BlueprintType)
struct FPathfindingResult
{
	GENERATED_BODY()

	/** 请求寻路的NPC */
	UPROPERTY()
	TWeakObjectPtr<ANPC> RequestingNPC;

	/** 是否成功找到路径 */
	UPROPERTY()
	bool bSuccess;

	/** 找到的路径 */
	UPROPERTY()
	TArray<FNPCNavigationState> Path;

	/** 寻路耗时（毫秒） */
	UPROPERTY()
	float ElapsedTime;

	/** 任务ID */
	UPROPERTY()
	int32 TaskId;

	/** 构造函数 */
	FPathfindingResult()
		: bSuccess(false)
		, ElapsedTime(0.0f)
		, TaskId(0)
	{
	}
};

/**
 * 简化的NPC路径管理器
 *
 * 功能：
 * - 提供随机生成NavMesh内可达终点的功能
 * - 支持指定搜索中心和半径
 * - 集中管理所有NPC的路径信息，收集未到达点供A*算法使用
 * - 串行执行所有寻路请求，避免多线程冲突
 */
UCLASS()
class CPP_3P_API ANPCPathfindingManager : public AActor
{
	GENERATED_BODY()

public:
	// 构造函数
	ANPCPathfindingManager();

protected:
	// 游戏开始时调用
	virtual void BeginPlay() override;

public:
	// 每帧调用（用于处理寻路任务队列）
	virtual void Tick(float DeltaTime) override;

	/** ==================== 主要功能 ==================== */

	/**
	 * 生成一个随机可导航的目标点（带碰撞检测）
	 * @param NPC NPC指针
	 * @param Radius 搜索半径（cm）
	 * @param OutLocation 输出的随机位置
	 * @return 是否成功生成有效位置
	 */
	UFUNCTION(BlueprintCallable, Category = "NPC Pathfinding Manager")
	bool GenerateRandomNavigableLocation(ANPC* NPC, float Radius, FVector& OutLocation, int32 MaxRandomAttempts = 5);

	/**
	 * 分析指定NavMesh区域内的胶囊体碰撞
	 * 按照A*网格大小遍历圆形区域内的NavMesh点，检测每个点的胶囊体碰撞状态
	 *
	 * @param CenterPos 检测区域的中心位置
	 * @param Radius 检测区域半径（cm）
	 * @param NPC 用于获取胶囊体参数的NPC引用
	 * @param OutCollisionLocations 输出：有碰撞的位置数组
	 * @param OutSafeLocations 输出：无碰撞的位置数组
	 * @return 总共检测的点数量
	 */
	UFUNCTION(BlueprintCallable, Category = "NPC Pathfinding Manager|Analysis")
	int32 AnalyzeNavMeshCapsuleCollisions(
	    const FVector& CenterPos,
	    float Radius,
	    ANPC* NPC,
	    TArray<FVector>& OutCollisionLocations,
	    TArray<FVector>& OutSafeLocations
	);

	/**
	 * 可视化NavMesh碰撞分析结果
	 * 在场景中绘制永久调试点直观显示碰撞分析结果
	 *
	 * @param CollisionLocations 有碰撞的位置数组（显示为红色）
	 * @param SafeLocations 无碰撞的位置数组（显示为绿色）
	 */
	UFUNCTION(BlueprintCallable, Category = "NPC Pathfinding Manager|Visualization")
	void VisualizeNavMeshCollisionAnalysis(
	    const TArray<FVector>& CollisionLocations,
	    const TArray<FVector>& SafeLocations
	);

	/**
	 * 可视化Reachable碰撞检测缓存
	 * 在场景中绘制永久调试点直观显示碰撞分析结果
	 *
	 * @param NavMeshBoundsVolume
	 * @param CheckNPC
	 */
	UFUNCTION(BlueprintCallable, Category = "NPC Pathfinding Manager|Visualization")
	void VisualizeReachableMap(ANavMeshBoundsVolume* NavMeshBoundsVolume, ANPC* CheckNPC);

	/** ==================== 路径跟踪系统 ==================== */

	/**
	 * 在管理器中注册或更新NPC的路径
	 * @param NPC 要注册/更新的NPC
	 * @param Path 完整路径（空路径表示移除该NPC）
	 * @param CurrentIndex 当前进度索引
	 */
	UFUNCTION(BlueprintCallable, Category = "NPC Pathfinding Manager")
	void RegisterOrUpdateNPCPath(ANPC* NPC, const TArray<FNPCNavigationState>& Path, int32 CurrentIndex = 0);

	/**
	 * 更新NPC的当前路径索引（NPC移动时调用）
	 * @param NPC 要更新的NPC
	 * @param NewIndex 新路径索引（应 >= 0 且 < Path.Num()）
	 * @return 更新是否成功
	 */
	UFUNCTION(BlueprintCallable, Category = "NPC Pathfinding Manager")
	bool UpdateNPCPathProgress(ANPC* NPC, int32 NewIndex);

	/**
	 * 从路径跟踪中移除NPC
	 * @param NPC 要移除的NPC
	 * @return 移除是否成功
	 */
	UFUNCTION(BlueprintCallable, Category = "NPC Pathfinding Manager")
	bool RemoveNPCFromTracking(ANPC* NPC);

	/**
	 * 从所有已注册NPC获取未到达的路径点
	 * @param OutPoints 输出的未到达点数组
	 * @param ExcludeNPC 要排除的NPC（正在寻路的NPC自己）
	 * @return 收集到的点数量
	 */
	UFUNCTION(BlueprintCallable, Category = "NPC Pathfinding Manager")
	int32 GetAllUnreachedPoints(TArray<FVector>& OutPoints, ANPC* ExcludeNPC = nullptr) const;

	/**
	 * 获取当前跟踪的NPC数量
	 * @return 已注册且有效的NPC数量
	 */
	UFUNCTION(BlueprintCallable, Category = "NPC Pathfinding Manager")
	int32 GetRegisteredNPCCount() const;

	/**
	 * 打印所有已注册NPC的调试信息
	 */
	UFUNCTION(BlueprintCallable, Category = "NPC Pathfinding Manager")
	void PrintRegisteredNPCsInfo() const;

	/** ==================== 串行寻路系统 ==================== */

	/**
	 * 注册NPC列表（用于碰撞检测时忽略）
	 * @param NPCs 要注册的NPC数组
	 */
	 UFUNCTION(BlueprintCallable, Category = "NPC Pathfinding Manager")
	void RegisterNPCs(const TArray<ANPC*>& NPCs);

	/**
	 * 请求异步寻路（NPC调用此接口提交任务）
	 * @param NPC 请求寻路的NPC
	 * @param StartPos 起点位置
	 * @param StartAngle 起点摄像机角度（弧度）
	 * @param GoalPos 终点位置
	 * @return 任务ID（-1表示失败）
	 */
	int32 RequestPathfinding(ANPC* NPC, const FVector& StartPos, float StartAngle, const FVector& GoalPos);

	/**
	 * 寻路完成委托
	 * 参数：
	 *   - NPC: 触发事件的 NPC 指针
	 *   - bSuccess: 寻路是否成功
	 *   - Path: 找到的路径（成功时有效）
	 *   - ElapsedTime: 寻路耗时（毫秒）
	 */
	DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnPathfindingComplete, ANPC*, NPC, bool, bSuccess, const TArray<FNPCNavigationState>&, Path);

	/** 寻路完成事件，可在蓝图中绑定 */
	UPROPERTY(BlueprintAssignable, Category = "NPC Pathfinding Manager")
	FOnPathfindingComplete OnPathfindingComplete;

private:
	/**
	 * 打印调试日志
	 */
	void PrintDebugLog(const FString& Message) const;

	/** ==================== 寻路核心函数 ==================== */

	/**
	 * 获取指定状态的所有邻居节点
	 * @param CurrentState 当前状态
	 * @param OutNeighbors 输出的邻居状态数组
	 * @param GoalState 目标状态（用于距离检查）
	 * @param InaccessiblePoints 预设的不可达点数组
	 * @param World 世界指针（从FindPathAstarInternal传入）
	 * @param NavSys 导航系统指针（从FindPathAstarInternal传入）
	 * @param NPC 请求寻路的NPC指针（从FindPathAstarInternal传入）
	 */
	void GetNeighborStates(const FNPCNavigationState& CurrentState, TArray<FNPCNavigationState>& OutNeighbors, const FNPCNavigationState& GoalState, const TArray<FVector>& InaccessiblePoints, UWorld* World, UNavigationSystemV1* NavSys, ANPC* NPC);

	/**
	 * 计算两个状态之间的启发式距离（h值）
	 * @param From 起始状态
	 * @param To 目标状态
	 * @return 估计的成本值
	 */
	float CalculateHeuristic(const FNPCNavigationState& From, const FNPCNavigationState& To) const;

	/**
	 * 检查位置是否与胶囊体碰撞
	 * @param NPC 要检查的NPC
	 * @param Position 要检查的位置
	 * @return 如果位置没有碰撞则返回true
	 */
	bool CheckCapsuleCollision(ANPC* NPC, const FVector& Position) const;

	/**
	 * 检查摄像机在指定位置和角度是否会碰撞
	 * @param NPC 要检查的NPC
	 * @param NPCPosition NPC位置
	 * @param CameraYawAngle 摄像机角度（弧度）
	 * @return 如果摄像机将发生碰撞则返回true
	 */
	bool CheckCameraCollision(ANPC* NPC, const FVector& NPCPosition, float CameraYawAngle) const;

	/**
	 * 从目标节点索引回溯重建完整路径
	 * @param AllStates 所有节点的数组
	 * @param GoalIndex 目标节点索引
	 * @param OutPath 输出的路径数组（从起点到终点）
	 */
	void ReconstructPath(const TArray<FNPCNavigationState>& AllStates, int32 GoalIndex, TArray<FNPCNavigationState>& OutPath);

	/**
	 * 处理寻路任务队列（每帧调用）
	 */
	void ProcessPathfindingQueue();

	/**
	 * 执行单个寻路任务的一个时间分片(最多20ms)
	 * @param Task 要执行的任务(非const,因为需要修改状态)
	 * @return 任务是否完成
	 */
	bool ExecutePathfindingTaskSlice(FPathfindingTask& Task);

	/** ==================== 路径跟踪数据成员 ==================== */

	/** 存储所有已注册NPC的路径数据 */
	UPROPERTY()
	TArray<FNPCPathData> RegisteredNPCPaths;

	/** 按NPC指针查找的映射表（O(1)查找） */
	UPROPERTY()
	TMap<TWeakObjectPtr<ANPC>, int32> NPCPathIndexMap;

	/** 读写锁，保护多线程访问 */
	mutable FRWLock PathDataLock;

	/** ==================== 寻路系统数据成员 ==================== */

	/** 寻路任务队列 */
	UPROPERTY()
	TArray<FPathfindingTask> PathfindingTaskQueue;

	/** 当前任务ID计数器 */
	int32 CurrentTaskId;

	/** NavSys访问互斥锁（保护GetRandomReachablePointInRadius等调用） */
	mutable FCriticalSection NavSysLock;

	/** 已注册的NPC列表（用于碰撞检测时忽略） */
	UPROPERTY()
	TArray<TWeakObjectPtr<ANPC>> RegisteredNPCs;

public:
	/** A*算法配置（与NPC相同） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|AStar")
	float GridSize;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|AStar")
	float CameraAngleStep;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|AStar")
	float MaxDistance;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Navigation|AStar")
	float MaxStepHeight;

	/** 摄像机配置 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Settings")
	float CameraBoomPitch;

	/** ==================== 可达性统计系统 ==================== */

	/** 网格可达性统计：Key为网格坐标"X_Y"，Value为连通分量ID（-1表示不可达） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Navigation|Reachability")
	TMap<FString, int32> Reachable;

	/** 并查集：Key为子节点GridKey，Value为父节点GridKey */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Navigation|Reachability")
	TMap<FString, FString> Father;

	/** 网格高度缓存：Key为网格坐标"X_Y"，Value为NavMesh投影高度（Z坐标） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Navigation|Reachability")
	TMap<FString, float> GridHeight;

	/**
	 * 计算可达性统计
	 * 分析NavMesh覆盖的整个区域，检测每个点的胶囊体碰撞状态
	 * @param NavMeshBoundsVolume 导航网格边界体积（外部传入）
	 * @param CheckNPC 用于碰撞检测的NPC
	 */
	UFUNCTION(BlueprintCallable, Category = "Navigation|Reachability")
	void CalculateReachability(ANavMeshBoundsVolume* NavMeshBoundsVolume, ANPC* CheckNPC);

	/**
	 * 检查指定位置是否可达
	 * 如果未计算过，会进行实时检测并更新Reachable缓存
	 * 如果提供了NPC，还会检查是否与NPC位置在同一连通分量
	 * @param Position 要检查的位置
	 * @param NPC 用于碰撞检测和连通分量检查的NPC
	 * @return 是否可达且与NPC在同一连通分量
	 */
	bool IsPositionReachable(const FVector& Position, ANPC* NPC);

	/**
	 * 生成有偏好的终点
	 * 输入NPC，根据Visited统计选择倾向于未访问过的终点
	 * @param NPC 请求NPC
	 * @param CenterPos 搜索中心
	 * @param Radius 搜索半径
	 * @param OutLocation 输出的终点位置
	 * @return 是否成功生成终点
	 */
	UFUNCTION(BlueprintCallable, Category = "NPC Pathfinding Manager")
	bool GenerateBiasedDestination(ANPC* NPC, float Radius, FVector& OutLocation);

	/**
	 * 检查NPC是否已完成对所有NavMesh可达点的探索
	 * 遍历NavMesh中所有可达点，检查该NPC的Visited程度是否都不为0
	 * @param NavMeshBoundsVolume 导航网格边界体积
	 * @param NPC 要检查的NPC
	 * @return 如果所有可达点的Visited程度都不为0，返回true（表示已探索完成）
	 */
	UFUNCTION(BlueprintCallable, Category = "NPC Pathfinding Manager")
	bool IsNPCExplorationComplete(ANavMeshBoundsVolume* NavMeshBoundsVolume, ANPC* NPC);

private:
	/** 将世界位置转换为网格键 */
	FString PositionToGridKey(const FVector& Position) const;

	/** 从网格键获取世界位置（网格中心） */
	FVector GridKeyToPosition(const FString& GridKey) const;

	/** 获取网格高度（优先从缓存读取，避免重复NavMesh查询）*/
	bool GetGridHeight(const FString& GridKey, float& OutHeight);

	/** 并查集：查找根节点（带路径压缩） */
	FString Find(const FString& GridKey);

	/** 并查集：合并两个集合 */
	void Union(const FString& Key1, const FString& Key2);
};
