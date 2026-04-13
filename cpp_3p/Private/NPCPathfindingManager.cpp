// Fill out your copyright notice in the Description page of Project Settings.

#include "NPCPathfindingManager.h"
#include "NPC.h"
#include "NavigationSystem.h"
#include "AI/NavigationSystemBase.h"
#include "NavigationPath.h"
#include "Logging/LogMacros.h"
#include "HAL/PlatformTime.h"
#include "NPCPathfindingHelpers.h"
#include "Components/CapsuleComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "DrawDebugHelpers.h"
#include "NavMesh/NavMeshBoundsVolume.h"

// ==================== 构造函数：初始化所有参数 ====================
ANPCPathfindingManager::ANPCPathfindingManager()
{
	// 启用Tick以处理寻路队列
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;

	// 初始化寻路系统状态
	CurrentTaskId = 0;

	// 初始化A*导航参数（与NPC相同）
	GridSize = 50.0f;
	CameraAngleStep = PI / 8.0f;
	MaxDistance = 5000.0f;
	MaxStepHeight = 50.0f;

	// 初始化摄像机参数
	CameraBoomPitch = -15.0f;
}

// ==================== 游戏开始时调用 ====================
void ANPCPathfindingManager::BeginPlay()
{
	Super::BeginPlay();

	PrintDebugLog(FString::Printf(TEXT("NPCPathfindingManager::BeginPlay - Manager已初始化，位置: %s"), *GetActorLocation().ToString()));
}

// ==================== 每帧更新：处理寻路任务队列 ====================
void ANPCPathfindingManager::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// 处理寻路任务队列
	ProcessPathfindingQueue();
}

// ==================== 生成随机可导航位置 ====================
bool ANPCPathfindingManager::GenerateRandomNavigableLocation(ANPC* NPC, float Radius, FVector& OutLocation, int32 MaxRandomAttempts)
{
	// 增强检查：如果提供了NPC指针，先验证其有效性
	if (!NPC)
	{
		PrintDebugLog(TEXT("GenerateRandomNavigableLocation - 警告：NPC指针无效"));
		return false;
	}

	if (!IsValid(NPC) || NPC->IsPendingKillPending())
	{
		PrintDebugLog(TEXT("GenerateRandomNavigableLocation - 警告：提供的NPC指针无效或已销毁"));
		return false;
	}

	// 从NPC获取当前位置作为搜索中心
	FVector CenterPos = NPC->GetActorLocation();

	UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (!NavSys)
	{
		PrintDebugLog(TEXT("NPCPathfindingManager::GenerateRandomNavigableLocation - 错误：NavigationSystem为NULL"));
		return false;
	}

	for (int32 i = 0; i < MaxRandomAttempts; i++)
	{
		// 使用自旋锁保护NavSys访问
		FScopeLock Lock(&NavSysLock);

		// 使用UE内置的智能随机点获取方法（与蓝图Get Random Reachable Point in NavMesh相同）
		FNavLocation RandomNavLocation;
		if (!NavSys->GetRandomReachablePointInRadius(CenterPos, Radius, RandomNavLocation))
		{
			PrintDebugLog(TEXT("随机点获取失败！"));
			continue; // 尝试下一次
		}
		// 锁在FScopeLock析构时自动释放

		// 使用IsPositionReachable进行可达性判断（会自动进行实时检测并更新缓存）
		if (!IsPositionReachable(RandomNavLocation.Location, NPC))
		{
			PrintDebugLog(TEXT("未通过可达性检测"));
			Radius *= 0.8;
			PrintDebugLog(FString::Printf(TEXT("尝试半径：%f"), Radius));
			continue; // 未通过可达性检测，尝试下一次
		}

		// 成功找到有效点
		OutLocation = RandomNavLocation.Location;
		return true;
	}
	PrintDebugLog(TEXT("达到尝试次数上限"));
	return false;
}

// ==================== 打印调试日志 ====================
void ANPCPathfindingManager::PrintDebugLog(const FString& Message) const
{
	UE_LOG(LogTemp, Log, TEXT("%s"), *Message);
	if (GEngine)
	{
		GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan, Message);
	}
}

// ==================== 路径跟踪系统实现 ====================

// ==================== 注册或更新NPC路径 ====================
void ANPCPathfindingManager::RegisterOrUpdateNPCPath(ANPC* NPC, const TArray<FNPCNavigationState>& Path, int32 CurrentIndex)
{
	if (!NPC)
	{
		UE_LOG(LogTemp, Error, TEXT("NPCPathfindingManager - 错误：NPC为NULL"));
		PrintDebugLog(TEXT("NPCPathfindingManager::RegisterOrUpdateNPCPath - 错误：NPC为NULL"));
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("NPCPathfindingManager - 注册/更新NPC[%s] 路径 - 点数: %d, 当前索引: %d"), *NPC->GetName(), Path.Num(), CurrentIndex);

	// 获取写锁（独占访问）
	FRWScopeLock WriteLock(PathDataLock, SLT_Write);

	// 检查是否为空路径（移除NPC）
	if (Path.Num() == 0)
	{
		if (int32* ExistingIndexPtr = NPCPathIndexMap.Find(NPC))
		{
			int32 ExistingIndex = *ExistingIndexPtr;
			if (RegisteredNPCPaths.IsValidIndex(ExistingIndex))
			{
				// 从数组中移除
				RegisteredNPCPaths.RemoveAt(ExistingIndex);

				// 更新映射表中所有受影响的索引
				for (auto& Pair : NPCPathIndexMap)
				{
					if (Pair.Value > ExistingIndex)
					{
						Pair.Value--;
					}
				}

				// 从映射表中移除
				NPCPathIndexMap.Remove(NPC);

				PrintDebugLog(FString::Printf(TEXT("NPCPathfindingManager::RegisterOrUpdateNPCPath - 移除NPC: %s"), *NPC->GetName()));
			}
		}
		return;
	}

	// 检查NPC是否已存在
	if (int32* ExistingIndexPtr = NPCPathIndexMap.Find(NPC))
	{
		// 更新现有路径数据
		int32 ExistingIndex = *ExistingIndexPtr;
		if (RegisteredNPCPaths.IsValidIndex(ExistingIndex))
		{
			FNPCPathData& PathData = RegisteredNPCPaths[ExistingIndex];
			PathData.NPC = NPC;
			PathData.Path = Path;
			PathData.CurrentPathIndex = FMath::Clamp(CurrentIndex, 0, Path.Num() - 1);

			PrintDebugLog(FString::Printf(TEXT("NPCPathfindingManager::RegisterOrUpdateNPCPath - 更新NPC: %s, 路径点数: %d, 当前索引: %d"),
				*NPC->GetName(), Path.Num(), PathData.CurrentPathIndex));
		}
		else
		{
			// 索引无效，从映射表中移除并重新添加
			NPCPathIndexMap.Remove(NPC);
			goto AddNewEntry;
		}
	}
	else
	{
AddNewEntry:
		// 添加新路径数据
		FNPCPathData NewPathData;
		NewPathData.NPC = NPC;
		NewPathData.Path = Path;
		NewPathData.CurrentPathIndex = FMath::Clamp(CurrentIndex, 0, Path.Num() - 1);

		int32 NewIndex = RegisteredNPCPaths.Add(NewPathData);
		NPCPathIndexMap.Add(NPC, NewIndex);

		PrintDebugLog(FString::Printf(TEXT("NPCPathfindingManager::RegisterOrUpdateNPCPath - 注册新NPC: %s, 路径点数: %d, 当前索引: %d"),
			*NPC->GetName(), Path.Num(), NewPathData.CurrentPathIndex));
	}
}

// ==================== 更新NPC路径进度 ====================
bool ANPCPathfindingManager::UpdateNPCPathProgress(ANPC* NPC, int32 NewIndex)
{
	if (!NPC)
	{
		UE_LOG(LogTemp, Error, TEXT("NPCPathfindingManager - 错误：NPC为NULL"));
		PrintDebugLog(TEXT("NPCPathfindingManager::UpdateNPCPathProgress - 错误：NPC为NULL"));
		return false;
	}
	// 获取写锁（独占访问）
	FRWScopeLock WriteLock(PathDataLock, SLT_Write);

	// 查找NPC索引
	if (int32* IndexPtr = NPCPathIndexMap.Find(NPC))
	{
		int32 Index = *IndexPtr;
		if (RegisteredNPCPaths.IsValidIndex(Index))
		{
			FNPCPathData& PathData = RegisteredNPCPaths[Index];

			// 检查是否完成路径（NewIndex >= Path.Num() 表示到达终点）
			if (NewIndex >= PathData.Path.Num())
			{
				// NPC已完成路径，从跟踪中移除
				UE_LOG(LogTemp, Log, TEXT("NPCPathfindingManager - NPC[%s] 完成路径，从跟踪中移除"), *NPC->GetName());
				PrintDebugLog(FString::Printf(TEXT("NPCPathfindingManager::UpdateNPCPathProgress - NPC %s 已完成路径，正在移除"),
					*NPC->GetName()));

				// 从数组中移除
				RegisteredNPCPaths.RemoveAt(Index);

				// 更新映射表中所有受影响的索引
				for (auto& Pair : NPCPathIndexMap)
				{
					if (Pair.Value > Index)
					{
						Pair.Value--;
					}
				}

				// 从映射表中移除
				NPCPathIndexMap.Remove(NPC);

				return true;  // 返回true表示成功处理（虽然已移除）
			}

			// 验证新索引（正常范围）
			if (NewIndex >= 0 && NewIndex < PathData.Path.Num())
			{
				PathData.CurrentPathIndex = NewIndex;
				return true;
			}
			else
			{
				PrintDebugLog(FString::Printf(TEXT("NPCPathfindingManager::UpdateNPCPathProgress - 错误：索引为负数 %d"),
					NewIndex));
				return false;
			}
		}
	}

	PrintDebugLog(FString::Printf(TEXT("NPCPathfindingManager::UpdateNPCPathProgress - NPC未注册: %s"), *NPC->GetName()));
	return false;
}

// ==================== 从跟踪中移除NPC ====================
bool ANPCPathfindingManager::RemoveNPCFromTracking(ANPC* NPC)
{
	if (!NPC)
	{
		UE_LOG(LogTemp, Error, TEXT("NPCPathfindingManager - 错误：NPC为NULL"));
		PrintDebugLog(TEXT("NPCPathfindingManager::RemoveNPCFromTracking - 错误：NPC为NULL"));
		return false;
	}

	UE_LOG(LogTemp, Warning, TEXT("NPCPathfindingManager - 移除NPC[%s] 从跟踪"), *NPC->GetName());

	// 获取写锁（独占访问）
	FRWScopeLock WriteLock(PathDataLock, SLT_Write);

	// 查找NPC索引
	if (int32* ExistingIndexPtr = NPCPathIndexMap.Find(NPC))
	{
		int32 ExistingIndex = *ExistingIndexPtr;
		if (RegisteredNPCPaths.IsValidIndex(ExistingIndex))
		{
			// 从数组中移除
			RegisteredNPCPaths.RemoveAt(ExistingIndex);

			// 更新映射表中所有受影响的索引
			for (auto& Pair : NPCPathIndexMap)
			{
				if (Pair.Value > ExistingIndex)
				{
					Pair.Value--;
				}
			}

			// 从映射表中移除
			NPCPathIndexMap.Remove(NPC);

			PrintDebugLog(FString::Printf(TEXT("NPCPathfindingManager::RemoveNPCFromTracking - 移除NPC: %s"), *NPC->GetName()));
			return true;
		}
		else
		{
			// 索引无效，从映射表中移除
			NPCPathIndexMap.Remove(NPC);
		}
	}

	PrintDebugLog(FString::Printf(TEXT("NPCPathfindingManager::RemoveNPCFromTracking - NPC未注册: %s"), *NPC->GetName()));
	return false;
}

// ==================== 获取所有未到达点 ====================
int32 ANPCPathfindingManager::GetAllUnreachedPoints(TArray<FVector>& OutPoints, ANPC* ExcludeNPC) const
{
	OutPoints.Empty();

	int32 TotalPoints = 0;

	// 1. 添加所有已注册NPC的当前位置（排除ExcludeNPC）
	for (const TWeakObjectPtr<ANPC>& NPCPtr : RegisteredNPCs)
	{
		if (NPCPtr.IsValid() && NPCPtr.Get() != ExcludeNPC)
		{
			OutPoints.Add(NPCPtr->GetActorLocation());
			TotalPoints++;
		}
	}

	// 2. 获取读锁（多个线程可并发读取）
	FRWScopeLock ReadLock(PathDataLock, SLT_ReadOnly);

	// 3. 遍历所有已注册的NPC路径，添加未来要走的点（排除ExcludeNPC）
	for (const FNPCPathData& PathData : RegisteredNPCPaths)
	{
		// 跳过无效NPC或要排除的NPC
		if (!PathData.IsValid() || PathData.NPC.Get() == ExcludeNPC)
			continue;

		// 检查是否已完成路径
		if (PathData.CurrentPathIndex >= PathData.Path.Num())
			continue;

		// 收集从CurrentIndex到末尾的所有点
		for (int32 i = PathData.CurrentPathIndex; i < PathData.Path.Num(); ++i)
		{
			OutPoints.Add(PathData.Path[i].Position);
			TotalPoints++;
		}
	}

	return TotalPoints;
}

// ==================== 获取已注册NPC数量 ====================
int32 ANPCPathfindingManager::GetRegisteredNPCCount() const
{
	// 获取读锁
	FRWScopeLock ReadLock(PathDataLock, SLT_ReadOnly);

	int32 Count = 0;
	for (const FNPCPathData& PathData : RegisteredNPCPaths)
	{
		if (PathData.IsValid())
			Count++;
	}

	return Count;
}

// ==================== 打印已注册NPC信息 ====================
void ANPCPathfindingManager::PrintRegisteredNPCsInfo() const
{
	// 获取读锁
	FRWScopeLock ReadLock(PathDataLock, SLT_ReadOnly);

	FString Info = FString::Printf(TEXT("===== 已注册NPC信息 (总数: %d) ====="), RegisteredNPCPaths.Num());
	PrintDebugLog(Info);

	for (int32 i = 0; i < RegisteredNPCPaths.Num(); ++i)
	{
		const FNPCPathData& PathData = RegisteredNPCPaths[i];
		if (PathData.NPC.IsValid())
		{
			FString NPCInfo = FString::Printf(TEXT("  [%d] NPC: %s, 路径点数: %d, 当前索引: %d, 未到达点: %d"),
				i,
				*PathData.NPC->GetName(),
				PathData.Path.Num(),
				PathData.CurrentPathIndex,
				PathData.GetUnreachedPointCount());
			PrintDebugLog(NPCInfo);
		}
		else
		{
			FString InvalidInfo = FString::Printf(TEXT("  [%d] NPC: 无效, 路径点数: %d"), i, PathData.Path.Num());
			PrintDebugLog(InvalidInfo);
		}
	}

	PrintDebugLog(TEXT("========================================"));
}

// ==================== 串行寻路系统实现 ====================

// ==================== 请求异步寻路 ====================
int32 ANPCPathfindingManager::RequestPathfinding(ANPC* NPC, const FVector& StartPos, float StartAngle, const FVector& GoalPos)
{
	if (!NPC)
	{
		UE_LOG(LogTemp, Error, TEXT("NPCPathfindingManager::RequestPathfinding - NPC为NULL"));
		return -1;
	}

	FPathfindingTask NewTask(NPC, StartPos, StartAngle, GoalPos, ++CurrentTaskId);
	PathfindingTaskQueue.Add(NewTask);

	UE_LOG(LogTemp, Log, TEXT("NPCPathfindingManager::RequestPathfinding - NPC[%s] 任务已加入队列，任务ID: %d，队列长度: %d"),
		*NPC->GetName(), NewTask.TaskId, PathfindingTaskQueue.Num());

	return NewTask.TaskId;
}

// ==================== 处理寻路任务队列（时间分片版本）====================
void ANPCPathfindingManager::ProcessPathfindingQueue()
{
	// 如果队列为空，直接返回
	if (PathfindingTaskQueue.Num() == 0)
		return;

	// 获取队列头部的任务（不移除）
	FPathfindingTask& Task = PathfindingTaskQueue[0];

	// 检查NPC是否仍然有效
	if (!Task.RequestingNPC.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("NPCPathfindingManager::ProcessPathfindingQueue - NPC已失效，跳过任务"));
		PathfindingTaskQueue.RemoveAt(0); // 移除无效任务
		return;
	}

	// 记录开始时间
	double StartTime = FPlatformTime::Seconds();

	// 执行任务的一个时间分片（最多20ms）
	bool bTaskCompleted = ExecutePathfindingTaskSlice(Task);

	// 更新累计时间
	double EndTime = FPlatformTime::Seconds();
	float ElapsedMs = (EndTime - StartTime) * 1000.0f;
	Task.AccumulatedTime += ElapsedMs;

	// 如果任务完成，从队列移除并处理结果
	if (bTaskCompleted)
	{
		// 从队列移除已完成的任务

		// 在游戏线程中应用结果
		if (Task.RequestingNPC.IsValid())
		{
			ANPC* ValidNPC = Task.RequestingNPC.Get();
			if (Task.Status == EPathfindingTaskStatus::Completed && Task.FoundPath.Num() > 0)
			{
				UE_LOG(LogTemp, Log, TEXT("NPCPathfindingManager::ProcessPathfindingQueue - NPC[%s] 寻路成功，路径点: %d，耗时: %.2fms"),
					*ValidNPC->GetName(), Task.FoundPath.Num(), Task.AccumulatedTime);

				// 更新NPC状态
				ValidNPC->bIsPathfindingInProgress = false;

				// 触发Manager的完成事件
				OnPathfindingComplete.Broadcast(ValidNPC, true, Task.FoundPath);

				// 显示成功信息
				if (GEngine)
					GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Green,
						FString::Printf(TEXT("串行寻路: 成功 (用时%.2fms, 路径点%d)"), Task.AccumulatedTime, Task.FoundPath.Num()));
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("NPCPathfindingManager::ProcessPathfindingQueue - NPC[%s] 寻路失败，耗时: %.2fms"),
					*ValidNPC->GetName(), Task.AccumulatedTime);

				// 更新NPC状态
				ValidNPC->bIsPathfindingInProgress = false;

				// 触发Manager的失败事件
				OnPathfindingComplete.Broadcast(ValidNPC, false, Task.FoundPath);

				// 显示失败信息
				if (GEngine)
					GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red,
						FString::Printf(TEXT("串行寻路: 失败 (用时%.2fms)"), Task.AccumulatedTime));
			}
		}

		PathfindingTaskQueue.RemoveAt(0);
	}
}

// ==================== 执行单个寻路任务的一个时间分片 ====================
bool ANPCPathfindingManager::ExecutePathfindingTaskSlice(FPathfindingTask& Task)
{
	// 记录分片开始时间
	double SliceStartTime = FPlatformTime::Seconds();
	const float MaxSliceTime = 0.02; // 20ms = 0.02秒

	// ==================== 准备工作：获取必要资源 ====================
	UWorld* World = GetWorld();
	if (!World)
	{
		UE_LOG(LogTemp, Error, TEXT("ANPCPathfindingManager::ExecutePathfindingTaskSlice - World is NULL!"));
		Task.Status = EPathfindingTaskStatus::Failed;
		return true; // 任务失败,返回true表示完成
	}

	UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
	if (!NavSys)
	{
		UE_LOG(LogTemp, Error, TEXT("ANPCPathfindingManager::ExecutePathfindingTaskSlice - NavigationSystem is NULL!"));
		Task.Status = EPathfindingTaskStatus::Failed;
		return true;
	}

	// 获取请求寻路的NPC指针
	ANPC* NPC = Task.RequestingNPC.IsValid() ? Task.RequestingNPC.Get() : nullptr;

	// ==================== 任务初始化（首次执行）====================
	if (Task.Status == EPathfindingTaskStatus::NotStarted)
	{
		Task.Status = EPathfindingTaskStatus::InProgress;

		// 初始化起点和目标状态（使用整数格子坐标，与ReachabilityMap保持一致）
		FNPCNavigationState StartState;

		// 将起点位置转换为整数格子中心坐标
		int32 StartGridX = FMath::RoundToInt(Task.StartPos.X / GridSize);
		int32 StartGridY = FMath::RoundToInt(Task.StartPos.Y / GridSize);
		StartState.Position = FVector(StartGridX * GridSize, StartGridY * GridSize, Task.StartPos.Z);
		StartState.CameraYawAngle = Task.StartAngle;
		StartState.GCost = 0;
		StartState.ParentIndex = -1;

		// 将终点位置转换为整数格子中心坐标
		int32 GoalGridX = FMath::RoundToInt(Task.GoalPos.X / GridSize);
		int32 GoalGridY = FMath::RoundToInt(Task.GoalPos.Y / GridSize);
		Task.GoalState.Position = FVector(GoalGridX * GridSize, GoalGridY * GridSize, Task.GoalPos.Z);

		// 计算起点的h值和f值
		StartState.HCost = CalculateHeuristic(StartState, Task.GoalState);
		StartState.CalculateFCost();

		// 清空并初始化算法数据结构
		Task.OpenSet.Clear();
		Task.OpenSetLookup.Clear();
		Task.ClosedSet.Clear();
		Task.AllStates.Empty();

		Task.AllStates.Add(StartState);
		Task.OpenSet.Add(0, StartState.FCost);
		Task.OpenSetLookup.Add(StartState, 0);

		Task.IterationCount = 0;

		// 动态获取所有未到达点作为不可访问点（排除自己）
		GetAllUnreachedPoints(Task.InaccessiblePoints, NPC);

		if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Cyan,
			FString::Printf(TEXT("ANPCPathfindingManager::ExecutePathfindingTaskSlice - 开始寻路: 任务ID=%d, 起点=%s, 终点=%s"),
				Task.TaskId, *StartState.Position.ToString(), *Task.GoalState.Position.ToString()));
	}

	// ==================== A*主循环（增量执行）====================
	//while (!Task.OpenSet.IsEmpty())
	//{
	//	// 检查是否超过时间分片
	//	double CurrentTime = FPlatformTime::Seconds();
	//	if ((CurrentTime - SliceStartTime) >= MaxSliceTime)
	//	{
	//		// 时间分片用尽，暂停执行，返回false表示任务未完成
	//		return false;
	//	}

	//	// O(1)时间获取f值最小的节点
	//	int32 CurrentIndex = Task.OpenSet.PopMin();
	//	Task.OpenSetLookup.Remove(Task.AllStates[CurrentIndex]);

	//	const FNPCNavigationState& Current = Task.AllStates[CurrentIndex];

	//	// 检查是否到达目标
	//	if (Current.IsSameStateFinal(Task.GoalState, GridSize * 1.5f))
	//	{
	//		// 重建路径
	//		ReconstructPath(Task.AllStates, CurrentIndex, Task.FoundPath);

	//		UE_LOG(LogTemp, Log, TEXT("ANPCPathfindingManager::ExecutePathfindingTaskSlice - 寻路成功 - 任务ID: %d, 路径点数: %d"),
	//			Task.TaskId, Task.FoundPath.Num());

	//		if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Green,
	//			FString::Printf(TEXT("ANPCPathfindingManager::ExecutePathfindingTaskSlice - 找到路径! 任务ID=%d, 路径点数=%d"),
	//			Task.TaskId, Task.FoundPath.Num()));

	//		Task.Status = EPathfindingTaskStatus::Completed;
	//		return true; // 任务完成
	//	}

	//	// 将当前节点添加到关闭列表
	//	Task.ClosedSet.Add(Current, CurrentIndex);

	//	// 获取当前节点的所有邻居
	//	TArray<FNPCNavigationState> Neighbors;
	//	GetNeighborStates(Current, Neighbors, Task.GoalState, Task.InaccessiblePoints, World, NavSys, NPC);

	//	// 遍历所有邻居节点
	//	for (const FNPCNavigationState& Neighbor : Neighbors)
	//	{
	//		// O(1)时间检查是否在关闭列表中
	//		if (Task.ClosedSet.Contains(Neighbor))
	//			continue;

	//		// 计算经过当前节点到达邻居的新g值
	//		float DistanceCost = FVector::Dist(Current.Position, Neighbor.Position);
	//		float AngleDelta = FMath::FindDeltaAngleRadians(Current.CameraYawAngle, Neighbor.CameraYawAngle);
	//		float NormalizedAngleCost = FMath::Abs(AngleDelta) / PI * GridSize;

	//		float TentativeGCost = Current.GCost + DistanceCost + NormalizedAngleCost;

	//		// O(1)时间检查邻居是否已在开放列表中
	//		int32 ExistingNeighborIndex = Task.OpenSetLookup.Find(Neighbor);

	//		if (ExistingNeighborIndex == -1)
	//		{
	//			// 不在OpenSet中，添加新节点
	//			FNPCNavigationState NewNeighbor = Neighbor;
	//			NewNeighbor.GCost = TentativeGCost;
	//			NewNeighbor.HCost = CalculateHeuristic(NewNeighbor, Task.GoalState);
	//			NewNeighbor.CalculateFCost();
	//			NewNeighbor.ParentIndex = CurrentIndex;

	//			int32 NewIndex = Task.AllStates.Add(NewNeighbor);
	//			Task.OpenSet.Add(NewIndex, NewNeighbor.FCost);
	//			Task.OpenSetLookup.Add(NewNeighbor, NewIndex);
	//		}
	//		else if (TentativeGCost < Task.AllStates[ExistingNeighborIndex].GCost)
	//		{
	//			// 在OpenSet中，但找到了更优路径，更新节点
	//			Task.AllStates[ExistingNeighborIndex].GCost = TentativeGCost;
	//			Task.AllStates[ExistingNeighborIndex].CalculateFCost();
	//			Task.AllStates[ExistingNeighborIndex].ParentIndex = CurrentIndex;

	//			// 更新堆中的F值
	//			Task.OpenSet.Update(ExistingNeighborIndex, Task.AllStates[ExistingNeighborIndex].FCost);
	//		}
	//	}
	//}
	while (!Task.OpenSet.IsEmpty())
	{
		// 检查是否超过时间分片
		double CurrentTime = FPlatformTime::Seconds();
		if ((CurrentTime - SliceStartTime) >= MaxSliceTime)
		{
			return false;
		}

		// O(1)时间获取f值最小的节点索引
		int32 CurrentIndex = Task.OpenSet.PopMin();

		// 【修复点 1】不再使用引用，而是直接使用索引
		// 注意：不要在这里创建 const FNPCNavigationState& Current = Task.AllStates[CurrentIndex];
		// 因为后面的 Add 操作可能使这个引用失效。

		// 获取当前节点的副本用于比较和计算（或者只在需要时通过索引访问）
		// 为了性能，我们可以先取出关键数据，或者确保在 Add 之前不使用引用
		FNPCNavigationState CurrentStateCopy = Task.AllStates[CurrentIndex];

		// 从 Lookup 表中移除
		Task.OpenSetLookup.Remove(CurrentStateCopy);

		// 检查是否到达目标
		if (CurrentStateCopy.IsSameStateFinal(Task.GoalState, GridSize * 1.5f))
		{
			ReconstructPath(Task.AllStates, CurrentIndex, Task.FoundPath);
			// ... 日志 ...
			Task.Status = EPathfindingTaskStatus::Completed;
			return true;
		}

		// 将当前节点添加到关闭列表
		// 注意：这里传入的是副本或者索引，确保 ClosedSet 内部存储方式安全
		Task.ClosedSet.Add(CurrentStateCopy, CurrentIndex);

		// 获取当前节点的所有邻居
		TArray<FNPCNavigationState> Neighbors;
		// 传入副本，避免在 GetNeighborStates 内部意外修改原数据或持有引用
		GetNeighborStates(CurrentStateCopy, Neighbors, Task.GoalState, Task.InaccessiblePoints, World, NavSys, NPC);

		// 遍历所有邻居节点
		for (const FNPCNavigationState& Neighbor : Neighbors)
		{
			if (Task.ClosedSet.Contains(Neighbor))
				continue;

			// 【修复点 2】计算 G Cost 时使用副本 CurrentStateCopy，而不是可能失效的引用
			float DistanceCost = FVector::Dist(CurrentStateCopy.Position, Neighbor.Position);
			float AngleDelta = FMath::FindDeltaAngleRadians(CurrentStateCopy.CameraYawAngle, Neighbor.CameraYawAngle);
			float NormalizedAngleCost = FMath::Abs(AngleDelta) / PI * GridSize;

			float TentativeGCost = CurrentStateCopy.GCost + DistanceCost + NormalizedAngleCost;

			int32 ExistingNeighborIndex = Task.OpenSetLookup.Find(Neighbor);

			if (ExistingNeighborIndex == -1)
			{
				// 不在OpenSet中，添加新节点
				FNPCNavigationState NewNeighbor = Neighbor;
				NewNeighbor.GCost = TentativeGCost;
				NewNeighbor.HCost = CalculateHeuristic(NewNeighbor, Task.GoalState);
				NewNeighbor.CalculateFCost();
				NewNeighbor.ParentIndex = CurrentIndex;

				// 【关键点】这里会发生 Realloc，导致之前任何对 Task.AllStates 的引用失效
				// 但由于我们使用的是 CurrentStateCopy（栈上的副本），所以是安全的！
				int32 NewIndex = Task.AllStates.Add(NewNeighbor);
				Task.OpenSet.Add(NewIndex, NewNeighbor.FCost);
				Task.OpenSetLookup.Add(NewNeighbor, NewIndex);
			}
			else if (TentativeGCost < Task.AllStates[ExistingNeighborIndex].GCost)
			{
				// 在OpenSet中，但找到了更优路径，更新节点
				// 直接通过索引访问数组，这是安全的
				Task.AllStates[ExistingNeighborIndex].GCost = TentativeGCost;
				Task.AllStates[ExistingNeighborIndex].CalculateFCost();
				Task.AllStates[ExistingNeighborIndex].ParentIndex = CurrentIndex;

				// 更新堆中的F值
				Task.OpenSet.Update(ExistingNeighborIndex, Task.AllStates[ExistingNeighborIndex].FCost);
			}
		}
	}
	// ==================== 未找到路径 ====================
	UE_LOG(LogTemp, Error, TEXT("ANPCPathfindingManager::ExecutePathfindingTaskSlice - 开放列表为空 - 任务ID: %d"), Task.TaskId);
	if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Yellow,
		FString::Printf(TEXT("ANPCPathfindingManager::ExecutePathfindingTaskSlice - 开放列表为空 - 任务ID=%d"), Task.TaskId));

	Task.Status = EPathfindingTaskStatus::Failed;
	return true; // 任务失败,返回true表示完成
}

// ==================== 获取邻居节点（从NPC迁移并修改）====================
void ANPCPathfindingManager::GetNeighborStates(const FNPCNavigationState& CurrentState, TArray<FNPCNavigationState>& OutNeighbors, const FNPCNavigationState& GoalState, const TArray<FVector>& InaccessiblePoints, UWorld* World, UNavigationSystemV1* NavSys, ANPC* NPC)
{
	OutNeighbors.Empty();

	// 8个移动方向（仅在XY平面）
	FVector2D Directions2D[] = {
		FVector2D(1, 0), FVector2D(-1, 0),
		FVector2D(0, 1), FVector2D(0, -1),
		FVector2D(1, 1), FVector2D(-1, -1),
		FVector2D(1, -1), FVector2D(-1, 1)
	};

	// 生成移动邻居
	for (const FVector2D& Dir2D : Directions2D)
	{
		FNPCNavigationState Neighbor;

		// 获取当前状态的整数格子坐标
		int32 CurrentGridX = FMath::RoundToInt(CurrentState.Position.X / GridSize);
		int32 CurrentGridY = FMath::RoundToInt(CurrentState.Position.Y / GridSize);

		// 计算邻居的整数格子坐标（与ReachabilityMap保持一致）
		int32 NeighborGridX = CurrentGridX + FMath::RoundToInt(Dir2D.X);
		int32 NeighborGridY = CurrentGridY + FMath::RoundToInt(Dir2D.Y);

		// 将格子坐标转换为世界坐标（格子中心）
		FVector2D NewPos2D = FVector2D(NeighborGridX * GridSize, NeighborGridY * GridSize);

		// 检查XY距离是否超过最大搜索范围
		if (FVector2D::Distance(NewPos2D, FVector2D(GoalState.Position.X, GoalState.Position.Y)) > MaxDistance)
			continue;

		// 创建网格键并从缓存获取高度
		FString NeighborGridKey = FString::Printf(TEXT("%d_%d"), NeighborGridX, NeighborGridY);
		float GridZ = 0.0f;
		if (!GetGridHeight(NeighborGridKey, GridZ))
			continue;  // 投影失败，跳过此邻居

		// 使用整数格子坐标对应的XY位置，配合缓存中的Z高度
		Neighbor.Position = FVector(NewPos2D.X, NewPos2D.Y, GridZ);
		Neighbor.CameraYawAngle = CurrentState.CameraYawAngle;

		// 检查高度差
		float HeightDiff = FMath::Abs(Neighbor.Position.Z - CurrentState.Position.Z);
		if (HeightDiff > MaxStepHeight)
		{
			continue;
		}

		// 检查是否在预设的不可达点半径内
		if (InaccessiblePoints.Num() > 0)
		{
			bool bIsInInaccessibleRadius = false;
			for (const FVector& InaccessiblePoint : InaccessiblePoints)
			{
				float Distance = FVector::Dist2D(Neighbor.Position, InaccessiblePoint);
				if (Distance < GridSize * 2)
				{
					bIsInInaccessibleRadius = true;
					break;
				}
			}
			if (bIsInInaccessibleRadius)
			{
				continue;
			}
		}

		// 如果有NPC引用，进行碰撞检测
		if (NPC)
		{
			// 检查胶囊体碰撞
			if (!IsPositionReachable(Neighbor.Position, NPC))
			{
				continue;
			}

			// 检查摄像机碰撞
			if (CheckCameraCollision(NPC, Neighbor.Position, Neighbor.CameraYawAngle))
			{
				continue;
			}
		}

		OutNeighbors.Add(Neighbor);
	}

	// 生成原地旋转邻居
	for (int32 AngleStep = -1; AngleStep <= 1; AngleStep++)
	{
		if (AngleStep == 0)
			continue;

		FNPCNavigationState Neighbor;
		Neighbor.Position = CurrentState.Position;
		Neighbor.CameraYawAngle = CurrentState.CameraYawAngle + AngleStep * CameraAngleStep;

		// 角度规范化到 [0, 2π) 范围
		Neighbor.CameraYawAngle = FMath::Fmod(Neighbor.CameraYawAngle, 2.0f * PI);
		if (Neighbor.CameraYawAngle < 0)
			Neighbor.CameraYawAngle += 2.0f * PI;

		// 如果有NPC引用，检查摄像机碰撞
		if (NPC)
		{
			if (CheckCameraCollision(NPC, Neighbor.Position, Neighbor.CameraYawAngle))
			{
				continue;
			}
		}

		OutNeighbors.Add(Neighbor);
	}
}

// ==================== 启发式函数 ====================
float ANPCPathfindingManager::CalculateHeuristic(const FNPCNavigationState& From, const FNPCNavigationState& To) const
{
	FVector2D FromPos2D(From.Position.X, From.Position.Y);
	FVector2D ToPos2D(To.Position.X, To.Position.Y);
	return FVector2D::Distance(FromPos2D, ToPos2D);
}

// ==================== 注册NPC列表 ====================
void ANPCPathfindingManager::RegisterNPCs(const TArray<ANPC*>& NPCs)
{
	RegisteredNPCs.Empty();
	for (ANPC* NPC : NPCs)
	{
		if (NPC)
		{
			RegisteredNPCs.Add(NPC);
		}
	}
}

// ==================== 检查胶囊体碰撞 ====================
bool ANPCPathfindingManager::CheckCapsuleCollision(ANPC* NPC, const FVector& Position) const
{
	if (!NPC || !GetWorld())
		return false;

	float CapsuleRadius = NPC->GetCapsuleComponent()->GetScaledCapsuleRadius() + 10.0f;
	float CapsuleHalfHeight = NPC->GetCapsuleComponent()->GetScaledCapsuleHalfHeight();

	FVector CapsuleCenter = FVector(Position.X, Position.Y, 10.0f + Position.Z + CapsuleHalfHeight);

	FCollisionShape CapsuleShape = FCollisionShape::MakeCapsule(CapsuleRadius, CapsuleHalfHeight);

	FCollisionQueryParams Params;
	Params.AddIgnoredActor(NPC);

	// 忽略所有已注册的NPC
	for (const TWeakObjectPtr<ANPC>& RegisteredNPC : RegisteredNPCs)
	{
		if (RegisteredNPC.IsValid())
		{
			Params.AddIgnoredActor(RegisteredNPC.Get());
		}
	}

	bool bHasOverlap = false;

	TArray<ECollisionChannel> Channels = {
		ECollisionChannel::ECC_Pawn,
		// ECollisionChannel::ECC_WorldStatic,
		// ECollisionChannel::ECC_WorldDynamic,
	};

	for (const ECollisionChannel& Channel : Channels)
	{
		bHasOverlap = GetWorld()->OverlapAnyTestByChannel(
			CapsuleCenter,
			FQuat::Identity,
			Channel,
			CapsuleShape,
			Params
		);

		if (bHasOverlap)
		{
			return false;
		}
	}

	return true;
}

// ==================== 检查摄像机碰撞 ====================
bool ANPCPathfindingManager::CheckCameraCollision(ANPC* NPC, const FVector& NPCPosition, float CameraYawAngle) const
{
	if (!NPC)
		return false;

	USpringArmComponent* CameraBoom = NPC->GetCameraBoom();
	if (!CameraBoom)
		return false;

	// 计算摄像机的世界位置
	FRotator CameraRot(CameraBoomPitch + 180.0f, FMath::RadiansToDegrees(CameraYawAngle) + 180.0f, 0);
	FVector CameraOffset = CameraRot.RotateVector(FVector(CameraBoom->TargetArmLength, 0, 0));
	FVector CameraPos = NPCPosition + CameraOffset;

	float CameraRadius = 20.0f;
	FCollisionQueryParams Params;
	Params.AddIgnoredActor(NPC);

	// 忽略所有已注册的NPC
	for (const TWeakObjectPtr<ANPC>& RegisteredNPC : RegisteredNPCs)
	{
		if (RegisteredNPC.IsValid())
		{
			Params.AddIgnoredActor(RegisteredNPC.Get());
		}
	}

	bool bHasOverlap = GetWorld()->OverlapAnyTestByChannel(
		CameraPos,
		FQuat::Identity,
		ECollisionChannel::ECC_Camera,
		FCollisionShape::MakeSphere(CameraRadius),
		Params
	);

	FHitResult HitResult;
	bool bHit = GetWorld()->LineTraceSingleByChannel(
		HitResult,
		NPCPosition + FVector(0, 0, 50),
		CameraPos,
		ECollisionChannel::ECC_Camera,
		Params
	);

	return (bHit && HitResult.Distance < CameraBoom->TargetArmLength) || bHasOverlap;
}

// ==================== 路径重建 ====================
void ANPCPathfindingManager::ReconstructPath(const TArray<FNPCNavigationState>& AllStates, int32 GoalIndex, TArray<FNPCNavigationState>& OutPath)
{
	OutPath.Empty();

	int32 CurrentIndex = GoalIndex;
	while (CurrentIndex != -1)
	{
		OutPath.Insert(AllStates[CurrentIndex], 0);
		CurrentIndex = AllStates[CurrentIndex].ParentIndex;
	}
}

// ==================== NavMesh碰撞分析功能 ====================

// ==================== 分析NavMesh胶囊体碰撞 ====================
int32 ANPCPathfindingManager::AnalyzeNavMeshCapsuleCollisions(
    const FVector& CenterPos,
    float Radius,
    ANPC* NPC,
    TArray<FVector>& OutCollisionLocations,
    TArray<FVector>& OutSafeLocations)
{
	OutCollisionLocations.Empty();
	OutSafeLocations.Empty();

	// 验证输入参数
	if (!NPC || !GetWorld())
	{
		PrintDebugLog(TEXT("AnalyzeNavMeshCapsuleCollisions - 无效的NPC或World"));
		return 0;
	}

	// 获取导航系统
	UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (!NavSys)
	{
		PrintDebugLog(TEXT("AnalyzeNavMeshCapsuleCollisions - NavigationSystem为NULL"));
		return 0;
	}

	int32 TotalPointsChecked = 0;

	// 获取中心点的网格坐标
	int32 CenterGridX = FMath::RoundToInt(CenterPos.X / GridSize);
	int32 CenterGridY = FMath::RoundToInt(CenterPos.Y / GridSize);

	// 遍历圆形区域内的网格点
	int32 GridSteps = FMath::CeilToInt(Radius / GridSize);
	for (int32 X = -GridSteps; X <= GridSteps; X++)
	{
		for (int32 Y = -GridSteps; Y <= GridSteps; Y++)
		{
			// 计算网格点位置
			FVector2D Offset2D = FVector2D(X, Y) * GridSize;
			float DistanceFromCenter = Offset2D.Length();

			// 跳过超出半径的点
			if (DistanceFromCenter > Radius)
				continue;

			// 计算网格坐标
			int32 GridX = CenterGridX + X;
			int32 GridY = CenterGridY + Y;
			FString GridKey = FString::Printf(TEXT("%d_%d"), GridX, GridY);

			// 从缓存获取高度
			float GridZ = 0.0f;
			if (!GetGridHeight(GridKey, GridZ))
				continue;  // 投影失败，跳过此点

			// 构造位置
			FVector Location(GridX * GridSize, GridY * GridSize, GridZ);

			// 投影成功，进行碰撞检测
			TotalPointsChecked++;

			// 复用现有的CheckCapsuleCollision函数
			bool bHasCollision = !CheckCapsuleCollision(NPC, Location);

			// 根据碰撞结果分类
			if (bHasCollision)
			{
				OutCollisionLocations.Add(Location);
			}
			else
			{
				OutSafeLocations.Add(Location);
			}
		}
	}

	PrintDebugLog(FString::Printf(TEXT("NavMesh碰撞分析完成 - 总检测点数: %d, 碰撞点: %d, 安全点: %d"),
	    TotalPointsChecked, OutCollisionLocations.Num(), OutSafeLocations.Num()));

	return TotalPointsChecked;
}

// ==================== 可视化NavMesh碰撞分析 ====================
void ANPCPathfindingManager::VisualizeNavMeshCollisionAnalysis(
    const TArray<FVector>& CollisionLocations,
    const TArray<FVector>& SafeLocations)
{
	if (!GetWorld())
	{
		PrintDebugLog(TEXT("VisualizeNavMeshCollisionAnalysis - World为NULL"));
		return;
	}

	const float PointSize = 15.0f;  // 调试点大小（cm）
	const bool bPersistentLines = true;  // 永久显示
	const float LifeTime = 0.0f;  // 永久显示

	// 绘制碰撞点（红色）
	for (const FVector& Location : CollisionLocations)
	{
		DrawDebugPoint(
		    GetWorld(),
		    Location,
		    PointSize,
		    FColor::Red,
		    bPersistentLines,
		    LifeTime
		);
	}

	// 绘制安全点（绿色）
	for (const FVector& Location : SafeLocations)
	{
		DrawDebugPoint(
		    GetWorld(),
		    Location,
		    PointSize,
		    FColor::Green,
		    bPersistentLines,
		    LifeTime
		);
	}

	PrintDebugLog(FString::Printf(TEXT("NavMesh碰撞可视化完成 - 碰撞点: %d (红色), 安全点: %d (绿色)"),
	    CollisionLocations.Num(), SafeLocations.Num()));
}

void ANPCPathfindingManager::VisualizeReachableMap(ANavMeshBoundsVolume* NavMeshBoundsVolume, ANPC* CheckNPC)
{
	if (!GetWorld() || !NavMeshBoundsVolume)
	{
		PrintDebugLog(TEXT("VisualizeReachableMap - 无效的 World 或 NavMeshBoundsVolumn"));
		return;
	}
	const float PointSize = 15.0f;  // 调试点大小（cm）
	const bool bPersistentLines = true;  // 永久显示
	const float LifeTime = 0.0f;  // 永久显示

	FVector Origin, Extent;
	NavMeshBoundsVolume->GetActorBounds(false, Origin, Extent);

	FVector MinPoint = Origin - Extent;
	FVector MaxPoint = Origin + Extent;

	int32 MinGridX = FMath::FloorToInt(MinPoint.X / GridSize);
	int32 MaxGridX = FMath::CeilToInt(MaxPoint.X / GridSize);
	int32 MinGridY = FMath::FloorToInt(MinPoint.Y / GridSize);
	int32 MaxGridY = FMath::CeilToInt(MaxPoint.Y / GridSize);

	for (int32 GridX = MinGridX; GridX <= MaxGridX; GridX++)
	{
		for (int32 GridY = MinGridY; GridY <= MaxGridY; GridY++)
		{
			// 创建网格键
			FString GridKey = FString::Printf(TEXT("%d_%d"), GridX, GridY);

			// 从缓存获取高度
			float GridZ = 0.0f;
			bool bHasHeight = GetGridHeight(GridKey, GridZ);

			// 如果没有高度数据，跳过可视化
			if (!bHasHeight)
				continue;

			FVector Location = FVector(GridX * GridSize, GridY * GridSize, GridZ);

			if (IsPositionReachable(Location, CheckNPC))
			{
				DrawDebugPoint(
					GetWorld(),
					Location,
					PointSize,
					FColor::Green,
					bPersistentLines,
					LifeTime
				);
			}
			else
			{
				DrawDebugPoint(
					GetWorld(),
					Location,
					PointSize,
					FColor::Red,
					bPersistentLines,
					LifeTime
				);
			}
		}
	}
}

// ==================== 可达性统计系统实现 ====================

// ==================== 将世界位置转换为网格键 ====================
FString ANPCPathfindingManager::PositionToGridKey(const FVector& Position) const
{
	int32 GridX = FMath::RoundToInt(Position.X / GridSize);
	int32 GridY = FMath::RoundToInt(Position.Y / GridSize);
	return FString::Printf(TEXT("%d_%d"), GridX, GridY);
}

// ==================== 从网格键获取世界位置（网格中心）====================
FVector ANPCPathfindingManager::GridKeyToPosition(const FString& GridKey) const
{
	// 解析网格键 "X_Y"
	FString XStr, YStr;
	if (GridKey.Split(TEXT("_"), &XStr, &YStr))
	{
		int32 GridX = FCString::Atoi(*XStr);
		int32 GridY = FCString::Atoi(*YStr);

		// 网格中心位置
		return FVector(GridX * GridSize, GridY * GridSize, 0.0f);
	}
	return FVector::ZeroVector;
}

// ==================== 获取网格高度（优先从缓存读取）====================
bool ANPCPathfindingManager::GetGridHeight(const FString& GridKey, float& OutHeight)
{
	// 首先检查缓存
	if (GridHeight.Contains(GridKey))
	{
		OutHeight = GridHeight[GridKey];
		return true;
	}

	// 缓存未命中，需要NavMesh查询
	if (!GetWorld())
		return false;

	UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (!NavSys)
		return false;

	// 从网格键解析出格子坐标
	FString XStr, YStr;
	if (!GridKey.Split(TEXT("_"), &XStr, &YStr))
		return false;

	int32 GridX = FCString::Atoi(*XStr);
	int32 GridY = FCString::Atoi(*YStr);

	// 构造测试位置（Z=0，让NavMesh投影返回实际高度）
	FVector TestPos(GridX * GridSize, GridY * GridSize, 0.0f);

	// 使用自旋锁保护NavSys访问
	FNavLocation ProjectedLoc;
	bool bProjectionSuccess = false;
	{
		FScopeLock Lock(&NavSysLock);
		FVector Extent = FVector(GridSize, GridSize, 1000.0f);
		bProjectionSuccess = NavSys->ProjectPointToNavigation(TestPos, ProjectedLoc, Extent);
	}

	if (bProjectionSuccess)
	{
		// 更新缓存
		GridHeight.Add(GridKey, ProjectedLoc.Location.Z);
		OutHeight = ProjectedLoc.Location.Z;
		return true;
	}

	return false;
}

// ==================== 并查集：查找根节点（带路径压缩）====================
FString ANPCPathfindingManager::Find(const FString& GridKey)
{
	if (!Father.Contains(GridKey))
		return GridKey;

	FString& Parent = Father[GridKey];

	// 如果父节点是自己，说明是根节点
	if (Parent == GridKey)
		return GridKey;

	// 路径压缩：直接指向根节点
	Parent = Find(Parent);
	return Parent;
}

// ==================== 并查集：合并两个集合 ====================
void ANPCPathfindingManager::Union(const FString& Key1, const FString& Key2)
{
	FString Root1 = Find(Key1);
	FString Root2 = Find(Key2);

	// 如果已经在同一集合，无需合并
	if (Root1 == Root2)
		return;

	// 合并：将Root1的父节点指向Root2
	Father[Root1] = Root2;
}

// ==================== 计算可达性统计（带连通分量分析）====================
void ANPCPathfindingManager::CalculateReachability(ANavMeshBoundsVolume* NavMeshBoundsVolume, ANPC* CheckNPC)
{
	// 清空之前的可达性数据
	Reachable.Empty();
	Father.Empty();

	// 验证输入参数
	if (!NavMeshBoundsVolume || !CheckNPC || !GetWorld())
	{
		PrintDebugLog(TEXT("CalculateReachability - 无效的NavMeshBoundsVolume、NPC或World"));
		return;
	}

	// 获取NavMesh边界 - 从BrushComponent获取
	FVector Origin, Extent;
	NavMeshBoundsVolume->GetActorBounds(false, Origin, Extent);

	FVector MinPoint = Origin - Extent;
	FVector MaxPoint = Origin + Extent;

	int32 ReachableCount = 0;
	int32 UnreachableCount = 0;

	int32 MinGridX = FMath::FloorToInt(MinPoint.X / GridSize);
	int32 MaxGridX = FMath::CeilToInt(MaxPoint.X / GridSize);
	int32 MinGridY = FMath::FloorToInt(MinPoint.Y / GridSize);
	int32 MaxGridY = FMath::CeilToInt(MaxPoint.Y / GridSize);

	PrintDebugLog(FString::Printf(TEXT("CalculateReachability - 网格范围: X[%d, %d], Y[%d, %d]"),
		MinGridX, MaxGridX, MinGridY, MaxGridY));

	// 获取NPC的Z坐标用于投影
	float NPCZ = CheckNPC->GetActorLocation().Z;

	// 获取导航系统
	UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (!NavSys)
	{
		PrintDebugLog(TEXT("CalculateReachability - NavigationSystem为NULL"));
		return;
	}

	// ==================== 第一阶段：标记可达网格并初始化并查集 ====================
	TArray<FString> ReachableGridKeys;  // 存储所有可达网格的键，用于后续连通分量计算

	for (int32 GridX = MinGridX; GridX <= MaxGridX; GridX++)
	{
		for (int32 GridY = MinGridY; GridY <= MaxGridY; GridY++)
		{
			// 计算网格中心位置，使用NPC的Z坐标
			FVector TestPos = FVector(GridX * GridSize, GridY * GridSize, NPCZ);

			// 使用自旋锁保护NavSys访问
			FNavLocation ProjectedLoc;
			bool bProjectionSuccess = false;
			{
				FScopeLock Lock(&NavSysLock);
				// 使用Extent参数进行投影，Z设置大一点
				FVector ProjExtent = FVector(GridSize / 2, GridSize / 2, 1000.0f);
				bProjectionSuccess = NavSys->ProjectPointToNavigation(TestPos, ProjectedLoc, ProjExtent);
			}

			// 创建网格键
			FString GridKey = FString::Printf(TEXT("%d_%d"), GridX, GridY);

			// 如果投影成功，进行碰撞检测
			if (bProjectionSuccess)
			{
				// 缓存投影高度（无论是否可达，都缓存以便后续使用）
				GridHeight.Add(GridKey, ProjectedLoc.Location.Z);

				// 检查胶囊体碰撞
				bool bHasCollision = !CheckCapsuleCollision(CheckNPC, FVector(TestPos.X, TestPos.Y, ProjectedLoc.Location.Z));

				if (!bHasCollision)
				{
					// 标记为可达（先用0表示，后续会分配连通分量ID）
					Reachable.Add(GridKey, 0);
					// 初始化并查集：父节点指向自己
					Father.Add(GridKey, GridKey);
					ReachableGridKeys.Add(GridKey);
					ReachableCount++;
				}
				else
				{
					// 标记为不可达
					Reachable.Add(GridKey, -1);
					UnreachableCount++;
				}
			}
			else
			{
				// 投影失败，标记为不可达（没有高度数据）
				Reachable.Add(GridKey, -1);
				UnreachableCount++;
			}
		}
	}

	PrintDebugLog(FString::Printf(TEXT("第一阶段完成 - 可达网格: %d, 不可达网格: %d, 高度缓存: %d"),
		ReachableCount, UnreachableCount, GridHeight.Num()));

	// ==================== 第二阶段：计算连通分量（使用并查集）====================
	int32 UnionCount = 0;

	for (const FString& GridKey : ReachableGridKeys)
	{
		// 从缓存获取当前网格的高度
		if (!GridHeight.Contains(GridKey))
			continue;

		float CurrentHeight = GridHeight[GridKey];

		// 解析网格坐标
		FString Left, Right;
		if (!GridKey.Split(TEXT("_"), &Left, &Right))
			continue;

		int32 GridX = FCString::Atoi(*Left);
		int32 GridY = FCString::Atoi(*Right);

		// 检查四个相邻网格（上、下、左、右）
		TArray<FString> NeighborKeys = {
			FString::Printf(TEXT("%d_%d"), GridX + 1, GridY),  // 右
			FString::Printf(TEXT("%d_%d"), GridX - 1, GridY),  // 左
			FString::Printf(TEXT("%d_%d"), GridX, GridY + 1),  // 上
			FString::Printf(TEXT("%d_%d"), GridX, GridY - 1)   // 下
		};

		for (const FString& NeighborKey : NeighborKeys)
		{
			// 检查邻居是否可达
			if (!Reachable.Contains(NeighborKey) || Reachable[NeighborKey] == -1)
				continue;

			// 从缓存获取邻居网格的高度
			if (!GridHeight.Contains(NeighborKey))
				continue;

			float NeighborHeight = GridHeight[NeighborKey];

			// 检查高度差是否在允许范围内
			float HeightDiff = FMath::Abs(CurrentHeight - NeighborHeight);
			if (HeightDiff <= MaxStepHeight)
			{
				// 合并两个集合
				Union(GridKey, NeighborKey);
				UnionCount++;
			}
		}
	}

	PrintDebugLog(FString::Printf(TEXT("第二阶段完成 - 执行了 %d 次合并操作"), UnionCount));

	// ==================== 第三阶段：分配连通分量ID ====================
	// 收集所有唯一的根节点
	TSet<FString> UniqueRoots;
	for (const FString& GridKey : ReachableGridKeys)
	{
		FString Root = Find(GridKey);
		UniqueRoots.Add(Root);
	}

	// 为每个根节点分配唯一的连通分量ID
	TMap<FString, int32> RootToComponentId;
	int32 ComponentId = 0;
	for (const FString& Root : UniqueRoots)
	{
		RootToComponentId.Add(Root, ComponentId++);
	}

	// 更新所有可达网格的连通分量ID
	for (const FString& GridKey : ReachableGridKeys)
	{
		FString Root = Find(GridKey);
		Reachable[GridKey] = RootToComponentId[Root];
	}

	PrintDebugLog(FString::Printf(TEXT("连通分量计算完成 - 发现 %d 个连通分量"), ComponentId));
}

// ==================== 检查位置是否可达（带连通分量检测）====================
bool ANPCPathfindingManager::IsPositionReachable(const FVector& Position, ANPC* NPC)
{
	FString GridKey = PositionToGridKey(Position);

	// 如果已计算过，检查连通分量
	if (Reachable.Contains(GridKey))
	{
		int32 TargetComponentId = Reachable[GridKey];

		// 如果目标不可达
		if (TargetComponentId == -1)
			return false;

		// 如果提供了NPC，检查是否与NPC在同一连通分量
		if (NPC)
		{
			FString NPCGridKey = PositionToGridKey(NPC->GetActorLocation());

			// 如果NPC的位置在Reachable中且可达
			if (Reachable.Contains(NPCGridKey) && Reachable[NPCGridKey] != -1)
			{
				int32 NPCComponentId = Reachable[NPCGridKey];
				// 返回是否在同一连通分量
				return TargetComponentId == NPCComponentId;
			}
		}
		else
		{
			return true;
		}
	}
	
	return false;
}

// ==================== 生成有偏好的终点 ====================
bool ANPCPathfindingManager::GenerateBiasedDestination(ANPC* NPC, float Radius, FVector& OutLocation)
{
	if (!NPC)
	{
		PrintDebugLog(TEXT("GenerateBiasedDestination - NPC为NULL"));
		return false;
	}

	// 获取导航系统
	UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (!NavSys)
	{
		PrintDebugLog(TEXT("GenerateBiasedDestination - NavigationSystem为NULL"));
		return false;
	}

	// 获取NPC当前位置
	FVector CenterPos = NPC->GetActorLocation();

	// 候选点结构
	struct FCandidatePoint
	{
		FVector Position;
		float Score;
	};

	TArray<FCandidatePoint> Candidates;

	// 获取中心点的网格坐标
	int32 CenterGridX = FMath::RoundToInt(CenterPos.X / GridSize);
	int32 CenterGridY = FMath::RoundToInt(CenterPos.Y / GridSize);

	// 遍历圆形区域内的网格点
	int32 GridSteps = FMath::CeilToInt(Radius / GridSize);
	for (int32 X = -GridSteps; X <= GridSteps; X++)
	{
		for (int32 Y = -GridSteps; Y <= GridSteps; Y++)
		{
			// 计算网格点位置
			FVector2D Offset2D = FVector2D(X, Y) * GridSize;
			float DistanceFromCenter = Offset2D.Length();

			// 跳过超出半径的点
			if (DistanceFromCenter > Radius)
				continue;

			// 计算网格坐标
			int32 GridX = CenterGridX + X;
			int32 GridY = CenterGridY + Y;
			FString GridKey = FString::Printf(TEXT("%d_%d"), GridX, GridY);

			// 从缓存获取高度
			float GridZ = 0.0f;
			if (!GetGridHeight(GridKey, GridZ))
				continue;  // 投影失败，跳过此点

			// 构造位置
			FVector Location(GridX * GridSize, GridY * GridSize, GridZ);

			// 使用IsPositionReachable进行可达性判断
			if (!IsPositionReachable(Location, NPC))
			{
				continue; // 跳过不可达的点
			}

			// 计算访问权重
			float VisitedWeight = 0.0f;
			if (NPC->Visited.Contains(GridKey))
			{
				VisitedWeight = NPC->Visited[GridKey];
			}

			// 计算距离
			float Distance = FVector::Dist2D(Location, CenterPos);

			// 计算分数：score = -visited_weight - distance
			// 访问权重越低、距离越近，分数越高
			float Score = -VisitedWeight;

			// 添加候选点
			FCandidatePoint Candidate;
			Candidate.Position = Location;
			Candidate.Score = Score;
			Candidates.Add(Candidate);
		}
	}

	// 如果没有候选点，返回失败
	if (Candidates.Num() == 0)
	{
		PrintDebugLog(TEXT("GenerateBiasedDestination - 没有找到有效的候选点"));
		return false;
	}

	// 按分数排序（降序）
	Candidates.Sort([](const FCandidatePoint& A, const FCandidatePoint& B)
	{
		return A.Score > B.Score;
	});

	// 取前K个候选点
	int32 TopK = FMath::Min(20, Candidates.Num());

	// 从前K个中随机选择一个
	int32 RandomIndex = FMath::RandRange(0, TopK - 1);
	OutLocation = Candidates[RandomIndex].Position;

	PrintDebugLog(FString::Printf(TEXT("GenerateBiasedDestination - 找到终点: %s, 候选点数: %d, 选中索引: %d, 分数: %.2f"),
		*OutLocation.ToString(), Candidates.Num(), RandomIndex, Candidates[RandomIndex].Score));

	return true;
}

// ==================== 检查NPC是否已完成对所有NavMesh可达点的探索 ====================
bool ANPCPathfindingManager::IsNPCExplorationComplete(ANavMeshBoundsVolume* NavMeshBoundsVolume, ANPC* NPC)
{
	// 验证输入参数
	if (!NavMeshBoundsVolume || !NPC || !GetWorld())
	{
		PrintDebugLog(TEXT("IsNPCExplorationComplete - 无效的NavMeshBoundsVolume、NPC或World"));
		return false;
	}

	// 获取NavMesh边界 - 从BrushComponent获取
	FVector Origin, Extent;
	NavMeshBoundsVolume->GetActorBounds(false, Origin, Extent);

	FVector MinPoint = Origin - Extent;
	FVector MaxPoint = Origin + Extent;

	// 计算网格范围
	int32 MinGridX = FMath::FloorToInt(MinPoint.X / GridSize);
	int32 MaxGridX = FMath::CeilToInt(MaxPoint.X / GridSize);
	int32 MinGridY = FMath::FloorToInt(MinPoint.Y / GridSize);
	int32 MaxGridY = FMath::CeilToInt(MaxPoint.Y / GridSize);

	// 统计变量
	int32 TotalReachablePoints = 0;
	int32 UnvisitedPoints = 0;
	int32 VisitedPoints = 0;

	// 遍历所有网格点
	for (int32 GridX = MinGridX; GridX <= MaxGridX; GridX++)
	{
		for (int32 GridY = MinGridY; GridY <= MaxGridY; GridY++)
		{
			FVector TestPos = FVector(GridX * GridSize, GridY * GridSize, 0);
			FString GridKey = FString::Printf(TEXT("%d_%d"), GridX, GridY);

			// 如果可达，检查Visited程度
			if (IsPositionReachable(TestPos, NPC))
			{
				TotalReachablePoints++;

				// 检查NPC的Visited统计
				float VisitedWeight = 0.0f;
				if (NPC->Visited.Contains(GridKey))
				{
					VisitedWeight = NPC->Visited[GridKey];
				}

				// 如果Visited为0，表示该点未被访问过
				if (VisitedWeight == 0.0f)
				{
					UnvisitedPoints++;
					PrintDebugLog(FString::Printf(TEXT("IsNPCExplorationComplete - 发现未访问点: %s"), *GridKey));

					// 可以选择立即返回false，或者继续统计所有未访问点
					// 这里选择立即返回false以提高性能
					return false;
				}
				else
				{
					VisitedPoints++;
				}
			}
		}
	}

	// 所有可达点都已访问过
	PrintDebugLog(FString::Printf(TEXT("IsNPCExplorationComplete - 探索完成！总可达点: %d, 已访问点: %d"),
		TotalReachablePoints, VisitedPoints));

	return true;
}
