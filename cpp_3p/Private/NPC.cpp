// 在项目设置的描述页面填写版权声明

#include "NPC.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/StaticMeshComponent.h"  // 静态网格体组件
#include "GameFramework/CharacterMovementComponent.h"
#include "NavigationSystem.h"
#include "NavigationPath.h"
#include "DrawDebugHelpers.h"
#include "Logging/LogMacros.h"  // 添加日志宏支持
#include "Components/SkeletalMeshComponent.h"  // 添加 skeletal mesh 组件支持
#include "Animation/AnimBlueprint.h"  // 添加动画蓝图支持
#include "Async/Async.h"  // 异步任务支持
#include "HAL/PlatformTime.h"  // 性能计时
#include "NPCPathfindingHelpers.h"  // A*优化辅助类
#include "NPCMovementRecorder.h"     // NPC移动记录器
#include "CollisionQueryParams.h"    // 碰撞查询参数
#include "Components/PrimitiveComponent.h"  // 碰撞组件
#include "Engine/OverlapResult.h"  // FOverlapResult定义
#include "Kismet/GameplayStatics.h"  // UGameplayStatics需要
#include "NPCPathfindingManager.h"  // 路径管理器

// ==================== 构造函数：初始化所有组件和参数 ====================
ANPC::ANPC()
{
 	// 设置此角色每帧调用Tick，如果不需要可以关闭以提升性能
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;

	// 构造函数调试日志
	if (GEngine)
		GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Green, TEXT("NPC Constructor called!"));

	// ==================== 初始化A*导航参数 ====================

	GridSize = 50.0f;  // 网格单元大小：100cm
	KernelSigma = 3.0f * GridSize;  // 正态分布核标准差，默认等于GridSize
	KernelInfluenceRadius = 0.0f;  // 将在BeginPlay中计算
	CameraAngleStep = PI / 8.0f;  // 角度步长：22.5度（π/8弧度）
	MaxDistance = 5000.0f;  // 最大搜索距离：5000cm（50米）
	MaxStepHeight = 60.0f;  // 最大允许高度差：50cm

	// ==================== 初始化路径跟随参数 ====================

	CurrentPathIndex = 0;  // 当前路径点索引
	bHasBroadcastDestinationReached = false;  // 初始状态未广播过到达事件
	MovementSpeed = 150.0f;  // 移动速度：150单位/秒
	CameraRotationSpeed = 1.0f;  // 摄像机旋转插值速度（相应调低）

	// ==================== 初始化摄像机设置 ====================

	CameraBoomLength = 200.0f;  // 弹簧臂长度：200cm
	CameraBoomPitch = -15.0f;  // 摄像机俯仰角：15度（向下倾斜）
	bEnableCameraCollision = false;  // 禁用碰撞检测

	// ==================== 初始化调试参数 ====================

	bDebugDrawPath = true;  // 是否绘制路径
	bDebugDrawCameraCollision = true;  // 是否绘制摄像机碰撞检测

	// ==================== 初始化异步寻路系统 ====================

	bIsPathfindingInProgress = false;  // 是否正在寻路

	// 设置碰撞胶囊体大小（半径42cm，高度96cm）
	GetCapsuleComponent()->InitCapsuleSize(42.f, 96.0f);

	// 设置胶囊体碰撞为QueryAndPhysics（修复：从QueryOnly改为QueryAndPhysics以启用实际物理碰撞）
	GetCapsuleComponent()->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

	// ==================== 设置角色网格体 ====================
	if (GetMesh())
	{
		// 设置网格体相对位置和旋转
		GetMesh()->SetRelativeLocation(FVector(0.0f, 0.0f, -96.0f));  // 向下偏移，使脚部对齐
		GetMesh()->SetRelativeRotation(FRotator(0.0f, -90.0f, 0.0f));  // 旋转以匹配UE的向前方向

		// 加载并设置Manny网格体（使用硬引用路径）
		static ConstructorHelpers::FObjectFinder<USkeletalMesh> MannyMeshRef(TEXT("/Game/Characters/Mannequins/Meshes/SKM_Manny"));
		if (MannyMeshRef.Succeeded())
		{
			GetMesh()->SetSkeletalMesh(MannyMeshRef.Object);
		}

		// 加载并设置Manny动画蓝图（使用硬引用路径）
		static ConstructorHelpers::FObjectFinder<UClass> MannyAnimBPRef(TEXT("/Game/Characters/Mannequins/Animations/ABP_Manny.ABP_Manny_C"));
		if (MannyAnimBPRef.Succeeded())
		{
			GetMesh()->SetAnimationMode(EAnimationMode::AnimationBlueprint);
			GetMesh()->SetAnimInstanceClass(MannyAnimBPRef.Object);
		}
	}

	// 配置角色移动组件
	GetCharacterMovement()->bOrientRotationToMovement = true;  // 角色自动朝向移动方向
	GetCharacterMovement()->RotationRate = FRotator(0.0f, 180.0f, 0.0f);  // 旋转速率（快速转向）
	GetCharacterMovement()->JumpZVelocity = 700.f;  // 跳跃速度
	GetCharacterMovement()->AirControl = 0.35f;  // 空中控制力
	GetCharacterMovement()->MaxWalkSpeed = 150.f;  // 最大行走速度（降低到150单位/秒）
	GetCharacterMovement()->MinAnalogWalkSpeed = 20.f;  // 最小模拟行走速度
	GetCharacterMovement()->BrakingDecelerationWalking = 2000.f;  // 行走制动减速度

	// 控制器旋转时不旋转角色，仅影响摄像机
	bUseControllerRotationPitch = false;
	bUseControllerRotationYaw = false;
	bUseControllerRotationRoll = false;

	// ==================== 创建摄像机系统 ====================

	// 创建弹簧臂（碰撞时自动拉近摄像机）
	CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
	CameraBoom->SetupAttachment(RootComponent);  // 附加到根组件

	// 先设置变换模式
	CameraBoom->SetAbsolute(false, true, false);  // 旋转使用世界坐标，位置和缩放使用相对空间

	// 设置弹簧臂的其他参数
	CameraBoom->TargetArmLength = CameraBoomLength;  // 摄像机跟随距离（200cm）
	CameraBoom->bUsePawnControlRotation = false;  // 不根据控制器旋转弹簧臂
	CameraBoom->bDoCollisionTest = false;  // 强制禁用碰撞检测（弹簧臂不会因碰撞缩短）
	CameraBoom->SetWorldRotation(FRotator(CameraBoomPitch, 0, 0));
	// CameraBoom->SetRelativeLocation(FVector(0.0f, 0.0f, 50.0f));  // 连接点提高50cm

	// ==================== 创建摄像机碰撞可视化球体 ====================
	/*
	CameraCollisionVisualizer = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CameraCollisionVisualizer"));
	CameraCollisionVisualizer->SetupAttachment(CameraBoom, USpringArmComponent::SocketName);  // 附加到弹簧臂末端（与摄像机同位置）

	// 加载球体网格体
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMeshRef(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	if (SphereMeshRef.Succeeded())
	{
		CameraCollisionVisualizer->SetStaticMesh(SphereMeshRef.Object);
	}

	// 设置球体大小（碰撞半径20cm，直径40cm）
	CameraCollisionVisualizer->SetRelativeScale3D(FVector(0.4f, 0.4f, 0.4f));  // 默认球体半径100cm，缩放到40cm

	// 设置为仅可视化（不参与碰撞）
	CameraCollisionVisualizer->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	CameraCollisionVisualizer->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);

	// 设置可视性和材质
	CameraCollisionVisualizer->SetHiddenInGame(false);  // 游戏中也显示
	CameraCollisionVisualizer->SetVisibility(true);     // 始终可见
	CameraCollisionVisualizer->SetCastShadow(false);    // 不投射阴影

	// 设置半透明白色材质
	CameraCollisionVisualizer->SetTranslucentSortPriority(-1);  // 优先渲染排序
	// 注意：如果需要特殊颜色效果，可以在蓝图中或运行时创建动态材质
	*/
	// ==================== 创建NPC移动记录器 ====================
	MovementRecorder = CreateDefaultSubobject<UNPCMovementRecorder>(TEXT("MovementRecorder"));
	MovementRecorder->SetComponentTickEnabled(false);  // 不使用自动Tick
}

// ==================== 游戏开始时调用 ====================
void ANPC::BeginPlay()
{
	Super::BeginPlay();

	// 计算核函数影响半径：F(d) < 0.1 时的距离
	// exp(-d²/(2σ²)) < 0.1
	// d > σ * sqrt(-2 * ln(0.1)) ≈ 2.146 * σ
	KernelInfluenceRadius = KernelSigma * FMath::Sqrt(-2.0f * FMath::Loge(0.1f));

	UE_LOG(LogTemp, Log, TEXT("NPC[%s] 核函数参数 - Sigma: %.2fcm, 影响半径: %.2fcm"),
		*GetName(), KernelSigma, KernelInfluenceRadius);

	// 强制禁用弹簧臂碰撞检测（确保不会因碰撞缩短）
	if (CameraBoom)
	{
		CameraBoom->SetRelativeLocation(FVector(0.0f, 0.0f, 50.0f));
		CameraBoom->bDoCollisionTest = false;  // 运行时再次确保碰撞检测被禁用
	}
}

// ==================== 摄像机组件访问器 ====================

float ANPC::GetCameraYaw() const
{
	if (CameraBoom)
	{
		// 获取弹簧臂的偏航角（弧度）
		float Yaw = FMath::DegreesToRadians(CameraBoom->GetRelativeRotation().Yaw);

		// 归一化到 [0, 2π) 范围
		if (Yaw < 0.0f)
		{
			Yaw += 2.0f * PI;
		}

		return Yaw;
	}

	return 0.0f;
}

// ==================== 每帧更新：处理路径跟随逻辑 ====================
void ANPC::ExecuteCurrentPath(float DeltaTime, ANPCPathfindingManager* PathfindingManager)
{
	// 如果没有路径或已到达终点，直接返回
	if (CurrentPath.Num() == 0)
	{
		UE_LOG(LogTemp, Verbose, TEXT("NPC[%s] 路径为空，无法执行"), *GetName());
		return;
	}

	// 检测是否刚刚到达终点（CurrentPathIndex >= CurrentPath.Num() 且尚未广播过）
	if (CurrentPathIndex >= CurrentPath.Num())
	{
		// 如果还没有广播过到达事件，现在广播
		if (!bHasBroadcastDestinationReached)
		{
			bHasBroadcastDestinationReached = true;

			UE_LOG(LogTemp, Log, TEXT("NPC[%s] 到达终点!"), *GetName());

			// 广播到达目标点事件，同时传递完成的路径
			OnDestinationReached.Broadcast(this, CurrentPath);

			// 可选：显示调试信息
			if (GEngine)
				GEngine->AddOnScreenDebugMessage(3, 3.0f, FColor::Green,
					FString::Printf(TEXT("NPC %s 已到达目标点!"), *GetName()));
		}
		UE_LOG(LogTemp, Verbose, TEXT("NPC[%s] 已到达终点"), *GetName());
		return;  // 已到达终点，不再处理
	}

	// 获取当前目标路径点
	const FNPCNavigationState& TargetState = CurrentPath[CurrentPathIndex];

	// 用于 action json 记录
	bool bIsActuallyMoving = false;

	// ==================== 摄像机角度跟随（优先处理） ====================
	if (CameraBoom)
	{
		float CurrentAngle = CameraBoom->GetComponentRotation().Yaw;  // 当前世界角度（度）
		float TargetAngle = FMath::RadiansToDegrees(TargetState.CameraYawAngle) + 180.0f;  // 目标角度（度）

		// 确保角度在 [0, 360) 范围内
		while (CurrentAngle >= 360.0f) CurrentAngle -= 360.0f;
		while (CurrentAngle < 0.0f) CurrentAngle += 360.0f;
		while (TargetAngle >= 360.0f) TargetAngle -= 360.0f;
		while (TargetAngle < 0.0f) TargetAngle += 360.0f;

		// 处理角度环绕：确保沿着最短路径旋转
		float DeltaAngle = TargetAngle - CurrentAngle;
		while (DeltaAngle > 180.0f) DeltaAngle -= 360.0f;
		while (DeltaAngle < -180.0f) DeltaAngle += 360.0f;

		// 计算绝对角度误差
		float AngleError = FMath::Abs(DeltaAngle);

		// 使用恒定角速度旋转（匀速，不会卡顿）
		float MaxRotationPerFrame = CameraRotationSpeed * DeltaTime * 20.0f;  // 每帧最大旋转角度
		float RotationAmount = FMath::Sign(DeltaAngle) * FMath::Min(AngleError, MaxRotationPerFrame);
		float NewAngle = CurrentAngle + RotationAmount;

		// 规范化到 [0, 360) 范围
		while (NewAngle >= 360.0f) NewAngle -= 360.0f;
		while (NewAngle < 0.0f) NewAngle += 360.0f;

		CameraBoom->SetWorldRotation(FRotator(CameraBoomPitch, NewAngle, 0));

		// 重新计算角度误差
		float FinalAngle = CameraBoom->GetComponentRotation().Yaw;
		float FinalDelta = TargetAngle - FinalAngle;
		while (FinalDelta > 180.0f) FinalDelta -= 360.0f;
		while (FinalDelta < -180.0f) FinalDelta += 360.0f;
		float FinalAngleError = FMath::Abs(FinalDelta);

		// 如果摄像机角度还没有对准（误差>5度），不进行移动，直接返回
		if (FinalAngleError > 5.0f)
		{
			static int32 RotateLogCounter = 0;
			if (++RotateLogCounter % 60 == 0)  // 每秒打印一次旋转日志
			{
				UE_LOG(LogTemp, Verbose, TEXT("NPC[%s] 摄像机旋转中 - 角度误差: %.2f°"), *GetName(), FinalAngleError);
			}

			// 记录当前帧状态（只在原地旋转相机，不移动）
			if (MovementRecorder && MovementRecorder->bIsRecording)
			{
				MovementRecorder->RecordFrameFromNPC(this, CurrentPath, CurrentPathIndex, DeltaTime, false);
			}
			return;  // 等待摄像机旋转到位
		}
	}

	// ==================== 位置跟随（摄像机对准后才执行） ====================
	FVector CurrentPos = GetActorLocation();  // 当前位置
	FVector TargetPos = TargetState.Position;  // 目标位置

	// 计算XY平面的距离（忽略Z轴高度差）
	FVector2D CurrentPos2D(CurrentPos.X, CurrentPos.Y);
	FVector2D TargetPos2D(TargetPos.X, TargetPos.Y);
	float Distance = FVector2D::Distance(CurrentPos2D, TargetPos2D);  // XY平面距离

	if (Distance > 10.0f)  // XY距离大于10cm，继续移动
	{
		static int32 MoveLogCounter = 0;
		if (++MoveLogCounter % 60 == 0)  // 每秒打印一次移动日志
		{
			UE_LOG(LogTemp, Verbose, TEXT("NPC[%s] 移动中 - 路径点: %d/%d, 距离: %.2f"), *GetName(), CurrentPathIndex, CurrentPath.Num(), Distance);
		}

		// 计算XY平面的移动方向（忽略Z轴差异）
		FVector MoveDirection = TargetPos - CurrentPos;
		MoveDirection.Z = 0;  // 忽略高度差
		MoveDirection.Normalize();

		// 使用Character的移动系统（这会正确处理物理、碰撞等）
		AddMovementInput(MoveDirection, MovementSpeed * DeltaTime);

		bIsActuallyMoving = true;
	}else{
		// 到达当前路径点，移动到下一个
		int32 OldIndex = CurrentPathIndex;
		CurrentPathIndex++;

		UE_LOG(LogTemp, Log, TEXT("NPC[%s] 到达路径点 %d/%d"), *GetName(), CurrentPathIndex, CurrentPath.Num());

		if (GEngine) GEngine->AddOnScreenDebugMessage(2, 2.0f, FColor::Yellow, FString::Printf(TEXT("Reached waypoint %d/%d"), CurrentPathIndex, CurrentPath.Num()));

		// 通知管理器更新路径进度
		if (PathfindingManager)
		{
			PathfindingManager->UpdateNPCPathProgress(this, CurrentPathIndex);
		}
	}

	// ==================== 记录当前帧状态 ====================
	if (MovementRecorder && MovementRecorder->bIsRecording)
	{
		MovementRecorder->RecordFrameFromNPC(this, CurrentPath, CurrentPathIndex, DeltaTime, bIsActuallyMoving);
	}
}

// ==================== 绑定输入到功能（当前为空） ====================
void ANPC::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}

// ==================== 检查胶囊体碰撞 ====================
// ==================== 公共接口：检查位置是否适合NPC站立 ====================
bool ANPC::IsLocationValidForNPC(const FVector& Position) const
{
	if (!GetWorld())
		return false;

	// 使用相机臂长度作为检测半径（更保守的碰撞检测）
	float CapsuleRadius = GetCapsuleComponent()->GetScaledCapsuleRadius();
	float CapsuleHalfHeight = GetCapsuleComponent()->GetScaledCapsuleHalfHeight();

	// 重要：Position是脚部位置（NavMesh投影点），但胶囊体是中心对齐的
	// 需要将胶囊体向上移动一半高度，这样胶囊体底部才对齐脚部位置
	FVector CapsuleCenter = FVector(Position.X, Position.Y, 50 + CapsuleHalfHeight);  // 向上偏移半高度

	// 在目标位置进行胶囊体重叠检测
	FCollisionShape CapsuleShape = FCollisionShape::MakeCapsule(CapsuleRadius, CapsuleHalfHeight);

	// 检测是否与世界的静态网格体等碰撞
	FCollisionQueryParams Params;
	Params.AddIgnoredActor(this);  // 忽略NPC自身

	// 使用Overlap检测，检查该位置是否有其他物体
	bool bHasOverlap = false;

	// 检测多个碰撞通道（按优先级排序）
	TArray<ECollisionChannel> Channels = {
		ECollisionChannel::ECC_Pawn,           // Pawn通道（最常用）
		//ECollisionChannel::ECC_WorldStatic,    // 静态物体
		//ECollisionChannel::ECC_WorldDynamic,   // 动态物体
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
			return false;  // 有碰撞，不可站立
		}
	}

	return true;  // 无碰撞，可以站立
}

// ==================== 检查摄像机碰撞（球形检测）====================
// ==================== 开始路径跟随 ====================
void ANPC::StartPathFollowing(const TArray<FNPCNavigationState>& Path, ANPCPathfindingManager* PathfindingManager)
{
	CurrentPath = Path;         // 保存路径
	CurrentPathIndex = 0;       // 从第一个路径点开始
	bHasBroadcastDestinationReached = false;  // 重置到达标志，准备检测新的到达

	UE_LOG(LogTemp, Log, TEXT("NPC[%s] 开始路径跟随 - 路径点数: %d"), *GetName(), Path.Num());

	// 在管理器中注册路径
	if (PathfindingManager)
	{
		PathfindingManager->RegisterOrUpdateNPCPath(this, CurrentPath, CurrentPathIndex);
	}
}

// ==================== 停止路径跟随 ====================
void ANPC::StopPathFollowing(ANPCPathfindingManager* PathfindingManager)
{
	UE_LOG(LogTemp, Warning, TEXT("NPC[%s] 停止路径跟随 - 当前索引: %d/%d"), *GetName(), CurrentPathIndex, CurrentPath.Num());

	CurrentPath.Empty();        // 清空路径
	CurrentPathIndex = 0;       // 重置索引
	bHasBroadcastDestinationReached = false;  // 重置到达标志

	// 从管理器中移除路径
	if (PathfindingManager)
	{
		PathfindingManager->RemoveNPCFromTracking(this);
	}
}

// ==================== 检查是否到达目的地 ====================
bool ANPC::HasReachedDestination() const
{
	// 如果当前索引>=路径数量且路径非空，说明已到达终点
	return CurrentPathIndex >= CurrentPath.Num() && CurrentPath.Num() > 0;
}

// ==================== 绘制导航调试信息 ====================
void ANPC::DrawDebugNavigation()
{
	// 路径点少于2个时不绘制
	if (CurrentPath.Num() < 2)
		return;

	// 遍历路径上的所有线段
	for (int32 i = 0; i < CurrentPath.Num() - 1; i++)
	{
		FVector Start = CurrentPath[i].Position;
		FVector End = CurrentPath[i + 1].Position;

		// 根据是否已走过选择颜色
		// 绿色：已走过的路径段
		// 红色：未走过的路径段
		FColor Color = (i < CurrentPathIndex) ? FColor::Green : FColor::Red;

		// 绘制路径线（线宽2.0）
		DrawDebugLine(GetWorld(), Start, End, Color, false, 0.0f, 0, 2.0f);

		// 绘制路径点（大小10cm的点）
		DrawDebugPoint(GetWorld(), Start, 10.0f, Color, false, 0.0f);

		// 绘制摄像机方向（黄色线，长度50cm）
		FRotator CameraRot(0, FMath::RadiansToDegrees(CurrentPath[i].CameraYawAngle), 0);
		FVector CameraEnd = Start + CameraRot.Vector() * 50.0f;
		DrawDebugLine(GetWorld(), Start, CameraEnd, FColor::Yellow, false, 0.0f, 0, 1.0f);
	}

	// 绘制最后一个路径点（红色点）
	DrawDebugPoint(GetWorld(), CurrentPath.Last().Position, 10.0f, FColor::Red, false, 0.0f);
}

// ==================== 异步A*寻路（委托给Manager串行执行）====================
void ANPC::FindPathAStarAsync(const FVector& StartPos, float StartAngle, const FVector& GoalPos, ANPCPathfindingManager* PathfindingManager)
{
	// 如果已经在寻路，拒绝新请求
	if (bIsPathfindingInProgress)
	{
		UE_LOG(LogTemp, Warning, TEXT("NPC[%s] 异步寻路正在进行中，忽略新请求"), *GetName());
		if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Yellow, TEXT("ANPC::FindPathAStarAsync - 寻路正在进行中，忽略新请求"));
		return;
	}

	// 检查Manager是否有效
	if (!PathfindingManager)
	{
		UE_LOG(LogTemp, Error, TEXT("NPC[%s] 异步寻路失败 - PathfindingManager为NULL"), *GetName());
		if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, TEXT("ANPC::FindPathAStarAsync - PathfindingManager为NULL!"));
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("NPC[%s] 开始异步A*寻路（委托给Manager） - 起点: %s, 终点: %s"), *GetName(), *StartPos.ToString(), *GoalPos.ToString());

	// ==================== 主线程：准备数据 ====================
	UWorld* World = GetWorld();
	if (!World)
	{
		if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, TEXT("ANPC::FindPathAStarAsync - World is NULL!"));
		return;
	}

	UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
	if (!NavSys)
	{
		if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, TEXT("ANPC::FindPathAStarAsync - NavigationSystem is NULL!"));
		return;
	}

	// 在主线程投影起点和终点
	FNavLocation StartProjectedLoc;
	if (!NavSys->ProjectPointToNavigation(StartPos, StartProjectedLoc))
	{
		if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, TEXT("ANPC::FindPathAStarAsync - StartPos不在NavMesh上!"));
		return;
	}

	FNavLocation GoalProjectedLoc;
	if (!NavSys->ProjectPointToNavigation(GoalPos, GoalProjectedLoc))
	{
		if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, TEXT("ANPC::FindPathAStarAsync - GoalPos不在NavMesh上!"));
		return;
	}

	// 标记寻路开始
	bIsPathfindingInProgress = true;

	// 委托给Manager的串行寻路系统
	int32 TaskId = PathfindingManager->RequestPathfinding(this, StartProjectedLoc.Location, StartAngle, GoalProjectedLoc.Location);

	if (TaskId == -1)
	{
		UE_LOG(LogTemp, Error, TEXT("NPC[%s] 异步寻路失败 - 无法提交任务到Manager"), *GetName());
		bIsPathfindingInProgress = false;
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("NPC[%s] 已将寻路任务提交给Manager，任务ID: %d"), *GetName(), TaskId);
}

// ==================== NPC录制系统实现 ====================

// ==================== 粗网格访问统计实现 ====================

// ==================== 更新访问统计（使用正态分布核函数） ====================
void ANPC::UpdateVisitedStats(const TArray<FNPCNavigationState>& Path)
{
	// 计算需要遍历的网格范围（基于影响半径）
	int32 GridRadius = FMath::CeilToInt(KernelInfluenceRadius / GridSize);
	float SigmaSquared = 2.0f * KernelSigma * KernelSigma;  // 2σ²，用于核函数

	// 遍历路径上的每个点
	for (const FNPCNavigationState& State : Path)
	{
		// 计算当前点所在的网格坐标
		int32 CenterGridX = FMath::RoundToInt(State.Position.X / GridSize);
		int32 CenterGridY = FMath::RoundToInt(State.Position.Y / GridSize);

		// 遍历周围影响范围内的所有网格
		for (int32 OffsetX = -GridRadius; OffsetX <= GridRadius; OffsetX++)
		{
			for (int32 OffsetY = -GridRadius; OffsetY <= GridRadius; OffsetY++)
			{
				int32 GridX = CenterGridX + OffsetX;
				int32 GridY = CenterGridY + OffsetY;

				// 计算网格中心的世界位置
				FVector GridWorldPos(GridX * GridSize, GridY * GridSize, State.Position.Z);

				// 计算距离（只考虑XY平面）
				float Distance = FVector2D::Distance(
					FVector2D(State.Position.X, State.Position.Y),
					FVector2D(GridWorldPos.X, GridWorldPos.Y)
				);

				// 正态分布核函数：F(d) = exp(-d²/(2σ²))
				float KernelValue = FMath::Exp(-(Distance * Distance) / SigmaSquared);

				// 忽略贡献太小的点（< 0.1）
				if (KernelValue < 0.1f)
					continue;

				// 创建网格键
				FString GridKey = FString::Printf(TEXT("%d_%d"), GridX, GridY);

				// 累加访问权重
				float* CurrentWeight = Visited.Find(GridKey);
				if (CurrentWeight)
				{
					Visited[GridKey] = *CurrentWeight + KernelValue;
				}
				else
				{
					Visited.Add(GridKey, KernelValue);
				}
			}
		}
	}

	UE_LOG(LogTemp, Log, TEXT("NPC[%s] 更新访问统计 - 路径点数: %d, 已访问网格数: %d"),
		*GetName(), Path.Num(), Visited.Num());
}

// ==================== 根据当前位置更新访问统计（简化版）====================
void ANPC::UpdateVisitedStatsAtCurrentPosition()
{
	// 获取NPC当前位置
	FNPCNavigationState CurrentState;
	CurrentState.Position = GetActorLocation();
	TArray<FNPCNavigationState> arr = {CurrentState};
	UpdateVisitedStats(arr);
}

// ==================== 开始记录NPC移动 ====================
void ANPC::StartMovementRecording(bool bClearOldData)
{
	if (MovementRecorder)
	{
		MovementRecorder->StartRecording(bClearOldData);
		if (GEngine)
		{
			GEngine->AddOnScreenDebugMessage(-1, 3.0f, FColor::Green,
				FString::Printf(TEXT("NPC::StartMovementRecording - 开始记录NPC移动")));
		}
	}
	else
	{
		if (GEngine)
		{
			GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red,
				TEXT("NPC::StartMovementRecording - MovementRecorder为空！"));
		}
	}
}

// ==================== 保存NPC移动记录到JSON文件 ====================
bool ANPC::SaveMovementRecording(const FString& SavePath)
{
	if (MovementRecorder)
	{
		bool bSuccess = MovementRecorder->SaveRecordingToFile(SavePath);
		if (GEngine)
		{
			FString Message = bSuccess ?
				FString::Printf(TEXT("NPC::SaveMovementRecording - 保存成功: %s"), *SavePath) :
				TEXT("NPC::SaveMovementRecording - 保存失败");
			GEngine->AddOnScreenDebugMessage(-1, 5.0f, bSuccess ? FColor::Green : FColor::Red, Message);
		}
		return bSuccess;
	}
	return false;
}

// ==================== 可视化周围半径内访问状况 ====================
void ANPC::VisualizeVisitedInRadius(float Radius)
{
	if (!GetWorld())
		return;

	// 获取NPC当前位置
	FVector CenterPos = GetActorLocation();

	// 计算半径范围内的网格范围（GridSize步长）
	int32 GridRadius = FMath::CeilToInt(Radius / GridSize);
	int32 CenterGridX = FMath::RoundToInt(CenterPos.X / GridSize);
	int32 CenterGridY = FMath::RoundToInt(CenterPos.Y / GridSize);

	// 遍历周围所有网格点
	for (int32 OffsetX = -GridRadius; OffsetX <= GridRadius; OffsetX++)
	{
		for (int32 OffsetY = -GridRadius; OffsetY <= GridRadius; OffsetY++)
		{
			int32 GridX = CenterGridX + OffsetX;
			int32 GridY = CenterGridY + OffsetY;

			// 计算网格的世界位置
			FVector GridWorldPos(GridX * GridSize, GridY * GridSize, CenterPos.Z);

			// 检查是否在半径范围内（只检查XY平面）
			float Distance = FVector2D::Distance(FVector2D(CenterPos.X, CenterPos.Y), FVector2D(GridWorldPos.X, GridWorldPos.Y));
			if (Distance > Radius)
				continue;

			// 查询访问权重
			FString GridKey = FString::Printf(TEXT("%d_%d"), GridX, GridY);
			const float* VisitWeightPtr = Visited.Find(GridKey);
			float VisitWeight = VisitWeightPtr ? *VisitWeightPtr : 0.0f;

			// 根据访问权重计算颜色：平滑渐变
			// 0.0 = 绿色 (0,255,0)
			// >=3.0 = 红色 (255,0,0)
			FColor Color;
			if (VisitWeight < 3.0f)
			{
				// 绿色 -> 红色
				float T = FMath::Clamp(VisitWeight, 0.0f, 3.0f);
				Color = FColor(
					FMath::RoundToInt((T / 3.0) * 255),
					255 - FMath::RoundToInt((T / 3.0) * 255),
					0
				);
			}
			else
			{
				Color = FColor::Red;
			}

			// 绘制网格点（访问权重越大点越大）
			float PointSize = FMath::Lerp(10.0f, 20.0f, FMath::Clamp(VisitWeight / 3.0f, 0.0f, 1.0f));
			DrawDebugPoint(GetWorld(), GridWorldPos, PointSize, Color, false, 0.0f, 0);

			// 绘制网格边框（使用DrawDebugBox）
			FVector BoxCenter(GridWorldPos.X, GridWorldPos.Y, CenterPos.Z);
			FVector BoxExtent(GridSize * 0.5f, GridSize * 0.5f, 5.0f);
			DrawDebugBox(GetWorld(), BoxCenter, BoxExtent, FQuat::Identity, Color, false, 0.0f, 0, 1.0f);
		}
	}

	UE_LOG(LogTemp, Log, TEXT("NPC[%s] 可视化访问状况 - 半径: %.2fcm, 网格数: %d, 已访问: %d"),
		*GetName(), Radius, (GridRadius * 2 + 1) * (GridRadius * 2 + 1), Visited.Num());
}