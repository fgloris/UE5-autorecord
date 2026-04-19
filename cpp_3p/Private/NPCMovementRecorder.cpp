// Fill out your copyright notice in the Description page of Project Settings.

#include "NPCMovementRecorder.h"
#include "NPC.h"  // 需要完整定义以访问FNPCNavigationState
#include "NPC_new.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "HAL/PlatformFileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/DateTime.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

// ==================== 构造函数 ====================
UNPCMovementRecorder::UNPCMovementRecorder()
{
	// 设置组件默认值
	PrimaryComponentTick.bCanEverTick = false; // 不需要每帧Tick，由NPC手动调用
	bIsRecording = false;
	CurrentFrame = 0;
	ElapsedTime = 0.0f;
	StartTime = 0.0;
}

// ==================== 组件开始播放 ====================
void UNPCMovementRecorder::BeginPlay()
{
	Super::BeginPlay();

	// 重置状态
	bIsRecording = false;
	CurrentFrame = 0;
	ElapsedTime = 0.0f;
}

// ==================== 每帧调用（不使用，由NPC手动调用） ====================
void UNPCMovementRecorder::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	// 不使用自动Tick，由NPC在RecordFrameFromNPC中手动调用记录逻辑
}

// ==================== 开始记录 ====================
void UNPCMovementRecorder::StartRecording(bool bClearOldData)
{
	if (bIsRecording)
	{
		UE_LOG(LogTemp, Warning, TEXT("UNPCMovementRecorder::StartRecording - 已在记录中，忽略请求"));
		return;
	}

	bIsRecording = true;
	StartTime = FPlatformTime::Seconds();
	CurrentFrame = 0;
	ElapsedTime = 0.0f;

	UE_LOG(LogTemp, Log, TEXT("UNPCMovementRecorder::StartRecording - 开始记录"));
}

// ==================== 保存记录到文件 ====================
bool UNPCMovementRecorder::SaveRecordingToFile(const FString& SavePath)
{
	if (RecordingData.GetFrameCount() == 0)
	{
		UE_LOG(LogTemp, Error, TEXT("UNPCMovementRecorder::SaveRecordingToFile - 没有记录数据可保存"));
		return false;
	}

	FString JsonString = BuildNPCRecordingJSON(RecordingData);

	// 写入文件
	if (FFileHelper::SaveStringToFile(JsonString, *SavePath))
	{
		UE_LOG(LogTemp, Log, TEXT("UNPCMovementRecorder::SaveRecordingToFile - 成功保存到: %s"), *SavePath);
		return true;
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("UNPCMovementRecorder::SaveRecordingToFile - 保存失败: %s"), *SavePath);
		return false;
	}
}

// ==================== 获取默认保存路径 ====================
FString UNPCMovementRecorder::GetDefaultSavePath(ANPC* NPC) const
{
	FString ProjectDir = FPaths::ProjectDir();
	FString SaveDir = FPaths::Combine(ProjectDir, TEXT("Saved"), TEXT("Recordings"));

	FString TimeStamp = FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S"));
	FString NPCName = NPC ? NPC->GetActorLabel() : TEXT("UnknownNPC");
	FString FileName = FString::Printf(TEXT("Recording_of_%s_at_%s.json"), *NPCName, *TimeStamp);

	return FPaths::Combine(SaveDir, FileName);
}

// ==================== 事件记录系统 ====================

// ==================== 添加事件 ====================
void UNPCMovementRecorder::AddEvent(const FString& EventName)
{
	// 使用当前Elapsed Time作为事件时间
	float EventTime = ElapsedTime;

	// 创建新事件记录
	FNPCEventRecord NewEvent(EventTime, EventName);

	// 添加到事件数组
	EventRecords.Add(NewEvent);

	UE_LOG(LogTemp, Log, TEXT("UNPCMovementRecorder::AddEvent - 添加事件: %s, 时间: %.2f秒"), *EventName, EventTime);

	// 如果在调试模式下，显示屏幕消息
	if (GEngine)
	{
		FString DebugMessage = FString::Printf(TEXT("Event: %s at %.2fs"), *EventName, EventTime);
		GEngine->AddOnScreenDebugMessage(-1, 2.0f, FColor::Yellow, DebugMessage);
	}
}

// ==================== 从NPC记录当前帧 ====================
void UNPCMovementRecorder::RecordFrameFromNPC(ANPC* NPC, const TArray<FNPCNavigationState>& CurrentPath, int32 CurrentPathIndex, float DeltaTime, bool bIsActuallyMoving)
{
	if (!bIsRecording || !NPC)
	{
		return;
	}

	// 计算当前时间戳
	this->ElapsedTime = FPlatformTime::Seconds() - this->StartTime;

	// 创建新帧数据
	FNPCFrameData FrameData;
	FrameData.Time = ElapsedTime;

	// 获取NPC当前位置和旋转
	FrameData.ActorPos = NPC->GetActorLocation();
	FrameData.ActorRPY = NPC->GetActorRotation();

	// 获取相机位置和旋转（从CameraBoom的endpoint获取）
	if (USpringArmComponent* CameraBoom = NPC->GetCameraBoom())
	{
		FrameData.CameraPos = CameraBoom->GetSocketLocation(USpringArmComponent::SocketName); // 获取CameraBoom的endpoint位置
		FrameData.CameraRot = CameraBoom->GetComponentRotation(); // 获取CameraBoom的世界旋转
	}

	// 初始化控制值
	FrameData.ws = 0;
	FrameData.ad = 0;
	FrameData.ud = 0; // ud始终为0
	FrameData.lr = 0;

	if (const ANPC_new* NPCNew = Cast<ANPC_new>(NPC))
	{
		if (bIsActuallyMoving)
		{
			NPCNew->GetCurrentRecorderControlSignals(FrameData.ws, FrameData.ad, FrameData.lr, FrameData.ud);
		}

		RecordingData.AddFrame(FrameData);
		this->CurrentFrame++;

		if (GEngine && this->CurrentFrame % 60 == 0)
		{
			GEngine->AddOnScreenDebugMessage(-1, 1.0f, FColor::Cyan,
				FString::Printf(TEXT("Recording: Frame=%d, Time=%.2fs, ws=%d, ad=%d, lr=%d, ud=%d"),
					this->CurrentFrame, this->ElapsedTime, FrameData.ws, FrameData.ad, FrameData.lr, FrameData.ud));
		}
		return;
	}

	// 计算移动方向和相机旋转方向
	if (CurrentPath.IsValidIndex(CurrentPathIndex))
	{
		const FNPCNavigationState& TargetState = CurrentPath[CurrentPathIndex];

		// 只有在真的在移动时才计算移动方向（ws/ad）
		// 如果只是在原地旋转相机，ws和ad都保持为0
		if (bIsActuallyMoving)
		{
			// 计算移动方向
			CalculateMovementDirection(FrameData.ActorPos, TargetState.Position, FrameData.ws, FrameData.ad);
		}

		// 计算相机旋转方向（即使不在移动也要记录相机旋转）
		float CurrentYaw = FrameData.CameraRot.Yaw;
		float TargetYaw = FMath::RadiansToDegrees(TargetState.CameraYawAngle) + 180.0f;
		FrameData.lr = CalculateCameraRotationDirection(CurrentYaw, TargetYaw);
	}

	// 添加帧数据到记录
	RecordingData.AddFrame(FrameData);
	this->CurrentFrame++;

	// 调试信息
	if (GEngine && this->CurrentFrame % 60 == 0) // 每60帧打印一次
	{
		GEngine->AddOnScreenDebugMessage(-1, 1.0f, FColor::Cyan,
			FString::Printf(TEXT("Recording: Frame=%d, Time=%.2fs, ws=%d, ad=%d, lr=%d"),
				this->CurrentFrame, this->ElapsedTime, FrameData.ws, FrameData.ad, FrameData.lr));
	}
}

// ==================== 计算移动方向 ====================
void UNPCMovementRecorder::CalculateMovementDirection(const FVector& CurrentPos, const FVector& TargetPos, int32& OutWS, int32& OutAD) const
{
	// 计算方向向量
	FVector Direction = TargetPos - CurrentPos;
	Direction.Z = 0; // 忽略Z轴差异

	// 归一化方向向量
	Direction.Normalize();

	// 计算ws值（x轴方向）
	if (Direction.X > 0.1f)
		OutWS = 1;  // x增加（向前）
	else if (Direction.X < -0.1f)
		OutWS = 2;  // x减少（向后）
	else
		OutWS = 0;  // 无x轴移动

	// 计算ad值（y轴方向）
	if (Direction.Y > 0.1f)
		OutAD = 1;  // y增加（向右）
	else if (Direction.Y < -0.1f)
		OutAD = 2;  // y减少（向左）
	else
		OutAD = 0;  // 无y轴移动
}

// ==================== 计算相机旋转方向 ====================
int32 UNPCMovementRecorder::CalculateCameraRotationDirection(float CurrentYaw, float TargetYaw) const
{
	// 标准化角度到 [0, 360) 范围
	while (CurrentYaw >= 360.0f) CurrentYaw -= 360.0f;
	while (CurrentYaw < 0.0f) CurrentYaw += 360.0f;
	while (TargetYaw >= 360.0f) TargetYaw -= 360.0f;
	while (TargetYaw < 0.0f) TargetYaw += 360.0f;

	// 计算角度差
	float DeltaYaw = TargetYaw - CurrentYaw;

	// 标准化角度差到 [-180, 180]
	while (DeltaYaw > 180.0f) DeltaYaw -= 360.0f;
	while (DeltaYaw < -180.0f) DeltaYaw += 360.0f;

	// 根据角度差确定旋转方向
	if (DeltaYaw > 0.1f)
		return 2;  // 顺时针旋转
	else if (DeltaYaw < -0.1f)
		return 1;  // 逆时针旋转
	else
		return 0;  // 无旋转
}

// ==================== 构建JSON字符串 ====================
FString UNPCMovementRecorder::BuildNPCRecordingJSON(const FNPCRecordingData& Recording) const
{
	TSharedPtr<FJsonObject> RootObject = MakeShareable(new FJsonObject);

	// 添加total_time
	RootObject->SetNumberField(TEXT("total_time"), Recording.TotalTime);

	// 构建events数组
	TArray<TSharedPtr<FJsonValue>> EventArray;
	for (const FNPCEventRecord& Event : EventRecords)
	{
		TSharedPtr<FJsonObject> EventObject = MakeShareable(new FJsonObject);
		EventObject->SetNumberField(TEXT("time"), Event.Time);
		EventObject->SetStringField(TEXT("event_name"), Event.EventName);
		EventArray.Add(MakeShareable(new FJsonValueObject(EventObject)));
	}
	RootObject->SetArrayField(TEXT("events"), EventArray);

	// 构建data数组
	TArray<TSharedPtr<FJsonValue>> DataArray;
	for (const FNPCFrameData& Frame : Recording.Data)
	{
		TSharedPtr<FJsonObject> FrameObject = MakeShareable(new FJsonObject);

		// 添加基本字段
		FrameObject->SetNumberField(TEXT("time"), Frame.Time);
		FrameObject->SetNumberField(TEXT("ws"), Frame.ws);
		FrameObject->SetNumberField(TEXT("ad"), Frame.ad);
		FrameObject->SetNumberField(TEXT("ud"), Frame.ud);
		FrameObject->SetNumberField(TEXT("lr"), Frame.lr);

		// 添加actor_pos对象
		TSharedPtr<FJsonObject> ActorPosObj = MakeShareable(new FJsonObject);
		ActorPosObj->SetNumberField(TEXT("x"), Frame.ActorPos.X);
		ActorPosObj->SetNumberField(TEXT("y"), Frame.ActorPos.Y);
		ActorPosObj->SetNumberField(TEXT("z"), Frame.ActorPos.Z);
		FrameObject->SetObjectField(TEXT("actor_pos"), ActorPosObj);

		// 添加actor_rpy对象
		TSharedPtr<FJsonObject> ActorRPYObj = MakeShareable(new FJsonObject);
		ActorRPYObj->SetNumberField(TEXT("x"), Frame.ActorRPY.Roll);
		ActorRPYObj->SetNumberField(TEXT("y"), Frame.ActorRPY.Pitch);
		ActorRPYObj->SetNumberField(TEXT("z"), Frame.ActorRPY.Yaw);
		FrameObject->SetObjectField(TEXT("actor_rpy"), ActorRPYObj);

		// 添加camera_pos对象
		TSharedPtr<FJsonObject> CameraPosObj = MakeShareable(new FJsonObject);
		CameraPosObj->SetNumberField(TEXT("x"), Frame.CameraPos.X);
		CameraPosObj->SetNumberField(TEXT("y"), Frame.CameraPos.Y);
		CameraPosObj->SetNumberField(TEXT("z"), Frame.CameraPos.Z);
		FrameObject->SetObjectField(TEXT("camera_pos"), CameraPosObj);

		// 添加camera_rot对象
		TSharedPtr<FJsonObject> CameraRotObj = MakeShareable(new FJsonObject);
		CameraRotObj->SetNumberField(TEXT("x"), Frame.CameraRot.Roll);
		CameraRotObj->SetNumberField(TEXT("y"), Frame.CameraRot.Pitch);
		CameraRotObj->SetNumberField(TEXT("z"), Frame.CameraRot.Yaw);
		FrameObject->SetObjectField(TEXT("camera_rot"), CameraRotObj);

		DataArray.Add(MakeShareable(new FJsonValueObject(FrameObject)));
	}

	RootObject->SetArrayField(TEXT("data"), DataArray);

	// 序列化为字符串
	FString OutputString;
	TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&OutputString);
	FJsonSerializer::Serialize(RootObject.ToSharedRef(), Writer);

	return OutputString;
}

// ==================== 确保输出目录存在 ====================
void UNPCMovementRecorder::EnsureOutputDirectoryExists(const FString& FilePath) const
{
	FString DirectoryPath = FPaths::GetPath(FilePath);

	if (!FPaths::DirectoryExists(DirectoryPath))
	{
		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		PlatformFile.CreateDirectoryTree(*DirectoryPath);
		UE_LOG(LogTemp, Log, TEXT("UNPCMovementRecorder::EnsureOutputDirectoryExists - 创建目录: %s"), *DirectoryPath);
	}
}
