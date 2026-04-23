// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "NPCRecordingStructs.h"
#include "NPCMovementRecorder.generated.h"

// 前向声明避免循环依赖
class ANPC;
struct FNPCNavigationState;

/**
 * NPC事件数据结构：记录事件发生的时间和名称
 */
USTRUCT(BlueprintType)
struct FNPCEventRecord
{
	GENERATED_BODY()

	/** 事件发生时间（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Event")
	float Time;

	/** 事件名称 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Event")
	FString EventName;

	/** 构造函数 */
	FNPCEventRecord()
		: Time(0.0f)
		, EventName(TEXT(""))
	{
	}

	/** 参数化构造函数 */
	FNPCEventRecord(float InTime, const FString& InEventName)
		: Time(InTime)
		, EventName(InEventName)
	{
	}
};

/**
 * NPC移动记录器组件
 * 负责记录NPC在自动导航过程中的每一帧状态，并导出为JSON格式
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class CPP_3P_API UNPCMovementRecorder : public UActorComponent
{
	GENERATED_BODY()

public:
	// 构造函数
	UNPCMovementRecorder();

	// ==================== 记录控制接口 ====================

	/**
	 * 开始记录NPC移动
	 * @param bClearOldData 是否清空之前的记录数据，默认true
	 */
	UFUNCTION(BlueprintCallable, Category = "Recording")
	void StartRecording(bool bClearOldData = true);

	// ==================== 数据导出 ====================

	/**
	 * 保存记录到JSON文件
	 * @param SavePath 保存路径
	 * @return 是否保存成功
	 */
	UFUNCTION(BlueprintCallable, Category = "Recording")
	bool SaveRecordingToFile(const FString& SavePath = TEXT(""));

	/**
	 * 获取默认保存路径
	 * @param NPC 用于获取名字
	 * @return 默认的保存文件路径
	 */
	UFUNCTION(BlueprintCallable, Category = "Recording")
	FString GetDefaultSavePath(ANPC* NPC) const;

	/**
	 * Append the skeletal mesh currently bound on the NPC instance to a JSON file.
	 * This is a one-shot helper and does not depend on recorder state.
	 * @param NPC NPC instance, including Blueprint subclasses with mesh assets assigned in Blueprint.
	 * @param SavePath Save path. If empty, Saved/Recordings/NPCMeshRecords.json is used.
	 * @return Whether the record was appended successfully.
	 */
	UFUNCTION(BlueprintCallable, Category = "Recording|Mesh")
	static bool AppendCurrentSkeletalMeshNameToJson(ANPC* NPC, const FString& SaveName = TEXT("skeletal_mesh_records.json"));

	/**
	 * 获取当前记录数据（只读）
	 * @return 当前记录数据的常量引用
	 */
	const FNPCRecordingData& GetRecordingData() const { return RecordingData; }

	// ==================== 状态管理 ====================

	/** 是否正在记录 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Recording")
	bool bIsRecording;

	/** 当前帧数 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Recording")
	int32 CurrentFrame;

	/** 已记录时长（秒） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Recording")
	float ElapsedTime;

	// ==================== 事件记录系统 ====================

	/**
	 * 添加事件记录
	 * @param EventName 事件名称
	 */
	UFUNCTION(BlueprintCallable, Category = "Recording|Event")
	void AddEvent(const FString& EventName);

	/**
	 * 获取所有事件记录
	 * @return 事件记录数组的常量引用
	 */
	const TArray<FNPCEventRecord>& GetEventRecords() const { return EventRecords; }

	/** 事件记录数组（只读） */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Recording|Event")
	TArray<FNPCEventRecord> EventRecords;

	// ==================== 记录接口（供NPC调用） ====================

	/**
	 * 从NPC记录当前帧状态
	 * @param NPC NPC指针
	 * @param CurrentPath 当前路径
	 * @param CurrentPathIndex 当前路径点索引
	 * @param DeltaTime 时间增量
	 * @param bIsActuallyMoving 是否真的在移动（true=移动中，false=原地旋转相机）
	 */
	void RecordFrameFromNPC(ANPC* NPC, const TArray<FNPCNavigationState>& CurrentPath, int32 CurrentPathIndex, float DeltaTime, bool bIsActuallyMoving = true);

protected:
	// 组件开始播放时调用
	virtual void BeginPlay() override;

	// 每帧调用
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	/** 记录数据 */
	FNPCRecordingData RecordingData;

	/** 记录开始时间（用于计算时间戳） */
	double StartTime;

	/** 构建JSON字符串 */
	FString BuildNPCRecordingJSON(const FNPCRecordingData& Recording) const;

	/** 确保输出目录存在 */
	void EnsureOutputDirectoryExists(const FString& FilePath) const;

	/** 计算移动方向ws/ad值 */
	void CalculateMovementDirection(const FVector& CurrentPos, const FVector& TargetPos, int32& OutWs, int32& OutAd) const;

	/** 计算相机旋转方向lr值 */
	int32 CalculateCameraRotationDirection(float CurrentYaw, float TargetYaw) const;
};
