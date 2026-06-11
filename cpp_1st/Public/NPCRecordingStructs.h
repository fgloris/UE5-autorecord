// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NPCRecordingStructs.generated.h"

/**
 * 单帧NPC状态数据结构
 * 用于记录NPC在自动导航过程中的每一帧状态
 */
USTRUCT(BlueprintType)
struct FNPCFrameData
{
	GENERATED_BODY()

	/** 时间戳（秒），从记录开始计算 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Recording")
	float Time;

	/** 移动方向ws值：0=无x轴移动, 1=x增加(向前), 2=x减少(向后) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Recording")
	int32 ws;

	/** 移动方向ad值：0=无y轴移动, 1=y增加(向右), 2=y减少(向左) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Recording")
	int32 ad;

	/** 相机俯仰旋转ud值：始终为0（不支持俯仰旋转） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Recording")
	int32 ud;

	/** 相机偏航旋转lr值：0=无旋转, 1=顺时针旋转, 2=逆时针旋转 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Recording")
	int32 lr;

	/** 角色位置坐标 (x, y, z) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Recording")
	FVector ActorPos;

	/** 角色旋转角度 (Roll, Pitch, Yaw) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Recording")
	FRotator ActorRPY;

	/** 相机位置坐标 (x, y, z) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Recording")
	FVector CameraPos;

	/** 相机旋转角度 (Roll, Pitch, Yaw) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Recording")
	FRotator CameraRot;

	/** 默认构造函数 */
	FNPCFrameData()
		: Time(0.0f)
		, ws(0)
		, ad(0)
		, ud(0)
		, lr(0)
		, ActorPos(FVector::ZeroVector)
		, ActorRPY(FRotator::ZeroRotator)
		, CameraPos(FVector::ZeroVector)
		, CameraRot(FRotator::ZeroRotator)
	{
	}
};

/**
 * 完整的NPC录制数据结构
 * 包含所有帧数据和元信息
 */
USTRUCT(BlueprintType)
struct FNPCRecordingData
{
	GENERATED_BODY()

	/** 总录制时长（秒） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Recording")
	float TotalTime;

	/** 所有帧数据数组 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Recording")
	TArray<FNPCFrameData> Data;

	/** 默认构造函数 */
	FNPCRecordingData()
		: TotalTime(0.0f)
	{
		Data.Empty();
	}

	/** 添加一帧数据 */
	void AddFrame(const FNPCFrameData& FrameData)
	{
		Data.Add(FrameData);
		TotalTime = FrameData.Time; // 更新总时间为最后一帧的时间
	}

	/** 获取帧数 */
	int32 GetFrameCount() const
	{
		return Data.Num();
	}

	/** 清空所有数据 */
	void Clear()
	{
		Data.Empty();
		TotalTime = 0.0f;
	}
};