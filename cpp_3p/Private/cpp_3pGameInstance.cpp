// Fill out your copyright notice in the Description page of Project Settings.

#include "cpp_3pGameInstance.h"
#include "Kismet/GameplayStatics.h"
#include "Logging/LogMacros.h"

DEFINE_LOG_CATEGORY_STATIC(LogGameInstance, Log, All);

// ==================== 构造函数 ====================
Ucpp_3pGameInstance::Ucpp_3pGameInstance()
	: CurrentLevelIndex(0)
	, bIsLevelListInitialized(false)
{
}

// ==================== 初始化 ====================
void Ucpp_3pGameInstance::Init()
{
	Super::Init();

	// 初始化关卡列表
	InitializeLevelList();
	UE_LOG(LogGameInstance, Log, TEXT("cpp_3pGameInstance初始化完成，共 %d 个关卡"), LevelList.Num());
}

// ==================== 关卡管理 ====================

bool Ucpp_3pGameInstance::SwitchToNextLevel()
{
	if (!bIsLevelListInitialized || LevelList.Num() == 0)
	{
		UE_LOG(LogGameInstance, Warning, TEXT("SwitchToNextLevel - 关卡列表未初始化或为空"));
		return false;
	}

	// 计算下一个关卡索引（循环）
	CurrentLevelIndex = (CurrentLevelIndex + 1) % LevelList.Num();

	// 获取关卡信息
	const FLevelInfo& NextLevel = LevelList[CurrentLevelIndex];

	UE_LOG(LogGameInstance, Log, TEXT("切换到下一个关卡：[%d] %s -> %s"),
		CurrentLevelIndex, *NextLevel.LevelName, *NextLevel.LevelPath);

	UGameplayStatics::OpenLevel(this, FName(*NextLevel.LevelPath), true);

	return true;
}

bool Ucpp_3pGameInstance::SwitchToLevelByIndex(int32 LevelIndex)
{
	if (!bIsLevelListInitialized || LevelList.Num() == 0)
	{
		UE_LOG(LogGameInstance, Warning, TEXT("SwitchToLevelByIndex - 关卡列表未初始化或为空"));
		return false;
	}

	// 验证索引
	if (LevelIndex < 0 || LevelIndex >= LevelList.Num())
	{
		UE_LOG(LogGameInstance, Warning, TEXT("SwitchToLevelByIndex - 无效的关卡索引：%d（有效范围：0-%d）"),
			LevelIndex, LevelList.Num() - 1);
		return false;
	}

	// 获取关卡信息
	const FLevelInfo& TargetLevel = LevelList[LevelIndex];

	UE_LOG(LogGameInstance, Log, TEXT("切换到指定关卡：[%d] %s -> %s"),
		LevelIndex, *TargetLevel.LevelName, *TargetLevel.LevelPath);

	UGameplayStatics::OpenLevel(this, FName(*TargetLevel.LevelPath), true);

	return true;
}

// ==================== 关卡信息查询 ====================

int32 Ucpp_3pGameInstance::GetCurrentLevelIndex() const
{
	return CurrentLevelIndex;
}

FString Ucpp_3pGameInstance::GetCurrentLevelName() const
{
	if (CurrentLevelIndex >= 0 && CurrentLevelIndex < LevelList.Num())
	{
		return LevelList[CurrentLevelIndex].LevelName;
	}
	return FString(TEXT(""));
}

FString Ucpp_3pGameInstance::GetCurrentLevelPath() const
{
	if (CurrentLevelIndex >= 0 && CurrentLevelIndex < LevelList.Num())
	{
		return LevelList[CurrentLevelIndex].LevelPath;
	}
	return FString(TEXT(""));
}

bool Ucpp_3pGameInstance::GetLevelInfoByIndex(int32 LevelIndex, FString& OutLevelName, FString& OutLevelPath) const
{
	if (!bIsLevelListInitialized || LevelIndex < 0 || LevelIndex >= LevelList.Num())
	{
		OutLevelName = FString(TEXT(""));
		OutLevelPath = FString(TEXT(""));
		return false;
	}

	OutLevelName = LevelList[LevelIndex].LevelName;
	OutLevelPath = LevelList[LevelIndex].LevelPath;
	return true;
}

int32 Ucpp_3pGameInstance::GetTotalLevelCount() const
{
	return LevelList.Num();
}

void Ucpp_3pGameInstance::PrintAllLevelsInfo() const
{
	UE_LOG(LogGameInstance, Log, TEXT("========== 关卡列表信息 =========="));
	UE_LOG(LogGameInstance, Log, TEXT("当前关卡索引：%d"), CurrentLevelIndex);
	UE_LOG(LogGameInstance, Log, TEXT("关卡总数：%d"), LevelList.Num());

	for (int32 i = 0; i < LevelList.Num(); i++)
	{
		FString Prefix = (i == CurrentLevelIndex) ? TEXT("[当前] ") : TEXT("       ");
		UE_LOG(LogGameInstance, Log, TEXT("%s[%d] %s -> %s"),
			*Prefix, i, *LevelList[i].LevelName, *LevelList[i].LevelPath);
	}

	UE_LOG(LogGameInstance, Log, TEXT("=================================="));
}

// ==================== 私有函数 ====================

void Ucpp_3pGameInstance::InitializeLevelList()
{
	// 清空现有列表
	LevelList.Empty();

	// ==================== 在这里添加所有关卡 ====================
	// 注意：路径格式为 "/Game/Maps/MapName"（不需要 .umap 后缀）

	// 示例关卡（请根据你的实际项目修改）
	LevelList.Add(FLevelInfo(TEXT("Level1"), TEXT("/Game/Maps/ThirdPersonMap")));
	LevelList.Add(FLevelInfo(TEXT("Level2"), TEXT("/Game/Maps/Rural_Cabins")));

	// ====================================================

	bIsLevelListInitialized = true;

	UE_LOG(LogGameInstance, Log, TEXT("关卡列表初始化完成，共 %d 个关卡"), LevelList.Num());
}