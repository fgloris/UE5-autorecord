// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "cpp_3pGameInstance.generated.h"

/**
 * 关卡信息结构体
 * 存储关卡的显示名称和对应的UMap资产路径
 */
USTRUCT(BlueprintType)
struct FLevelInfo
{
	GENERATED_BODY()

	/** 关卡显示名称（用于UI显示等） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level Info")
	FString LevelName;

	/** 关卡UMap资产路径（例如："/Game/Maps/Level1"） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Level Info")
	FString LevelPath;

	/** 默认构造函数 */
	FLevelInfo()
		: LevelName(TEXT(""))
		, LevelPath(TEXT(""))
	{
	}

	/** 便捷构造函数 */
	FLevelInfo(const FString& InName, const FString& InPath)
		: LevelName(InName)
		, LevelPath(InPath)
	{
	}
};

/**
 * 自定义GameInstance类
 *
 * 功能：
 * - 管理关卡列表和当前关卡索引
 * - 提供顺序切换关卡的功能
 * - 查询当前关卡信息
 * - 在关卡切换时保持数据持久化
 */
UCLASS()
class CPP_3P_API Ucpp_3pGameInstance : public UGameInstance
{
	GENERATED_BODY()

public:
	// 构造函数
	Ucpp_3pGameInstance();

protected:
	// 游戏实例初始化时调用
	virtual void Init() override;

public:
	/** ==================== 关卡管理 ==================== */

	/**
	 * 切换到下一个关卡
	 * 如果已经是最后一个关卡，会循环回到第一个关卡
	 * @return 是否成功开始切换流程
	 */
	UFUNCTION(BlueprintCallable, Category = "Level Management")
	bool SwitchToNextLevel();

	/**
	 * 切换到指定索引的关卡
	 * @param LevelIndex 目标关卡索引（0-based）
	 * @return 是否成功开始切换流程
	 */
	UFUNCTION(BlueprintCallable, Category = "Level Management")
	bool SwitchToLevelByIndex(int32 LevelIndex);

	/** ==================== 关卡信息查询 ==================== */

	/**
	 * 获取当前关卡索引（0-based）
	 * @return 当前关卡索引，如果无效则返回-1
	 */
	UFUNCTION(BlueprintCallable, Category = "Level Management")
	int32 GetCurrentLevelIndex() const;

	/**
	 * 获取当前关卡名称
	 * @return 当前关卡显示名称，如果无效则返回空字符串
	 */
	UFUNCTION(BlueprintCallable, Category = "Level Management")
	FString GetCurrentLevelName() const;

	/**
	 * 获取当前关卡路径
	 * @return 当前关卡UMap路径，如果无效则返回空字符串
	 */
	UFUNCTION(BlueprintCallable, Category = "Level Management")
	FString GetCurrentLevelPath() const;

	/**
	 * 获取指定索引的关卡信息
	 * @param LevelIndex 关卡索引
	 * @param OutLevelName 输出：关卡名称
	 * @param OutLevelPath 输出：关卡路径
	 * @return 是否成功获取（索引有效）
	 */
	UFUNCTION(BlueprintCallable, Category = "Level Management")
	bool GetLevelInfoByIndex(int32 LevelIndex, FString& OutLevelName, FString& OutLevelPath) const;

	/**
	 * 获取关卡总数
	 * @return 关卡列表中的关卡数量
	 */
	UFUNCTION(BlueprintCallable, Category = "Level Management")
	int32 GetTotalLevelCount() const;

	/**
	 * 打印所有关卡信息（调试用）
	 */
	UFUNCTION(BlueprintCallable, Category = "Level Management|Debug")
	void PrintAllLevelsInfo() const;

private:
	/**
	 * 初始化关卡列表
	 * 在这里添加所有关卡的配置
	 */
	void InitializeLevelList();

private:
	/** 关卡列表 */
	UPROPERTY()
	TArray<FLevelInfo> LevelList;

	/** 当前关卡索引（0-based） */
	UPROPERTY()
	int32 CurrentLevelIndex;

	/** 是否已初始化关卡列表 */
	UPROPERTY()
	bool bIsLevelListInitialized;
};
