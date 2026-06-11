// Copyright Epic Games, Inc. All Rights Reserved.

#include "cpp_1stGameMode.h"
#include "UObject/ConstructorHelpers.h"

Acpp_1stGameMode::Acpp_1stGameMode()
{
	
	// 配置为观察者模式
	DefaultPawnClass = nullptr;  // 没有pawn，玩家可以自由飞行观察
	bStartPlayersAsSpectators = true;  // 玩家以观察者模式开始
}
