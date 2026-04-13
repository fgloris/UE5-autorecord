// Copyright Epic Games, Inc. All Rights Reserved.

#include "cpp_3pGameMode.h"
#include "GameFramework/SpectatorPawn.h"

Acpp_3pGameMode::Acpp_3pGameMode()
{
	// 不使用任何Pawn，玩家处于上帝视角（Spectator模式）
	DefaultPawnClass = nullptr;  // nullptr表示没有pawn，玩家可以自由飞行观察

	// 配置为观察者模式
	bStartPlayersAsSpectators = true;  // 玩家以观察者模式开始
}
