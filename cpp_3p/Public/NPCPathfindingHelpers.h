// NPC Pathfinding Helpers - A*算法优化辅助类

#pragma once

#include "CoreMinimal.h"
#include "NPC.h"
#include "Containers/Map.h"

/**
 * 简单的最小堆实现，用于快速获取F值最小的节点
 * 使用索引而非指针，避免TArray扩容导致的指针失效
 */
class FMinHeap
{
public:
	FMinHeap() = default;

	void Clear()
	{
		Heap.Empty();
	}

	bool IsEmpty() const
	{
		return Heap.Num() == 0;
	}

	void Add(int32 StateIndex, float FCost)
	{
		Heap.Add({StateIndex, FCost});
		HeapifyUp(Heap.Num() - 1);
	}

	int32 PopMin()
	{
		if (Heap.Num() == 0) return -1;

		int32 MinIndex = Heap[0].StateIndex;
		Heap[0] = Heap.Last();
		Heap.Pop();
		if (Heap.Num() > 0)
		{
			HeapifyDown(0);
		}
		return MinIndex;
	}

	bool Update(int32 StateIndex, float NewFCost)
	{
		for (int32 i = 0; i < Heap.Num(); i++)
		{
			if (Heap[i].StateIndex == StateIndex)
			{
				if (NewFCost < Heap[i].FCost)
				{
					Heap[i].FCost = NewFCost;
					HeapifyUp(i);
				}
				return true;
			}
		}
		return false;
	}

	bool Contains(int32 StateIndex) const
	{
		for (const auto& Node : Heap)
		{
			if (Node.StateIndex == StateIndex)
				return true;
		}
		return false;
	}

private:
	struct FHeapNode
	{
		int32 StateIndex;
		float FCost;
	};

	TArray<FHeapNode> Heap;

	void HeapifyUp(int32 Index)
	{
		while (Index > 0)
		{
			int32 Parent = (Index - 1) / 2;
			if (Heap[Index].FCost >= Heap[Parent].FCost) break;

			Swap(Index, Parent);
			Index = Parent;
		}
	}

	void HeapifyDown(int32 Index)
	{
		while (true)
		{
			int32 LeftChild = 2 * Index + 1;
			int32 RightChild = 2 * Index + 2;
			int32 Smallest = Index;

			if (LeftChild < Heap.Num() && Heap[LeftChild].FCost < Heap[Smallest].FCost)
				Smallest = LeftChild;
			if (RightChild < Heap.Num() && Heap[RightChild].FCost < Heap[Smallest].FCost)
				Smallest = RightChild;

			if (Smallest == Index) break;

			Swap(Index, Smallest);
			Index = Smallest;
		}
	}

	void Swap(int32 A, int32 B)
	{
		FHeapNode Temp = Heap[A];
		Heap[A] = Heap[B];
		Heap[B] = Temp;
	}
};

/**
 * 空间哈希实现：将状态映射到网格坐标
 * 将连续空间离散化为固定大小的网格，用于快速查找已访问状态
 */
struct FStateHash
{
	static FIntVector ComputeHash(const FNPCNavigationState& State)
	{
		// 位置哈希：每50cm一个网格单元（只有X和Y维度）
		int32 GridX = FMath::FloorToInt(State.Position.X / 50.0f);
		int32 GridY = FMath::FloorToInt(State.Position.Y / 50.0f);

		// 角度哈希：每22.5度（PI/8）一个角度单元，存储在Z分量中
		int32 AngleBin = FMath::FloorToInt(State.CameraYawAngle / (PI / 8.0f));

		return FIntVector(GridX, GridY, AngleBin);
	}

	FIntVector operator()(const FNPCNavigationState& State) const
	{
		return ComputeHash(State);
	}
};

/**
 * 状态集合管理器：使用空间哈希快速查找已访问状态
 */
class FStateSet
{
public:
	enum ESetType
	{
		OpenSet,
		ClosedSet
	};

	FStateSet() = default;

	void Add(const FNPCNavigationState& State, int32 StateIndex)
	{
		FIntVector Hash = FStateHash::ComputeHash(State);
		StateMap.Add(Hash, StateIndex);
	}

	bool Contains(const FNPCNavigationState& State) const
	{
		FIntVector Hash = FStateHash::ComputeHash(State);
		return StateMap.Contains(Hash);
	}

	int32 Find(const FNPCNavigationState& State) const
	{
		FIntVector Hash = FStateHash::ComputeHash(State);
		if (const int32* IndexPtr = StateMap.Find(Hash))
		{
			return *IndexPtr;
		}
		return -1;
	}

	void Remove(const FNPCNavigationState& State)
	{
		FIntVector Hash = FStateHash::ComputeHash(State);
		StateMap.Remove(Hash);
	}

	void Clear()
	{
		StateMap.Empty();
	}

	int32 Num() const
	{
		return StateMap.Num();
	}

private:
	TMap<FIntVector, int32> StateMap;
};
