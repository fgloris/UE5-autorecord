// 在项目设置的描述页面填写版权声明

#include "NPC.h"
#include "Components/CapsuleComponent.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/SpringArmComponent.h"
#include "NavigationSystem.h"
#include "NPCMovementRecorder.h"

ANPC::ANPC()
{
	PrimaryActorTick.bCanEverTick = false;
	PrimaryActorTick.bStartWithTickEnabled = false;

	GridSize = 50.0f;
	MaxStepHeight = 60.0f;
	CurrentPathIndex = 0;

	CameraBoomLength = 200.0f;
	CameraBoomPitch = -15.0f;

	KernelSigma = 3.0f * GridSize;
	KernelInfluenceRadius = 0.0f;

	CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
	CameraBoom->SetupAttachment(RootComponent);
	CameraBoom->SetAbsolute(false, true, false);
	CameraBoom->SetRelativeLocation(FVector(0.0f, 0.0f, 30.0f));
	CameraBoom->TargetArmLength = CameraBoomLength;
	CameraBoom->bUsePawnControlRotation = false;
	CameraBoom->bDoCollisionTest = false;
	CameraBoom->SetWorldRotation(FRotator(CameraBoomPitch, 0, 0));

	MovementRecorder = CreateDefaultSubobject<UNPCMovementRecorder>(TEXT("MovementRecorder"));
	MovementRecorder->SetComponentTickEnabled(false);
}

void ANPC::BeginPlay()
{
	Super::BeginPlay();

	if (CameraBoom)
	{
		CameraBoom->SetRelativeLocation(FVector(0.0f, 0.0f, 30.0f));
		CameraBoom->TargetArmLength = CameraBoomLength;
		CameraBoom->SetWorldRotation(FRotator(CameraBoomPitch, 0.0f, 0.0f));
	}

	KernelInfluenceRadius = KernelSigma * FMath::Sqrt(-2.0f * FMath::Loge(0.1f));
}

void ANPC::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}

bool ANPC::IsLocationValidForNPC(const FVector& Position) const
{
	UWorld* World = GetWorld();
	const UCapsuleComponent* Capsule = GetCapsuleComponent();
	if (!World || !Capsule)
	{
		return false;
	}

	const float CapsuleRadius = Capsule->GetScaledCapsuleRadius();
	const float CapsuleHalfHeight = Capsule->GetScaledCapsuleHalfHeight();
	const FCollisionShape CapsuleShape = FCollisionShape::MakeCapsule(CapsuleRadius - 5, CapsuleHalfHeight - 5);

	FCollisionQueryParams Params(SCENE_QUERY_STAT(NPCCapsuleOverlap), false, this);
	Params.AddIgnoredActor(this);

	return !World->OverlapAnyTestByChannel(
		Position + FVector(0.0f, 0.0f, CapsuleHalfHeight),
		FQuat::Identity,
		ECollisionChannel::ECC_Pawn,
		CapsuleShape,
		Params);
}

void ANPC::UpdateVisitedStats(const TArray<FNPCNavigationState>& Path)
{
	if (Path.Num() == 0 || GridSize <= KINDA_SMALL_NUMBER || KernelSigma <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	const int32 GridRadius = FMath::CeilToInt(KernelInfluenceRadius / GridSize);
	const float SigmaSquared = 2.0f * KernelSigma * KernelSigma;

	for (const FNPCNavigationState& State : Path)
	{
		const int32 CenterGridX = FMath::RoundToInt(State.Position.X / GridSize);
		const int32 CenterGridY = FMath::RoundToInt(State.Position.Y / GridSize);

		for (int32 OffsetX = -GridRadius; OffsetX <= GridRadius; ++OffsetX)
		{
			for (int32 OffsetY = -GridRadius; OffsetY <= GridRadius; ++OffsetY)
			{
				const int32 GridX = CenterGridX + OffsetX;
				const int32 GridY = CenterGridY + OffsetY;
				const FVector GridWorldPos(GridX * GridSize, GridY * GridSize, State.Position.Z);
				const float Dist2D = FVector2D::Distance(FVector2D(GridWorldPos.X, GridWorldPos.Y), FVector2D(State.Position.X, State.Position.Y));

				if (Dist2D > KernelInfluenceRadius)
				{
					continue;
				}

				const float KernelValue = FMath::Exp(-(Dist2D * Dist2D) / SigmaSquared);
				const FString GridKey = FString::Printf(TEXT("%d_%d"), GridX, GridY);
				Visited.FindOrAdd(GridKey) += KernelValue;
			}
		}
	}
}

void ANPC::UpdateVisitedStatsAtCurrentPosition()
{
	FNPCNavigationState State;
	State.Position = GetActorLocation();
	TArray<FNPCNavigationState> SingleStatePath;
	SingleStatePath.Add(State);
	UpdateVisitedStats(SingleStatePath);
}

void ANPC::StartMovementRecording(bool bClearOldData)
{
	if (MovementRecorder)
	{
		MovementRecorder->StartRecording(bClearOldData);
	}
}

bool ANPC::SaveMovementRecording(const FString& SavePath)
{
	if (!MovementRecorder)
	{
		return false;
	}

	const FString FinalPath = SavePath.IsEmpty() ? MovementRecorder->GetDefaultSavePath(this) : SavePath;
	return MovementRecorder->SaveRecordingToFile(FinalPath);
}

void ANPC::VisualizeVisitedInRadius(float Radius)
{
	if (!GetWorld() || GridSize <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	const FVector CenterPos = GetActorLocation();
	const int32 GridRadius = FMath::CeilToInt(Radius / GridSize);
	const int32 CenterGridX = FMath::RoundToInt(CenterPos.X / GridSize);
	const int32 CenterGridY = FMath::RoundToInt(CenterPos.Y / GridSize);

	for (int32 OffsetX = -GridRadius; OffsetX <= GridRadius; ++OffsetX)
	{
		for (int32 OffsetY = -GridRadius; OffsetY <= GridRadius; ++OffsetY)
		{
			const int32 GridX = CenterGridX + OffsetX;
			const int32 GridY = CenterGridY + OffsetY;
			const FVector GridWorldPos(GridX * GridSize, GridY * GridSize, CenterPos.Z + 5.0f);

			if (FVector::Dist2D(GridWorldPos, CenterPos) > Radius)
			{
				continue;
			}

			const FString GridKey = FString::Printf(TEXT("%d_%d"), GridX, GridY);
			const float VisitWeight = Visited.Contains(GridKey) ? Visited[GridKey] : 0.0f;
			const float ColorAlpha = FMath::Clamp(VisitWeight / 3.0f, 0.0f, 1.0f);
			const FColor GridColor = FColor::MakeRedToGreenColorFromScalar(1.0f - ColorAlpha);
			DrawDebugBox(GetWorld(), GridWorldPos, FVector(GridSize * 0.5f, GridSize * 0.5f, 5.0f), GridColor, false, 0.1f, 0, 1.5f);
		}
	}
}
