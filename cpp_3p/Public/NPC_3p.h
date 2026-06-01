#pragma once

#include "CoreMinimal.h"
#include "NPC_new.h"
#include "NPC_3p.generated.h"

/**
 * Compatibility-friendly third-person NPC class.
 *
 * NPC_new is intentionally kept for old Blueprints/assets/checkpoints that still
 * reference the original class name. NPC_3p simply derives from NPC_new so it
 * reuses the exact same third-person random exploration implementation without
 * duplicating enums or logic.
 */
UCLASS()
class CPP_3P_API ANPC_3p : public ANPC_new
{
    GENERATED_BODY()

public:
    ANPC_3p();
};
