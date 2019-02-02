// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "AnimNode_SkeletalControlBaseLocal.h"
#include "AnimNode_JacobianIKLocal.generated.h"

USTRUCT(BlueprintInternalUseOnly)
struct IKPLUGINSANDBOX_API FAnimNode_JacobianIKLocal : public FAnimNode_SkeletalControlBaseLocal
{
	GENERATED_USTRUCT_BODY()

	FAnimNode_JacobianIKLocal();

public:
	// FAnimNode_SkeletalControlBaseLocal interface
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
	virtual void EvaluateSkeletalControl_AnyThread(const FCSPose<FCompactPose>& InPose, FPoseContext& Output) override;
	// End of FAnimNode_SkeletalControlBaseLocal interface
};
