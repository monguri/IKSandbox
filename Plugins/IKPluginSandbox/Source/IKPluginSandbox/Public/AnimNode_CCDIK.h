// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "AnimNode_CCDIK.generated.h"

USTRUCT(BlueprintInternalUseOnly)
struct IKPLUGINSANDBOX_API FAnimNode_CCDIK : public FAnimNode_SkeletalControlBase
{
	GENERATED_USTRUCT_BODY()

	/** Name of ik start joint. **/
	UPROPERTY(EditAnywhere, Category=IK)
	FBoneReference IKRootJoint;

	/** Name of ik effector joint. **/
	UPROPERTY(EditAnywhere, Category=IK)
	FBoneReference EffectorJoint;

	/** Effector Target Location. Component space. **/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=IK, meta=(PinShownByDefault))
	FVector EffectorTargetLocation;

	/** The number of iteration. **/
	UPROPERTY(EditAnywhere, Category=IK)
	uint32 MaxIteration;

	FAnimNode_CCDIK();

public:
	// FAnimNode_SkeletalControlBase interface
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface

private:
	struct IKJointWorkData
	{
		FCompactPoseBoneIndex BoneIndex;
		FTransform Transform;

		IKJointWorkData() : BoneIndex(INDEX_NONE), Transform(FTransform::Identity) {}
		IKJointWorkData(FCompactPoseBoneIndex _BoneIndex, const FTransform& _Transform) : BoneIndex(_BoneIndex), Transform(_Transform) {}
	};

	TArray<IKJointWorkData> IKJointWorkDatas;

	// FAnimNode_SkeletalControlBase interface
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface
};
