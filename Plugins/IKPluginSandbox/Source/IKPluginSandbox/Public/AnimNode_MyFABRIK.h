// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "AnimNode_MyFABRIK.generated.h"

USTRUCT(BlueprintInternalUseOnly)
struct IKPLUGINSANDBOX_API FAnimNode_MyFABRIK : public FAnimNode_SkeletalControlBase
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

	/** Tolerance for IK Target and IK joint length, in unreal units. */
	UPROPERTY(EditAnywhere, Category=IK)
	float Precision;

	FAnimNode_MyFABRIK();

public:
	// FAnimNode_SkeletalControlBase interface
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface

private:
	//TODO: CCD-IK‚Æ“¯‚¶‚È‚Ì‚Å‹¤’Ê‰»‚µ‚½‚¢
	struct IKJointWorkData
	{
		FCompactPoseBoneIndex BoneIndex;
		FTransform Transform;
		float OriginalJointLengthToChild;

		IKJointWorkData() : BoneIndex(INDEX_NONE), Transform(FTransform::Identity), OriginalJointLengthToChild(0.0f) {}
		IKJointWorkData(FCompactPoseBoneIndex _BoneIndex, const FTransform& _Transform, float _OriginalJointLengthToChild) : BoneIndex(_BoneIndex), Transform(_Transform), OriginalJointLengthToChild(_OriginalJointLengthToChild) {}
	};

	TArray<IKJointWorkData> IKJointWorkDatas;
	FVector IKRootJointOriginalLocation;

	// FAnimNode_SkeletalControlBase interface
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface
};
