// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "AnimNode_TwoBoneIKPractice.generated.h"

/**
 * 
 */
USTRUCT(BlueprintInternalUseOnly)
struct IKPLUGINSANDBOX_API FAnimNode_TwoBoneIKPractice : public FAnimNode_SkeletalControlBase
{
	GENERATED_USTRUCT_BODY()
	
	/** Name of bone to control. This is the main bone chain to modify from. **/
	UPROPERTY(EditAnywhere, Category=IK)
	FBoneReference IKBone;

	/** Reference frame of Effector Location. */
	UPROPERTY(EditAnywhere, Category=Effector)
	TEnumAsByte<enum EBoneControlSpace> EffectorLocationSpace;

	/** Effector Location. Target Location to reach. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Effector, meta = (PinShownByDefault))
	FVector EffectorLocation;

	UPROPERTY(EditAnywhere, Category=Effector)
	FBoneSocketTarget EffectorTarget;

	/** Reference frame of Joint Target Location. */
	UPROPERTY(EditAnywhere, Category=JointTarget)
	TEnumAsByte<enum EBoneControlSpace> JointTargetLocationSpace;

	/** Joint Target Location. Location used to orient Joint bone. **/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=JointTarget, meta=(PinShownByDefault))
	FVector JointTargetLocation;

	UPROPERTY(EditAnywhere, Category = JointTarget)
	FBoneSocketTarget JointTarget;

	FAnimNode_TwoBoneIKPractice();

	// FAnimNode_Base interface
	virtual void Initialize_AnyThread(const FAnimationInitializeContext& Context) override;
	// End of FAnimNode_Base interface

	// FAnimNode_SkeletalControlBase interface
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;

private:
	// FAnimNode_SkeletalControlBase interface
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface

	// cached limb index for lower/upper
	FCompactPoseBoneIndex CachedUpperLimbIndex;
	FCompactPoseBoneIndex CachedLowerLimbIndex;
};
