// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_CCDIK.h"
#include "Animation/AnimInstanceProxy.h"

FAnimNode_CCDIK::FAnimNode_CCDIK()
{
}

void FAnimNode_CCDIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(Output.AnimInstanceProxy->GetSkelMeshComponent());
	check(OutBoneTransforms.Num() == 0);
}

bool FAnimNode_CCDIK::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	if (!IKRootJoint.IsValidToEvaluate(RequiredBones))
	{
		return false;
	}

	if (!EffectorJoint.IsValidToEvaluate(RequiredBones))
	{
		return false;
	}

	FCompactPoseBoneIndex ParentIndex = EffectorJoint.GetCompactPoseIndex(RequiredBones);
	while (ParentIndex != INDEX_NONE || ParentIndex != IKRootJoint.BoneIndex)
	{
		ParentIndex = RequiredBones.GetParentBoneIndex(ParentIndex);
	}

	return (ParentIndex == IKRootJoint.BoneIndex);
}

void FAnimNode_CCDIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
}
