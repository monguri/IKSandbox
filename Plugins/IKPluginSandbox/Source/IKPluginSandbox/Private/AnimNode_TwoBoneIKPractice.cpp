// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_TwoBoneIKPractice.h"

FAnimNode_TwoBoneIKPractice::FAnimNode_TwoBoneIKPractice()
	: EffectorLocationSpace(BCS_ComponentSpace)
	, EffectorLocation(FVector::ZeroVector)
	, JointTargetLocationSpace(BCS_ComponentSpace)
	, JointTargetLocation(FVector::ZeroVector)
	, CachedUpperLimbIndex(INDEX_NONE)
	, CachedLowerLimbIndex(INDEX_NONE)
{
}

void FAnimNode_TwoBoneIKPractice::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
}

bool FAnimNode_TwoBoneIKPractice::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	if (!IKBone.IsValidToEvaluate(RequiredBones))
	{
		return false;
	}

	if (CachedLowerLimbIndex == INDEX_NONE || CachedUpperLimbIndex == INDEX_NONE)
	{
		return false;
	}

	// check bone space here
	if (EffectorLocationSpace == BCS_ParentBoneSpace || EffectorLocationSpace == BCS_BoneSpace)
	{
		if (!EffectorTarget.IsValidToEvaluate(RequiredBones))
		{
			return false;
		}
	}

	if (JointTargetLocationSpace == BCS_ParentBoneSpace || JointTargetLocationSpace == BCS_BoneSpace)
	{
		if (!JointTarget.IsValidToEvaluate(RequiredBones))
		{
			return false;
		}
	}

	return true;
}

void FAnimNode_TwoBoneIKPractice::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	IKBone.Initialize(RequiredBones);

	EffectorTarget.InitializeBoneReferences(RequiredBones);
	JointTarget.InitializeBoneReferences(RequiredBones);

	FCompactPoseBoneIndex IKBoneCompactPoseIndex = IKBone.GetCompactPoseIndex(RequiredBones);
	CachedLowerLimbIndex = FCompactPoseBoneIndex(INDEX_NONE);
	CachedUpperLimbIndex = FCompactPoseBoneIndex(INDEX_NONE);

	if (IKBoneCompactPoseIndex != INDEX_NONE)
	{
		CachedLowerLimbIndex = RequiredBones.GetParentBoneIndex(IKBoneCompactPoseIndex);
		if (CachedLowerLimbIndex != INDEX_NONE)
		{
			CachedUpperLimbIndex = RequiredBones.GetParentBoneIndex(CachedLowerLimbIndex);
		}
	}
}

void FAnimNode_TwoBoneIKPractice::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
	FAnimNode_SkeletalControlBase::Initialize_AnyThread(Context);
	EffectorTarget.Initialize(Context.AnimInstanceProxy);
	JointTarget.Initialize(Context.AnimInstanceProxy);
}
