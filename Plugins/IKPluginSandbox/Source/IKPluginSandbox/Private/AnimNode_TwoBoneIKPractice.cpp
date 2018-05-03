// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_TwoBoneIKPractice.h"

FAnimNode_TwoBoneIKPractice::FAnimNode_TwoBoneIKPractice()
{
}

void FAnimNode_TwoBoneIKPractice::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
}

void FAnimNode_TwoBoneIKPractice::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
}

bool FAnimNode_TwoBoneIKPractice::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	return true;
}

void FAnimNode_TwoBoneIKPractice::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
}
