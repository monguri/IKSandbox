// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_LegIKPractice.h"

FAnimNode_LegIKPractice::FAnimNode_LegIKPractice()
{
	ReachPrecision = 0.01f;
	MaxIterations = 12;
}

void FAnimNode_LegIKPractice::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
}

bool FAnimNode_LegIKPractice::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	return true;
}

void FAnimNode_LegIKPractice::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
}
