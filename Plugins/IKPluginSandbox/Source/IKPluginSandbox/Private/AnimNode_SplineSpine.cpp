// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_SplineSpine.h"
#include "Animation/AnimInstanceProxy.h"

FAnimNode_SplineSpine::FAnimNode_SplineSpine()
{
}

void FAnimNode_SplineSpine::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
}

bool FAnimNode_SplineSpine::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	return true;
}

void FAnimNode_SplineSpine::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
}
