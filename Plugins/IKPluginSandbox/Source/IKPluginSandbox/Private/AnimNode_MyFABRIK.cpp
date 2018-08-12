// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_MyFABRIK.h"
#include "Animation/AnimInstanceProxy.h"

FAnimNode_MyFABRIK::FAnimNode_MyFABRIK()
{
}

void FAnimNode_MyFABRIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(Output.AnimInstanceProxy->GetSkelMeshComponent());
	check(OutBoneTransforms.Num() == 0);
}

bool FAnimNode_MyFABRIK::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	return true;
}

void FAnimNode_MyFABRIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
}
