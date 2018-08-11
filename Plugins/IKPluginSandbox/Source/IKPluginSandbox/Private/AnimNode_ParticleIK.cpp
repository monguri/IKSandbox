// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_ParticleIK.h"
#include "Animation/AnimInstanceProxy.h"

FAnimNode_ParticleIK::FAnimNode_ParticleIK()
{
}

void FAnimNode_ParticleIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(Output.AnimInstanceProxy->GetSkelMeshComponent());
	check(OutBoneTransforms.Num() == 0);
}

bool FAnimNode_ParticleIK::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	return true;
}

void FAnimNode_ParticleIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
}
