// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_JacobianIKLocal.h"
#include "Animation/AnimInstanceProxy.h"

FAnimNode_JacobianIKLocal::FAnimNode_JacobianIKLocal()
{
}

void FAnimNode_JacobianIKLocal::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
}

bool FAnimNode_JacobianIKLocal::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	return true;
}

void FAnimNode_JacobianIKLocal::EvaluateSkeletalControl_AnyThread(const FCSPose<FCompactPose>& InPose, FPoseContext& Output)
{
}
