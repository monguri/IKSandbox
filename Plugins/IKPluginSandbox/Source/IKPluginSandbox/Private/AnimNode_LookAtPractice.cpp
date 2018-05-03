// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_LookAtPractice.h"
#include "BoneContainer.h"


FAnimNode_LookAtPractice::FAnimNode_LookAtPractice()
	: LookAtLocation(FVector(100.f, 0.f, 0.f))
{
}

void FAnimNode_LookAtPractice::UpdateInternal(const FAnimationUpdateContext& Context)
{
}

void FAnimNode_LookAtPractice::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
}

void FAnimNode_LookAtPractice::EvaluateComponentSpaceInternal(FComponentSpacePoseContext& Context)
{
}

void FAnimNode_LookAtPractice::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
}

bool FAnimNode_LookAtPractice::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	return true;
}

void FAnimNode_LookAtPractice::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	BoneToModify.Initialize(RequiredBones);
	LookAtTarget.InitializeBoneReferences(RequiredBones);
}

