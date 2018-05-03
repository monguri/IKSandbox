// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_LookAtPractice.h"
#include "Animation/AnimInstanceProxy.h"
#include "AnimationCoreLibrary.h"

static const FVector DefaultLookAtAxis(0.0f, 1.0f, 0.0f);
static const FVector DefaultLookUpAxis(0.0f, 0.0f, 1.0f);

namespace
{
/** 
 * Aim solver
 *
 * This solves new transform that aims at the target based on inputs
 *
 * @param	CurrentTransform	Current Transform
 * @param	TargetPosition		Target to look at
 * @param	AimVector			Aim vector in Current Transform
 * @param	bUseUpVector		Whether or not to use Up vector
 * @param	UpVector			Up Vector in Current Transform if bUseUpVector is true
 * @param	AimClampInDegree	Clamp cone around the AimVector
 *
 * @return  Delta Rotation to turn
 */
FQuat SolveAim(const FTransform& CurrentTransform, const FVector& TargetPosition, const FVector& AimVector, bool bUseUpVector /*= false*/, const FVector& UpVector /*= FVector::UpVector*/)
{
	if (!ensureAlways(AimVector.IsNormalized())
		|| bUseUpVector && !ensureAlways(UpVector.IsNormalized())
		)
	{
		return FQuat::Identity;
	}

	FTransform NewTransform = CurrentTransform;
	FVector ToTarget = TargetPosition - NewTransform.GetLocation();
	ToTarget.Normalize();

	return FQuat::FindBetweenNormals(AimVector, ToTarget);
}
}

FAnimNode_LookAtPractice::FAnimNode_LookAtPractice()
	: LookAtLocation(FVector(100.f, 0.f, 0.f))
	, LookAt_Axis(DefaultLookAtAxis)
	, LookUp_Axis(DefaultLookUpAxis)
{
}

void FAnimNode_LookAtPractice::EvaluateComponentSpaceInternal(FComponentSpacePoseContext& Context)
{
	FAnimNode_SkeletalControlBase::EvaluateComponentSpaceInternal(Context);
}

void FAnimNode_LookAtPractice::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(OutBoneTransforms.Num() > 0);

	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();
	const FCompactPoseBoneIndex& ModifyBoneIndex = BoneToModify.GetCompactPoseIndex(BoneContainer);
	FTransform ComponentBoneTransform = Output.Pose.GetComponentSpaceTransform(ModifyBoneIndex);

	// get target location
	FTransform TargetTransform = LookAtTarget.GetTargetTransform(LookAtLocation, Output.Pose, Output.AnimInstanceProxy->GetComponentTransform());
	FVector TargetLocationInComponentSpace = TargetTransform.GetLocation();

	// lookat vector
	FVector LookAtVector = LookAt_Axis.GetTransformedAxis(ComponentBoneTransform);
	// find look up vector in local space
	FVector LookUpVector = LookUp_Axis.GetTransformedAxis(ComponentBoneTransform);
	// Find new transform from look at info
	FQuat DeltaRotation = SolveAim(ComponentBoneTransform, TargetLocationInComponentSpace, LookAtVector, bUseLookUpAxis, LookUpVector);
	ComponentBoneTransform.SetRotation(DeltaRotation * ComponentBoneTransform.GetRotation());
	// Set New Transform 
	OutBoneTransforms.Add(FBoneTransform(ModifyBoneIndex, ComponentBoneTransform));
}

bool FAnimNode_LookAtPractice::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	// if both bones are valid
	return (BoneToModify.IsValidToEvaluate(RequiredBones) &&
		// or if name isn't set (use Look At Location) or Look at bone is valid 
		// do not call isValid since that means if look at bone isn't in LOD, we won't evaluate
		// we still should evaluate as long as the BoneToModify is valid even LookAtBone isn't included in required bones
		(!LookAtTarget.HasTargetSetup() || LookAtTarget.IsValidToEvaluate(RequiredBones)));
}

void FAnimNode_LookAtPractice::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	BoneToModify.Initialize(RequiredBones);
	LookAtTarget.InitializeBoneReferences(RequiredBones);
}

void FAnimNode_LookAtPractice::UpdateInternal(const FAnimationUpdateContext& Context)
{
	FAnimNode_SkeletalControlBase::UpdateInternal(Context);
}

void FAnimNode_LookAtPractice::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
	// TODO:InitializeBoneReferencesÇ∆ÇÃà·Ç¢Ç¡ÇƒÇ»ÇÒÇæÇ¡ÇØÅH
	FAnimNode_SkeletalControlBase::Initialize_AnyThread(Context);

	LookAtTarget.Initialize(Context.AnimInstanceProxy);

	// initialize
	LookUp_Axis.Initialize();
	if (LookUp_Axis.Axis.IsZero())
	{
		UE_LOG(LogAnimation, Warning, TEXT("Zero-length look-up axis specified in LookUp node. Reverting to default."));
		LookUp_Axis.Axis = DefaultLookUpAxis;
	}
	LookAt_Axis.Initialize();
	if (LookAt_Axis.Axis.IsZero())
	{
		UE_LOG(LogAnimation, Warning, TEXT("Zero-length look-at axis specified in LookAt node. Reverting to default."));
		LookAt_Axis.Axis = DefaultLookAtAxis;
	}

}

