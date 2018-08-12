// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_CCDIK.h"
#include "Animation/AnimInstanceProxy.h"

FAnimNode_CCDIK::FAnimNode_CCDIK()
	: EffectorLocation(0.0f, 0.0f, 0.0f)
	, NumIteration(10)
{
}

void FAnimNode_CCDIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();

	for (uint32 i = 0; i < NumIteration; ++i)
	{
		FCompactPoseBoneIndex EffectorIndex = EffectorJoint.GetCompactPoseIndex(BoneContainer);
		bool FinishIteration = false;

		for (FCompactPoseBoneIndex ParentIndex = BoneContainer.GetParentBoneIndex(EffectorIndex);
			ParentIndex != IKRootJoint.BoneIndex;
			ParentIndex = BoneContainer.GetParentBoneIndex(ParentIndex))
		{
			const FVector& EffectorLocation = Output.Pose.GetComponentSpaceTransform(EffectorIndex).GetLocation();
			if ((EffectorLocation - EffectorTargetLocation).Size() < SMALL_NUMBER)
			{
				// エフェクタの現在位置と目標位置が十分小さくなったらイテレーション途中でも終了
				FinishIteration = true;
				break;
			}

			const FVector& ParentLocation = Output.Pose.GetComponentSpaceTransform(ParentIndex).GetLocation();

			FVector& ParentToEffectorDirection = EffectorLocation - ParentLocation;
			bool Success = ParentToEffectorDirection.Normalize();
			check(Success); // ParentとEffectorの位置が一致したときはNormalizeができなくなる

			FVector& ParentToEffectorTargetDirection = EffectorTargetLocation - ParentLocation;
			Success = ParentToEffectorTargetDirection.Normalize();
			check(Success) // ParentとEffectorTargetの位置が一致したときはNormalizeができなくなる;

			const FQuat& DiffRotation = FQuat::FindBetweenNormals(ParentToEffectorDirection, ParentToEffectorTargetDirection);

			// TODO:ジョイントに回転を加える !!!次はここから！

			ParentIndex = BoneContainer.GetParentBoneIndex(ParentIndex);
		}

		if (FinishIteration)
		{
			// エフェクタの現在位置と目標位置が十分小さくなったらイテレーション途中でも終了
			break;
		}
	}


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
