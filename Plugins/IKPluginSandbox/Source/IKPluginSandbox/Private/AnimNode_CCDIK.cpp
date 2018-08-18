// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_CCDIK.h"
#include "Animation/AnimInstanceProxy.h"

FAnimNode_CCDIK::FAnimNode_CCDIK()
	: EffectorTargetLocation(0.0f, 0.0f, 0.0f)
	, NumIteration(10)
{
}

void FAnimNode_CCDIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(Output.AnimInstanceProxy->GetSkelMeshComponent());
	check(OutBoneTransforms.Num() == 0);

	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();

	// ワークデータのRotationの初期化
	for (TPair<int32, FQuat>& WorkData  : IKJointWorkDatas)
	{
		WorkData.Value = Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(WorkData.Key)).GetRotation();
	}

	// CCDIKのメインアルゴリズム
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

			// CCDの各ジョイントでの回転角調整
			const FVector& ParentLocation = Output.Pose.GetComponentSpaceTransform(ParentIndex).GetLocation();

			const FVector& ParentToEffectorDirection = EffectorLocation - ParentLocation;
			const FVector& ParentToEffectorTargetDirection = EffectorTargetLocation - ParentLocation;

			const FQuat& DiffRotation = FQuat::FindBetweenVectors(ParentToEffectorDirection, ParentToEffectorTargetDirection);

			IKJointWorkDatas[ParentIndex.GetInt()] *= DiffRotation;
		}

		if (FinishIteration)
		{
			// エフェクタの現在位置と目標位置が十分小さくなったらイテレーション途中でも終了
			break;
		}
	}

	for (TPair<int32, FQuat>& WorkData  : IKJointWorkDatas)
	{
		FTransform Transform = Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(WorkData.Key));
		Transform.SetRotation(WorkData.Value);
		OutBoneTransforms.Add(FBoneTransform(FCompactPoseBoneIndex(WorkData.Key), Transform));
	}
	// 配列がボーンインデックスの降順に並んでるので昇順に直す
	OutBoneTransforms.Sort(FCompareBoneTransformIndex());
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

	if (ParentIndex == INDEX_NONE)
	{
		return false;
	}

	return (IKJointWorkDatas.Num() > 0); // InitializeBoneReferencesは確かコンパイル時に呼ばれるけどこっちは実行時に呼ばれる
}

void FAnimNode_CCDIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	IKRootJoint.Initialize(RequiredBones);
	EffectorJoint.Initialize(RequiredBones);

	FCompactPoseBoneIndex ParentIndex = EffectorJoint.GetCompactPoseIndex(RequiredBones);
	while (ParentIndex != INDEX_NONE || ParentIndex != IKRootJoint.BoneIndex)
	{
		IKJointWorkDatas.Add(ParentIndex.GetInt(), FQuat::Identity);

		ParentIndex = RequiredBones.GetParentBoneIndex(ParentIndex);
	}
}
