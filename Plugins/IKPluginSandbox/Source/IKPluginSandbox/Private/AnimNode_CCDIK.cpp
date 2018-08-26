// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_CCDIK.h"
#include "Animation/AnimInstanceProxy.h"

FAnimNode_CCDIK::FAnimNode_CCDIK()
	: EffectorTargetLocation(0.0f, 0.0f, 0.0f)
	, MaxIteration(10)
	, Precision(SMALL_NUMBER)
{
}

void FAnimNode_CCDIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(Output.AnimInstanceProxy->GetSkelMeshComponent());
	check(OutBoneTransforms.Num() == 0);

	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();

	// TODO:そもそもIKの解があるかの確認が毎フレーム必要

	// ワークデータのTransformの初期化
	for (IKJointWorkData& WorkData : IKJointWorkDatas)
	{
		WorkData.Transform = Output.Pose.GetComponentSpaceTransform(WorkData.BoneIndex);
	}

	// CCDIKのメインアルゴリズム
	uint32 iterCount = 0;
	for (; iterCount < MaxIteration; ++iterCount)
	{
		bool FinishIteration = false;
		// 毎イテレーション、エフェクタの直近の親ジョイントからIKRootJointに指定したジョイントまでループする
		for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i)
		{
			const FVector& EffectorLocation = IKJointWorkDatas[0].Transform.GetLocation();
			if ((EffectorLocation - EffectorTargetLocation).Size() < Precision)
			{
				// エフェクタの現在位置と目標位置が十分小さくなったらイテレーション途中でも終了
				FinishIteration = true;
				break;
			}

			// CCDの各ジョイントでの回転修正の計算
			const FVector& IKJointCurrentLocation = IKJointWorkDatas[i].Transform.GetLocation();

			const FVector& IKJointToEffectorDirection = EffectorLocation - IKJointCurrentLocation;
			const FVector& IKJointToEffectorTargetDirection = EffectorTargetLocation - IKJointCurrentLocation;

			const FQuat& DiffRotation = FQuat::FindBetweenVectors(IKJointToEffectorDirection, IKJointToEffectorTargetDirection);

			// 回転修正をEffectorまでのすべての子のコンポーネント座標でのRotationとLocationに反映させる
			// TODO:分岐は考慮していない
			for (int32 j = i - 1; j >= 0; --j)
			{
				IKJointWorkDatas[j].Transform.SetRotation(DiffRotation * IKJointWorkDatas[j].Transform.GetRotation());
				IKJointWorkDatas[j].Transform.SetLocation(IKJointCurrentLocation + DiffRotation * (IKJointWorkDatas[j].Transform.GetLocation() - IKJointCurrentLocation));
			}
		}

		if (FinishIteration)
		{
			// エフェクタの現在位置と目標位置が十分小さくなったらイテレーション途中でも終了
			break;
		}
	}

	UE_CLOG(iterCount >= MaxIteration, LogAnimation, Warning, TEXT("Did not converge at MaxIteration set."));

	// ボーンインデックスの昇順に渡したいので逆順にループする
	for (int32 i = IKJointWorkDatas.Num() - 1; i >= 0; --i)
	{
		OutBoneTransforms.Add(FBoneTransform(IKJointWorkDatas[i].BoneIndex, IKJointWorkDatas[i].Transform));
	}
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
	while (ParentIndex != INDEX_NONE && ParentIndex != IKRootJoint.BoneIndex)
	{
		ParentIndex = RequiredBones.GetParentBoneIndex(ParentIndex);
	}

	if (ParentIndex == INDEX_NONE)
	{
		// IKRootJointとEffectorJointが先祖子孫関係になってない
		return false;
	}

	return (IKJointWorkDatas.Num() > 0); // InitializeBoneReferencesは確かコンパイル時に呼ばれるけどこっちは実行時に呼ばれる
}

void FAnimNode_CCDIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	IKRootJoint.Initialize(RequiredBones);
	EffectorJoint.Initialize(RequiredBones);

	// EffectorJointから、IKRootJointまで、IKRootJointにぶつからなければルートジョイントまでワークデータを持たせる
	FCompactPoseBoneIndex IKJointIndex = EffectorJoint.GetCompactPoseIndex(RequiredBones);

	while (IKJointIndex != INDEX_NONE && IKJointIndex != IKRootJoint.BoneIndex)
	{
		IKJointWorkDatas.Emplace(IKJointIndex, FTransform::Identity);

		IKJointIndex = RequiredBones.GetParentBoneIndex(IKJointIndex);
	}

	if (IKJointIndex == IKRootJoint.BoneIndex)
	{
		IKJointWorkDatas.Emplace(IKJointIndex, FTransform::Identity);
	}
}
