// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_JacobianIK.h"
#include "Animation/AnimInstanceProxy.h"

FAnimNode_JacobianIK::FAnimNode_JacobianIK()
	: EffectorTargetLocation(0.0f, 0.0f, 0.0f)
	, MaxIteration(10)
	, Precision(SMALL_NUMBER)
{
}

void FAnimNode_JacobianIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(Output.AnimInstanceProxy->GetSkelMeshComponent());
	check(OutBoneTransforms.Num() == 0);

	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();

	// TODO:各IKノードと共通化しよう
	// そもそもジョイントの長さ的にIKの解に到達しうるかの確認
	float EffectorToIKRootLength = (Output.Pose.GetComponentSpaceTransform(IKJointWorkDatas.Last().BoneIndex).GetLocation() - Output.Pose.GetComponentSpaceTransform(IKJointWorkDatas[0].BoneIndex).GetLocation()).Size();
	float IKJointTotalLength = 0; // アニメーションにScaleがないなら、一度だけ計算してキャッシュしておけばよいが、今は毎回計算する
	for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i)
	{
		IKJointTotalLength += (Output.Pose.GetComponentSpaceTransform(IKJointWorkDatas[i].BoneIndex).GetLocation() - Output.Pose.GetComponentSpaceTransform(IKJointWorkDatas[i - 1].BoneIndex).GetLocation()).Size();
	}

	if (IKJointTotalLength < EffectorToIKRootLength)
	{
		UE_LOG(LogAnimation, Warning, TEXT("IK cannot reach effector target location. The total length of joints is not enough."));
		return;
	}

	// ワークデータのTransformの初期化
	for (IKJointWorkData& WorkData : IKJointWorkDatas)
	{
		WorkData.Transform = Output.Pose.GetComponentSpaceTransform(WorkData.BoneIndex);
	}

	// JacobianIKのメインアルゴリズム
	// JacobianIKアルゴリズムについてはComputer Graphics Gems JP 2012の8章を参照
	uint32 iterCount = 0;
	for (; iterCount < MaxIteration; ++iterCount)
	{
	}

	UE_CLOG(iterCount >= MaxIteration, LogAnimation, Warning, TEXT("Did not converge at MaxIteration set."));

	// ボーンインデックスの昇順に渡したいので逆順にループする
	for (int32 i = IKJointWorkDatas.Num() - 1; i >= 0; --i)
	{
		OutBoneTransforms.Add(FBoneTransform(IKJointWorkDatas[i].BoneIndex, IKJointWorkDatas[i].Transform));
	}
}

bool FAnimNode_JacobianIK::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
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

void FAnimNode_JacobianIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
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
