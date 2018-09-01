// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_ParticleIK.h"
#include "Animation/AnimInstanceProxy.h"

FAnimNode_ParticleIK::FAnimNode_ParticleIK()
	: EffectorTargetLocation(0.0f, 0.0f, 0.0f)
	, MaxIteration(10)
	, IKRootJointOriginalLocation(0.0f, 0.0f, 0.0f)
	, Precision(SMALL_NUMBER)
{
}

void FAnimNode_ParticleIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
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

	// ワークデータの初期化
	IKJointWorkData ChildWorkData = IKJointWorkDatas[0];
	for (int32 i = 0; i < IKJointWorkDatas.Num(); ++i)
	{
		IKJointWorkDatas[i].Transform = Output.Pose.GetComponentSpaceTransform(IKJointWorkDatas[i].BoneIndex);
		if (i == 0)
		{
			IKJointWorkDatas[i].OriginalJointLengthToChild = 0.0f;
		}
		else
		{
			IKJointWorkDatas[i].OriginalJointLengthToChild = (IKJointWorkDatas[i].Transform.GetLocation() - IKJointWorkDatas[i - 1].Transform.GetLocation()).Size();
		}
	}

	IKRootJointOriginalLocation = IKJointWorkDatas.Last().Transform.GetLocation();

	// 毎イテレーション、エフェクタの直近の親ジョイントからIKRootJointに指定したジョイントまでループする
	FCompactPoseBoneIndex IKRootJointIndex = IKRootJoint.GetCompactPoseIndex(BoneContainer);
	const FVector& IKRootJointLocation = Output.Pose.GetComponentSpaceTransform(IKRootJointIndex).GetLocation();

	// Particle-IKのメインアルゴリズム
	uint32 iterCount = 0;
	for (; iterCount < MaxIteration; ++iterCount)
	{
		// まずはエフェクタの位置を指定した位置に移動させる
		IKJointWorkDatas[0].Transform.SetLocation(EffectorTargetLocation);

		bool FinishIteration = false;
		// 毎イテレーション、エフェクタの直近の親ジョイントからIKRootJointに指定したジョイントまでループする
		for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i) // Effectorの一つ親のジョイントから、IK処理のルートに設定したジョイントまでループする
		{
			const FVector& ChildToThisJointDirection = IKJointWorkDatas[i].Transform.GetLocation() - IKJointWorkDatas[i - 1].Transform.GetLocation();
			FVector ExpandDiff = ChildToThisJointDirection;
			ExpandDiff *= (ChildToThisJointDirection.Size() - IKJointWorkDatas[i].OriginalJointLengthToChild) / ChildToThisJointDirection.Size() * 0.5f;

			// ここではLocationを修正するだけでRotationは修正しない
			IKJointWorkDatas[i].Transform.SetLocation(IKJointWorkDatas[i].Transform.GetLocation() - ExpandDiff);
			IKJointWorkDatas[i - 1].Transform.SetLocation(IKJointWorkDatas[i - 1].Transform.GetLocation() + ExpandDiff);
		}

		// IKのルートジョイントの位置をもとに戻す
		IKJointWorkDatas.Last().Transform.SetLocation(IKRootJointOriginalLocation);

		// 収束判定
		const FVector& EffectorLocation = IKJointWorkDatas[0].Transform.GetLocation();
		if ((EffectorLocation - EffectorTargetLocation).Size() < Precision)
		{
			// エフェクタの現在位置と目標位置が十分小さくなった上で、各ジョイント間の長さがオリジナルの長さとほっぼ一致していたらイテレーション途中でも終了
			// ジョイントの長さのずれにこだわらないならエフェクタ位置だけで収束判定をしてもいいが
			// TODO:ジョイント長さまで判定に含めると、判定のコストがかなり大きくなる
			bool FinishIteration = true;
			for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i) // Effectorの一つ親のジョイントから、IK処理のルートに設定したジョイントまでループする
			{
				const FVector& ChildToThisJointDirection = IKJointWorkDatas[i].Transform.GetLocation() - IKJointWorkDatas[i - 1].Transform.GetLocation();
				if (FMath::Abs(ChildToThisJointDirection.Size() - IKJointWorkDatas[i].OriginalJointLengthToChild) > Precision)
				{
					FinishIteration = false;
					break;
				}
			}

			if (FinishIteration)
			{
				break;
			}
		}
	}

	UE_CLOG(iterCount >= MaxIteration, LogAnimation, Warning, TEXT("Did not converge at MaxIteration set."));

	// UE4の場合はコンポーネント座標でのSRTからローカルのSRTをComponentToLocalノードが計算してくれるため、回転に対して手を加える必要がない
	// IKを加える前の回転角のまま、Translationのみ大幅にIKによって変わっているのが不自然に感じるならRotationの調整が必要

	// ボーンインデックスの昇順に渡したいので逆順にループする
	for (int32 i = IKJointWorkDatas.Num() - 1; i >= 0; --i)
	{
		OutBoneTransforms.Add(FBoneTransform(IKJointWorkDatas[i].BoneIndex, IKJointWorkDatas[i].Transform));
	}
}

bool FAnimNode_ParticleIK::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	//TODO: CCD-IKと同じなので共通化したい
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

void FAnimNode_ParticleIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	IKRootJoint.Initialize(RequiredBones);
	EffectorJoint.Initialize(RequiredBones);

	// EffectorJointから、IKRootJointまで、IKRootJointにぶつからなければルートジョイントまでワークデータを持たせる
	FCompactPoseBoneIndex IKJointIndex = EffectorJoint.GetCompactPoseIndex(RequiredBones);

	while (IKJointIndex != INDEX_NONE && IKJointIndex != IKRootJoint.BoneIndex)
	{
		IKJointWorkDatas.Emplace(IKJointIndex, FTransform::Identity, 0.0f);

		IKJointIndex = RequiredBones.GetParentBoneIndex(IKJointIndex);
	}

	if (IKJointIndex == IKRootJoint.BoneIndex)
	{
		IKJointWorkDatas.Emplace(IKJointIndex, FTransform::Identity, 0.0f);
	}
}
