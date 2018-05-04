// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_TwoBoneIKPractice.h"
#include "AnimationRuntime.h"
#include "Animation/AnimInstanceProxy.h"
#include "TwoBoneIK.h" //TODO:後で消す

FAnimNode_TwoBoneIKPractice::FAnimNode_TwoBoneIKPractice()
	: EffectorLocationSpace(BCS_ComponentSpace)
	, EffectorLocation(FVector::ZeroVector)
	, JointTargetLocationSpace(BCS_ComponentSpace)
	, JointTargetLocation(FVector::ZeroVector)
	, CachedUpperLimbIndex(INDEX_NONE)
	, CachedLowerLimbIndex(INDEX_NONE)
{
}

FTransform FAnimNode_TwoBoneIKPractice::GetTargetTransform(const FTransform& InComponentTransform, FCSPose<FCompactPose>& MeshBases, FBoneSocketTarget& InTarget, EBoneControlSpace Space, const FVector& InOffset)
{
	FTransform OutTransform;
	if (Space == BCS_BoneSpace)
	{
		// このときは単なるラッパー
		// InOffsetをLSからCSに変換
		OutTransform = InTarget.GetTargetTransform(InOffset, MeshBases, InComponentTransform);
	}
	else
	{
		// parent bone space still goes through this way
		// if your target is socket, it will try find parents of joint that socket belongs to
		OutTransform.SetLocation(InOffset);
		FAnimationRuntime::ConvertBoneSpaceTransformToCS(InComponentTransform, MeshBases, OutTransform, InTarget.GetCompactPoseBoneIndex(), Space);
		// TODO:この関数って任意のSpaceを受け入れるからif節って必要なくならない？
	}

	return OutTransform;
}

void FAnimNode_TwoBoneIKPractice::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(OutBoneTransforms.Num() == 0);

	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();

	// Get indices of the lower and upper limb bones and check validity.
	bool bInvalidLimb = false;

	// TwoBoneIK.cppの変数名をそのまま使ったが、IKBone、Effector、EndBoneとエフェクタを複数の名前で呼んでるのに注意
	FCompactPoseBoneIndex IKBoneCompactPoseIndex = IKBone.GetCompactPoseIndex(BoneContainer);

	// Get Local Space transforms for our bones. We do this first in case they already are local.
	// As right after we get them in component space. (And that does the auto conversion).
	// We might save one transform by doing local first...
	const FTransform EndBoneLocalTransform = Output.Pose.GetLocalSpaceTransform(IKBoneCompactPoseIndex);
	const FTransform LowerLimbLocalTransform = Output.Pose.GetLocalSpaceTransform(CachedLowerLimbIndex);
	const FTransform UpperLimbLocalTransform = Output.Pose.GetLocalSpaceTransform(CachedUpperLimbIndex);

	// Now get those in component space...
	FTransform EndBoneCSTransform = Output.Pose.GetComponentSpaceTransform(IKBoneCompactPoseIndex);
	FTransform LowerLimbCSTransform = Output.Pose.GetComponentSpaceTransform(CachedLowerLimbIndex);
	FTransform UpperLimbCSTransform = Output.Pose.GetComponentSpaceTransform(CachedUpperLimbIndex);

#if 0 // AnimNode_Two_BoneIK.cppでやってる処理だが、GetTranslationしてSetLocationしてるだけで、何も加工しておらず、LocationとTranslationは同じなのでコメントアウト
	// Get current position of root of limb.
	// All position are in Component space.
	const FVector RootPos = UpperLimbCSTransform.GetTranslation();
	const FVector InitialJointPos = LowerLimbCSTransform.GetTranslation();
	const FVector InitialEndPos = EndBoneCSTransform.GetTranslation();
#endif

	// Transform EffectorLocation from EffectorLocationSpace to ComponentSpace.
	// エフェクタのターゲット位置
	FTransform EffectorTargetTransform = GetTargetTransform(Output.AnimInstanceProxy->GetComponentTransform(), Output.Pose, EffectorTarget, EffectorLocationSpace, EffectorLocation);

	// This is our reach goal.
	FVector EffectorTargetPosition = EffectorTargetTransform.GetLocation();

	// Get joint target (used for defining plane that joint should be in).
	// エフェクタの向きを決めるためのターゲット
	FTransform JointTargetTransform = GetTargetTransform(Output.AnimInstanceProxy->GetComponentTransform(), Output.Pose, JointTarget, JointTargetLocationSpace, JointTargetLocation);

	FVector JointTargetPosition = JointTargetTransform.GetLocation();

#if 0 // AnimNode_Two_BoneIK.cppでやってる処理だが、GetTranslationしてSetLocationしてるだけで、何も加工しておらず、LocationとTranslationは同じなのでコメントアウト
	UpperLimbCSTransform.SetLocation(RootPos);
	LowerLimbCSTransform.SetLocation(InitialJointPos);
	EndBoneCSTransform.SetLocation(InitialEndPos);
#endif

	AnimationCore::SolveTwoBoneIK(UpperLimbCSTransform, LowerLimbCSTransform, EndBoneCSTransform, JointTargetPosition, EffectorTargetPosition, 0, 0, 0);

	// Update transform for upper bone.
	{
		// Order important. First bone is upper limb.
		OutBoneTransforms.Add(FBoneTransform(CachedUpperLimbIndex, UpperLimbCSTransform));
	}

	// Update transform for lower bone.
	{
		// Order important. Second bone is lower limb.
		OutBoneTransforms.Add(FBoneTransform(CachedLowerLimbIndex, LowerLimbCSTransform));
	}

	// Update transform for end bone.
	{
		// Order important. Third bone is End Bone.
		OutBoneTransforms.Add(FBoneTransform(IKBoneCompactPoseIndex, EndBoneCSTransform));
	}

	// Make sure we have correct number of bones
	check(OutBoneTransforms.Num() == 3);
}

bool FAnimNode_TwoBoneIKPractice::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	if (!IKBone.IsValidToEvaluate(RequiredBones))
	{
		return false;
	}

	if (CachedLowerLimbIndex == INDEX_NONE || CachedUpperLimbIndex == INDEX_NONE)
	{
		return false;
	}

	// check bone space here
	if (EffectorLocationSpace == BCS_ParentBoneSpace || EffectorLocationSpace == BCS_BoneSpace)
	{
		if (!EffectorTarget.IsValidToEvaluate(RequiredBones))
		{
			return false;
		}
	}

	if (JointTargetLocationSpace == BCS_ParentBoneSpace || JointTargetLocationSpace == BCS_BoneSpace)
	{
		if (!JointTarget.IsValidToEvaluate(RequiredBones))
		{
			return false;
		}
	}

	return true;
}

void FAnimNode_TwoBoneIKPractice::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	IKBone.Initialize(RequiredBones);

	//TODO:なぜかリンクエラーになるので必要な処理だがとりあえずコメントアウト
	//EffectorTarget.InitializeBoneReferences(RequiredBones);
	//JointTarget.InitializeBoneReferences(RequiredBones);

	FCompactPoseBoneIndex IKBoneCompactPoseIndex = IKBone.GetCompactPoseIndex(RequiredBones);
	CachedLowerLimbIndex = FCompactPoseBoneIndex(INDEX_NONE);
	CachedUpperLimbIndex = FCompactPoseBoneIndex(INDEX_NONE);

	if (IKBoneCompactPoseIndex != INDEX_NONE)
	{
		CachedLowerLimbIndex = RequiredBones.GetParentBoneIndex(IKBoneCompactPoseIndex);
		if (CachedLowerLimbIndex != INDEX_NONE)
		{
			CachedUpperLimbIndex = RequiredBones.GetParentBoneIndex(CachedLowerLimbIndex);
		}
	}
}

void FAnimNode_TwoBoneIKPractice::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
	FAnimNode_SkeletalControlBase::Initialize_AnyThread(Context);
	//TODO:なぜかリンクエラーになるので必要な処理だがとりあえずコメントアウト
	//EffectorTarget.Initialize(Context.AnimInstanceProxy);
	//JointTarget.Initialize(Context.AnimInstanceProxy);
}
