// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_TwoBoneIKPractice.h"
#include "AnimationRuntime.h"
#include "Animation/AnimInstanceProxy.h"

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
		// ���̂Ƃ��͒P�Ȃ郉�b�p�[
		// InOffset��LS����CS�ɕϊ�
		OutTransform = InTarget.GetTargetTransform(InOffset, MeshBases, InComponentTransform);
	}
	else
	{
		// parent bone space still goes through this way
		// if your target is socket, it will try find parents of joint that socket belongs to
		OutTransform.SetLocation(InOffset);
		FAnimationRuntime::ConvertBoneSpaceTransformToCS(InComponentTransform, MeshBases, OutTransform, InTarget.GetCompactPoseBoneIndex(), Space);
		// TODO:���̊֐����ĔC�ӂ�Space���󂯓���邩��if�߂��ĕK�v�Ȃ��Ȃ�Ȃ��H
	}

	return OutTransform;
}

namespace
{
void SolveTwoBoneIK(const FVector& RootPos, const FVector& JointPos, const FVector& EndPos, const FVector& JointTarget, const FVector& Effector, FVector& OutJointPos, FVector& OutEndPos, float UpperLimbLength, float LowerLimbLength)
{
	// This is our reach goal.
	FVector DesiredPos = Effector;
	FVector DesiredDelta = DesiredPos - RootPos;
	float DesiredLength = DesiredDelta.Size();

	// Find lengths of upper and lower limb in the ref skeleton.
	// Use actual sizes instead of ref skeleton, so we take into account translation and scaling from other bone controllers.
	float MaxLimbLength = LowerLimbLength + UpperLimbLength;

	// Check to handle case where DesiredPos is the same as RootPos.
	FVector DesiredDir;
	if (DesiredLength < (float)KINDA_SMALL_NUMBER)
	{
		DesiredLength = (float)KINDA_SMALL_NUMBER;
		DesiredDir = FVector(1, 0, 0);
	}
	else
	{
		DesiredDir = DesiredDelta.GetSafeNormal();
	}

	// Get joint target (used for defining plane that joint should be in).
	FVector JointTargetDelta = JointTarget - RootPos;
	const float JointTargetLengthSqr = JointTargetDelta.SizeSquared();

	// Same check as above, to cover case when JointTarget position is the same as RootPos.
	FVector JointPlaneNormal, JointBendDir;
	// JointBendDir�ƌ����Ă邪�AOutJoint�����ۂɋȂ�������ł͂Ȃ��AJointPlane�ɑ΂��ĕ��s�ŁADesiredDir�ɑ΂��Đ����Ȏ��ɂ�����
	if (JointTargetLengthSqr < FMath::Square((float)KINDA_SMALL_NUMBER))
	{
		JointBendDir = FVector(0, 1, 0);
		JointPlaneNormal = FVector(0, 0, 1);
	}
	else
	{
		JointPlaneNormal = DesiredDir ^ JointTargetDelta;

		// If we are trying to point the limb in the same direction that we are supposed to displace the joint in, 
		// we have to just pick 2 random vector perp to DesiredDir and each other.
		if (JointPlaneNormal.SizeSquared() < FMath::Square((float)KINDA_SMALL_NUMBER))
		{
			DesiredDir.FindBestAxisVectors(JointPlaneNormal, JointBendDir);
		}
		else
		{
			JointPlaneNormal.Normalize();

			// Find the final member of the reference frame by removing any component of JointTargetDelta along DesiredDir.
			// This should never leave a zero vector, because we've checked DesiredDir and JointTargetDelta are not parallel.
			JointBendDir = JointTargetDelta - (JointTargetDelta | DesiredDir) * DesiredDir;
			JointBendDir.Normalize();
		}
	}

	OutEndPos = DesiredPos;
	// ������������IK�v�Z�BTwoBoneIK�Ȃ�OutJointPos����͓I�ɉ�����
	OutJointPos = JointPos;

	// If we are trying to reach a goal beyond the length of the limb, clamp it to something solvable and extend limb fully.
	if (DesiredLength > MaxLimbLength)
	{
		// OutEndPos�͂������������S��������DesiredPos�Ɍ��܂�Ȃ̂����A�����I�ɖ����ȂƂ��͍ō��Ɋ֐߂��w������ɂ܂������ɐL�΂����`�ɂ���
		OutEndPos = RootPos + MaxLimbLength * DesiredDir;
		OutJointPos = RootPos + UpperLimbLength * DesiredDir;
	}
	else
	{
		// �����͊��S�ɎO�p�֐��̌v�Z�ɂ����OutJointPos�����߂镔���ł���

		// �]���藝��TwoBone�̂Ȃ��O�p�`��Root�����̊p�x�����߂�
		// So we have a triangle we know the side lengths of. We can work out the angle between DesiredDir and the direction of the upper limb
		// using the sin rule:
		const float TwoAB = 2.0f * UpperLimbLength * DesiredLength;

		const float CosAngle = ((UpperLimbLength * UpperLimbLength) + (DesiredLength * DesiredLength) - (LowerLimbLength * LowerLimbLength)) / TwoAB;

		// If CosAngle is less than 0, the upper arm actually points the opposite way to DesiredDir, so we handle that.
		const bool bReverseUpperBone = (CosAngle < 0.0f);

		// Angle between upper limb and DesiredDir
		// ACos clamps internally so we dont need to worry about out-of-range values here.
		const float Angle = FMath::Acos(CosAngle);

		// Now we calculate the distance of the joint from the root -> effector line.
		// This forms a right-angle triangle, with the upper limb as the hypotenuse.
		const float JointLineDist = UpperLimbLength * FMath::Sin(Angle);

		// And the final side of that triangle - distance along DesiredDir of perpendicular.
		// ProjJointDistSqr can't be neg, because JointLineDist must be <= UpperLimbLength because appSin(Angle) is <= 1.
		// �s�^�S���X�̒藝
		const float ProjJointDistSqr = (UpperLimbLength * UpperLimbLength) - (JointLineDist * JointLineDist);
		float ProjJointDist = (ProjJointDistSqr > 0.0f) ? FMath::Sqrt(ProjJointDistSqr) : 0.0f;
		if (bReverseUpperBone)
		{
			ProjJointDist *= -1.0f; // ����͉��C�ɕK�v���ɋC�Â��ɂ�����ȁB�x�N�g���v�Z�ƎO�p�֐��v�Z�̈Ⴂ�𖄂߂����
		}

		// So now we can work out where to put the joint!
		OutJointPos = RootPos + (ProjJointDist * DesiredDir) + (JointLineDist * JointBendDir);
	}
}

void SolveTwoBoneIK(FTransform& InOutRootTransform, FTransform& InOutJointTransform, FTransform& InOutEndTransform, const FVector& JointTarget, const FVector& Effector, float UpperLimbLength, float LowerLimbLength)
{
	FVector OutJointPos, OutEndPos;

	FVector RootPos = InOutRootTransform.GetLocation();
	FVector JointPos = InOutJointTransform.GetLocation();
	FVector EndPos = InOutEndTransform.GetLocation();

	SolveTwoBoneIK(RootPos, JointPos, EndPos, JointTarget, Effector, OutJointPos, OutEndPos, UpperLimbLength, LowerLimbLength);

	// Update transform for upper bone.
	{
		// Get difference in direction for old and new joint orientations
		FVector const OldDir = (JointPos - RootPos).GetSafeNormal();
		FVector const NewDir = (OutJointPos - RootPos).GetSafeNormal();
		// Find Delta Rotation take takes us from Old to New dir
		FQuat const DeltaRotation = FQuat::FindBetweenNormals(OldDir, NewDir);
		// Rotate our Joint quaternion by this delta rotation
		InOutRootTransform.SetRotation(DeltaRotation * InOutRootTransform.GetRotation());
		// And put joint where it should be.
		InOutRootTransform.SetTranslation(RootPos);
	}

	// update transform for middle bone
	{
		// Get difference in direction for old and new joint orientations
		FVector const OldDir = (EndPos - JointPos).GetSafeNormal();
		FVector const NewDir = (OutEndPos - OutJointPos).GetSafeNormal();
		// Find Delta Rotation take takes us from Old to New dir
		FQuat const DeltaRotation = FQuat::FindBetweenNormals(OldDir, NewDir);
		// Rotate our Joint quaternion by this delta rotation
		InOutJointTransform.SetRotation(DeltaRotation * InOutJointTransform.GetRotation());
		// And put joint where it should be.
		InOutJointTransform.SetTranslation(OutJointPos);
	}

	// Update transform for end bone.
	// currently not doing anything to rotation
	// keeping input rotation
	// Set correct location for end bone.
	InOutEndTransform.SetTranslation(OutEndPos);
	// TODO:Location��Translation�͓����Ȃ̂ɂȂ�Get��Location��Set��Translation���g���񂾂낤�H
}

void SolveTwoBoneIK(FTransform& InOutRootTransform, FTransform& InOutJointTransform, FTransform& InOutEndTransform, const FVector& JointTarget, const FVector& Effector)
{
	float LowerLimbLength = (InOutEndTransform.GetLocation() - InOutJointTransform.GetLocation()).Size();
	float UpperLimbLength = (InOutJointTransform.GetLocation() - InOutRootTransform.GetLocation()).Size();
	SolveTwoBoneIK(InOutRootTransform, InOutJointTransform, InOutEndTransform, JointTarget, Effector, UpperLimbLength, LowerLimbLength);
}
}

void FAnimNode_TwoBoneIKPractice::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(OutBoneTransforms.Num() == 0);

	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();

	// Get indices of the lower and upper limb bones and check validity.
	bool bInvalidLimb = false;

	// TwoBoneIK.cpp�̕ϐ��������̂܂܎g�������AIKBone�AEffector�AEndBone�ƃG�t�F�N�^�𕡐��̖��O�ŌĂ�ł�̂ɒ���
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

#if 0 // AnimNode_Two_BoneIK.cpp�ł���Ă鏈�������AGetTranslation����SetLocation���Ă邾���ŁA�������H���Ă��炸�ALocation��Translation�͓����Ȃ̂ŃR�����g�A�E�g
	// Get current position of root of limb.
	// All position are in Component space.
	const FVector RootPos = UpperLimbCSTransform.GetTranslation();
	const FVector InitialJointPos = LowerLimbCSTransform.GetTranslation();
	const FVector InitialEndPos = EndBoneCSTransform.GetTranslation();
#endif

	// Transform EffectorLocation from EffectorLocationSpace to ComponentSpace.
	// �G�t�F�N�^�̃^�[�Q�b�g�ʒu
	FTransform EffectorTargetTransform = GetTargetTransform(Output.AnimInstanceProxy->GetComponentTransform(), Output.Pose, EffectorTarget, EffectorLocationSpace, EffectorLocation);

	// This is our reach goal.
	FVector EffectorTargetPosition = EffectorTargetTransform.GetLocation();

	// Get joint target (used for defining plane that joint should be in).
	// �G�t�F�N�^�̌��������߂邽�߂̃^�[�Q�b�g
	FTransform JointTargetTransform = GetTargetTransform(Output.AnimInstanceProxy->GetComponentTransform(), Output.Pose, JointTarget, JointTargetLocationSpace, JointTargetLocation);

	FVector JointTargetPosition = JointTargetTransform.GetLocation();

#if 0 // AnimNode_Two_BoneIK.cpp�ł���Ă鏈�������AGetTranslation����SetLocation���Ă邾���ŁA�������H���Ă��炸�ALocation��Translation�͓����Ȃ̂ŃR�����g�A�E�g
	UpperLimbCSTransform.SetLocation(RootPos);
	LowerLimbCSTransform.SetLocation(InitialJointPos);
	EndBoneCSTransform.SetLocation(InitialEndPos);
#endif

	SolveTwoBoneIK(UpperLimbCSTransform, LowerLimbCSTransform, EndBoneCSTransform, JointTargetPosition, EffectorTargetPosition);

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

	//TODO:�Ȃ��������N�G���[�ɂȂ�̂ŕK�v�ȏ��������Ƃ肠�����R�����g�A�E�g
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
	//TODO:�Ȃ��������N�G���[�ɂȂ�̂ŕK�v�ȏ��������Ƃ肠�����R�����g�A�E�g
	//EffectorTarget.Initialize(Context.AnimInstanceProxy);
	//JointTarget.Initialize(Context.AnimInstanceProxy);
}
