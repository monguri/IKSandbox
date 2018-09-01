// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_MyFABRIK.h"
#include "Animation/AnimInstanceProxy.h"

FAnimNode_MyFABRIK::FAnimNode_MyFABRIK()
	: EffectorTargetLocation(0.0f, 0.0f, 0.0f)
	, MaxIteration(10)
	, IKRootJointOriginalLocation(0.0f, 0.0f, 0.0f)
	, Precision(SMALL_NUMBER)
{
}

void FAnimNode_MyFABRIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(Output.AnimInstanceProxy->GetSkelMeshComponent());
	check(OutBoneTransforms.Num() == 0);

	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();

	// TODO:�eIK�m�[�h�Ƌ��ʉ����悤
	// ���������W���C���g�̒����I��IK�̉��ɓ��B�����邩�̊m�F
	float EffectorToIKRootLength = (Output.Pose.GetComponentSpaceTransform(IKJointWorkDatas.Last().BoneIndex).GetLocation() - Output.Pose.GetComponentSpaceTransform(IKJointWorkDatas[0].BoneIndex).GetLocation()).Size();
	float IKJointTotalLength = 0; // �A�j���[�V������Scale���Ȃ��Ȃ�A��x�����v�Z���ăL���b�V�����Ă����΂悢���A���͖���v�Z����
	for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i)
	{
		IKJointTotalLength += (Output.Pose.GetComponentSpaceTransform(IKJointWorkDatas[i].BoneIndex).GetLocation() - Output.Pose.GetComponentSpaceTransform(IKJointWorkDatas[i - 1].BoneIndex).GetLocation()).Size();
	}

	if (IKJointTotalLength < EffectorToIKRootLength)
	{
		UE_LOG(LogAnimation, Warning, TEXT("IK cannot reach effector target location. The total length of joints is not enough."));
		return;
	}

	// ���[�N�f�[�^�̏�����
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

	// ���C�e���[�V�����A�G�t�F�N�^�̒��߂̐e�W���C���g����IKRootJoint�Ɏw�肵���W���C���g�܂Ń��[�v����
	FCompactPoseBoneIndex IKRootJointIndex = IKRootJoint.GetCompactPoseIndex(BoneContainer);
	const FVector& IKRootJointLocation = Output.Pose.GetComponentSpaceTransform(IKRootJointIndex).GetLocation();

	// Particle-IK�̃��C���A���S���Y��
	// https://www.robloxdev.com/articles/Inverse-Kinematics-for-Animation#FABRIK
	uint32 iterCount = 0;
	for (; iterCount < MaxIteration; ++iterCount)
	{
		// ParticleIK�Ƃ̈Ⴂ
		// �EParitcleIK�̓{�[���̒������炸�ꂽ����e�Ǝq���������Ĉړ����邪�AFABRIK��
		// Backward(�e�ɂ����̂ڂ郋�[�v)�ł͐e�̃W���C���g�݈̂ړ�������BForward�ł͋t�Ɏq�̃W���C���g�݈̂ړ�������B
		// �EParitcleIK�̓C�e���[�V�����̊J�n���_�ŃG�t�F�N�^��ڕW�ʒu�Ɉړ������A�C�e���[�V�����̍Ō�̓��[�g�����̈ʒu�ɖ߂�
		// FABRIK�ł�Backward�̊J�n���_�ŃG�t�F�N�^��ڕW�ʒu�Ɉړ������AForward�̊J�n���_�ł̓��[�g�����Ƃ̈ʒu�ɖ߂�

		// Backward
		{
			// �܂��̓G�t�F�N�^�̈ʒu���w�肵���ʒu�Ɉړ�������
			IKJointWorkDatas[0].Transform.SetLocation(EffectorTargetLocation);

			// ���C�e���[�V�����A�G�t�F�N�^�̒��߂̐e�W���C���g����IKRootJoint�Ɏw�肵���W���C���g�܂Ń��[�v����
			for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i) // Effector�̈�e�̃W���C���g����AIK�����̃��[�g�ɐݒ肵���W���C���g�܂Ń��[�v����
			{
				const FVector& ChildToThisJointDirection = IKJointWorkDatas[i].Transform.GetLocation() - IKJointWorkDatas[i - 1].Transform.GetLocation();
				FVector ExpandDiff = ChildToThisJointDirection;
				ExpandDiff *= (ChildToThisJointDirection.Size() - IKJointWorkDatas[i].OriginalJointLengthToChild) / ChildToThisJointDirection.Size();

				// �����ł�Location���C�����邾����Rotation�͏C�����Ȃ�
				IKJointWorkDatas[i].Transform.SetLocation(IKJointWorkDatas[i].Transform.GetLocation() - ExpandDiff);
			}
		}

		// Forward
		{
			// IK�̃��[�g�W���C���g�̈ʒu�����Ƃɖ߂�
			IKJointWorkDatas.Last().Transform.SetLocation(IKRootJointOriginalLocation);

			// ���C�e���[�V�����A�G�t�F�N�^�̒��߂̐e�W���C���g����IKRootJoint�Ɏw�肵���W���C���g�܂Ń��[�v����
			for (int32 i = IKJointWorkDatas.Num() - 1; i > 0; --i) // Effector�̈�e�̃W���C���g����AIK�����̃��[�g�ɐݒ肵���W���C���g�܂Ń��[�v����
			{
				const FVector& ChildToThisJointDirection = IKJointWorkDatas[i].Transform.GetLocation() - IKJointWorkDatas[i - 1].Transform.GetLocation();
				FVector ExpandDiff = ChildToThisJointDirection;
				ExpandDiff *= (ChildToThisJointDirection.Size() - IKJointWorkDatas[i].OriginalJointLengthToChild) / ChildToThisJointDirection.Size();

				// �����ł�Location���C�����邾����Rotation�͏C�����Ȃ�
				IKJointWorkDatas[i - 1].Transform.SetLocation(IKJointWorkDatas[i - 1].Transform.GetLocation() + ExpandDiff);
			}
		}

		// ��������
		const FVector& EffectorLocation = IKJointWorkDatas[0].Transform.GetLocation();
		const FVector& IKRootLocation = IKJointWorkDatas.Last().Transform.GetLocation();
		if ((EffectorLocation - EffectorTargetLocation).Size() < Precision && (IKRootLocation - IKRootJointOriginalLocation).Size() < Precision)
		{
			break;
		}
	}

	UE_CLOG(iterCount >= MaxIteration, LogAnimation, Warning, TEXT("Did not converge at MaxIteration set."));

	// �{�[���C���f�b�N�X�̏����ɓn�������̂ŋt���Ƀ��[�v����
	for (int32 i = IKJointWorkDatas.Num() - 1; i >= 0; --i)
	{
		OutBoneTransforms.Add(FBoneTransform(IKJointWorkDatas[i].BoneIndex, IKJointWorkDatas[i].Transform));
	}
}

bool FAnimNode_MyFABRIK::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
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
		// IKRootJoint��EffectorJoint����c�q���֌W�ɂȂ��ĂȂ�
		return false;
	}

	return (IKJointWorkDatas.Num() > 0); // InitializeBoneReferences�͊m���R���p�C�����ɌĂ΂�邯�ǂ������͎��s���ɌĂ΂��
}

void FAnimNode_MyFABRIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	IKRootJoint.Initialize(RequiredBones);
	EffectorJoint.Initialize(RequiredBones);

	// EffectorJoint����AIKRootJoint�܂ŁAIKRootJoint�ɂԂ���Ȃ���΃��[�g�W���C���g�܂Ń��[�N�f�[�^����������
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
