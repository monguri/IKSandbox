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
	check(OutBoneTransforms.Num() == 0);

	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();

	// TODO:�eIK�m�[�h�Ƌ��ʉ����悤
	// ���������W���C���g�̒����I��IK�̉��ɓ��B�����邩�̊m�F
	float EffectorToIKRootLength = (IKJointWorkDatas.Last().Transform.GetLocation() - IKJointWorkDatas[0].Transform.GetLocation()).Size();
	float IKJointTotalLength = 0; // �A�j���[�V������Scale���Ȃ��Ȃ�A��x�����v�Z���ăL���b�V�����Ă����΂悢���A���͖���v�Z����
	for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i)
	{
		IKJointTotalLength += (IKJointWorkDatas[i].Transform.GetLocation() - IKJointWorkDatas[i - 1].Transform.GetLocation()).Size();
	}

	if (IKJointTotalLength < EffectorToIKRootLength)
	{
		UE_LOG(LogAnimation, Warning, TEXT("IK cannot reach effector target location. The total length of joints is not enough."));
		return;
	}

	// ���[�N�f�[�^��Transform�̏�����
	for (IKJointWorkData& WorkData : IKJointWorkDatas)
	{
		WorkData.Transform = Output.Pose.GetComponentSpaceTransform(WorkData.BoneIndex);
	}

	// CCDIK�̃��C���A���S���Y��
	uint32 iterCount = 0;
	for (; iterCount < MaxIteration; ++iterCount)
	{
		bool FinishIteration = false;
		// ���C�e���[�V�����A�G�t�F�N�^�̒��߂̐e�W���C���g����IKRootJoint�Ɏw�肵���W���C���g�܂Ń��[�v����
		for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i)
		{
			const FVector& EffectorLocation = IKJointWorkDatas[0].Transform.GetLocation();
			if ((EffectorLocation - EffectorTargetLocation).Size() < Precision)
			{
				// �G�t�F�N�^�̌��݈ʒu�ƖڕW�ʒu���\���������Ȃ�����C�e���[�V�����r���ł��I��
				FinishIteration = true;
				break;
			}

			// CCD�̊e�W���C���g�ł̉�]�C���̌v�Z
			const FVector& IKJointCurrentLocation = IKJointWorkDatas[i].Transform.GetLocation();

			const FVector& IKJointToEffectorDirection = EffectorLocation - IKJointCurrentLocation;
			const FVector& IKJointToEffectorTargetDirection = EffectorTargetLocation - IKJointCurrentLocation;

			const FQuat& DiffRotation = FQuat::FindBetweenVectors(IKJointToEffectorDirection, IKJointToEffectorTargetDirection);

			// ��]�C����Effector�܂ł̂��ׂĂ̎q�̃R���|�[�l���g���W�ł�Rotation��Location�ɔ��f������
			// TODO:����͍l�����Ă��Ȃ�
			for (int32 j = i - 1; j >= 0; --j)
			{
				IKJointWorkDatas[j].Transform.SetRotation(DiffRotation * IKJointWorkDatas[j].Transform.GetRotation());
				IKJointWorkDatas[j].Transform.SetLocation(IKJointCurrentLocation + DiffRotation * (IKJointWorkDatas[j].Transform.GetLocation() - IKJointCurrentLocation));
			}
		}

		if (FinishIteration)
		{
			// �G�t�F�N�^�̌��݈ʒu�ƖڕW�ʒu���\���������Ȃ�����C�e���[�V�����r���ł��I��
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
		// IKRootJoint��EffectorJoint����c�q���֌W�ɂȂ��ĂȂ�
		return false;
	}

	return (IKJointWorkDatas.Num() > 0); // InitializeBoneReferences�͊m���R���p�C�����ɌĂ΂�邯�ǂ������͎��s���ɌĂ΂��
}

void FAnimNode_CCDIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	IKRootJoint.Initialize(RequiredBones);
	EffectorJoint.Initialize(RequiredBones);

	// EffectorJoint����AIKRootJoint�܂ŁAIKRootJoint�ɂԂ���Ȃ���΃��[�g�W���C���g�܂Ń��[�N�f�[�^����������
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
