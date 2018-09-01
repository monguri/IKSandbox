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
	uint32 iterCount = 0;
	for (; iterCount < MaxIteration; ++iterCount)
	{
		// �܂��̓G�t�F�N�^�̈ʒu���w�肵���ʒu�Ɉړ�������
		IKJointWorkDatas[0].Transform.SetLocation(EffectorTargetLocation);

		bool FinishIteration = false;
		// ���C�e���[�V�����A�G�t�F�N�^�̒��߂̐e�W���C���g����IKRootJoint�Ɏw�肵���W���C���g�܂Ń��[�v����
		for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i) // Effector�̈�e�̃W���C���g����AIK�����̃��[�g�ɐݒ肵���W���C���g�܂Ń��[�v����
		{
			const FVector& ChildToThisJointDirection = IKJointWorkDatas[i].Transform.GetLocation() - IKJointWorkDatas[i - 1].Transform.GetLocation();
			FVector ExpandDiff = ChildToThisJointDirection;
			ExpandDiff *= (ChildToThisJointDirection.Size() - IKJointWorkDatas[i].OriginalJointLengthToChild) / ChildToThisJointDirection.Size() * 0.5f;

			// �����ł�Location���C�����邾����Rotation�͏C�����Ȃ�
			IKJointWorkDatas[i].Transform.SetLocation(IKJointWorkDatas[i].Transform.GetLocation() - ExpandDiff);
			IKJointWorkDatas[i - 1].Transform.SetLocation(IKJointWorkDatas[i - 1].Transform.GetLocation() + ExpandDiff);
		}

		// IK�̃��[�g�W���C���g�̈ʒu�����Ƃɖ߂�
		IKJointWorkDatas.Last().Transform.SetLocation(IKRootJointOriginalLocation);

		// ��������
		const FVector& EffectorLocation = IKJointWorkDatas[0].Transform.GetLocation();
		if ((EffectorLocation - EffectorTargetLocation).Size() < Precision)
		{
			// �G�t�F�N�^�̌��݈ʒu�ƖڕW�ʒu���\���������Ȃ�����ŁA�e�W���C���g�Ԃ̒������I���W�i���̒����Ƃق��ڈ�v���Ă�����C�e���[�V�����r���ł��I��
			// �W���C���g�̒����̂���ɂ������Ȃ��Ȃ�G�t�F�N�^�ʒu�����Ŏ�����������Ă�������
			// TODO:�W���C���g�����܂Ŕ���Ɋ܂߂�ƁA����̃R�X�g�����Ȃ�傫���Ȃ�
			bool FinishIteration = true;
			for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i) // Effector�̈�e�̃W���C���g����AIK�����̃��[�g�ɐݒ肵���W���C���g�܂Ń��[�v����
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

	// UE4�̏ꍇ�̓R���|�[�l���g���W�ł�SRT���烍�[�J����SRT��ComponentToLocal�m�[�h���v�Z���Ă���邽�߁A��]�ɑ΂��Ď��������K�v���Ȃ�
	// IK��������O�̉�]�p�̂܂܁ATranslation�̂ݑ啝��IK�ɂ���ĕς���Ă���̂��s���R�Ɋ�����Ȃ�Rotation�̒������K�v

	// �{�[���C���f�b�N�X�̏����ɓn�������̂ŋt���Ƀ��[�v����
	for (int32 i = IKJointWorkDatas.Num() - 1; i >= 0; --i)
	{
		OutBoneTransforms.Add(FBoneTransform(IKJointWorkDatas[i].BoneIndex, IKJointWorkDatas[i].Transform));
	}
}

bool FAnimNode_ParticleIK::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	//TODO: CCD-IK�Ɠ����Ȃ̂ŋ��ʉ�������
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

void FAnimNode_ParticleIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
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
