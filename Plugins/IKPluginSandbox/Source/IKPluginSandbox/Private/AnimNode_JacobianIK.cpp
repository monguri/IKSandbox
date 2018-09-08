// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_JacobianIK.h"
#include "Animation/AnimInstanceProxy.h"

namespace
{
// ��]������3�����AXYZ��3�����Ɏg���萔
const int32 AXIS_COUNT = 3;

// degree�P�ʂł�sin/cos�֐�
float SinD(float degree) { return FMath::Sin(FMath::DegreesToRadians(degree)); }
float CosD(float degree) { return FMath::Cos(FMath::DegreesToRadians(degree)); }

// �e���̉�]�s��
//TODO: �����̕�����A�s�D���D��ɂ��Ă͂��܂肫����ƍl���͂��Ă��炸�ق�������̃\�[�X���Q�l�ɂ��Ă���
FMatrix RotationX(float Roll)
{
	return FMatrix(
		FPlane(1, 0, 0, 0),
		FPlane(0, CosD(Roll), -SinD(Roll), 0),
		FPlane(0, SinD(Roll), CosD(Roll), 0),
		FPlane(0, 0, 0, 1)
	);
}

FMatrix RotationY(float Pitch)
{
	return FMatrix(
		FPlane(CosD(Pitch), 0, SinD(Pitch), 0),
		FPlane(0, 1, 0, 0),
		FPlane(-SinD(Pitch), 0, CosD(Pitch), 0),
		FPlane(0, 0, 0, 1)
	);
}

FMatrix RotationZ(float Yaw)
{
	return FMatrix(
		FPlane(CosD(Yaw), SinD(Yaw), 0, 0),
		FPlane(-SinD(Yaw), CosD(Yaw), 0, 0),
		FPlane(0, 0, 1, 0),
		FPlane(0, 0, 0, 1)
	);
}

// �e���̉�]�s�����]�p�̕ϐ��Ŕ��������s��
FMatrix RotationDifferentialX(float Roll)
{
	return FMatrix(
		FPlane(0, 0, 0, 0),
		FPlane(0, -SinD(Roll), -CosD(Roll), 0),
		FPlane(0, CosD(Roll), -SinD(Roll), 0),
		FPlane(0, 0, 0, 0)
	);
}

FMatrix RotationDifferentialY(float Pitch)
{
	return FMatrix(
		FPlane(-SinD(Pitch), 0, CosD(Pitch), 0),
		FPlane(0, 0, 0, 0),
		FPlane(-CosD(Pitch), 0, -SinD(Pitch), 0),
		FPlane(0, 0, 0, 0)
	);
}

FMatrix RotationDifferentialZ(float Yaw)
{
	return FMatrix(
		FPlane(-SinD(Yaw), CosD(Yaw), 0, 0),
		FPlane(-CosD(Yaw), -SinD(Yaw), 0, 0),
		FPlane(0, 0, 0, 0),
		FPlane(0, 0, 0, 0)
	);
}
} // namespace

FAnimNode_JacobianIK::AnySizeMatrix::AnySizeMatrix()
{
}

FAnimNode_JacobianIK::AnySizeMatrix::AnySizeMatrix(uint8 _NumRow, uint8 _NumColumn)
{
	Elements.AddZeroed(_NumRow * _NumColumn);
	NumRow = _NumRow;
	NumColumn = _NumColumn;
}

void FAnimNode_JacobianIK::AnySizeMatrix::Set(uint8 Row, uint8 Column, float Value)
{
	// TODO:operator[][]�̉��Z�q�I�[�o�[���C�h�͍��Ȃ����ȁH
	Elements[Row * NumColumn + Column] = Value;
}

void FAnimNode_JacobianIK::AnySizeMatrix::ZeroClear()
{
	memset(Elements.GetData(), 0, NumRow * NumColumn);
}

FAnimNode_JacobianIK::FAnimNode_JacobianIK()
	: EffectorTargetLocation(0.0f, 0.0f, 0.0f)
	, NumIteration(10)
	, Precision(SMALL_NUMBER)
	, IKRootJointParent(INDEX_NONE)
{
}

void FAnimNode_JacobianIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
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

	// ���[�N�f�[�^��Transform�̏�����
	for (IKJointWorkData& WorkData : IKJointWorkDatas)
	{
		WorkData.Transform = Output.Pose.GetComponentSpaceTransform(WorkData.BoneIndex);
	}

	// JacobianIK�̃��C���A���S���Y��
	// JacobianIK�A���S���Y���ɂ��Ă�Computer Graphics Gems JP 2012��8�͂��Q��
	uint32 iterCount = 0;
	for (; iterCount < NumIteration; ++iterCount)
	{
		// Jacobian�̌v�Z
		{
			Jacobian.ZeroClear();

			// �G�t�F�N�^�̐e����IK���[�g�W���C���g�܂ł���]�̓��͂Ƃ���̂ł���炩��Jacobian�����
			for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i)
			{
				// Jacobian�̃W���C���g�ɑΉ�����s�����߂�
				// �v�Z���@�ɂ��Ă�Computer Graphics Gems JP 2012��8�͂��Q�Ƃ̂���

				FTransform LocalTransform;
				if (i == IKJointWorkDatas.Num() - 1) // IK���[�g�W���C���g�̂Ƃ�
				{
					if (IKRootJointParent.GetInt() == INDEX_NONE) // IK���[�g�W���C���g���X�P���g���̃��[�g�̂Ƃ�
					{
						LocalTransform = FTransform::Identity;
					}
					else // IK���[�g�W���C���g�ɐe�̃W���C���g������Ƃ�
					{
						LocalTransform = IKJointWorkDatas[i].Transform * Output.Pose.GetComponentSpaceTransform(IKRootJointParent).Inverse(); 
					}
				}
				else
				{
					LocalTransform = IKJointWorkDatas[i].Transform * IKJointWorkDatas[i + 1].Transform.Inverse();
				}

				const FRotator& LocalRotation = LocalTransform.GetRotation().Rotator();

				// �W���C���g�̃��[�J���s��̉�]����������s��ɒu�����������̂����߂�
				FTransform LocalTransformRotationDifferential[AXIS_COUNT]; // ��]��3�v�f���ꂼ��ł̔���
				LocalTransformRotationDifferential[0] = LocalTransformRotationDifferential[1] = LocalTransformRotationDifferential[2] = LocalTransform;

				LocalTransformRotationDifferential[0].SetRotation((RotationDifferentialX(LocalRotation.Roll) * RotationY(LocalRotation.Pitch) * RotationZ(LocalRotation.Yaw)).ToQuat());
				LocalTransformRotationDifferential[1].SetRotation((RotationX(LocalRotation.Roll) * RotationDifferentialY(LocalRotation.Pitch) * RotationZ(LocalRotation.Yaw)).ToQuat());
				LocalTransformRotationDifferential[2].SetRotation((RotationX(LocalRotation.Roll) * RotationY(LocalRotation.Pitch) * RotationDifferentialZ(LocalRotation.Yaw)).ToQuat());

				// �W���C���g�̍��W�n���猩�����݂̃G�t�F�N�^�ʒu�����߂�
				const FVector& EffectorLocationAtThisJointSpace = (IKJointWorkDatas[0].Transform * IKJointWorkDatas[i].Transform.Inverse()).TransformFVector4(FVector4(0, 0, 0, 1));
				const FTransform& ParentRestTransform = IKJointWorkDatas[i + 1].Transform;

				for (int32 Axis = 0; Axis < AXIS_COUNT; ++Axis)
				{
					const FVector& JacobianColumn = (LocalTransformRotationDifferential[0] * ParentRestTransform).TransformPosition(EffectorLocationAtThisJointSpace);
					Jacobian.Set((i - 1) * AXIS_COUNT + Axis, 0, JacobianColumn.X);	
					Jacobian.Set((i - 1) * AXIS_COUNT + Axis, 1, JacobianColumn.Y);	
					Jacobian.Set((i - 1) * AXIS_COUNT + Axis, 2, JacobianColumn.Z);	
				}
			}
		}
	}

	// �{�[���C���f�b�N�X�̏����ɓn�������̂ŋt���Ƀ��[�v����
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
		// IKRootJoint��EffectorJoint����c�q���֌W�ɂȂ��ĂȂ�
		return false;
	}

	IKRootJointParent = FCompactPoseBoneIndex(RequiredBones.GetParentBoneIndex(IKRootJoint.BoneIndex));

	return (IKJointWorkDatas.Num() > 0); // InitializeBoneReferences�͊m���R���p�C�����ɌĂ΂�邯�ǂ������͎��s���ɌĂ΂��
}

void FAnimNode_JacobianIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
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

	// ���R�r�A���̍s���́A�ڕW�l��ݒ肷��S�G�t�F�N�^�̏o�͕ϐ��̐��Ȃ̂ŁA���݂�1�����̈ʒu�w�肾���Ȃ̂�3
	// ���R�r�A���̗񐔂́AIK�̓��͕ϐ��̐��Ȃ̂Łi�W���C���g��-1�j�~��]�̎��R�x3�B-1�Ȃ̂̓G�t�F�N�^�̉�]���R�x�͎g��Ȃ�����
	Jacobian = AnySizeMatrix((IKJointWorkDatas.Num() - 1) * AXIS_COUNT, AXIS_COUNT);
}
