// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_JacobianIK.h"
#include "Animation/AnimInstanceProxy.h"

namespace
{
// XYZ��3�����Ɏg���萔
const int32 AXIS_COUNT = 3;
// ��]������3�����Ɏg���萔
const int32 ROTATION_AXIS_COUNT = 3;

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

float FAnimNode_JacobianIK::AnySizeMatrix::Get(uint8 Row, uint8 Column) const
{
	return Elements[Row * NumColumn + Column];
}

void FAnimNode_JacobianIK::AnySizeMatrix::Set(uint8 Row, uint8 Column, float Value)
{
	// TODO:operator[][]�̉��Z�q�I�[�o�[���C�h�͍��Ȃ����ȁH
	Elements[Row * NumColumn + Column] = Value;
}

void FAnimNode_JacobianIK::AnySizeMatrix::ZeroClear()
{
	FMemory::Memzero(Elements.GetData(), sizeof(float) * NumRow * NumColumn);
}

void FAnimNode_JacobianIK::AnySizeMatrix::Transpose(const AnySizeMatrix& InMatrix, AnySizeMatrix& OutMatrix)
{
	for (int32 Row = 0; Row < InMatrix.NumRow; ++Row)
	{
		for (int32 Column = 0; Column < InMatrix.NumColumn; ++Column)
		{
			OutMatrix.Set(Column, Row, InMatrix.Get(Row, Column));
		}
	}
}

void FAnimNode_JacobianIK::AnySizeMatrix::Multiply(const AnySizeMatrix& A, const AnySizeMatrix& B, AnySizeMatrix& OutResult)
{
	check(A.NumColumn == B.NumRow);
	check(OutResult.NumRow == A.NumRow);
	check(OutResult.NumColumn == B.NumColumn);

	for (int32 Row = 0; Row < OutResult.NumRow; ++Row)
	{
		for (int32 Column = 0; Column < OutResult.NumColumn; ++Column)
		{
			OutResult.Set(Row, Column, 0);

			for (int32 i = 0; i < A.NumColumn; ++i)
			{
				OutResult.Set(Row, Column, OutResult.Get(Row, Column) + A.Get(Row, i) * B.Get(i, Column));
			}
		}
	}
}

float FAnimNode_JacobianIK::AnySizeMatrix::Inverse3x3(const AnySizeMatrix& InMatrix, AnySizeMatrix& OutMatrix)
{
	float Determinant =
		  InMatrix.Get(0, 0) * InMatrix.Get(1, 1) * InMatrix.Get(2, 2)
		+ InMatrix.Get(1, 0) * InMatrix.Get(2, 1) * InMatrix.Get(0, 2)
		+ InMatrix.Get(2, 0) * InMatrix.Get(0, 1) * InMatrix.Get(1, 2)
		- InMatrix.Get(0, 0) * InMatrix.Get(2, 1) * InMatrix.Get(1, 2)
		- InMatrix.Get(2, 0) * InMatrix.Get(1, 1) * InMatrix.Get(0, 2)
		- InMatrix.Get(1, 0) * InMatrix.Get(0, 1) * InMatrix.Get(2, 2);

	if (Determinant == 0)
	{
		return Determinant;
	}

	OutMatrix.Set(0, 0, (InMatrix.Get(1, 1) * InMatrix.Get(2, 2) - InMatrix.Get(1, 2) * InMatrix.Get(2, 1)) / Determinant);
	OutMatrix.Set(0, 1, (InMatrix.Get(0, 2) * InMatrix.Get(2, 1) - InMatrix.Get(0, 1) * InMatrix.Get(2, 2)) / Determinant);
	OutMatrix.Set(0, 2, (InMatrix.Get(0, 1) * InMatrix.Get(1, 2) - InMatrix.Get(0, 2) * InMatrix.Get(1, 1)) / Determinant);

	OutMatrix.Set(1, 0, (InMatrix.Get(1, 2) * InMatrix.Get(2, 0) - InMatrix.Get(1, 0) * InMatrix.Get(2, 2)) / Determinant);
	OutMatrix.Set(1, 1, (InMatrix.Get(0, 0) * InMatrix.Get(2, 2) - InMatrix.Get(0, 2) * InMatrix.Get(2, 0)) / Determinant);
	OutMatrix.Set(1, 2, (InMatrix.Get(0, 2) * InMatrix.Get(1, 0) - InMatrix.Get(0, 0) * InMatrix.Get(1, 2)) / Determinant);

	OutMatrix.Set(2, 0, (InMatrix.Get(1, 0) * InMatrix.Get(2, 1) - InMatrix.Get(1, 1) * InMatrix.Get(2, 0)) / Determinant);
	OutMatrix.Set(2, 1, (InMatrix.Get(0, 1) * InMatrix.Get(2, 0) - InMatrix.Get(0, 0) * InMatrix.Get(2, 1)) / Determinant);
	OutMatrix.Set(2, 2, (InMatrix.Get(0, 0) * InMatrix.Get(1, 1) - InMatrix.Get(0, 1) * InMatrix.Get(1, 0)) / Determinant);

	return Determinant;
}

void FAnimNode_JacobianIK::AnySizeMatrix::TransformVector(const AnySizeMatrix& InMatrix, const TArray<float>& InVector, TArray<float>& OutVector)
{
	check((int32)InMatrix.NumRow == InVector.Num());
	check((int32)InMatrix.NumColumn == OutVector.Num());

	for (int32 Column = 0; Column < OutVector.Num(); ++Column)
	{
		OutVector[Column] = 0.0f;
	}

	for (int32 Column = 0; Column < InMatrix.NumColumn; ++Column)
	{
		for (int32 Row = 0; Row < InMatrix.NumRow; ++Row)
		{
			OutVector[Column] += InVector[Row] * InMatrix.Get(Row, Column);
		}
	}
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
		WorkData.ComponentTransform = Output.Pose.GetComponentSpaceTransform(WorkData.BoneIndex);
	}
	// �G�t�F�N�^��LocalTransform�̓C�e���[�V�����̒��ōX�V���Ȃ��̂ł����Ōv�Z���Ă���
	IKJointWorkDatas[0].LocalTransform = Output.Pose.GetLocalSpaceTransform(IKJointWorkDatas[0].BoneIndex);

	// �m�[�h�̓��͂��ꂽ�G�t�F�N�^�̈ʒu����ڕW�ʒu�ւ̍����x�N�g��
	const FVector& DeltaLocation = EffectorTargetLocation - Output.Pose.GetComponentSpaceTransform(IKJointWorkDatas[0].BoneIndex).GetLocation();
	// ���C�e���[�V�����ł̈ʒu�ړ�
	const FVector& IterationStep = DeltaLocation / NumIteration;
	IterationStepPosition[0] = IterationStep.X;
	IterationStepPosition[1] = IterationStep.Y;
	IterationStepPosition[2] = IterationStep.Z;

	// JacobianIK�̃��C���A���S���Y��
	// JacobianIK�A���S���Y���ɂ��Ă�Computer Graphics Gems JP 2012��8�͂��Q��
	uint32 iterCount = 0;
	for (; iterCount < NumIteration; ++iterCount)
	{
		// Jacobian�̌v�Z
		{
			Jacobian.ZeroClear(); // �ŏI�I�ɑS�v�f�ɒl������̂�0�N���A����K�v�͂Ȃ����f�o�b�O�̂��₷���̂��߂�0�N���A����

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
						IKJointWorkDatas[i].LocalTransform = FTransform::Identity;
					}
					else // IK���[�g�W���C���g�ɐe�̃W���C���g������Ƃ�
					{
						IKJointWorkDatas[i].LocalTransform = IKJointWorkDatas[i].ComponentTransform * Output.Pose.GetComponentSpaceTransform(IKRootJointParent).Inverse(); 
					}
				}
				else
				{
					IKJointWorkDatas[i].LocalTransform = IKJointWorkDatas[i].ComponentTransform * IKJointWorkDatas[i + 1].ComponentTransform.Inverse();
				}

				const FRotator& LocalRotation = IKJointWorkDatas[i].LocalTransform.GetRotation().Rotator();

				// �W���C���g�̃��[�J���s��̉�]����������s��ɒu�����������̂����߂�

				FMatrix LocalTranslateMatrix = FMatrix::Identity;
				LocalTranslateMatrix = LocalTranslateMatrix.ConcatTranslation(IKJointWorkDatas[i].LocalTransform.GetTranslation());

				FMatrix LocalMatrix[ROTATION_AXIS_COUNT];
				LocalMatrix[0] = (RotationDifferentialX(LocalRotation.Roll) * RotationY(LocalRotation.Pitch) * RotationZ(LocalRotation.Yaw)) * LocalTranslateMatrix;
				LocalMatrix[1] = (RotationX(LocalRotation.Roll) * RotationDifferentialY(LocalRotation.Pitch) * RotationZ(LocalRotation.Yaw)) * LocalTranslateMatrix;
				LocalMatrix[2] = (RotationX(LocalRotation.Roll) * RotationY(LocalRotation.Pitch) * RotationDifferentialZ(LocalRotation.Yaw)) * LocalTranslateMatrix;

				// �W���C���g�̍��W�n���猩�����݂̃G�t�F�N�^�ʒu�����߂�
				const FMatrix& ChildRestMatrix = (IKJointWorkDatas[0].ComponentTransform.ToMatrixWithScale() * IKJointWorkDatas[i].ComponentTransform.Inverse().ToMatrixWithScale());
				FMatrix ParentRestMatrix;
				if (i == IKJointWorkDatas.Num() - 1) // IK���[�g�W���C���g�̂Ƃ�
				{
					if (IKRootJointParent.GetInt() == INDEX_NONE) // IK���[�g�W���C���g���X�P���g���̃��[�g�̂Ƃ�
					{
						ParentRestMatrix = FMatrix::Identity;
					}
					else // IK���[�g�W���C���g�ɐe�̃W���C���g������Ƃ�
					{
						ParentRestMatrix = Output.Pose.GetComponentSpaceTransform(IKRootJointParent).ToMatrixWithScale(); 
					}
				}
				else
				{
					ParentRestMatrix = IKJointWorkDatas[i + 1].ComponentTransform.ToMatrixWithScale();
				}

				for (int32 RotAxis = 0; RotAxis < ROTATION_AXIS_COUNT; ++RotAxis)
				{
					const FVector& JacobianRow = (ChildRestMatrix * LocalMatrix[RotAxis] * ParentRestMatrix).TransformPosition(FVector::ZeroVector);
					Jacobian.Set((i - 1) * ROTATION_AXIS_COUNT + RotAxis, 0, JacobianRow.X);
					Jacobian.Set((i - 1) * ROTATION_AXIS_COUNT + RotAxis, 1, JacobianRow.Y);	
					Jacobian.Set((i - 1) * ROTATION_AXIS_COUNT + RotAxis, 2, JacobianRow.Z);	
				}
			}
		}

		// Jacobian�̋^���t�s��̌v�Z
		{
			// TODO:�v�Z���s�D��ɂȂ��ĂȂ��̂ł́H
			Jt.ZeroClear(); // �ŏI�I�ɑS�v�f�ɒl������̂�0�N���A����K�v�͂Ȃ����f�o�b�O�̂��₷���̂��߂�0�N���A����
			AnySizeMatrix::Transpose(Jacobian, Jt);

			JJt.ZeroClear(); // �ŏI�I�ɑS�v�f�ɒl������̂�0�N���A����K�v�͂Ȃ����f�o�b�O�̂��₷���̂��߂�0�N���A����
			AnySizeMatrix::Multiply(Jt, Jacobian, JJt);

			JJti.ZeroClear(); // �ŏI�I�ɑS�v�f�ɒl������̂�0�N���A����K�v�͂Ȃ����f�o�b�O�̂��₷���̂��߂�0�N���A����
			float Determinant = AnySizeMatrix::Inverse3x3(JJt, JJti);
			//if (FMath::Abs(Determinant) < KINDA_SMALL_NUMBER)
			if (FMath::Abs(Determinant) < SMALL_NUMBER)
			{
				return;
			}

			PseudoInverseJacobian.ZeroClear(); // �ŏI�I�ɑS�v�f�ɒl������̂�0�N���A����K�v�͂Ȃ����f�o�b�O�̂��₷���̂��߂�0�N���A����
			AnySizeMatrix::Multiply(JJti, Jt, PseudoInverseJacobian);

			// TODO:�Ƃ肠�����֐ߊp�ψʖڕW�l�т�0�ɂ��Ă����A�v�Z���Ȃ�
		}

		// ��]�p�ψʂ����߁A���[�N�f�[�^�̌��݊p���X�V����
		{
			AnySizeMatrix::TransformVector(PseudoInverseJacobian, IterationStepPosition, IterationStepAngles);

			// �G�t�F�N�^��IKJointWorkDatas[0].LocalTransform�͏����l��FTransform::Identity�̂܂�
			for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i)
			{
				FRotator LocalRotation = IKJointWorkDatas[i].LocalTransform.Rotator();
				LocalRotation.Roll += IterationStepAngles[0];
				LocalRotation.Pitch += IterationStepAngles[1];
				LocalRotation.Yaw += IterationStepAngles[2];
				IKJointWorkDatas[i].LocalTransform.SetRotation(FQuat(LocalRotation));
			}

			// �e���珇��LocalTransform�̕ω���ComponentTransform�ɔ��f���Ă����̂ŋt���Ƀ��[�v����
			for (int32 i = IKJointWorkDatas.Num() - 1; i >= 0; --i)
			{
				FTransform ParentTransform;
				if (i == IKJointWorkDatas.Num() - 1) // IK���[�g�W���C���g�̂Ƃ�
				{
					if (IKRootJointParent.GetInt() == INDEX_NONE) // IK���[�g�W���C���g���X�P���g���̃��[�g�̂Ƃ�
					{
						ParentTransform = FTransform::Identity;
					}
					else // IK���[�g�W���C���g�ɐe�̃W���C���g������Ƃ�
					{
						ParentTransform = Output.Pose.GetComponentSpaceTransform(IKRootJointParent);
					}
				}
				else
				{
					ParentTransform = IKJointWorkDatas[i + 1].ComponentTransform;
				}

				IKJointWorkDatas[i].ComponentTransform = IKJointWorkDatas[i].LocalTransform * ParentTransform;
			}
		}
	}

	// �{�[���C���f�b�N�X�̏����ɓn�������̂ŋt���Ƀ��[�v����
	for (int32 i = IKJointWorkDatas.Num() - 1; i >= 0; --i)
	{
		OutBoneTransforms.Add(FBoneTransform(IKJointWorkDatas[i].BoneIndex, IKJointWorkDatas[i].ComponentTransform));
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

	return (IKJointWorkDatas.Num() >= 2); // �Œ���G�t�F�N�^�ƁA���IK����W���C���g���Ȃ��ƈӖ����Ȃ�
}

void FAnimNode_JacobianIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	IKRootJoint.Initialize(RequiredBones);
	EffectorJoint.Initialize(RequiredBones);

	IKRootJointParent = FCompactPoseBoneIndex(RequiredBones.GetParentBoneIndex(IKRootJoint.BoneIndex));

	// EffectorJoint����AIKRootJoint�܂ŁAIKRootJoint�ɂԂ���Ȃ���΃��[�g�W���C���g�܂Ń��[�N�f�[�^����������
	FCompactPoseBoneIndex IKJointIndex = EffectorJoint.GetCompactPoseIndex(RequiredBones);

	while (IKJointIndex != INDEX_NONE && IKJointIndex != IKRootJoint.BoneIndex)
	{
		IKJointWorkDatas.Emplace(IKJointIndex, FTransform::Identity, FTransform::Identity);

		IKJointIndex = RequiredBones.GetParentBoneIndex(IKJointIndex);
	}

	if (IKJointIndex == IKRootJoint.BoneIndex)
	{
		IKJointWorkDatas.Emplace(IKJointIndex, FTransform::Identity, FTransform::Identity);
	}

	Jacobian = AnySizeMatrix((IKJointWorkDatas.Num() - 1) * ROTATION_AXIS_COUNT, AXIS_COUNT);
	Jt = AnySizeMatrix(AXIS_COUNT, (IKJointWorkDatas.Num() - 1) * ROTATION_AXIS_COUNT);
	JJt = AnySizeMatrix(AXIS_COUNT, AXIS_COUNT); // J * Jt
	JJti = AnySizeMatrix(AXIS_COUNT, AXIS_COUNT); // (J^t * J)^-1
	PseudoInverseJacobian = AnySizeMatrix(AXIS_COUNT, (IKJointWorkDatas.Num() - 1) * ROTATION_AXIS_COUNT); // J^t * (J^t * J)^-1
	IterationStepPosition.SetNumZeroed(AXIS_COUNT);
	IterationStepAngles.SetNumZeroed((IKJointWorkDatas.Num() - 1) * ROTATION_AXIS_COUNT);
}
