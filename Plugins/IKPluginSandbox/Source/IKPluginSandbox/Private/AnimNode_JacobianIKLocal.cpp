// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_JacobianIKLocal.h"
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

FAnimNode_JacobianIKLocal::AnySizeMatrix::AnySizeMatrix()
{
}

FAnimNode_JacobianIKLocal::AnySizeMatrix::AnySizeMatrix(uint8 _NumRow, uint8 _NumColumn)
{
	Elements.AddZeroed(_NumRow * _NumColumn);
	NumRow = _NumRow;
	NumColumn = _NumColumn;
}

float FAnimNode_JacobianIKLocal::AnySizeMatrix::Get(uint8 Row, uint8 Column) const
{
	return Elements[Row * NumColumn + Column];
}

void FAnimNode_JacobianIKLocal::AnySizeMatrix::Set(uint8 Row, uint8 Column, float Value)
{
	// TODO:operator[][]�̉��Z�q�I�[�o�[���C�h�͍��Ȃ����ȁH
	Elements[Row * NumColumn + Column] = Value;
}

void FAnimNode_JacobianIKLocal::AnySizeMatrix::ZeroClear()
{
	FMemory::Memzero(Elements.GetData(), sizeof(float) * NumRow * NumColumn);
}

void FAnimNode_JacobianIKLocal::AnySizeMatrix::Transpose(const AnySizeMatrix& InMatrix, AnySizeMatrix& OutMatrix)
{
	for (int32 Row = 0; Row < InMatrix.NumRow; ++Row)
	{
		for (int32 Column = 0; Column < InMatrix.NumColumn; ++Column)
		{
			OutMatrix.Set(Column, Row, InMatrix.Get(Row, Column));
		}
	}
}

void FAnimNode_JacobianIKLocal::AnySizeMatrix::Multiply(const AnySizeMatrix& A, const AnySizeMatrix& B, AnySizeMatrix& OutResult)
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

void FAnimNode_JacobianIKLocal::AnySizeMatrix::Add(const AnySizeMatrix& A, const AnySizeMatrix& B, AnySizeMatrix& OutResult)
{
	check(A.NumRow == B.NumRow);
	check(A.NumColumn == B.NumColumn);
	check(OutResult.NumRow == A.NumRow);
	check(OutResult.NumColumn == B.NumColumn);

	for (int32 Row = 0; Row < OutResult.NumRow; ++Row)
	{
		for (int32 Column = 0; Column < OutResult.NumColumn; ++Column)
		{
			OutResult.Set(Row, Column, A.Get(Row, Column) + B.Get(Row, Column));
		}
	}
}

float FAnimNode_JacobianIKLocal::AnySizeMatrix::Inverse3x3(const AnySizeMatrix& InMatrix, AnySizeMatrix& OutMatrix)
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

void FAnimNode_JacobianIKLocal::AnySizeMatrix::TransformVector(const AnySizeMatrix& InMatrix, const TArray<float>& InVector, TArray<float>& OutVector)
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

FAnimNode_JacobianIKLocal::FAnimNode_JacobianIKLocal()
	: EffectorTargetLocation(0.0f, 0.0f, 0.0f)
	, NumIteration(10)
	, Precision(SMALL_NUMBER)
	, IKRootJointParent(INDEX_NONE)
	, Lambda(0.0f)
{
}

void FAnimNode_JacobianIKLocal::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	bool Success = IKRootJoint.Initialize(RequiredBones);
	if (!Success)
	{
		return;
	}

	EffectorJoint.Initialize(RequiredBones);
	if (!Success)
	{
		return;
	}

	IKRootJointParent = FCompactPoseBoneIndex(RequiredBones.GetParentBoneIndex(IKRootJoint.BoneIndex));

	// EffectorJoint����AIKRootJoint�܂ŁAIKRootJoint�ɂԂ���Ȃ���΃��[�g�W���C���g�܂Ń��[�N�f�[�^����������
	FCompactPoseBoneIndex IKJointIndex = EffectorJoint.GetCompactPoseIndex(RequiredBones);

	IKJointWorkDatas.Reset(IKJointWorkDatas.Num());

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

	LambdaI = AnySizeMatrix(AXIS_COUNT, AXIS_COUNT); // lambda * I
	LambdaI.ZeroClear();
	LambdaI.Set(0, 0, Lambda);
	LambdaI.Set(1, 1, Lambda);
	LambdaI.Set(2, 2, Lambda);

	JJtPlusLambdaI = AnySizeMatrix(AXIS_COUNT, AXIS_COUNT); // J * J^t + lambda * I
	JJti = AnySizeMatrix(AXIS_COUNT, AXIS_COUNT); // (J * J^t + lambda * I)^-1
	PseudoInverseJacobian = AnySizeMatrix(AXIS_COUNT, (IKJointWorkDatas.Num() - 1) * ROTATION_AXIS_COUNT); // J^t * (J * J^t)^-1

	IterationStepPosition.SetNumZeroed(AXIS_COUNT);
	IterationStepAngles.SetNumZeroed((IKJointWorkDatas.Num() - 1) * ROTATION_AXIS_COUNT);
}

bool FAnimNode_JacobianIKLocal::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
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

void FAnimNode_JacobianIKLocal::EvaluateSkeletalControl_AnyThread(const FCSPose<FCompactPose>& InPose, FPoseContext& Output)
{
}
