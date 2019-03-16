// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_JacobianIK.h"
#include "Animation/AnimInstanceProxy.h"

FAnimNode_JacobianIK::FAnimNode_JacobianIK()
	: EffectorTargetLocation(0.0f, 0.0f, 0.0f)
	, NumIteration(10)
	, Precision(SMALL_NUMBER)
	, IKRootJointParent(INDEX_NONE)
	, Lambda(0.0f)
{
}

void FAnimNode_JacobianIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
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

			JJtPlusLambdaI.ZeroClear();
			AnySizeMatrix::Add(JJt, LambdaI, JJtPlusLambdaI);

			JJti.ZeroClear(); // �ŏI�I�ɑS�v�f�ɒl������̂�0�N���A����K�v�͂Ȃ����f�o�b�O�̂��₷���̂��߂�0�N���A����
#if 0
			float Determinant = AnySizeMatrix::Inverse3x3(JJt, JJti);
#else
			float Determinant = AnySizeMatrix::Inverse3x3(JJtPlusLambdaI, JJti);
#endif
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
				LocalRotation.Roll += FMath::RadiansToDegrees(IterationStepAngles[(i - 1) * ROTATION_AXIS_COUNT + 0]);
				LocalRotation.Pitch += FMath::RadiansToDegrees(IterationStepAngles[(i - 1) * ROTATION_AXIS_COUNT + 1]);
				LocalRotation.Yaw += FMath::RadiansToDegrees(IterationStepAngles[(i - 1) * ROTATION_AXIS_COUNT + 2]);
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

