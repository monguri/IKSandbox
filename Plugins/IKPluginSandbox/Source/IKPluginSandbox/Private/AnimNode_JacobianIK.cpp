// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_JacobianIK.h"
#include "Animation/AnimInstanceProxy.h"

FAnimNode_JacobianIK::FAnimNode_JacobianIK()
	: NumIteration(10)
	, Precision(SMALL_NUMBER)
	, Lambda(1.0f)
{
}

void FAnimNode_JacobianIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	for (FIKJoint& IKJoint : IKSkeleton)
	{
		bool Success = IKJoint.Joint.Initialize(RequiredBones);
		if (!Success)
		{
			// TODO:UE_LOG
			return;
		}

		Success = IKJoint.ParentJoint.Initialize(RequiredBones);
		if (!Success)
		{
			// TODO:UE_LOG
			return;
		}

		check(IKJoint.Joint.BoneIndex != INDEX_NONE);
		check(IKJoint.ParentJoint.BoneIndex != INDEX_NONE);
		// TODO:IKSkeleton���̃W���C���g�̏d���`�F�b�N�����Ȃ��Ƃ�
	}

	IKJointWorkDataMap.Reset();

	for (const FIKJoint& IKJoint : IKSkeleton)
	{
		IKJointWorkDataMap.Emplace(IKJoint.Joint.GetCompactPoseIndex(RequiredBones).GetInt(), IKJointWorkData(IKJoint.ParentJoint.GetCompactPoseIndex(RequiredBones), FTransform::Identity, FTransform::Identity, IKJoint.Constraints));
	}

	IKJointWorkDataMap.KeySort(TLess<int32>());

	Jacobian = AnySizeMatrix((IKJointWorkDataMap.Num() - 1) * ROTATION_AXIS_COUNT, AXIS_COUNT);
	Jt = AnySizeMatrix(AXIS_COUNT, (IKJointWorkDataMap.Num() - 1) * ROTATION_AXIS_COUNT);
	JJt = AnySizeMatrix(AXIS_COUNT, AXIS_COUNT); // J * Jt

	LambdaI = AnySizeMatrix(AXIS_COUNT, AXIS_COUNT); // lambda * I
	LambdaI.ZeroClear();
	LambdaI.Set(0, 0, Lambda);
	LambdaI.Set(1, 1, Lambda);
	LambdaI.Set(2, 2, Lambda);

	JJtPlusLambdaI = AnySizeMatrix(AXIS_COUNT, AXIS_COUNT); // J * J^t + lambda * I
	JJti = AnySizeMatrix(AXIS_COUNT, AXIS_COUNT); // (J * J^t + lambda * I)^-1
	PseudoInverseJacobian = AnySizeMatrix(AXIS_COUNT, (IKJointWorkDataMap.Num() - 1) * ROTATION_AXIS_COUNT); // J^t * (J * J^t)^-1

	IterationStepPosition.SetNumZeroed(AXIS_COUNT);
	IterationStepAngles.SetNumZeroed((IKJointWorkDataMap.Num() - 1) * ROTATION_AXIS_COUNT);
}

bool FAnimNode_JacobianIK::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	int32 NumRootIKJoint = 0;
	int32 NumConstraint = 0;
	bool isExistConstraint = false;

	//TODO; ���t���[�����肷��悤�Ȃ��Ƃł͂Ȃ��B�ꕔ��Initialize_AnyThread�AInitializeBoneReference�ł̃`�F�b�N�ŏ\��
	for (FIKJoint& IKJoint : IKSkeleton)
	{
		if (!IKJoint.Joint.IsValidToEvaluate(RequiredBones))
		{
			return false;
		}

		if (!IKJoint.ParentJoint.IsValidToEvaluate(RequiredBones))
		{
			return false;
		}

		if (IKJoint.Joint == IKJoint.ParentJoint)
		{
			NumRootIKJoint++;
		}

		if (IKJoint.Constraints.Num() > 0)
		{
			NumConstraint++;
		}
	}

	return ((NumRootIKJoint == 1) // ���[�g�ݒ�̃W���C���g�͕K�����
		&& (NumConstraint == 1) //TODO: �R���X�g���C���g�͈�Ƃ���������܂�����Ă���
		&& IKJointWorkDataMap.Num() >= 2); // �Œ���G�t�F�N�^�ƁA���IK����W���C���g���Ȃ��ƈӖ����Ȃ�
}

void FAnimNode_JacobianIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(Output.AnimInstanceProxy->GetSkelMeshComponent());
	check(OutBoneTransforms.Num() == 0);

	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();

	// TODO:����A�����R���X�g���C���g�ɑΉ��ł��ĂȂ��̂ł��ꂪ�B��Ή�����R���X�g���C���g
	FCompactPoseBoneIndex OnlyOneConstraintJoint(INDEX_NONE);
	FIKConstraint OnlyOneConstraint;

	// TODO:�eIK�m�[�h�Ƌ��ʉ����悤
	// ���������W���C���g�̒����I��IK�̉��ɓ��B�����邩�̊m�F
	// �R���X�g���C���g���ƂɊm�F����
	for (const TPair<int32, IKJointWorkData>& WorkData : IKJointWorkDataMap)
	{
		if (WorkData.Value.Constraints.Num() == 0)
		{
			continue;
		}

		//TODO: ����A�ʒu�Œ�R���X�g���C���g�����Ȃ�
		for (const FIKConstraint& Constraint : WorkData.Value.Constraints)
		{
			OnlyOneConstraintJoint = FCompactPoseBoneIndex(WorkData.Key);
			OnlyOneConstraint = Constraint;

			float IKJointTotalLength = 0; // ���̃A�j���[�V�����m�[�h�ɓ��͂����|�[�Y��Scale���Ȃ��Ȃ�A��x�����v�Z���ăL���b�V�����Ă����΂悢���A���͖���v�Z����
			// TODO:�����`�F�b�N�����ɂ��Ă͖��t���[���̌v�Z�R�X�g����������

			FCompactPoseBoneIndex CurrentJointIndex = FCompactPoseBoneIndex(WorkData.Key);
			FCompactPoseBoneIndex ParentJointIndex = WorkData.Value.ParentJointIndex;
			IKJointWorkData CurrentWorkData = WorkData.Value;
			IKJointWorkData ParentWorkData = IKJointWorkDataMap[ParentJointIndex.GetInt()];
			while (CurrentJointIndex != ParentJointIndex) // IKSkeleton���̃��[�g�ݒ�̃W���C���g�Ɏ���܂Ń��[�v
			{
				IKJointTotalLength += (Output.Pose.GetComponentSpaceTransform(ParentJointIndex).GetLocation() - Output.Pose.GetComponentSpaceTransform(CurrentJointIndex).GetLocation()).Size();

				CurrentJointIndex = ParentJointIndex;
				ParentJointIndex = ParentWorkData.ParentJointIndex;
				CurrentWorkData = ParentWorkData;
				ParentWorkData = IKJointWorkDataMap[ParentJointIndex.GetInt()];
			}

			float EffectorToIKRootLength = (Output.Pose.GetComponentSpaceTransform(CurrentJointIndex).GetLocation() - Output.Pose.GetComponentSpaceTransform(OnlyOneConstraintJoint).GetLocation()).Size();
			if (IKJointTotalLength < EffectorToIKRootLength)
			{
				UE_LOG(LogAnimation, Warning, TEXT("IK cannot reach effector target location. The total length of joints is not enough."));
				return;
			}
		}
	}

	// ���[�N�f�[�^��Transform�̏�����
	for (TPair<int32, IKJointWorkData>& WorkData : IKJointWorkDataMap)
	{
		WorkData.Value.ComponentTransform = Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(WorkData.Key));

		// �G�t�F�N�^��LocalTransform�̓C�e���[�V�����̒��ōX�V���Ȃ��̂ł����Ōv�Z���Ă���
		//TODO: ����A�ʒu�Œ�R���X�g���C���g�����Ȃ�
		if (WorkData.Value.Constraints.Num() > 0)
		{
			WorkData.Value.LocalTransform = Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(WorkData.Key)) * Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(WorkData.Value.ParentJointIndex)).Inverse();
		}
	}

	// �m�[�h�̓��͂��ꂽ�G�t�F�N�^�̈ʒu����ڕW�ʒu�ւ̍����x�N�g��
	const FVector& DeltaLocation = OnlyOneConstraint.Position - Output.Pose.GetComponentSpaceTransform(OnlyOneConstraintJoint).GetLocation();
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
			int32 i = 0;
			for (TPair<int32, IKJointWorkData>& WorkDataPair : IKJointWorkDataMap)
			{
				if (WorkDataPair.Key == OnlyOneConstraintJoint.GetInt())
				{
					continue;
				}

				// Jacobian�̃W���C���g�ɑΉ�����s�����߂�
				// �v�Z���@�ɂ��Ă�Computer Graphics Gems JP 2012��8�͂��Q�Ƃ̂���
				const FCompactPoseBoneIndex& JointIndex = FCompactPoseBoneIndex(WorkDataPair.Key);
				IKJointWorkData& WorkData = WorkDataPair.Value;

				FTransform LocalTransform;
				if (JointIndex == WorkData.ParentJointIndex) // IK���[�g�W���C���g�̂Ƃ�
				{
					if (JointIndex.IsRootBone()) // IK���[�g�W���C���g���X�P���g���̃��[�g�̂Ƃ�
					{
						WorkData.LocalTransform = FTransform::Identity;
					}
					else // IK���[�g�W���C���g�ɐe�̃W���C���g������Ƃ�
					{
						WorkData.LocalTransform = WorkData.ComponentTransform * Output.Pose.GetComponentSpaceTransform(BoneContainer.GetParentBoneIndex(JointIndex)).Inverse(); 
					}
				}
				else
				{
					const IKJointWorkData& ParentWorkData = IKJointWorkDataMap[WorkData.ParentJointIndex.GetInt()];
					WorkData.LocalTransform = WorkData.ComponentTransform * ParentWorkData.ComponentTransform.Inverse();
				}

				const FRotator& LocalRotation = WorkData.LocalTransform.GetRotation().Rotator();

				// �W���C���g�̃��[�J���s��̉�]����������s��ɒu�����������̂����߂�

				FMatrix LocalTranslateMatrix = FMatrix::Identity;
				LocalTranslateMatrix = LocalTranslateMatrix.ConcatTranslation(WorkData.LocalTransform.GetTranslation());

				FMatrix LocalMatrix[ROTATION_AXIS_COUNT];
				LocalMatrix[0] = (RotationDifferentialX(LocalRotation.Roll) * RotationY(LocalRotation.Pitch) * RotationZ(LocalRotation.Yaw)) * LocalTranslateMatrix;
				LocalMatrix[1] = (RotationX(LocalRotation.Roll) * RotationDifferentialY(LocalRotation.Pitch) * RotationZ(LocalRotation.Yaw)) * LocalTranslateMatrix;
				LocalMatrix[2] = (RotationX(LocalRotation.Roll) * RotationY(LocalRotation.Pitch) * RotationDifferentialZ(LocalRotation.Yaw)) * LocalTranslateMatrix;

				// �W���C���g�̍��W�n���猩�����݂̃G�t�F�N�^�ʒu�����߂�
				const FMatrix& ChildRestMatrix = (IKJointWorkDataMap[OnlyOneConstraintJoint.GetInt()].ComponentTransform.ToMatrixWithScale() * WorkData.ComponentTransform.Inverse().ToMatrixWithScale());
				FMatrix ParentRestMatrix;
				if (JointIndex == WorkData.ParentJointIndex) // IK���[�g�W���C���g�̂Ƃ�
				{
					if (JointIndex.IsRootBone()) // IK���[�g�W���C���g���X�P���g���̃��[�g�̂Ƃ�
					{
						ParentRestMatrix = FMatrix::Identity;
					}
					else // IK���[�g�W���C���g�ɐe�̃W���C���g������Ƃ�
					{
						ParentRestMatrix = Output.Pose.GetComponentSpaceTransform(BoneContainer.GetParentBoneIndex(JointIndex)).ToMatrixWithScale(); 
					}
				}
				else
				{
					const IKJointWorkData& ParentWorkData = IKJointWorkDataMap[WorkData.ParentJointIndex.GetInt()];
					ParentRestMatrix = ParentWorkData.ComponentTransform.ToMatrixWithScale();
				}

				for (int32 RotAxis = 0; RotAxis < ROTATION_AXIS_COUNT; ++RotAxis)
				{
					const FVector& JacobianRow = (ChildRestMatrix * LocalMatrix[RotAxis] * ParentRestMatrix).TransformPosition(FVector::ZeroVector);
					Jacobian.Set(i * ROTATION_AXIS_COUNT + RotAxis, 0, JacobianRow.X);
					Jacobian.Set(i * ROTATION_AXIS_COUNT + RotAxis, 1, JacobianRow.Y);	
					Jacobian.Set(i * ROTATION_AXIS_COUNT + RotAxis, 2, JacobianRow.Z);	
				}

				i++;
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

			// �G�t�F�N�^��IKJointWorkDataMap[0].LocalTransform�͏����l��FTransform::Identity�̂܂�
			int32 i = 0;
			for (TPair<int32, IKJointWorkData>& WorkDataPair : IKJointWorkDataMap)
			{
				if (WorkDataPair.Key == OnlyOneConstraintJoint.GetInt())
				{
					continue;
				}

				IKJointWorkData& WorkData = WorkDataPair.Value;

				FRotator LocalRotation = WorkData.LocalTransform.Rotator();
				LocalRotation.Roll += FMath::RadiansToDegrees(IterationStepAngles[i * ROTATION_AXIS_COUNT + 0]);
				LocalRotation.Pitch += FMath::RadiansToDegrees(IterationStepAngles[i * ROTATION_AXIS_COUNT + 1]);
				LocalRotation.Yaw += FMath::RadiansToDegrees(IterationStepAngles[i * ROTATION_AXIS_COUNT + 2]);
				WorkData.LocalTransform.SetRotation(FQuat(LocalRotation));

				i++;
			}

			// �e���珇��LocalTransform�̕ω���ComponentTransform�ɔ��f���Ă���
			for (TPair<int32, IKJointWorkData>& WorkDataPair : IKJointWorkDataMap)
			{
				const FCompactPoseBoneIndex& JointIndex = FCompactPoseBoneIndex(WorkDataPair.Key);
				IKJointWorkData& WorkData = WorkDataPair.Value;

				FTransform ParentTransform;

				if (JointIndex == WorkData.ParentJointIndex) // IK���[�g�W���C���g�̂Ƃ�
				{
					if (JointIndex.IsRootBone()) // IK���[�g�W���C���g���X�P���g���̃��[�g�̂Ƃ�
					{
						ParentTransform = FTransform::Identity;
					}
					else // IK���[�g�W���C���g�ɐe�̃W���C���g������Ƃ�
					{
						ParentTransform = Output.Pose.GetComponentSpaceTransform(BoneContainer.GetParentBoneIndex(JointIndex));
					}
				}
				else
				{
					const IKJointWorkData& ParentWorkData = IKJointWorkDataMap[WorkData.ParentJointIndex.GetInt()];
					ParentTransform = ParentWorkData.ComponentTransform;
				}

				WorkData.ComponentTransform = WorkData.LocalTransform * ParentTransform;
			}
		}
	}

	// �{�[���C���f�b�N�X�̏����ɓn�������̂ŋt���Ƀ��[�v����
	for (const TPair<int32, IKJointWorkData>& WorkDataPair : IKJointWorkDataMap)
	{
		OutBoneTransforms.Add(FBoneTransform(FCompactPoseBoneIndex(WorkDataPair.Key), WorkDataPair.Value.ComponentTransform));
	}
}

