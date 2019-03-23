// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_JacobianIK.h"
#include "Animation/AnimInstanceProxy.h"

FAnimNode_JacobianIK::FAnimNode_JacobianIK()
	: NumIteration(10)
	, Precision(SMALL_NUMBER)
	, Lambda(1.0f)
	, bValidToEvaluate(false)
{
}

void FAnimNode_JacobianIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	bValidToEvaluate = false;

	// �Œ���G�t�F�N�^�ƁA���IK����W���C���g���Ȃ��ƈӖ����Ȃ�
	if (IKSkeleton.Num() < 2)
	{
		return;
	}

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

		// TODO:IKSkeleton���̃W���C���g�̏d���`�F�b�N�����Ȃ��Ƃ�

		for (FIKConstraint& IKConstraint : IKJoint.Constraints)
		{
			// EffectiveRootJoint�͂��łɂǂꂩ��FIKJoint�Ŏw�肳��Ă���Initialize����Ă���͂�
			if (!IKConstraint.EffectiveRootJoint.Initialize(RequiredBones))
			{
				return;
			}
		}
	}

	int32 NumRootIKJoint = 0;

	//TODO; ���t���[�����肷��悤�Ȃ��Ƃł͂Ȃ��B�ꕔ��Initialize_AnyThread�AInitializeBoneReference�ł̃`�F�b�N�ŏ\��
	for (FIKJoint& IKJoint : IKSkeleton)
	{
		if (!IKJoint.Joint.IsValidToEvaluate(RequiredBones))
		{
			return;
		}

		if (!IKJoint.ParentJoint.IsValidToEvaluate(RequiredBones))
		{
			return;
		}

		if (IKJoint.Joint == IKJoint.ParentJoint)
		{
			NumRootIKJoint++;
			if (NumRootIKJoint > 1)
			{
				return;
			}
		}
	}

	IKJointWorkDataMap.Reset();

	for (const FIKJoint& IKJoint : IKSkeleton)
	{
		IKJointWorkDataMap.Emplace(IKJoint.Joint.GetCompactPoseIndex(RequiredBones).GetInt(), IKJointWorkData(IKJoint.ParentJoint.GetCompactPoseIndex(RequiredBones), FTransform::Identity, FTransform::Identity, IKJoint.Constraints));
	}

	// JointIndex�̏����ɂ���
	IKJointWorkDataMap.KeySort(TLess<int32>());


	IKConstraintWorkDataArray.Empty(IKConstraintWorkDataArray.Num());

	for (FIKJoint& IKJoint : IKSkeleton)
	{
		for (const FIKConstraint& IKConstraint : IKJoint.Constraints)
		{
			if (!IKConstraint.EffectiveRootJoint.IsValidToEvaluate(RequiredBones))
			{
				return;
			}

			const FCompactPoseBoneIndex& ConstraintJointIndex = IKJoint.Joint.GetCompactPoseIndex(RequiredBones);
			const FCompactPoseBoneIndex& EffectiveRootIndex = IKConstraint.EffectiveRootJoint.GetCompactPoseIndex(RequiredBones);
			if (ConstraintJointIndex == EffectiveRootIndex)
			{
				return;
			}

			TArray<FCompactPoseBoneIndex> EffectiveJointIndices;

			FCompactPoseBoneIndex JointIndex = ConstraintJointIndex;
			IKJointWorkData WorkData = IKJointWorkDataMap[JointIndex.GetInt()];

			for (JointIndex = WorkData.ParentJointIndex, WorkData = IKJointWorkDataMap[JointIndex.GetInt()]; // ������EffectiveJointIndices�ɂ͊܂߂Ȃ�
				JointIndex != EffectiveRootIndex && JointIndex != WorkData.ParentJointIndex; // IK�X�P���g���̃��[�g�ɒB�����Ƃ������[�v�͔�����
				JointIndex = WorkData.ParentJointIndex, WorkData = IKJointWorkDataMap[JointIndex.GetInt()])
			{
				EffectiveJointIndices.Add(JointIndex);
			}
			EffectiveJointIndices.Add(JointIndex); // EffectiveRootIndex���܂߂�

			EffectiveJointIndices.Sort(
				[](const FCompactPoseBoneIndex& A, const FCompactPoseBoneIndex& B)
				{
					return A.GetInt() < B.GetInt();
				}
			);

			IKConstraintWorkDataArray.Emplace(ConstraintJointIndex, EffectiveJointIndices, IKConstraint.Position);
		}
	}

	// �R���X�g���C���g�͕K����ȏ゠��
	if (IKConstraintWorkDataArray.Num() == 0)
	{
		return;
	}

	// JointIndex�̏����ɂ���
	IKConstraintWorkDataArray.StableSort(
		[](const IKConstraintWorkData& A, const IKConstraintWorkData& B)
		{
			return A.JointIndex.GetInt() < B.JointIndex.GetInt();
		}
	);


	Jacobian = AnySizeMatrix(IKJointWorkDataMap.Num() * ROTATION_AXIS_COUNT, AXIS_COUNT * IKConstraintWorkDataArray.Num());
	Jt = AnySizeMatrix(AXIS_COUNT * IKConstraintWorkDataArray.Num(), IKJointWorkDataMap.Num() * ROTATION_AXIS_COUNT);
	JJt = AnySizeMatrix(AXIS_COUNT * IKConstraintWorkDataArray.Num(), AXIS_COUNT * IKConstraintWorkDataArray.Num()); // J * Jt

	LambdaI = AnySizeMatrix(AXIS_COUNT * IKConstraintWorkDataArray.Num(), AXIS_COUNT * IKConstraintWorkDataArray.Num()); // lambda * I
	LambdaI.ZeroClear();
	for (int32 i = 0; i < AXIS_COUNT * IKConstraintWorkDataArray.Num(); i++)
	{
		LambdaI.Set(i, i, Lambda);
	}

	JJtPlusLambdaI = AnySizeMatrix(AXIS_COUNT * IKConstraintWorkDataArray.Num(), AXIS_COUNT * IKConstraintWorkDataArray.Num()); // J * J^t + lambda * I
	JJti = AnySizeMatrix(AXIS_COUNT * IKConstraintWorkDataArray.Num(), AXIS_COUNT * IKConstraintWorkDataArray.Num()); // (J * J^t + lambda * I)^-1
	PseudoInverseJacobian = AnySizeMatrix(AXIS_COUNT * IKConstraintWorkDataArray.Num(), IKJointWorkDataMap.Num() * ROTATION_AXIS_COUNT); // J^t * (J * J^t)^-1

	IterationStepPosition.SetNumZeroed(AXIS_COUNT * IKConstraintWorkDataArray.Num());
	IterationStepAngles.SetNumZeroed(IKJointWorkDataMap.Num() * ROTATION_AXIS_COUNT);

	bValidToEvaluate = true;
}

bool FAnimNode_JacobianIK::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	return bValidToEvaluate;
}

void FAnimNode_JacobianIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(Output.AnimInstanceProxy->GetSkelMeshComponent());
	check(OutBoneTransforms.Num() == 0);

	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();

	// TODO:�eIK�m�[�h�Ƌ��ʉ����悤
	// ���������W���C���g�̒����I��IK�̉��ɓ��B�����邩�̊m�F
	// �R���X�g���C���g���ƂɊm�F����
	int32 ConstraintIndex = 0;
	for (const IKConstraintWorkData& Constraint : IKConstraintWorkDataArray)
	{
		float IKJointTotalLength = 0; // ���̃A�j���[�V�����m�[�h�ɓ��͂����|�[�Y��Scale���Ȃ��Ȃ�A��x�����v�Z���ăL���b�V�����Ă����΂悢���A���͖���v�Z����
		// TODO:�����`�F�b�N�����ɂ��Ă͖��t���[���̌v�Z�R�X�g����������

		// �G�t�F�N�^�̃{�[���ȊO�̒����̍��v
		for (int32 i = 0; i < Constraint.EffectiveJointIndices.Num() - 1; i++)
		{
			IKJointTotalLength += (Output.Pose.GetComponentSpaceTransform(Constraint.EffectiveJointIndices[i + 1]).GetLocation() - Output.Pose.GetComponentSpaceTransform(Constraint.EffectiveJointIndices[i]).GetLocation()).Size();
		}
		// �G�t�F�N�^�̃{�[�������v����
		IKJointTotalLength += (Output.Pose.GetComponentSpaceTransform(Constraint.JointIndex).GetLocation() - Output.Pose.GetComponentSpaceTransform(Constraint.EffectiveJointIndices[Constraint.EffectiveJointIndices.Num() - 1]).GetLocation()).Size();

		float EffectorToIKRootLength = (Constraint.Position - Output.Pose.GetComponentSpaceTransform(Constraint.EffectiveJointIndices[0]).GetLocation()).Size();
		if (IKJointTotalLength < EffectorToIKRootLength)
		{
			UE_LOG(LogAnimation, Warning, TEXT("IK cannot reach effector target location. The total length of joints is not enough."));
			return;
		}

		// �m�[�h�̓��͂��ꂽ�G�t�F�N�^�̈ʒu����ڕW�ʒu�ւ̍����x�N�g��
		const FVector& DeltaLocation = Constraint.Position - Output.Pose.GetComponentSpaceTransform(Constraint.JointIndex).GetLocation();

		// ���C�e���[�V�����ł̈ʒu�ړ�
		const FVector& IterationStep = DeltaLocation / NumIteration;
		IterationStepPosition[ConstraintIndex + 0] = IterationStep.X;
		IterationStepPosition[ConstraintIndex + 1] = IterationStep.Y;
		IterationStepPosition[ConstraintIndex + 2] = IterationStep.Z;
		ConstraintIndex++;
	}

	// ���[�N�f�[�^��Transform�̏�����
	for (TPair<int32, IKJointWorkData>& WorkData : IKJointWorkDataMap)
	{
		WorkData.Value.ComponentTransform = Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(WorkData.Key));
		WorkData.Value.LocalTransform = WorkData.Value.ComponentTransform * Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(WorkData.Value.ParentJointIndex)).Inverse();
	}

	// JacobianIK�̃��C���A���S���Y��
	// JacobianIK�A���S���Y���ɂ��Ă�Computer Graphics Gems JP 2012��8�͂��Q��
	uint32 iterCount = 0;
	for (; iterCount < NumIteration; ++iterCount)
	{
		// Jacobian�̌v�Z
		{
			Jacobian.ZeroClear(); // �ŏI�I�ɑS�v�f�ɒl������̂�0�N���A����K�v�͂Ȃ����f�o�b�O�̂��₷���̂��߂�0�N���A����

			int32 RotationIndex = 0;
			for (TPair<int32, IKJointWorkData>& WorkDataPair : IKJointWorkDataMap)
			{
				// Jacobian�̃W���C���g�ɑΉ�����s�����߂�
				// �v�Z���@�ɂ��Ă�Computer Graphics Gems JP 2012��8�͂��Q�Ƃ̂���
				IKJointWorkData& WorkData = WorkDataPair.Value;
				const FCompactPoseBoneIndex& JointIndex = FCompactPoseBoneIndex(WorkDataPair.Key);

				for (int32 ConstraintIndex = 0; ConstraintIndex < IKConstraintWorkDataArray.Num(); ConstraintIndex++)
				{
					const IKConstraintWorkData& Constraint = IKConstraintWorkDataArray[ConstraintIndex];
					bool bEffectiveJoint = false;
					for (const FCompactPoseBoneIndex& EffectiveJoint : Constraint.EffectiveJointIndices)
					{
						if (JointIndex == EffectiveJoint)
						{
							bEffectiveJoint = true;
							break;
						}
					}

					if (bEffectiveJoint)
					{
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
						const FMatrix& ChildRestMatrix = (IKJointWorkDataMap[Constraint.JointIndex.GetInt()].ComponentTransform.ToMatrixWithScale() * WorkData.ComponentTransform.Inverse().ToMatrixWithScale());
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
							Jacobian.Set(RotationIndex * ROTATION_AXIS_COUNT + RotAxis, ConstraintIndex + 0, JacobianRow.X);
							Jacobian.Set(RotationIndex * ROTATION_AXIS_COUNT + RotAxis, ConstraintIndex + 1, JacobianRow.Y);	
							Jacobian.Set(RotationIndex * ROTATION_AXIS_COUNT + RotAxis, ConstraintIndex + 2, JacobianRow.Z);	
						}
					}
					else
					{
						// ���̃W���C���g�����̃R���X�g���C���g�ɉe�����y�ڂ��W���C���g�łȂ��Ƃ��̓��R�r�A���̗v�f��0
						for (int32 RotAxis = 0; RotAxis < ROTATION_AXIS_COUNT; ++RotAxis)
						{
							Jacobian.Set(RotationIndex * ROTATION_AXIS_COUNT + RotAxis, ConstraintIndex + 0, 0.0f);
							Jacobian.Set(RotationIndex * ROTATION_AXIS_COUNT + RotAxis, ConstraintIndex + 1, 0.0f);	
							Jacobian.Set(RotationIndex * ROTATION_AXIS_COUNT + RotAxis, ConstraintIndex + 2, 0.0f);	
						}
					}
				}

				RotationIndex++;
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

			int32 i = 0;
			for (TPair<int32, IKJointWorkData>& WorkDataPair : IKJointWorkDataMap)
			{
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

