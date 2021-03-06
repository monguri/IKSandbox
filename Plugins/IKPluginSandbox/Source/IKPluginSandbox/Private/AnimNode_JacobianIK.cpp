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

	// 最低限エフェクタと、一つIK制御ジョイントがないと意味がない
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

		// TODO:IKSkeleton内のジョイントの重複チェックもしないとな

		for (FIKConstraint& IKConstraint : IKJoint.Constraints)
		{
			if (IKConstraint.Type == EIKConstraintType::KEEP_POSITION)
			{
				// EffectiveRootJointはすでにどれかのFIKJointで指定されておりInitializeされているはず
				if (!IKConstraint.EffectiveRootJoint.Initialize(RequiredBones))
				{
					// TODO:UE_LOG
					return;
				}
			}
		}
	}

	int32 NumRootIKJoint = 0;

	//TODO; 毎フレーム判定するようなことではない。一部はInitialize_AnyThread、InitializeBoneReferenceでのチェックで十分
	for (FIKJoint& IKJoint : IKSkeleton)
	{
		if (!IKJoint.Joint.IsValidToEvaluate(RequiredBones))
		{
			// TODO:UE_LOG
			return;
		}

		if (!IKJoint.ParentJoint.IsValidToEvaluate(RequiredBones))
		{
			// TODO:UE_LOG
			return;
		}

		if (IKJoint.Joint == IKJoint.ParentJoint)
		{
			NumRootIKJoint++;
			if (NumRootIKJoint > 1)
			{
				// TODO:UE_LOG
				return;
			}
		}
	}

	IKJointWorkDataMap.Reset();

	for (const FIKJoint& IKJoint : IKSkeleton)
	{
		IKJointWorkDataMap.Emplace(IKJoint.Joint.GetCompactPoseIndex(RequiredBones).GetInt(), IKJointWorkData(IKJoint.ParentJoint.GetCompactPoseIndex(RequiredBones), FTransform::Identity, FTransform::Identity, IKJoint.Constraints));
	}

	// JointIndexの昇順にする
	IKJointWorkDataMap.KeySort(TLess<int32>());


	IKConstraintWorkDataArray.Empty(IKConstraintWorkDataArray.Num());

	uint32 NumPositionConstraint = 0;

	for (FIKJoint& IKJoint : IKSkeleton)
	{
		for (const FIKConstraint& IKConstraint : IKJoint.Constraints)
		{
			if (IKConstraint.Type == EIKConstraintType::INVALID)
			{
				continue;
			}

			const FCompactPoseBoneIndex& ConstraintJointIndex = IKJoint.Joint.GetCompactPoseIndex(RequiredBones);
			TArray<FCompactPoseBoneIndex> EffectiveJointIndices;

			if (IKConstraint.Type == EIKConstraintType::KEEP_POSITION)
			{
				// PositionコンストレイントのときはIK計算に加えるジョイントをすべて洗い出す

				if (!IKConstraint.EffectiveRootJoint.IsValidToEvaluate(RequiredBones))
				{
					// TODO:UE_LOG
					return;
				}

				const FCompactPoseBoneIndex& EffectiveRootIndex = IKConstraint.EffectiveRootJoint.GetCompactPoseIndex(RequiredBones);
				if (ConstraintJointIndex == EffectiveRootIndex)
				{
					// TODO:UE_LOG
					return;
				}

				FCompactPoseBoneIndex JointIndex = ConstraintJointIndex;
				IKJointWorkData WorkData = IKJointWorkDataMap[JointIndex.GetInt()];

				for (JointIndex = WorkData.ParentJointIndex, WorkData = IKJointWorkDataMap[JointIndex.GetInt()]; // 自分はEffectiveJointIndicesには含めない
					JointIndex != EffectiveRootIndex && JointIndex != WorkData.ParentJointIndex; // IKスケルトンのルートに達したときもループは抜ける
					JointIndex = WorkData.ParentJointIndex, WorkData = IKJointWorkDataMap[JointIndex.GetInt()])
				{
					EffectiveJointIndices.Add(JointIndex);
				}
				EffectiveJointIndices.Add(JointIndex); // EffectiveRootIndexも含める

				EffectiveJointIndices.Sort(
					[](const FCompactPoseBoneIndex& A, const FCompactPoseBoneIndex& B)
				{
					return A.GetInt() < B.GetInt();
				}
				);

				NumPositionConstraint++;
			}

			IKConstraintWorkDataArray.Emplace(ConstraintJointIndex, EffectiveJointIndices, IKConstraint.Type, IKConstraint.Position, IKConstraint.Rotation);
		}
	}

	// コンストレイントは必ず一つ以上ある
	if (IKConstraintWorkDataArray.Num() == 0)
	{
		// TODO:UE_LOG
		return;
	}

	// JointIndexの昇順にする
	IKConstraintWorkDataArray.StableSort(
		[](const IKConstraintWorkData& A, const IKConstraintWorkData& B)
		{
			return A.JointIndex.GetInt() < B.JointIndex.GetInt();
		}
	);


	Jacobian = AnySizeMatrix(IKJointWorkDataMap.Num() * ROTATION_AXIS_COUNT, AXIS_COUNT * NumPositionConstraint);
	Jt = AnySizeMatrix(AXIS_COUNT * NumPositionConstraint, IKJointWorkDataMap.Num() * ROTATION_AXIS_COUNT);
	JJt = AnySizeMatrix(AXIS_COUNT * NumPositionConstraint, AXIS_COUNT * NumPositionConstraint); // J * Jt

	LambdaI = AnySizeMatrix(AXIS_COUNT * NumPositionConstraint, AXIS_COUNT * NumPositionConstraint); // lambda * I
	LambdaI.ZeroClear();
	for (uint32 i = 0; i < AXIS_COUNT * NumPositionConstraint; i++)
	{
		LambdaI.Set(i, i, Lambda);
	}

	JJtPlusLambdaI = AnySizeMatrix(AXIS_COUNT * NumPositionConstraint, AXIS_COUNT * NumPositionConstraint); // J * J^t + lambda * I
	JJti = AnySizeMatrix(AXIS_COUNT * NumPositionConstraint, AXIS_COUNT * NumPositionConstraint); // (J * J^t + lambda * I)^-1
	PseudoInverseJacobian = AnySizeMatrix(AXIS_COUNT * NumPositionConstraint, IKJointWorkDataMap.Num() * ROTATION_AXIS_COUNT); // J^t * (J * J^t)^-1

	IterationStepPosition.SetNumZeroed(AXIS_COUNT * NumPositionConstraint);
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

	// ワークデータのTransformの初期化
	for (TPair<int32, IKJointWorkData>& WorkData : IKJointWorkDataMap)
	{
		WorkData.Value.ComponentTransform = Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(WorkData.Key));
		WorkData.Value.LocalTransform = WorkData.Value.ComponentTransform * Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(WorkData.Value.ParentJointIndex)).Inverse();
	}

	// TODO:各IKノードと共通化しよう
	uint32 PositionConstraintIndex = 0;
	for (const IKConstraintWorkData& Constraint : IKConstraintWorkDataArray)
	{
		switch (Constraint.Type)
		{
			case EIKConstraintType::KEEP_POSITION:
			{
				// そもそもジョイントの長さ的にIKの解に到達しうるかの確認
				// コンストレイントごとに確認する
				float IKJointTotalLength = 0; // このアニメーションノードに入力されるポーズにScaleがないなら、一度だけ計算してキャッシュしておけばよいが、今は毎回計算する
				// TODO:だがチェック処理にしては毎フレームの計算コスト高すぎかも

				// エフェクタのボーン以外の長さの合計
				for (int32 i = 0; i < Constraint.EffectiveJointIndices.Num() - 1; i++)
				{
					IKJointTotalLength += (Output.Pose.GetComponentSpaceTransform(Constraint.EffectiveJointIndices[i + 1]).GetLocation() - Output.Pose.GetComponentSpaceTransform(Constraint.EffectiveJointIndices[i]).GetLocation()).Size();
				}
				// エフェクタのボーンも合計する
				IKJointTotalLength += (Output.Pose.GetComponentSpaceTransform(Constraint.JointIndex).GetLocation() - Output.Pose.GetComponentSpaceTransform(Constraint.EffectiveJointIndices[Constraint.EffectiveJointIndices.Num() - 1]).GetLocation()).Size();

				float EffectorToIKRootLength = (Constraint.Position - Output.Pose.GetComponentSpaceTransform(Constraint.EffectiveJointIndices[0]).GetLocation()).Size();
				if (IKJointTotalLength < EffectorToIKRootLength)
				{
					UE_LOG(LogAnimation, Warning, TEXT("IK cannot reach effector target location. The total length of joints is not enough."));
					return;
				}

				// ノードの入力されたエフェクタの位置から目標位置への差分ベクトル
				const FVector& DeltaLocation = Constraint.Position - Output.Pose.GetComponentSpaceTransform(Constraint.JointIndex).GetLocation();

				// 毎イテレーションでの位置移動
				const FVector& IterationStep = DeltaLocation / NumIteration;
				IterationStepPosition[PositionConstraintIndex * AXIS_COUNT + 0] = IterationStep.X;
				IterationStepPosition[PositionConstraintIndex * AXIS_COUNT + 1] = IterationStep.Y;
				IterationStepPosition[PositionConstraintIndex * AXIS_COUNT + 2] = IterationStep.Z;
				PositionConstraintIndex++;
			}
				break;
			case EIKConstraintType::KEEP_ROTATION:
			{
				IKJointWorkData& WorkData = IKJointWorkDataMap.FindChecked(Constraint.JointIndex.GetInt());

				const FRotator& DeltaRotation = Constraint.Rotation - Output.Pose.GetComponentSpaceTransform(Constraint.JointIndex).GetRotation().Rotator();
				WorkData.LocalTransform.SetRotation(FQuat(Constraint.Rotation));
				WorkData.LocalTransform.NormalizeRotation();

				WorkData.ComponentTransform = WorkData.LocalTransform * Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(WorkData.ParentJointIndex));
			}
				break;
			case EIKConstraintType::INVALID:
				// fall through
			default:
				check(false)
				break;
		}
	}

	// JacobianIKのメインアルゴリズム
	// JacobianIKアルゴリズムについてはComputer Graphics Gems JP 2012の8章を参照
	uint32 iterCount = 0;
	for (; iterCount < NumIteration; ++iterCount)
	{
		// Jacobianの計算
		{
			Jacobian.ZeroClear(); // 最終的に全要素に値が入るので0クリアする必要はないがデバッグのしやすさのために0クリアする

			int32 RotationIndex = 0;
			for (TPair<int32, IKJointWorkData>& WorkDataPair : IKJointWorkDataMap)
			{
				// Jacobianのジョイントに対応する行を求める
				// 計算方法についてはComputer Graphics Gems JP 2012の8章を参照のこと
				IKJointWorkData& WorkData = WorkDataPair.Value;
				const FCompactPoseBoneIndex& JointIndex = FCompactPoseBoneIndex(WorkDataPair.Key);

				PositionConstraintIndex = 0;
				for (IKConstraintWorkData& Constraint : IKConstraintWorkDataArray)
				{
					if (Constraint.Type != EIKConstraintType::KEEP_POSITION)
					{
						continue;
					}

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
						if (JointIndex == WorkData.ParentJointIndex) // IKルートジョイントのときは、LocalTransformは通常のスケルトンのLocalTransrormと同じ
						{
							if (JointIndex.IsRootBone()) // IKルートジョイントがスケルトンのルートのとき
							{
								WorkData.LocalTransform = FTransform::Identity;
							}
							else // IKルートジョイントに親のジョイントがあるとき
							{
								WorkData.LocalTransform = WorkData.ComponentTransform * Output.Pose.GetComponentSpaceTransform(BoneContainer.GetParentBoneIndex(JointIndex)).Inverse(); 
							}
						}
						else // IKルートジョイントのときは、LocalTransformはIKスケルトン基準で計算する
						{
							const IKJointWorkData& ParentWorkData = IKJointWorkDataMap[WorkData.ParentJointIndex.GetInt()];
							WorkData.LocalTransform = WorkData.ComponentTransform * ParentWorkData.ComponentTransform.Inverse();
						}

						const FRotator& LocalRotation = WorkData.LocalTransform.GetRotation().Rotator();

						// ジョイントのローカル行列の回転部分を微分行列に置き換えたものを求める

						FMatrix LocalTranslateMatrix = FMatrix::Identity;
						LocalTranslateMatrix = LocalTranslateMatrix.ConcatTranslation(WorkData.LocalTransform.GetTranslation());

						FMatrix LocalMatrix[ROTATION_AXIS_COUNT];
						LocalMatrix[0] = (RotationDifferentialX(LocalRotation.Roll) * RotationY(LocalRotation.Pitch) * RotationZ(LocalRotation.Yaw)) * LocalTranslateMatrix;
						LocalMatrix[1] = (RotationX(LocalRotation.Roll) * RotationDifferentialY(LocalRotation.Pitch) * RotationZ(LocalRotation.Yaw)) * LocalTranslateMatrix;
						LocalMatrix[2] = (RotationX(LocalRotation.Roll) * RotationY(LocalRotation.Pitch) * RotationDifferentialZ(LocalRotation.Yaw)) * LocalTranslateMatrix;

						// ジョイントの座標系から見た現在のエフェクタ位置を求める
						const FMatrix& ChildRestMatrix = (IKJointWorkDataMap[Constraint.JointIndex.GetInt()].ComponentTransform.ToMatrixWithScale() * WorkData.ComponentTransform.Inverse().ToMatrixWithScale());
						FMatrix ParentRestMatrix;
						if (JointIndex == WorkData.ParentJointIndex) // IKルートジョイントのとき
						{
							if (JointIndex.IsRootBone()) // IKルートジョイントがスケルトンのルートのとき
							{
								ParentRestMatrix = FMatrix::Identity;
							}
							else // IKルートジョイントに親のジョイントがあるとき
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
							Jacobian.Set(RotationIndex * ROTATION_AXIS_COUNT + RotAxis, PositionConstraintIndex * AXIS_COUNT + 0, JacobianRow.X);
							Jacobian.Set(RotationIndex * ROTATION_AXIS_COUNT + RotAxis, PositionConstraintIndex * AXIS_COUNT + 1, JacobianRow.Y);	
							Jacobian.Set(RotationIndex * ROTATION_AXIS_COUNT + RotAxis, PositionConstraintIndex * AXIS_COUNT + 2, JacobianRow.Z);	
						}
					}
					else
					{
						// このジョイントがこのコンストレイントに影響を及ぼすジョイントでないときはヤコビアンの要素は0
						for (int32 RotAxis = 0; RotAxis < ROTATION_AXIS_COUNT; ++RotAxis)
						{
							Jacobian.Set(RotationIndex * ROTATION_AXIS_COUNT + RotAxis, PositionConstraintIndex * AXIS_COUNT + 0, 0.0f);
							Jacobian.Set(RotationIndex * ROTATION_AXIS_COUNT + RotAxis, PositionConstraintIndex * AXIS_COUNT + 1, 0.0f);	
							Jacobian.Set(RotationIndex * ROTATION_AXIS_COUNT + RotAxis, PositionConstraintIndex * AXIS_COUNT + 2, 0.0f);	
						}
					}

					PositionConstraintIndex++;
				}

				RotationIndex++;
			}
		}

		// Jacobianの疑似逆行列の計算
		{
			// TODO:計算が行優先になってないのでは？
			Jt.ZeroClear(); // 最終的に全要素に値が入るので0クリアする必要はないがデバッグのしやすさのために0クリアする
			AnySizeMatrix::Transpose(Jacobian, Jt);

			JJt.ZeroClear(); // 最終的に全要素に値が入るので0クリアする必要はないがデバッグのしやすさのために0クリアする
			AnySizeMatrix::Multiply(Jt, Jacobian, JJt);

			JJtPlusLambdaI.ZeroClear();
			AnySizeMatrix::Add(JJt, LambdaI, JJtPlusLambdaI);

			JJti.ZeroClear(); // 最終的に全要素に値が入るので0クリアする必要はないがデバッグのしやすさのために0クリアする
#if 0
			float Determinant = AnySizeMatrix::InverseNxN(JJt, JJti);
#else
			float Determinant = AnySizeMatrix::InverseNxN(JJtPlusLambdaI, JJti);
#endif
			//if (FMath::Abs(Determinant) < KINDA_SMALL_NUMBER)
			if (FMath::Abs(Determinant) < SMALL_NUMBER)
			{
				// TODO:UE_LOG
				return;
			}

			PseudoInverseJacobian.ZeroClear(); // 最終的に全要素に値が入るので0クリアする必要はないがデバッグのしやすさのために0クリアする
			AnySizeMatrix::Multiply(JJti, Jt, PseudoInverseJacobian);

			// TODO:とりあえず関節角変位目標値τは0にしておき、計算しない
		}

		// 回転角変位を求め、ワークデータの現在角を更新する
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

			// 親から順にLocalTransformの変化をComponentTransformに反映していく
			for (TPair<int32, IKJointWorkData>& WorkDataPair : IKJointWorkDataMap)
			{
				const FCompactPoseBoneIndex& JointIndex = FCompactPoseBoneIndex(WorkDataPair.Key);
				IKJointWorkData& WorkData = WorkDataPair.Value;

				FTransform ParentTransform;

				if (JointIndex == WorkData.ParentJointIndex) // IKルートジョイントのときは、ComponentTransformは通常のスケルトンのComponentTransformと同じ
				{
					if (JointIndex.IsRootBone()) // IKルートジョイントがスケルトンのルートのとき
					{
						ParentTransform = FTransform::Identity;
					}
					else // IKルートジョイントに親のジョイントがあるとき
					{
						ParentTransform = Output.Pose.GetComponentSpaceTransform(BoneContainer.GetParentBoneIndex(JointIndex));
					}
				}
				else // IKルートジョイントのときは、ComponentTransformはIKスケルトン基準で計算する
				{
					const IKJointWorkData& ParentWorkData = IKJointWorkDataMap[WorkData.ParentJointIndex.GetInt()];
					ParentTransform = ParentWorkData.ComponentTransform;
				}

				WorkData.ComponentTransform = WorkData.LocalTransform * ParentTransform;
			}
		}
	}

	// ボーンインデックスの昇順に渡したいので逆順にループする
	for (const TPair<int32, IKJointWorkData>& WorkDataPair : IKJointWorkDataMap)
	{
		OutBoneTransforms.Add(FBoneTransform(FCompactPoseBoneIndex(WorkDataPair.Key), WorkDataPair.Value.ComponentTransform));
	}
}

