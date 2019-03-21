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
		// TODO:IKSkeleton内のジョイントの重複チェックもしないとな
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

	//TODO; 毎フレーム判定するようなことではない。一部はInitialize_AnyThread、InitializeBoneReferenceでのチェックで十分
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

	return ((NumRootIKJoint == 1) // ルート設定のジョイントは必ず一つ
		&& (NumConstraint == 1) //TODO: コンストレイントは一つという仮定をまだ入れておく
		&& IKJointWorkDataMap.Num() >= 2); // 最低限エフェクタと、一つIK制御ジョイントがないと意味がない
}

void FAnimNode_JacobianIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(Output.AnimInstanceProxy->GetSkelMeshComponent());
	check(OutBoneTransforms.Num() == 0);

	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();

	// TODO:現状、複数コンストレイントに対応できてないのでこれが唯一対応するコンストレイント
	FCompactPoseBoneIndex OnlyOneConstraintJoint(INDEX_NONE);
	FIKConstraint OnlyOneConstraint;

	// TODO:各IKノードと共通化しよう
	// そもそもジョイントの長さ的にIKの解に到達しうるかの確認
	// コンストレイントごとに確認する
	for (const TPair<int32, IKJointWorkData>& WorkData : IKJointWorkDataMap)
	{
		if (WorkData.Value.Constraints.Num() == 0)
		{
			continue;
		}

		//TODO: 現状、位置固定コンストレイントしかない
		for (const FIKConstraint& Constraint : WorkData.Value.Constraints)
		{
			OnlyOneConstraintJoint = FCompactPoseBoneIndex(WorkData.Key);
			OnlyOneConstraint = Constraint;

			float IKJointTotalLength = 0; // このアニメーションノードに入力されるポーズにScaleがないなら、一度だけ計算してキャッシュしておけばよいが、今は毎回計算する
			// TODO:だがチェック処理にしては毎フレームの計算コスト高すぎかも

			FCompactPoseBoneIndex CurrentJointIndex = FCompactPoseBoneIndex(WorkData.Key);
			FCompactPoseBoneIndex ParentJointIndex = WorkData.Value.ParentJointIndex;
			IKJointWorkData CurrentWorkData = WorkData.Value;
			IKJointWorkData ParentWorkData = IKJointWorkDataMap[ParentJointIndex.GetInt()];
			while (CurrentJointIndex != ParentJointIndex) // IKSkeleton内のルート設定のジョイントに至るまでループ
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

	// ワークデータのTransformの初期化
	for (TPair<int32, IKJointWorkData>& WorkData : IKJointWorkDataMap)
	{
		WorkData.Value.ComponentTransform = Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(WorkData.Key));

		// エフェクタのLocalTransformはイテレーションの中で更新しないのでここで計算しておく
		//TODO: 現状、位置固定コンストレイントしかない
		if (WorkData.Value.Constraints.Num() > 0)
		{
			WorkData.Value.LocalTransform = Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(WorkData.Key)) * Output.Pose.GetComponentSpaceTransform(FCompactPoseBoneIndex(WorkData.Value.ParentJointIndex)).Inverse();
		}
	}

	// ノードの入力されたエフェクタの位置から目標位置への差分ベクトル
	const FVector& DeltaLocation = OnlyOneConstraint.Position - Output.Pose.GetComponentSpaceTransform(OnlyOneConstraintJoint).GetLocation();
	// 毎イテレーションでの位置移動
	const FVector& IterationStep = DeltaLocation / NumIteration;
	IterationStepPosition[0] = IterationStep.X;
	IterationStepPosition[1] = IterationStep.Y;
	IterationStepPosition[2] = IterationStep.Z;

	// JacobianIKのメインアルゴリズム
	// JacobianIKアルゴリズムについてはComputer Graphics Gems JP 2012の8章を参照
	uint32 iterCount = 0;
	for (; iterCount < NumIteration; ++iterCount)
	{
		// Jacobianの計算
		{
			Jacobian.ZeroClear(); // 最終的に全要素に値が入るので0クリアする必要はないがデバッグのしやすさのために0クリアする

			// エフェクタの親からIKルートジョイントまでを回転の入力とするのでそれらからJacobianを作る
			int32 i = 0;
			for (TPair<int32, IKJointWorkData>& WorkDataPair : IKJointWorkDataMap)
			{
				if (WorkDataPair.Key == OnlyOneConstraintJoint.GetInt())
				{
					continue;
				}

				// Jacobianのジョイントに対応する行を求める
				// 計算方法についてはComputer Graphics Gems JP 2012の8章を参照のこと
				const FCompactPoseBoneIndex& JointIndex = FCompactPoseBoneIndex(WorkDataPair.Key);
				IKJointWorkData& WorkData = WorkDataPair.Value;

				FTransform LocalTransform;
				if (JointIndex == WorkData.ParentJointIndex) // IKルートジョイントのとき
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
				else
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
				const FMatrix& ChildRestMatrix = (IKJointWorkDataMap[OnlyOneConstraintJoint.GetInt()].ComponentTransform.ToMatrixWithScale() * WorkData.ComponentTransform.Inverse().ToMatrixWithScale());
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
					Jacobian.Set(i * ROTATION_AXIS_COUNT + RotAxis, 0, JacobianRow.X);
					Jacobian.Set(i * ROTATION_AXIS_COUNT + RotAxis, 1, JacobianRow.Y);	
					Jacobian.Set(i * ROTATION_AXIS_COUNT + RotAxis, 2, JacobianRow.Z);	
				}

				i++;
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
			float Determinant = AnySizeMatrix::Inverse3x3(JJt, JJti);
#else
			float Determinant = AnySizeMatrix::Inverse3x3(JJtPlusLambdaI, JJti);
#endif
			//if (FMath::Abs(Determinant) < KINDA_SMALL_NUMBER)
			if (FMath::Abs(Determinant) < SMALL_NUMBER)
			{
				return;
			}

			PseudoInverseJacobian.ZeroClear(); // 最終的に全要素に値が入るので0クリアする必要はないがデバッグのしやすさのために0クリアする
			AnySizeMatrix::Multiply(JJti, Jt, PseudoInverseJacobian);

			// TODO:とりあえず関節角変位目標値τは0にしておき、計算しない
		}

		// 回転角変位を求め、ワークデータの現在角を更新する
		{
			AnySizeMatrix::TransformVector(PseudoInverseJacobian, IterationStepPosition, IterationStepAngles);

			// エフェクタのIKJointWorkDataMap[0].LocalTransformは初期値のFTransform::Identityのまま
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

			// 親から順にLocalTransformの変化をComponentTransformに反映していく
			for (TPair<int32, IKJointWorkData>& WorkDataPair : IKJointWorkDataMap)
			{
				const FCompactPoseBoneIndex& JointIndex = FCompactPoseBoneIndex(WorkDataPair.Key);
				IKJointWorkData& WorkData = WorkDataPair.Value;

				FTransform ParentTransform;

				if (JointIndex == WorkData.ParentJointIndex) // IKルートジョイントのとき
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
				else
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

