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

	// EffectorJointから、IKRootJointまで、IKRootJointにぶつからなければルートジョイントまでワークデータを持たせる
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
		// IKRootJointとEffectorJointが先祖子孫関係になってない
		return false;
	}

	return (IKJointWorkDatas.Num() >= 2); // 最低限エフェクタと、一つIK制御ジョイントがないと意味がない
}

void FAnimNode_JacobianIK::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(Output.AnimInstanceProxy->GetSkelMeshComponent());
	check(OutBoneTransforms.Num() == 0);

	const FBoneContainer& BoneContainer = Output.Pose.GetPose().GetBoneContainer();

	// TODO:各IKノードと共通化しよう
	// そもそもジョイントの長さ的にIKの解に到達しうるかの確認
	float EffectorToIKRootLength = (Output.Pose.GetComponentSpaceTransform(IKJointWorkDatas.Last().BoneIndex).GetLocation() - Output.Pose.GetComponentSpaceTransform(IKJointWorkDatas[0].BoneIndex).GetLocation()).Size();
	float IKJointTotalLength = 0; // アニメーションにScaleがないなら、一度だけ計算してキャッシュしておけばよいが、今は毎回計算する
	for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i)
	{
		IKJointTotalLength += (Output.Pose.GetComponentSpaceTransform(IKJointWorkDatas[i].BoneIndex).GetLocation() - Output.Pose.GetComponentSpaceTransform(IKJointWorkDatas[i - 1].BoneIndex).GetLocation()).Size();
	}

	if (IKJointTotalLength < EffectorToIKRootLength)
	{
		UE_LOG(LogAnimation, Warning, TEXT("IK cannot reach effector target location. The total length of joints is not enough."));
		return;
	}

	// ワークデータのTransformの初期化
	for (IKJointWorkData& WorkData : IKJointWorkDatas)
	{
		WorkData.ComponentTransform = Output.Pose.GetComponentSpaceTransform(WorkData.BoneIndex);
	}
	// エフェクタのLocalTransformはイテレーションの中で更新しないのでここで計算しておく
	IKJointWorkDatas[0].LocalTransform = Output.Pose.GetLocalSpaceTransform(IKJointWorkDatas[0].BoneIndex);

	// ノードの入力されたエフェクタの位置から目標位置への差分ベクトル
	const FVector& DeltaLocation = EffectorTargetLocation - Output.Pose.GetComponentSpaceTransform(IKJointWorkDatas[0].BoneIndex).GetLocation();
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
			for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i)
			{
				// Jacobianのジョイントに対応する行を求める
				// 計算方法についてはComputer Graphics Gems JP 2012の8章を参照のこと

				FTransform LocalTransform;
				if (i == IKJointWorkDatas.Num() - 1) // IKルートジョイントのとき
				{
					if (IKRootJointParent.GetInt() == INDEX_NONE) // IKルートジョイントがスケルトンのルートのとき
					{
						IKJointWorkDatas[i].LocalTransform = FTransform::Identity;
					}
					else // IKルートジョイントに親のジョイントがあるとき
					{
						IKJointWorkDatas[i].LocalTransform = IKJointWorkDatas[i].ComponentTransform * Output.Pose.GetComponentSpaceTransform(IKRootJointParent).Inverse(); 
					}
				}
				else
				{
					IKJointWorkDatas[i].LocalTransform = IKJointWorkDatas[i].ComponentTransform * IKJointWorkDatas[i + 1].ComponentTransform.Inverse();
				}

				const FRotator& LocalRotation = IKJointWorkDatas[i].LocalTransform.GetRotation().Rotator();

				// ジョイントのローカル行列の回転部分を微分行列に置き換えたものを求める

				FMatrix LocalTranslateMatrix = FMatrix::Identity;
				LocalTranslateMatrix = LocalTranslateMatrix.ConcatTranslation(IKJointWorkDatas[i].LocalTransform.GetTranslation());

				FMatrix LocalMatrix[ROTATION_AXIS_COUNT];
				LocalMatrix[0] = (RotationDifferentialX(LocalRotation.Roll) * RotationY(LocalRotation.Pitch) * RotationZ(LocalRotation.Yaw)) * LocalTranslateMatrix;
				LocalMatrix[1] = (RotationX(LocalRotation.Roll) * RotationDifferentialY(LocalRotation.Pitch) * RotationZ(LocalRotation.Yaw)) * LocalTranslateMatrix;
				LocalMatrix[2] = (RotationX(LocalRotation.Roll) * RotationY(LocalRotation.Pitch) * RotationDifferentialZ(LocalRotation.Yaw)) * LocalTranslateMatrix;

				// ジョイントの座標系から見た現在のエフェクタ位置を求める
				const FMatrix& ChildRestMatrix = (IKJointWorkDatas[0].ComponentTransform.ToMatrixWithScale() * IKJointWorkDatas[i].ComponentTransform.Inverse().ToMatrixWithScale());
				FMatrix ParentRestMatrix;
				if (i == IKJointWorkDatas.Num() - 1) // IKルートジョイントのとき
				{
					if (IKRootJointParent.GetInt() == INDEX_NONE) // IKルートジョイントがスケルトンのルートのとき
					{
						ParentRestMatrix = FMatrix::Identity;
					}
					else // IKルートジョイントに親のジョイントがあるとき
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

			// エフェクタのIKJointWorkDatas[0].LocalTransformは初期値のFTransform::Identityのまま
			for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i)
			{
				FRotator LocalRotation = IKJointWorkDatas[i].LocalTransform.Rotator();
				LocalRotation.Roll += FMath::RadiansToDegrees(IterationStepAngles[(i - 1) * ROTATION_AXIS_COUNT + 0]);
				LocalRotation.Pitch += FMath::RadiansToDegrees(IterationStepAngles[(i - 1) * ROTATION_AXIS_COUNT + 1]);
				LocalRotation.Yaw += FMath::RadiansToDegrees(IterationStepAngles[(i - 1) * ROTATION_AXIS_COUNT + 2]);
				IKJointWorkDatas[i].LocalTransform.SetRotation(FQuat(LocalRotation));
			}

			// 親から順にLocalTransformの変化をComponentTransformに反映していくので逆順にループする
			for (int32 i = IKJointWorkDatas.Num() - 1; i >= 0; --i)
			{
				FTransform ParentTransform;
				if (i == IKJointWorkDatas.Num() - 1) // IKルートジョイントのとき
				{
					if (IKRootJointParent.GetInt() == INDEX_NONE) // IKルートジョイントがスケルトンのルートのとき
					{
						ParentTransform = FTransform::Identity;
					}
					else // IKルートジョイントに親のジョイントがあるとき
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

	// ボーンインデックスの昇順に渡したいので逆順にループする
	for (int32 i = IKJointWorkDatas.Num() - 1; i >= 0; --i)
	{
		OutBoneTransforms.Add(FBoneTransform(IKJointWorkDatas[i].BoneIndex, IKJointWorkDatas[i].ComponentTransform));
	}
}

