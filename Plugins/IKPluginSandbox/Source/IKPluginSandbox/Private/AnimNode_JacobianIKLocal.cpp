// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_JacobianIKLocal.h"
#include "Animation/AnimInstanceProxy.h"

namespace
{
// XYZの3成分に使う定数
const int32 AXIS_COUNT = 3;
// 回転方向の3成分に使う定数
const int32 ROTATION_AXIS_COUNT = 3;

// degree単位でのsin/cos関数
float SinD(float degree) { return FMath::Sin(FMath::DegreesToRadians(degree)); }
float CosD(float degree) { return FMath::Cos(FMath::DegreesToRadians(degree)); }

// 各軸の回転行列
//TODO: 正負の符号や、行優先列優先についてはあまりきちんと考慮はしておらずほげたつさんのソースを参考にしている
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

// 各軸の回転行列を回転角の変数で微分した行列
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
	// TODO:operator[][]の演算子オーバーライドは作れないかな？
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
		// IKRootJointとEffectorJointが先祖子孫関係になってない
		return false;
	}

	return (IKJointWorkDatas.Num() >= 2); // 最低限エフェクタと、一つIK制御ジョイントがないと意味がない
}

void FAnimNode_JacobianIKLocal::EvaluateSkeletalControl_AnyThread(FCSPose<FCompactPose>& InPose, FPoseContext& Output)
{
	check(Output.AnimInstanceProxy->GetSkelMeshComponent());

	const FBoneContainer& BoneContainer = InPose.GetPose().GetBoneContainer();

	// TODO:各IKノードと共通化しよう
	// そもそもジョイントの長さ的にIKの解に到達しうるかの確認
	float EffectorToIKRootLength = (InPose.GetComponentSpaceTransform(IKJointWorkDatas.Last().BoneIndex).GetLocation() - InPose.GetComponentSpaceTransform(IKJointWorkDatas[0].BoneIndex).GetLocation()).Size();
	float IKJointTotalLength = 0; // アニメーションにScaleがないなら、一度だけ計算してキャッシュしておけばよいが、今は毎回計算する
	for (int32 i = 1; i < IKJointWorkDatas.Num(); ++i)
	{
		IKJointTotalLength += (InPose.GetComponentSpaceTransform(IKJointWorkDatas[i].BoneIndex).GetLocation() - InPose.GetComponentSpaceTransform(IKJointWorkDatas[i - 1].BoneIndex).GetLocation()).Size();
	}

	if (IKJointTotalLength < EffectorToIKRootLength)
	{
		UE_LOG(LogAnimation, Warning, TEXT("IK cannot reach effector target location. The total length of joints is not enough."));
		return;
	}

	// ワークデータのTransformの初期化
	for (IKJointWorkData& WorkData : IKJointWorkDatas)
	{
		WorkData.LocalTransform = InPose.GetLocalSpaceTransform(WorkData.BoneIndex);
	}
	for (IKJointWorkData& WorkData : IKJointWorkDatas)
	{
		WorkData.ComponentTransform = InPose.GetComponentSpaceTransform(WorkData.BoneIndex);
	}

	// ノードの入力されたエフェクタの位置から目標位置への差分ベクトル
	const FVector& DeltaLocation = EffectorTargetLocation - InPose.GetComponentSpaceTransform(IKJointWorkDatas[0].BoneIndex).GetLocation();
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
						ParentRestMatrix = InPose.GetComponentSpaceTransform(IKRootJointParent).ToMatrixWithScale(); 
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
						ParentTransform = InPose.GetComponentSpaceTransform(IKRootJointParent);
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
		Output.Pose[IKJointWorkDatas[i].BoneIndex] = IKJointWorkDatas[i].LocalTransform;
	}
}
