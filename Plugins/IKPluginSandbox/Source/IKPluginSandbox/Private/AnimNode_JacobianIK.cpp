// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_JacobianIK.h"
#include "Animation/AnimInstanceProxy.h"

namespace
{
// 回転方向の3成分、XYZの3成分に使う定数
const int32 AXIS_COUNT = 3;

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
	// TODO:operator[][]の演算子オーバーライドは作れないかな？
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
		WorkData.Transform = Output.Pose.GetComponentSpaceTransform(WorkData.BoneIndex);
	}

	// JacobianIKのメインアルゴリズム
	// JacobianIKアルゴリズムについてはComputer Graphics Gems JP 2012の8章を参照
	uint32 iterCount = 0;
	for (; iterCount < NumIteration; ++iterCount)
	{
		// Jacobianの計算
		{
			Jacobian.ZeroClear();

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
						LocalTransform = FTransform::Identity;
					}
					else // IKルートジョイントに親のジョイントがあるとき
					{
						LocalTransform = IKJointWorkDatas[i].Transform * Output.Pose.GetComponentSpaceTransform(IKRootJointParent).Inverse(); 
					}
				}
				else
				{
					LocalTransform = IKJointWorkDatas[i].Transform * IKJointWorkDatas[i + 1].Transform.Inverse();
				}

				const FRotator& LocalRotation = LocalTransform.GetRotation().Rotator();

				// ジョイントのローカル行列の回転部分を微分行列に置き換えたものを求める
				FTransform LocalTransformRotationDifferential[AXIS_COUNT]; // 回転の3要素それぞれでの微分
				LocalTransformRotationDifferential[0] = LocalTransformRotationDifferential[1] = LocalTransformRotationDifferential[2] = LocalTransform;

				LocalTransformRotationDifferential[0].SetRotation((RotationDifferentialX(LocalRotation.Roll) * RotationY(LocalRotation.Pitch) * RotationZ(LocalRotation.Yaw)).ToQuat());
				LocalTransformRotationDifferential[1].SetRotation((RotationX(LocalRotation.Roll) * RotationDifferentialY(LocalRotation.Pitch) * RotationZ(LocalRotation.Yaw)).ToQuat());
				LocalTransformRotationDifferential[2].SetRotation((RotationX(LocalRotation.Roll) * RotationY(LocalRotation.Pitch) * RotationDifferentialZ(LocalRotation.Yaw)).ToQuat());

				// ジョイントの座標系から見た現在のエフェクタ位置を求める
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

	// ボーンインデックスの昇順に渡したいので逆順にループする
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
		// IKRootJointとEffectorJointが先祖子孫関係になってない
		return false;
	}

	IKRootJointParent = FCompactPoseBoneIndex(RequiredBones.GetParentBoneIndex(IKRootJoint.BoneIndex));

	return (IKJointWorkDatas.Num() > 0); // InitializeBoneReferencesは確かコンパイル時に呼ばれるけどこっちは実行時に呼ばれる
}

void FAnimNode_JacobianIK::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	IKRootJoint.Initialize(RequiredBones);
	EffectorJoint.Initialize(RequiredBones);

	// EffectorJointから、IKRootJointまで、IKRootJointにぶつからなければルートジョイントまでワークデータを持たせる
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

	// ヤコビアンの行数は、目標値を設定する全エフェクタの出力変数の数なので、現在は1個だけの位置指定だけなので3
	// ヤコビアンの列数は、IKの入力変数の数なので（ジョイント数-1）×回転の自由度3。-1なのはエフェクタの回転自由度は使わないから
	Jacobian = AnySizeMatrix((IKJointWorkDatas.Num() - 1) * AXIS_COUNT, AXIS_COUNT);
}
