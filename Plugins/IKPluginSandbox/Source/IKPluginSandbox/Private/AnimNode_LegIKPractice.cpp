// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_LegIKPractice.h"
#include "Animation/AnimInstanceProxy.h"

FAnimNode_LegIKPractice::FAnimNode_LegIKPractice()
{
	ReachPrecision = 0.01f;
	MaxIterations = 12;
}

void FAnimLegIKDataPractice::InitializeTransforms(USkeletalMeshComponent* SkelComp, FCSPose<FCompactPose>& MeshBases)
{
	// Initialize bone transforms
	IKFootTransform = MeshBases.GetComponentSpaceTransform(IKFootBoneIndex);

	FKLegBoneTransforms.Reset();
	for (FCompactPoseBoneIndex LegBoneIndex : FKLegBoneIndices)
	{
		FKLegBoneTransforms.Add(MeshBases.GetComponentSpaceTransform(LegBoneIndex));
	}
}

namespace
{
bool RotateLegByQuat(const FQuat& InDeltaRotation, FAnimLegIKDataPractice& InLegData)
{
	if (!InDeltaRotation.IsIdentity())
	{
		const FVector HipLocation = InLegData.FKLegBoneTransforms.Last().GetLocation();

		// Rotate Leg so it is aligned with IK Target
		for (FTransform LegBoneTransform : InLegData.FKLegBoneTransforms)
		{
			LegBoneTransform.SetRotation(InDeltaRotation * LegBoneTransform.GetRotation());

			//TODO:Rotationをセットしたのに、それとは別にロケーションのセットがいるのか？コンポーネント座標だから？
			const FVector BoneLocation = LegBoneTransform.GetLocation();
			LegBoneTransform.SetLocation(HipLocation + InDeltaRotation.RotateVector(BoneLocation - HipLocation));
		}

		return true;
	}

	return false;
}

bool RotateLegByDeltaNormals(const FVector& InInitialDir, const FVector& InTargetDir, FAnimLegIKDataPractice& InLegData)
{
	if (!InInitialDir.IsZero() && !InTargetDir.IsZero())
	{
		// Find Delta Rotation take takes us from Old to New dir
		const FQuat DeltaRotation = FQuat::FindBetweenNormals(InInitialDir, InTargetDir);
		return RotateLegByQuat(DeltaRotation, InLegData);
	}

	return false;

}

void OrientLegTowardsIK(FAnimLegIKDataPractice& InLegData, USkeletalMeshComponent* SkelComp)
{
	check(InLegData.NumBones > 0);

	const FVector HipLocation = InLegData.FKLegBoneTransforms.Last().GetLocation();
	const FVector FootFKLocation = InLegData.FKLegBoneTransforms[0].GetLocation();
	const FVector FootIKLocation = InLegData.IKFootTransform.GetLocation();

	const FVector InitialDir = (FootFKLocation - HipLocation).GetSafeNormal();
	const FVector TargetDir = (FootIKLocation - HipLocation).GetSafeNormal();
}

void DoLegReachIK(FAnimLegIKDataPractice& InLegData, USkeletalMeshComponent* SkelComp, float InReachPrecision, int32 InMaxIterations)
{
	const FVector& FootFKLocation = InLegData.FKLegBoneTransforms[0].GetLocation();
	const FVector& FootIKLocation = InLegData.IKFootTransform.GetLocation();
	
	// If we're already reaching our IK Target, we have no work to do.
	if (FootFKLocation.Equals(FootIKLocation, InReachPrecision))
	{
		return;
	}

	//これから行うFABRIK計算のために用いるワークデータを作る
	FIKChainPractice IKChain;
	IKChain.InitializeFromLegData(InLegData, SkelComp);

	// FABRIK計算を行う
	IKChain.ReachTarget(FootIKLocation, InReachPrecision, InMaxIterations);

	// FABRIK計算した結果のFIKChainPracticeをスケルトンのポーズに反映する
	// Update bone transforms based on IKChain

	// Rotations
	for (int32 LinkIndex = InLegData.NumBones - 2; LinkIndex >= 0; LinkIndex--) // ヒップジョイントは含めず、その一つ子からエフェクタまでループ
	{
		// リンク配列の方はLocationしか持たない。ジョイントのRotation差分はリンク配列のLocationから計算する
		const FIKChainLinkPractice ParentLink = IKChain.Links[LinkIndex + 1];
		const FIKChainLinkPractice CurrentLink = IKChain.Links[LinkIndex];

		FTransform& ParentTransform = InLegData.FKLegBoneTransforms[LinkIndex + 1];
		const FTransform& CurrentTransform = InLegData.FKLegBoneTransforms[LinkIndex];

		// Calculate pre-translation vector between this bone and child
		const FVector InitialDir = (CurrentTransform.GetLocation() - ParentTransform.GetLocation()).GetSafeNormal();

		// Get vector from the post-translation bone to it's child
		const FVector TargetlDir = (CurrentLink.Location - ParentLink.Location).GetSafeNormal();

		const FQuat DeltaRotation = FQuat::FindBetweenNormals(TargetlDir, InitialDir);
		ParentTransform.SetRotation(DeltaRotation * ParentTransform.GetRotation());
	}

	// Translations
	for (int32 LinkIndex = InLegData.NumBones - 2; LinkIndex >= 0; LinkIndex--) // ヒップジョイントは含めず、その一つ子からエフェクタまでループ
	{
		const FIKChainLinkPractice CurrentLink = IKChain.Links[LinkIndex];
		FTransform& CurrentTransform = InLegData.FKLegBoneTransforms[LinkIndex];
		CurrentTransform.SetTranslation(CurrentLink.Location);
	}
}
}

void FIKChainPractice::InitializeFromLegData(const FAnimLegIKDataPractice& InLegData, USkeletalMeshComponent* InSkelMeshComp)
{
	// FAnimLegIKDataPracticeではジョイント単位でデータをもっていたが、IK計算ではジョイントの間をつなぐリンク(それこそボーン的なもの)単位で
	// ワークデータを持つようにする
	Links.Reset(InLegData.NumBones);
	MaximumReach = 0.0f;

	check(InLegData.NumBones > 1);
	for (int32 Index = 0; Index < InLegData.NumBones - 1; Index++)
	{
		const FVector BoneLocation = InLegData.FKLegBoneTransforms[Index].GetLocation();
		const FVector ParentLocation = InLegData.FKLegBoneTransforms[Index + 1].GetLocation();
		const float BoneLength = FVector::Dist(BoneLocation, ParentLocation);
		Links.Add(FIKChainLinkPractice(BoneLocation, BoneLength)); //親からでなく子を基準にしてチェインリンクを作るんだね
		MaximumReach += BoneLength;
	}

	// Add root bone last
	// ヒップジョイントは長さ0という扱いで最後に入れる。これでNumBones個のリンクができる。配列の扱いをFKLegBoneIndices/FKLegBoneTransformsと
	// 同じにできるように、あえて不要なヒップジョイントを入れている模様
	const FVector RootLocation = InLegData.FKLegBoneTransforms.Last().GetLocation();
	Links.Add(FIKChainLinkPractice(RootLocation, 0.0f));
	NumLinks = Links.Num();
	check(NumLinks == InLegData.NumBones);

	if (InLegData.LegDefPtr != NULL)
	{
		bEnableRotationLimit = InLegData.LegDefPtr->bEnableRotationLimit;
		if (bEnableRotationLimit)
		{
			MinRotationAngleRadians = FMath::DegreesToRadians(FMath::Clamp(InLegData.LegDefPtr->MinRotationAngle, 0.0f, 90.0f));
		}
	}

	SkelMeshComp = InSkelMeshComp;
	bInitialized = (SkelMeshComp != NULL);
}

void FIKChainPractice::ReachTarget(const FVector& InTargetLocation, float InReachPrecision, int32 InMaxIterations)
{
	if (!bInitialized) //bInitialized変数を使ったこの判定って必要か？InitializeFromLegDataで失敗を返して、ReachTargetに来ないようにしておけばよかっただけでは
	{
		return;
	}

	const FVector RootLocation = Links.Last().Location;

	// TwoBoneIKでもやった判定だね
	// If we can't reach, we just go in a straight line towards the target,
	if ((NumLinks <= 2) // ジョイントが2個しかないとき
		|| (FVector::DistSquared(RootLocation, InTargetLocation) >= FMath::Square(GetMaximumReach()))) // ジョイントを伸ばし切ってもターゲットに届かないとき
	{
		// 伸ばしきったポーズにする
		const FVector Direction = (InTargetLocation - RootLocation).GetSafeNormal();
		OrientAllLinksToDirection(Direction);
	}
	else
	{
		SolveFABRIK(InTargetLocation, InReachPrecision, InMaxIterations);
	}
}

void FIKChainPractice::OrientAllLinksToDirection(const FVector& InDirection)
{
	for (int32 Index = Links.Num() - 2; Index >= 0; Index--) // ヒップジョイントは含めず、その一つ子からエフェクタまでループ
	{
		Links[Index].Location = Links[Index + 1].Location + InDirection * Links[Index].Length;
	}
}

void FIKChainPractice::SolveFABRIK(const FVector& InTargetLocation, float InReachPrecision, int32 InMaxIterations)
{
}

void FAnimNode_LegIKPractice::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	check(Output.AnimInstanceProxy->GetSkelMeshComponent());
	check(OutBoneTransforms.Num() == 0);

	// Get transforms for each leg.
	{
		for (FAnimLegIKDataPractice& LegData : LegsData)
		{
			LegData.InitializeTransforms(Output.AnimInstanceProxy->GetSkelMeshComponent(), Output.Pose);

			// rotate hips so foot aligns with effector.
			OrientLegTowardsIK(LegData, Output.AnimInstanceProxy->GetSkelMeshComponent());

			// expand/compress leg, so foot reaches effector.
			DoLegReachIK(LegData, Output.AnimInstanceProxy->GetSkelMeshComponent(), ReachPrecision, MaxIterations);

			// Override Foot FK, with IK.
			//TODO:これはなぜやるのかわからない
			LegData.FKLegBoneTransforms[0].SetRotation(LegData.IKFootTransform.GetRotation());

			// Add transforms
			for (int32 Index = 0; Index < LegData.NumBones; Index++)
			{
				OutBoneTransforms.Add(FBoneTransform(LegData.FKLegBoneIndices[Index], LegData.FKLegBoneTransforms[Index]));
			}
		}
	}

	// Sort OutBoneTransforms so indices are in increasing order.
	OutBoneTransforms.Sort(FCompareBoneTransformIndex());
}

bool FAnimNode_LegIKPractice::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	return (LegsData.Num() > 0); // InitializeBoneReferencesは確かコンパイル時に呼ばれるけどこっちは実行時に呼ばれる
}

namespace
{
void PopulateLegBoneIndices(FAnimLegIKDataPractice& InLegData, const FCompactPoseBoneIndex& InFootBoneIndex, const int32& NumBonesInLimb, const FBoneContainer& RequiredBones)
{
	FCompactPoseBoneIndex BoneIndex = InFootBoneIndex;
	if (BoneIndex != INDEX_NONE)
	{
		InLegData.FKLegBoneIndices.Add(BoneIndex); // インデックス0に入れるのはエフェクタIKBone
		FCompactPoseBoneIndex ParentBoneIndex = RequiredBones.GetParentBoneIndex(BoneIndex);

		int32 NumIterations = NumBonesInLimb;
		while (NumIterations-- > 0 && ParentBoneIndex != INDEX_NONE) // NumBonesInLimbの数だけ、あるいはRootにつくまでさかのぼってFKLegBoneIndicesに登録する
		{
			BoneIndex = ParentBoneIndex;
			InLegData.FKLegBoneIndices.Add(BoneIndex); // インデックス0に入れるのはエフェクタIKBone
			ParentBoneIndex = RequiredBones.GetParentBoneIndex(BoneIndex);
		}
	}
}
}

void FAnimNode_LegIKPractice::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	LegsData.Reset();
	for (FAnimLegIKDefinitionPractice& LegDef : LegsDefinition)
	{
		LegDef.IKFootBone.Initialize(RequiredBones);
		LegDef.FKFootBone.Initialize(RequiredBones);

		FAnimLegIKDataPractice LegData;
		LegData.IKFootBoneIndex = LegDef.IKFootBone.GetCompactPoseIndex(RequiredBones);

		const FCompactPoseBoneIndex FKFootBoneIndex = LegDef.FKFootBone.GetCompactPoseIndex(RequiredBones);
		if ((LegData.IKFootBoneIndex != INDEX_NONE) && (FKFootBoneIndex != INDEX_NONE))
		{
			PopulateLegBoneIndices(LegData, LegData.IKFootBoneIndex, FMath::Max(LegDef.NumBonesInLimb, 1), RequiredBones);

			// We need at least three joints for this to work (hip, knee and foot).
			if (LegData.FKLegBoneIndices.Num() >= 3)
			{
				LegData.NumBones = LegData.FKLegBoneIndices.Num();
				LegData.LegDefPtr = &LegDef;
				LegsData.Add(LegData);
			}
		}
	}
}
