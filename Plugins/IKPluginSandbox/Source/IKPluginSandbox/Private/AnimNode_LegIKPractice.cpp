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

	RotateLegByDeltaNormals(InitialDir, TargetDir, InLegData);
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

namespace
{
FVector FindPlaneNormal(const TArray<FIKChainLinkPractice>& Links, const FVector& RootLocation, const FVector& TargetLocation)
{
	// UE4では、通常はボーンの方向がX方向。
	const FVector AxisX = (TargetLocation - RootLocation).GetSafeNormal();

	// ヒップから順にリンクをたどっていき、AxisXと平面を定義できるベクトルが見つかればそれで平面を定義する
	for (int32 LinkIdx = Links.Num() - 2; LinkIdx >= 0; LinkIdx--)
	{
		const FVector AxisY = (Links[LinkIdx].Location - RootLocation).GetSafeNormal();
		const FVector PlaneNormal = AxisX ^ AxisY;

		// Make sure we have a valid normal (Axes were not coplanar).
		if (PlaneNormal.SizeSquared() > SMALL_NUMBER)
		{
			return PlaneNormal.GetUnsafeNormal();
		}
	}

	// All links are co-planar?
	// すでにのびきって一直線になっているとこうなる
	return FVector::UpVector; //(0.0f, 0.0f, 1.0f) 適当
}

void FABRIK_ForwardReach(const FVector& InTargetLocation, FIKChainPractice& IKChain)
{
	//TODO:まずは追加機能は除いてアルゴリズムのもっとも基本部分のみ実装する
	// Move end effector towards target
	// If we are compressing the chain, limit displacement.
	// Due to how FABRIK works, if we push the target past the parent's joint, we flip the bone.
	{
		FVector EndEffectorToTarget = InTargetLocation - IKChain.Links[0].Location;

		FVector EndEffectorToTargetDir;
		float EndEffectorToTargetSize;
		EndEffectorToTarget.ToDirectionAndLength(EndEffectorToTargetDir, EndEffectorToTargetSize); //便利な関数があるんだなー

		float Displacement = EndEffectorToTargetSize;
		// もっとも単純なのは、エフェクタをそのまま目標位置にもっていくこと。IKChain.Links[0].Location = InTargetLocation;
		// もとのソースは、理由は不明だが、親のジョイント位置を考慮したり、移動値にパーセンテージを入れたりしている
		// TODO:それらの処理は後で書く
		IKChain.Links[0].Location += Displacement * EndEffectorToTargetDir;
	}

	// "Forward Reaching" stage - adjust bones from end effector.
	for (int32 LinkIndex = 1; LinkIndex < IKChain.Links.Num(); LinkIndex++)
	{
		FIKChainLinkPractice& ChildLink = IKChain.Links[LinkIndex - 1];
		FIKChainLinkPractice& CurrentLink = IKChain.Links[LinkIndex];

		// リンクの方向は変えずに、リンクの長さを本来の長さに伸縮することを、エフェクタからヒップまでリンクをさかのぼって繰り返す
		CurrentLink.Location = ChildLink.Location + (CurrentLink.Location - ChildLink.Location).GetSafeNormal() * CurrentLink.Length;
		// 最終的にヒップの位置はもとの位置からずれるが、それは次にBackwardの処理で修正する

		//TODO: bEnableRotationLimit==trueのときの処理は後で書く
	}
}

void FABRIK_BackwardReach(const FVector& InRootTargetLocation, FIKChainPractice& IKChain)
{
	//TODO:実装
	// Move Root back towards RootTarget
	// If we are compressing the chain, limit displacement.
	// Due to how FABRIK works, if we push the target past the parent's joint, we flip the bone.
	{
		FVector RootToRootTarget = InRootTargetLocation - IKChain.Links.Last().Location;

		FVector RootToRoootTargetDir;
		float RootToRoootTargetSize;
		RootToRootTarget.ToDirectionAndLength(RootToRoootTargetDir, RootToRoootTargetSize); //便利な関数があるんだなー

		float Displacement = RootToRoootTargetSize;
		// もっとも単純なのは、ヒップをそのまま目標位置（もとの位置）にもっていくこと。IKChain.Links.Last().Location = InRootTargetLocation;
		// もとのソースは、理由は不明だが、子のジョイント位置を考慮したり、移動値にパーセンテージを入れたりしている
		// TODO:それらの処理は後で書く
		IKChain.Links.Last().Location += Displacement * RootToRoootTargetDir;
	}

	// "Backward Reaching" stage - adjust bones from root.
	for (int32 LinkIndex = IKChain.Links.Num() - 1; LinkIndex >= 1; LinkIndex--)
	{
		FIKChainLinkPractice& CurrentLink = IKChain.Links[LinkIndex];
		FIKChainLinkPractice& ChildLink = IKChain.Links[LinkIndex - 1];

		// リンクの方向は変えずに、リンクの長さを本来の長さに伸縮することを、ヒップからヒップまでリンクをさかのぼって繰り返す
		ChildLink.Location = CurrentLink.Location + (ChildLink.Location - CurrentLink.Location).GetSafeNormal() * ChildLink.Length;
		// 最終的にエフェクタの位置は目標位置からずれるが、それは次にForwardの処理で修正する

		//TODO: bEnableRotationLimit==trueのときの処理は後で書く
	}
}

void FABRIK_ApplyLinkConstraints_Forward(FIKChainPractice& IKChain, int32 LinkIndex)
{
}

void FABRIK_ApplyLinkConstraints_Backward(FIKChainPractice& IKChain, int32 LinkIndex)
{
}
}

void FIKChainPractice::SolveFABRIK(const FVector& InTargetLocation, float InReachPrecision, int32 InMaxIterations)
{
	// Make sure precision is not too small.
	const float ReachPrecision = FMath::Max(InReachPrecision, KINDA_SMALL_NUMBER);
	const float PullDistributionAlpha = 0.5f; //TODO:もとのソースだとCVarだが、ここではCVarのデフォルト値を採用

	const FVector RootTargetLocation = Links.Last().Location;

	// Check distance between foot and foot target location
	float Slop = FVector::Dist(Links[0].Location, InTargetLocation);
	if (Slop > ReachPrecision) // ReachPrecisionより差分が小さくなったらFABRIK処理はしない
	{
		if (bEnableRotationLimit)
		{
			// Since we've previously aligned the foot with the IK Target, we're solving IK in 2D space on a single plane.
			// Find Plane Normal, to use in rotation constraints.
			// TODO:↑2D spaceの計算になるわけないだろ。ジョイントがばねみたいにねじれた構造になってる可能性は大いにある
			// OrientLegTowardsIKはエフェクタのFKBoneのヒップからの方向をIKBoneの方向になるようにしたに過ぎない
			// おそらくだが、FABRIKだと、それでも2D spaceに射影して考える必要があるのではなかろうか。
			// だから固定した平面の法線方向として全ジョイントのFABRIK計算上のZ軸をさだめるのでは？

			const FVector PlaneNormal = FindPlaneNormal(Links, RootTargetLocation, InTargetLocation);

			for (int32 LinkIndex = 1; LinkIndex < NumLinks - 1; LinkIndex++)
			{
				const FIKChainLinkPractice& ChildLink = Links[LinkIndex - 1]; // エフェクタからそのひとつ上のジョイントへのリンクには手を入れない
				// だが、そのリンクが持つLocationは子側のジョイントのロケーションなので、結局はエフェクタのひとつうえのジョイントからLinkAxisZを持たせることになる
				FIKChainLinkPractice& CurrentLink = Links[LinkIndex];
				const FIKChainLinkPractice& ParentLink = Links[LinkIndex + 1]; // ヒップジョイントだけの最後のリンクには手を入れない
 
				const FVector ChildAxisX = (ChildLink.Location - CurrentLink.Location).GetSafeNormal();
				const FVector ChildAxisY = PlaneNormal ^ ChildAxisX; // Z ^ X = Y
				const FVector ParentAxisX = (ParentLink.Location - CurrentLink.Location).GetSafeNormal();

				// Orient Z, so that ChildAxisY points 'up' and produces positive Sin values.
				CurrentLink.LinkAxisZ = (ParentAxisX | ChildAxisY) > 0.0f ? PlaneNormal : -PlaneNormal; // TODO:この理屈はよくわからんな。あとの計算の都合か？
			}
		}

		// Re-position limb to distribute pull
		// これはたぶんFABRIK特有の調整項なのだろうが、全ジョイントにあらかじめ、エフェクタの目標への差分にアルファをかけたオフセットを加えておく
		const FVector PullDistributionOffset = PullDistributionAlpha * (InTargetLocation - Links[0].Location) + (1.0f - PullDistributionAlpha) * (RootTargetLocation - Links.Last().Location); // 後者は0ベクトルになるに決まっている。だから結局はエフェクタの元の位置と目標位置の差にアルファをかけたものになる
		for (int32 LinkIndex = 0; LinkIndex < NumLinks; LinkIndex++)
		{
			Links[LinkIndex].Location = Links[LinkIndex].Location + PullDistributionOffset;
		}

		// FABRIKループ
		int32 IterationCount = 1;
		const int32 MaxIterations = FMath::Max(InMaxIterations, 1);
		// 必ず一回はFABRIK計算をやるのでdo-whileを使う
		do
		{
			const float PreviousSlop = Slop;

			if (Slop > 1.0f) //TODO:1.0fと比較してるのがどういうことかが不明だ。。。
			{
				// フォワードとバックワード計算の結果の中間をとる
				//TODO:中間をとるアルゴリズムのオプションなんてあったんだ。。。どういう効果があるんだろう。収束が早くなったり？
				FIKChainPractice ForwardPull = *this;
				FABRIK_ForwardReach(InTargetLocation, ForwardPull);

				FIKChainPractice BackwardPull = *this;
				FABRIK_BackwardReach(InTargetLocation, BackwardPull);

				// Average pulls
				for (int32 LinkIndex = 0; LinkIndex < NumLinks; LinkIndex++)
				{
					Links[LinkIndex].Location = 0.5f * (ForwardPull.Links[LinkIndex].Location + BackwardPull.Links[LinkIndex].Location);
				}
			}
			else
			{
				// フォワードとバックワード計算の両方を続けて適用する
				FABRIK_ForwardReach(InTargetLocation, *this);
				FABRIK_BackwardReach(InTargetLocation, *this);
			}

			// Slopは初期値と違ってヒップも動かすのでヒップの差分距離も加算する
			Slop = FVector::Dist(Links[0].Location, InTargetLocation) + FVector::Dist(Links.Last().Location, RootTargetLocation);

			// Abort if we're not getting closer and enter a deadlock.
			//TODO:FABRIKはループの途中過程で距離がより離れることがないんだね
			if (Slop > PreviousSlop)
			{
				break;
			}
		} while ((Slop > ReachPrecision) && (IterationCount++ < MaxIterations));

		// Make sure our root is back at our root target.
		// 何かの原因でヒップが元の位置からずれていたらもう一回分だけbackwardをやる
		if (!Links.Last().Location.Equals(RootTargetLocation))
		{
			FABRIK_BackwardReach(RootTargetLocation, *this);
		}

		// If we reached, set target precisely
		if (Slop <= ReachPrecision)
		{
			Links[0].Location = InTargetLocation;
		}
	}
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
