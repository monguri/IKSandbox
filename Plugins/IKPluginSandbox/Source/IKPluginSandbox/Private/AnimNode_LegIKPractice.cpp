// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimNode_LegIKPractice.h"
#include "BoneContainer.h"

FAnimNode_LegIKPractice::FAnimNode_LegIKPractice()
{
	ReachPrecision = 0.01f;
	MaxIterations = 12;
}

void FAnimNode_LegIKPractice::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
}

bool FAnimNode_LegIKPractice::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	return true;
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
