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

			//TODO:Rotation���Z�b�g�����̂ɁA����Ƃ͕ʂɃ��P�[�V�����̃Z�b�g������̂��H�R���|�[�l���g���W������H
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

	//���ꂩ��s��FABRIK�v�Z�̂��߂ɗp���郏�[�N�f�[�^�����
	FIKChainPractice IKChain;
	IKChain.InitializeFromLegData(InLegData, SkelComp);

	// FABRIK�v�Z���s��
	IKChain.ReachTarget(FootIKLocation, InReachPrecision, InMaxIterations);

	// FABRIK�v�Z�������ʂ�FIKChainPractice���X�P���g���̃|�[�Y�ɔ��f����
	// Update bone transforms based on IKChain

	// Rotations
	for (int32 LinkIndex = InLegData.NumBones - 2; LinkIndex >= 0; LinkIndex--) // �q�b�v�W���C���g�͊܂߂��A���̈�q����G�t�F�N�^�܂Ń��[�v
	{
		// �����N�z��̕���Location���������Ȃ��B�W���C���g��Rotation�����̓����N�z���Location����v�Z����
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
	for (int32 LinkIndex = InLegData.NumBones - 2; LinkIndex >= 0; LinkIndex--) // �q�b�v�W���C���g�͊܂߂��A���̈�q����G�t�F�N�^�܂Ń��[�v
	{
		const FIKChainLinkPractice CurrentLink = IKChain.Links[LinkIndex];
		FTransform& CurrentTransform = InLegData.FKLegBoneTransforms[LinkIndex];
		CurrentTransform.SetTranslation(CurrentLink.Location);
	}
}
}

void FIKChainPractice::InitializeFromLegData(const FAnimLegIKDataPractice& InLegData, USkeletalMeshComponent* InSkelMeshComp)
{
	// FAnimLegIKDataPractice�ł̓W���C���g�P�ʂŃf�[�^�������Ă������AIK�v�Z�ł̓W���C���g�̊Ԃ��Ȃ������N(���ꂱ���{�[���I�Ȃ���)�P�ʂ�
	// ���[�N�f�[�^�����悤�ɂ���
	Links.Reset(InLegData.NumBones);
	MaximumReach = 0.0f;

	check(InLegData.NumBones > 1);
	for (int32 Index = 0; Index < InLegData.NumBones - 1; Index++)
	{
		const FVector BoneLocation = InLegData.FKLegBoneTransforms[Index].GetLocation();
		const FVector ParentLocation = InLegData.FKLegBoneTransforms[Index + 1].GetLocation();
		const float BoneLength = FVector::Dist(BoneLocation, ParentLocation);
		Links.Add(FIKChainLinkPractice(BoneLocation, BoneLength)); //�e����łȂ��q����ɂ��ă`�F�C�������N�����񂾂�
		MaximumReach += BoneLength;
	}

	// Add root bone last
	// �q�b�v�W���C���g�͒���0�Ƃ��������ōŌ�ɓ����B�����NumBones�̃����N���ł���B�z��̈�����FKLegBoneIndices/FKLegBoneTransforms��
	// �����ɂł���悤�ɁA�����ĕs�v�ȃq�b�v�W���C���g�����Ă���͗l
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
	if (!bInitialized) //bInitialized�ϐ����g�������̔�����ĕK�v���HInitializeFromLegData�Ŏ��s��Ԃ��āAReachTarget�ɗ��Ȃ��悤�ɂ��Ă����΂悩���������ł�
	{
		return;
	}

	const FVector RootLocation = Links.Last().Location;

	// TwoBoneIK�ł���������肾��
	// If we can't reach, we just go in a straight line towards the target,
	if ((NumLinks <= 2) // �W���C���g��2�����Ȃ��Ƃ�
		|| (FVector::DistSquared(RootLocation, InTargetLocation) >= FMath::Square(GetMaximumReach()))) // �W���C���g��L�΂��؂��Ă��^�[�Q�b�g�ɓ͂��Ȃ��Ƃ�
	{
		// �L�΂��������|�[�Y�ɂ���
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
	for (int32 Index = Links.Num() - 2; Index >= 0; Index--) // �q�b�v�W���C���g�͊܂߂��A���̈�q����G�t�F�N�^�܂Ń��[�v
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
			//TODO:����͂Ȃ����̂��킩��Ȃ�
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
	return (LegsData.Num() > 0); // InitializeBoneReferences�͊m���R���p�C�����ɌĂ΂�邯�ǂ������͎��s���ɌĂ΂��
}

namespace
{
void PopulateLegBoneIndices(FAnimLegIKDataPractice& InLegData, const FCompactPoseBoneIndex& InFootBoneIndex, const int32& NumBonesInLimb, const FBoneContainer& RequiredBones)
{
	FCompactPoseBoneIndex BoneIndex = InFootBoneIndex;
	if (BoneIndex != INDEX_NONE)
	{
		InLegData.FKLegBoneIndices.Add(BoneIndex); // �C���f�b�N�X0�ɓ����̂̓G�t�F�N�^IKBone
		FCompactPoseBoneIndex ParentBoneIndex = RequiredBones.GetParentBoneIndex(BoneIndex);

		int32 NumIterations = NumBonesInLimb;
		while (NumIterations-- > 0 && ParentBoneIndex != INDEX_NONE) // NumBonesInLimb�̐������A���邢��Root�ɂ��܂ł����̂ڂ���FKLegBoneIndices�ɓo�^����
		{
			BoneIndex = ParentBoneIndex;
			InLegData.FKLegBoneIndices.Add(BoneIndex); // �C���f�b�N�X0�ɓ����̂̓G�t�F�N�^IKBone
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
