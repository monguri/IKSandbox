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

namespace
{
FVector FindPlaneNormal(const TArray<FIKChainLinkPractice>& Links, const FVector& RootLocation, const FVector& TargetLocation)
{
	// UE4�ł́A�ʏ�̓{�[���̕�����X�����B
	const FVector AxisX = (TargetLocation - RootLocation).GetSafeNormal();

	// �q�b�v���珇�Ƀ����N�����ǂ��Ă����AAxisX�ƕ��ʂ��`�ł���x�N�g����������΂���ŕ��ʂ��`����
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
	// ���łɂ̂т����Ĉ꒼���ɂȂ��Ă���Ƃ����Ȃ�
	return FVector::UpVector; //(0.0f, 0.0f, 1.0f) �K��
}

void FABRIK_ForwardReach(const FVector& InTargetLocation, FIKChainPractice& IKChain)
{
}

void FABRIK_BackwardReach(const FVector& InRootTargetLocation, FIKChainPractice& IKChain)
{
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
	const float PullDistributionAlpha = 0.5f; //TODO:���Ƃ̃\�[�X����CVar�����A�����ł�CVar�̃f�t�H���g�l���̗p

	const FVector RootTargetLocation = Links.Last().Location;

	// Check distance between foot and foot target location
	float Slop = FVector::Dist(Links[0].Location, InTargetLocation);
	if (Slop > ReachPrecision) // ReachPrecision��荷�����������Ȃ�����FABRIK�����͂��Ȃ�
	{
		if (bEnableRotationLimit)
		{
			// Since we've previously aligned the foot with the IK Target, we're solving IK in 2D space on a single plane.
			// Find Plane Normal, to use in rotation constraints.
			// TODO:��2D space�̌v�Z�ɂȂ�킯�Ȃ�����B�W���C���g���΂˂݂����ɂ˂��ꂽ�\���ɂȂ��Ă�\���͑傢�ɂ���
			// OrientLegTowardsIK�̓G�t�F�N�^��FKBone�̃q�b�v����̕�����IKBone�̕����ɂȂ�悤�ɂ����ɉ߂��Ȃ�
			// �����炭�����AFABRIK���ƁA����ł�2D space�Ɏˉe���čl����K�v������̂ł͂Ȃ��낤���B
			// ������Œ肵�����ʂ̖@�������Ƃ��đS�W���C���g��FABRIK�v�Z���Z���������߂�̂ł́H

			const FVector PlaneNormal = FindPlaneNormal(Links, RootTargetLocation, InTargetLocation);

			for (int32 LinkIndex = 1; LinkIndex < NumLinks - 1; LinkIndex++)
			{
				const FIKChainLinkPractice& ChildLink = Links[LinkIndex - 1]; // �G�t�F�N�^���炻�̂ЂƂ�̃W���C���g�ւ̃����N�ɂ͎�����Ȃ�
				// �����A���̃����N������Location�͎q���̃W���C���g�̃��P�[�V�����Ȃ̂ŁA���ǂ̓G�t�F�N�^�̂ЂƂ����̃W���C���g����LinkAxisZ���������邱�ƂɂȂ�
				FIKChainLinkPractice& CurrentLink = Links[LinkIndex];
				const FIKChainLinkPractice& ParentLink = Links[LinkIndex + 1]; // �q�b�v�W���C���g�����̍Ō�̃����N�ɂ͎�����Ȃ�
 
				const FVector ChildAxisX = (ChildLink.Location - CurrentLink.Location).GetSafeNormal();
				const FVector ChildAxisY = PlaneNormal ^ ChildAxisX; // Z ^ X = Y
				const FVector ParentAxisX = (ParentLink.Location - CurrentLink.Location).GetSafeNormal();

				// Orient Z, so that ChildAxisY points 'up' and produces positive Sin values.
				CurrentLink.LinkAxisZ = (ParentAxisX | ChildAxisY) > 0.0f ? PlaneNormal : -PlaneNormal; // TODO:���̗����͂悭�킩���ȁB���Ƃ̌v�Z�̓s�����H
			}
		}

		// Re-position limb to distribute pull
		// ����͂��Ԃ�FABRIK���L�̒������Ȃ̂��낤���A�S�W���C���g�ɂ��炩���߁A�G�t�F�N�^�̖ڕW�ւ̍����ɃA���t�@���������I�t�Z�b�g�������Ă���
		const FVector PullDistributionOffset = PullDistributionAlpha * (InTargetLocation - Links[0].Location) + (1.0f - PullDistributionAlpha) * (RootTargetLocation - Links.Last().Location); // ��҂�0�x�N�g���ɂȂ�Ɍ��܂��Ă���B�����猋�ǂ̓G�t�F�N�^�̌��̈ʒu�ƖڕW�ʒu�̍��ɃA���t�@�����������̂ɂȂ�
		for (int32 LinkIndex = 0; LinkIndex < NumLinks; LinkIndex++)
		{
			Links[LinkIndex].Location = Links[LinkIndex].Location + PullDistributionOffset;
		}

		// FABRIK���[�v
		int32 IterationCount = 1;
		const int32 MaxIterations = FMath::Max(InMaxIterations, 1);
		// �K������FABRIK�v�Z�����̂�do-while���g��
		do
		{
			//TODO:����
		} while ((Slop > ReachPrecision) && (IterationCount++ < MaxIterations));

		// Make sure our root is back at our root target.
		// �����̌����Ńq�b�v�����̈ʒu���炸��Ă����������񕪂���backward�����
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
