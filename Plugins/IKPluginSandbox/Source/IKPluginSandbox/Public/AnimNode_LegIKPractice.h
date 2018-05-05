// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "AnimNode_LegIKPractice.generated.h"

/** Per foot definitions */
USTRUCT()
struct FAnimLegIKDefinitionPractice
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, Category = "Settings")
	FBoneReference IKFootBone;

	UPROPERTY(EditAnywhere, Category = "Settings")
	FBoneReference FKFootBone;

	UPROPERTY(EditAnywhere, Category = "Settings")
	int32 NumBonesInLimb;

	/** Forward Axis for Foot bone. */
	UPROPERTY(EditAnywhere, Category = "Settings")
	TEnumAsByte<EAxis::Type> FootBoneForwardAxis;

	/** If enabled, we prevent the leg from bending backwards and enforce a min compression angle */
	UPROPERTY(EditAnywhere, Category = "Settings")
	bool bEnableRotationLimit;

	/** Only used if bEnableRotationLimit is enabled. Prevents the leg from folding onto itself,
	* and forces at least this angle between Parent and Child bone. */
	UPROPERTY(EditAnywhere, Category = "Settings")
	float MinRotationAngle;

	FAnimLegIKDefinitionPractice()
		: NumBonesInLimb(2)
		, FootBoneForwardAxis(EAxis::Y)
		, bEnableRotationLimit(false)
		, MinRotationAngle(15.f)
	{}
};

/** Runtime foot data after validation, we guarantee these bones to exist */
USTRUCT()
struct FAnimLegIKDataPractice
{
	GENERATED_USTRUCT_BODY()

public:
	FCompactPoseBoneIndex IKFootBoneIndex;
	FTransform IKFootTransform;

	//TODO:���|�C���^�Ŏ����Ă��邪���v���H
	FAnimLegIKDefinitionPractice* LegDefPtr;

	int32 NumBones;
	TArray<FCompactPoseBoneIndex> FKLegBoneIndices;
	TArray<FTransform > FKLegBoneTransforms;

public:
	FAnimLegIKDataPractice()
		: IKFootBoneIndex(INDEX_NONE)
		, LegDefPtr(nullptr)
		, NumBones(INDEX_NONE)
	{}
};

USTRUCT()
struct FIKChainLinkPractice
{
	GENERATED_USTRUCT_BODY()
	// TODO:��ŕK�v�ɂȂ������ɕK�v�ɂȂ������̂��`����
};

USTRUCT()
struct FIKChainPractice
{
	GENERATED_USTRUCT_BODY()
	// TODO:��ŕK�v�ɂȂ������ɕK�v�ɂȂ������̂��`����
};

USTRUCT(BlueprintInternalUseOnly)
struct IKPLUGINSANDBOX_API FAnimNode_LegIKPractice : public FAnimNode_SkeletalControlBase
{
	GENERATED_USTRUCT_BODY()

	FAnimNode_LegIKPractice();

	/** Tolerance for reaching IK Target, in unreal units. */
	UPROPERTY(EditAnywhere, Category = "Settings")
	float ReachPrecision;

	/** Max Number of Iterations. */
	UPROPERTY(EditAnywhere, Category = "Settings")
	int32 MaxIterations;

	UPROPERTY(EditAnywhere, Category = "Settings")
	TArray<FAnimLegIKDefinitionPractice> LegsDefinition;

	UPROPERTY(Transient)
	TArray<FAnimLegIKDataPractice> LegsData;

public:
	// FAnimNode_SkeletalControlBase interface
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface

private:
	// FAnimNode_SkeletalControlBase interface
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface
};
