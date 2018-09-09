// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "AnimNode_JacobianIK.generated.h"

USTRUCT(BlueprintInternalUseOnly)
struct IKPLUGINSANDBOX_API FAnimNode_JacobianIK : public FAnimNode_SkeletalControlBase
{
	GENERATED_USTRUCT_BODY()

	/** Name of ik start joint. **/
	UPROPERTY(EditAnywhere, Category=IK)
	FBoneReference IKRootJoint;

	/** Name of ik effector joint. **/
	UPROPERTY(EditAnywhere, Category=IK)
	FBoneReference EffectorJoint;

	/** Effector Target Location. Component space. **/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category=IK, meta=(PinShownByDefault))
	FVector EffectorTargetLocation;

	/** The number of iteration. **/
	UPROPERTY(EditAnywhere, Category=IK)
	uint32 NumIteration;

	/** Tolerance for IK Target and IK joint length, in unreal units. */
	UPROPERTY(EditAnywhere, Category=IK)
	float Precision;

	FAnimNode_JacobianIK();

public:
	// FAnimNode_SkeletalControlBase interface
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface

private:
	struct IKJointWorkData
	{
		FCompactPoseBoneIndex BoneIndex;
		FTransform Transform;

		IKJointWorkData() : BoneIndex(INDEX_NONE), Transform(FTransform::Identity) {}
		IKJointWorkData(FCompactPoseBoneIndex _BoneIndex, const FTransform& _Transform) : BoneIndex(_BoneIndex), Transform(_Transform) {}
	};

	TArray<IKJointWorkData> IKJointWorkDatas;

	struct AnySizeMatrix
	{
		AnySizeMatrix();
		AnySizeMatrix(uint8 _NumRow, uint8 _NumColumn);
		float Get(uint8 Row, uint8 Column) const;
		void Set(uint8 Row, uint8 Column, float Value);
		void ZeroClear();
		static void Transpose(const AnySizeMatrix& InMatrix, AnySizeMatrix& OutMatrix);
		static void Multiply(const AnySizeMatrix& A, const AnySizeMatrix& B, AnySizeMatrix& OutResult);
		static float Inverse3x3(const AnySizeMatrix& InMatrix, AnySizeMatrix& OutMatrix);
		static void TransformVector(const AnySizeMatrix& InMatrix, const TArray<float>& InVector, TArray<float>& OutVector);

		TArray<float> Elements; // 1-dimensional array for access speed.
		uint8 NumRow;
		uint8 NumColumn;
	};

	// Parent joint of ik root joint
	FCompactPoseBoneIndex IKRootJointParent;

	AnySizeMatrix Jacobian;
	AnySizeMatrix Jt;
	AnySizeMatrix JtJ;
	AnySizeMatrix JtJi;
	AnySizeMatrix PseudoInverseJacobian;
	TArray<float> IterationStepPosition;
	TArray<float> IterationStepAngles;

	// FAnimNode_SkeletalControlBase interface
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface
};
