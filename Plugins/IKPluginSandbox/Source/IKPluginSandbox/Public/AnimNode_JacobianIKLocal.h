// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "AnimNode_SkeletalControlBaseLocal.h"
#include "BoneIndices.h"
#include "AnimNode_JacobianIKLocal.generated.h"

USTRUCT(BlueprintInternalUseOnly)
struct IKPLUGINSANDBOX_API FAnimNode_JacobianIKLocal : public FAnimNode_SkeletalControlBaseLocal
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

	/** Normalize parameter for singular pose. */
	UPROPERTY(EditAnywhere, Category=IK)
	float Lambda;

	FAnimNode_JacobianIKLocal();

private:
	struct IKJointWorkData
	{
		FCompactPoseBoneIndex BoneIndex;
		FTransform ComponentTransform;
		FTransform LocalTransform;

		IKJointWorkData() : BoneIndex(INDEX_NONE), ComponentTransform(FTransform::Identity), LocalTransform(FTransform::Identity) {}
		IKJointWorkData(FCompactPoseBoneIndex _BoneIndex, const FTransform& _ComponentTransform, const FTransform& _LocalTransform) : BoneIndex(_BoneIndex), ComponentTransform(_ComponentTransform), LocalTransform(_LocalTransform) {}
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
		static void Add(const AnySizeMatrix& A, const AnySizeMatrix& B, AnySizeMatrix& OutResult);
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
	AnySizeMatrix JJt;
	AnySizeMatrix JJtPlusLambdaI;
	AnySizeMatrix JJti;
	AnySizeMatrix PseudoInverseJacobian;
	AnySizeMatrix LambdaI;
	TArray<float> IterationStepPosition;
	TArray<float> IterationStepAngles;

	// FAnimNode_SkeletalControlBaseLocal interface
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
	virtual void EvaluateSkeletalControl_AnyThread(FCSPose<FCompactPose>& InPose, FPoseContext& Output) override;
	// End of FAnimNode_SkeletalControlBaseLocal interface
};
