// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "AnimNode_SkeletalControlBaseLocal.h"
#include "BoneIndices.h"
#include "JacobianIKUtility.h"
#include "AnimNode_JacobianIKLocal.generated.h"

using namespace JacobianIKUtility;

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

	/** Toggle debug draw. */
	UPROPERTY(EditAnywhere, Category=Debug)
	bool bDebugDraw;

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
