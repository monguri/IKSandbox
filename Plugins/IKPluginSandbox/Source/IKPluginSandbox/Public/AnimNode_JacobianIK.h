// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "JacobianIKUtility.h"
#include "AnimNode_JacobianIK.generated.h"

using namespace JacobianIKUtility;

USTRUCT()
struct FIKConstraint
{
	//TODO: JacobianIKUtility.hの中に置きたかったがコンパイルエラーを解消できずしょうがなく
	GENERATED_USTRUCT_BODY()

	/** fixed position constraint. **/
	UPROPERTY(EditAnywhere)
	FVector Position; // 今のところ位置のコンストレイントだけ。今後増やす
};

USTRUCT()
struct FIKJoint
{
	//TODO: JacobianIKUtility.hの中に置きたかったがコンパイルエラーを解消できずしょうがなく
	GENERATED_USTRUCT_BODY()

	/** target joint of constraint. **/
	UPROPERTY(EditAnywhere)
	FBoneReference Joint;

	/** parent joint of target joint. If it equals target joint, it is treated as root joint. **/
	UPROPERTY(EditAnywhere)
	FBoneReference ParentJoint; //TODO: parentをわざわざ指定させるというのはあまりきれいなやり方ではない

	/** constraints. **/
	UPROPERTY(EditAnywhere)
	TArray<FIKConstraint> Constraints;
};

USTRUCT(BlueprintInternalUseOnly)
struct IKPLUGINSANDBOX_API FAnimNode_JacobianIK : public FAnimNode_SkeletalControlBase
{
	GENERATED_USTRUCT_BODY()

public:
	/** Joints which are calculated by IK. **/
	UPROPERTY(EditAnywhere, Category=IK)
	TArray<FIKJoint> IKSkeleton;

	/** The number of iteration. **/
	UPROPERTY(EditAnywhere, Category=IK)
	uint32 NumIteration;

	/** Tolerance for IK Target and IK joint length, in unreal units. */
	UPROPERTY(EditAnywhere, Category=IK)
	float Precision;

	/** Normalize parameter for singular pose. */
	UPROPERTY(EditAnywhere, Category=IK)
	float Lambda;

	FAnimNode_JacobianIK();

private:
	struct IKJointWorkData
	{
		FCompactPoseBoneIndex ParentJointIndex; // TODO:JointIndexに名前を変えたい
		FTransform ComponentTransform;
		FTransform LocalTransform;
		TArray<FIKConstraint> Constraints;

		IKJointWorkData() : ParentJointIndex(INDEX_NONE), ComponentTransform(FTransform::Identity), LocalTransform(FTransform::Identity) {}
		IKJointWorkData(FCompactPoseBoneIndex _ParentJointIndex, const FTransform& _ComponentTransform, const FTransform& _LocalTransform, const TArray<FIKConstraint> _Constraints) : ParentJointIndex(_ParentJointIndex), ComponentTransform(_ComponentTransform), LocalTransform(_LocalTransform), Constraints(_Constraints) {}
	};

	TMap<int32, IKJointWorkData> IKJointWorkDataMap;

	AnySizeMatrix Jacobian;
	AnySizeMatrix Jt;
	AnySizeMatrix JJt;
	AnySizeMatrix JJtPlusLambdaI;
	AnySizeMatrix JJti;
	AnySizeMatrix PseudoInverseJacobian;
	AnySizeMatrix LambdaI;
	TArray<float> IterationStepPosition;
	TArray<float> IterationStepAngles;

	// FAnimNode_SkeletalControlBase interface
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface
};
