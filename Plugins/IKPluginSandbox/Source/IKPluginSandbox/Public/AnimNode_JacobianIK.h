// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "JacobianIKUtility.h"
#include "AnimNode_JacobianIK.generated.h"

using namespace JacobianIKUtility;

UENUM()
enum class EIKConstraintType : uint8
{
	INVALID,
	KEEP_POSITION,
	KEEP_ROTATION,
};

USTRUCT()
struct FIKConstraint
{
	//TODO: JacobianIKUtility.hの中に置きたかったがコンパイルエラーを解消できずしょうがなく
	GENERATED_USTRUCT_BODY()

	/** constraint type. **/
	UPROPERTY(EditAnywhere)
	EIKConstraintType Type;

	/** constraint position. **/
	UPROPERTY(EditAnywhere)
	FVector Position;

	/** constraint rotation. **/
	UPROPERTY(EditAnywhere)
	FRotator Rotation;

	/** root joint of joints chain which effects to this IK constraint. **/
	UPROPERTY(EditAnywhere)
	FBoneReference EffectiveRootJoint;
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
		/** parent joint index of this joint. If it equals this joint index, it is treated as root joint. **/
		FCompactPoseBoneIndex ParentJointIndex;
		/** Component space transform work data. **/
		FTransform ComponentTransform;
		/** Local space transform work data. **/
		FTransform LocalTransform;

		IKJointWorkData() : ParentJointIndex(INDEX_NONE), ComponentTransform(FTransform::Identity), LocalTransform(FTransform::Identity) {}
		IKJointWorkData(const FCompactPoseBoneIndex& _ParentJointIndex, const FTransform& _ComponentTransform, const FTransform& _LocalTransform, const TArray<FIKConstraint> _Constraints) : ParentJointIndex(_ParentJointIndex), ComponentTransform(_ComponentTransform), LocalTransform(_LocalTransform) {}
	};

	struct IKConstraintWorkData
	{
		/** target joint index of constraint. **/
		FCompactPoseBoneIndex JointIndex;
		/** joints chain indices which effects to this IK constraint. It does not include self joint. **/
		TArray<FCompactPoseBoneIndex> EffectiveJointIndices; // このコンストレイントIK計算を行う
		/** constraint type. **/
		EIKConstraintType Type;
		/** constraint position **/
		FVector Position;
		/** constraint rotation. **/
		FRotator Rotation;

		IKConstraintWorkData(const FCompactPoseBoneIndex& _JointIndex, const TArray<FCompactPoseBoneIndex>& _EffectiveJointIndices, EIKConstraintType _Type, const FVector& _Position, const FRotator& _Rotation) : JointIndex(_JointIndex), EffectiveJointIndices(_EffectiveJointIndices), Type(_Type), Position(_Position), Rotation(_Rotation) {};
	};

	TMap<int32, IKJointWorkData> IKJointWorkDataMap;
	TArray<IKConstraintWorkData> IKConstraintWorkDataArray;

	bool bValidToEvaluate;

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
