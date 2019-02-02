// Copyright 1998-2018 Epic Games, Inc. All Rights Reserved.

#include "AnimNode_SkeletalControlBaseLocal.h"
#include "Animation/AnimInstanceProxy.h"
#include "Engine/SkeletalMeshSocket.h"

/////////////////////////////////////////////////////
// FAnimNode_SkeletalControlBaseLocal

void FAnimNode_SkeletalControlBaseLocal::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
	FAnimNode_Base::Initialize_AnyThread(Context);

	Pose.Initialize(Context);

	AlphaBoolBlend.Reinitialize();
	AlphaScaleBiasClamp.Reinitialize();
}

void FAnimNode_SkeletalControlBaseLocal::CacheBones_AnyThread(const FAnimationCacheBonesContext& Context) 
{
	FAnimNode_Base::CacheBones_AnyThread(Context);
	InitializeBoneReferences(Context.AnimInstanceProxy->GetRequiredBones());
	Pose.CacheBones(Context);
}

void FAnimNode_SkeletalControlBaseLocal::UpdateInternal(const FAnimationUpdateContext& Context)
{
}

void FAnimNode_SkeletalControlBaseLocal::Update_AnyThread(const FAnimationUpdateContext& Context)
{
	Pose.Update(Context);

	ActualAlpha = 0.f;
	if (IsLODEnabled(Context.AnimInstanceProxy))
	{
		EvaluateGraphExposedInputs.Execute(Context);

		// Apply the skeletal control if it's valid
		switch (AlphaInputType)
		{
		case EAnimAlphaInputType::Float : 
			ActualAlpha = AlphaScaleBias.ApplyTo(AlphaScaleBiasClamp.ApplyTo(Alpha, Context.GetDeltaTime()));
			break;
		case EAnimAlphaInputType::Bool :
			ActualAlpha = AlphaBoolBlend.ApplyTo(bAlphaBoolEnabled, Context.GetDeltaTime());
			break;
		case EAnimAlphaInputType::Curve :
			if (UAnimInstance* AnimInstance = Cast<UAnimInstance>(Context.AnimInstanceProxy->GetAnimInstanceObject()))
			{
				ActualAlpha = AlphaScaleBiasClamp.ApplyTo(AnimInstance->GetCurveValue(AlphaCurveName), Context.GetDeltaTime());
			}
			break;
		};

		// Make sure Alpha is clamped between 0 and 1.
		ActualAlpha = FMath::Clamp<float>(ActualAlpha, 0.f, 1.f);

		if (FAnimWeight::IsRelevant(ActualAlpha) && IsValidToEvaluate(Context.AnimInstanceProxy->GetSkeleton(), Context.AnimInstanceProxy->GetRequiredBones()))
		{
			UpdateInternal(Context);
		}
	}
}

bool ContainsNaN(const TArray<FBoneTransform> & BoneTransforms)
{
	for (int32 i = 0; i < BoneTransforms.Num(); ++i)
	{
		if (BoneTransforms[i].Transform.ContainsNaN())
		{
			return true;
		}
	}

	return false;
}

void FAnimNode_SkeletalControlBaseLocal::EvaluateInternal(FPoseContext& Output)
{
}

void FAnimNode_SkeletalControlBaseLocal::Evaluate_AnyThread(FPoseContext& Output)
{
	Pose.Evaluate(Output);

	// save current pose before applying skeletal control to compute the exact gizmo location in AnimGraphNode
	// forwarded pose data from the wired node which current node's skeletal control is not applied yet
	FCSPose<FCompactPose> ForwardedPose;
	ForwardedPose.InitPose(Output.Pose);

	// this is to ensure Source data does not contain NaN
	ensure(Output.ContainsNaN() == false);

	// Apply the skeletal control if it's valid
	if (FAnimWeight::IsRelevant(ActualAlpha) && IsValidToEvaluate(Output.AnimInstanceProxy->GetSkeleton(), Output.AnimInstanceProxy->GetRequiredBones()))
	{
		EvaluateInternal(Output);
		EvaluateSkeletalControl_AnyThread(ForwardedPose, Output);
	}
}

void FAnimNode_SkeletalControlBaseLocal::AddDebugNodeData(FString& OutDebugData)
{
	OutDebugData += FString::Printf(TEXT("Alpha: %.1f%%"), ActualAlpha*100.f);
}

void FAnimNode_SkeletalControlBaseLocal::EvaluateSkeletalControl_AnyThread(const FCSPose<FCompactPose>& InPose, FPoseContext& Output)
{
}

