// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimGraphNode_SplineSpine.h"


/////////////////////////////////////////////////////
// UAnimGraphNode_LegIK

#define LOCTEXT_NAMESPACE "AnimGraph_SplineSpine"

UAnimGraphNode_SplineSpine::UAnimGraphNode_SplineSpine(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_SplineSpine::GetControllerDescription() const
{
	return LOCTEXT("SplineSpine", "Spline Spine");
}

FText UAnimGraphNode_SplineSpine::GetTooltipText() const
{
	return LOCTEXT("AnimGraphNode_SplineSpine_Tooltip", "IK node for multi-bone legs.");
}

FText UAnimGraphNode_SplineSpine::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return GetControllerDescription();
}

#undef LOCTEXT_NAMESPACE
