// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimGraphNode_LegIKPractice.h"


/////////////////////////////////////////////////////
// UAnimGraphNode_LegIK

#define LOCTEXT_NAMESPACE "AnimGraph_LegIKPractice"

UAnimGraphNode_LegIKPractice::UAnimGraphNode_LegIKPractice(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_LegIKPractice::GetControllerDescription() const
{
	return LOCTEXT("LegIK", "Leg IK");
}

FText UAnimGraphNode_LegIKPractice::GetTooltipText() const
{
	return LOCTEXT("AnimGraphNode_LegIK_Tooltip", "IK node for multi-bone legs.");
}

FText UAnimGraphNode_LegIKPractice::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return GetControllerDescription();
}

#undef LOCTEXT_NAMESPACE
