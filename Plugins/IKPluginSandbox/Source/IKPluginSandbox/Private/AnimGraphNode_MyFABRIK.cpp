// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimGraphNode_MyFABRIK.h"


/////////////////////////////////////////////////////
// UAnimGraphNode_LookAt

#define LOCTEXT_NAMESPACE "AnimGraph_MyFABRIK"

UAnimGraphNode_MyFABRIK::UAnimGraphNode_MyFABRIK(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_MyFABRIK::GetControllerDescription() const
{
	return LOCTEXT("My FABRIK Node", "MyFABRIK");
}

FText UAnimGraphNode_MyFABRIK::GetTooltipText() const
{
	return LOCTEXT("AnimGraphNode_MyFABRIK_Tooltip", "This node allow a bone to trace or follow another bone");
}

FText UAnimGraphNode_MyFABRIK::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return GetControllerDescription();
}

#undef LOCTEXT_NAMESPACE
