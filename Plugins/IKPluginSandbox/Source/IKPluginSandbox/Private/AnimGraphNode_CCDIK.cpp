// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimGraphNode_CCDIK.h"


/////////////////////////////////////////////////////
// UAnimGraphNode_LookAt

#define LOCTEXT_NAMESPACE "AnimGraph_CCDIK"

UAnimGraphNode_CCDIK::UAnimGraphNode_CCDIK(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_CCDIK::GetControllerDescription() const
{
	return LOCTEXT("CCDIKNode", "CCD-IK");
}

FText UAnimGraphNode_CCDIK::GetTooltipText() const
{
	return LOCTEXT("AnimGraphNode_CCDIK_Tooltip", "This node allow a bone to trace or follow another bone");
}

FText UAnimGraphNode_CCDIK::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return GetControllerDescription();
}

#undef LOCTEXT_NAMESPACE
