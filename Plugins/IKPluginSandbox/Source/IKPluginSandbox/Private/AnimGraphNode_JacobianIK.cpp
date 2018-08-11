// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimGraphNode_JacobianIK.h"


/////////////////////////////////////////////////////
// UAnimGraphNode_LookAt

#define LOCTEXT_NAMESPACE "AnimGraph_JacobianIK"

UAnimGraphNode_JacobianIK::UAnimGraphNode_JacobianIK(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_JacobianIK::GetControllerDescription() const
{
	return LOCTEXT("JacobianIKNode", "CCD-IK");
}

FText UAnimGraphNode_JacobianIK::GetTooltipText() const
{
	return LOCTEXT("AnimGraphNode_JacobianIK_Tooltip", "This node allow a bone to trace or follow another bone");
}

FText UAnimGraphNode_JacobianIK::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return GetControllerDescription();
}

#undef LOCTEXT_NAMESPACE
