// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimGraphNode_JacobianIKLocal.h"


/////////////////////////////////////////////////////
// UAnimGraphNode_LookAt

#define LOCTEXT_NAMESPACE "AnimGraph_JacobianIKLocal"

UAnimGraphNode_JacobianIKLocal::UAnimGraphNode_JacobianIKLocal(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_JacobianIKLocal::GetControllerDescription() const
{
	return LOCTEXT("JacobianIKLocalNode", "Jacobian-IK Local");
}

FText UAnimGraphNode_JacobianIKLocal::GetTooltipText() const
{
	return LOCTEXT("AnimGraphNode_JacobianIKLocal_Tooltip", "This node allow a bone to trace or follow another bone");
}

FText UAnimGraphNode_JacobianIKLocal::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return GetControllerDescription();
}

#undef LOCTEXT_NAMESPACE
