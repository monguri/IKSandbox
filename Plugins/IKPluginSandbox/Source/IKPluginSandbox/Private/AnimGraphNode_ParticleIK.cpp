// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimGraphNode_ParticleIK.h"


/////////////////////////////////////////////////////
// UAnimGraphNode_LookAt

#define LOCTEXT_NAMESPACE "AnimGraph_ParticleIK"

UAnimGraphNode_ParticleIK::UAnimGraphNode_ParticleIK(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_ParticleIK::GetControllerDescription() const
{
	return LOCTEXT("ParticleIKNode", "CCD-IK");
}

FText UAnimGraphNode_ParticleIK::GetTooltipText() const
{
	return LOCTEXT("AnimGraphNode_ParticleIK_Tooltip", "This node allow a bone to trace or follow another bone");
}

FText UAnimGraphNode_ParticleIK::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return GetControllerDescription();
}

#undef LOCTEXT_NAMESPACE
