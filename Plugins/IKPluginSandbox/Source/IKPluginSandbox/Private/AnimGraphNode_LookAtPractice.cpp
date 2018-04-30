// Fill out your copyright notice in the Description page of Project Settings.

#include "AnimGraphNode_LookAtPractice.h"


/////////////////////////////////////////////////////
// UAnimGraphNode_LookAt

#define LOCTEXT_NAMESPACE "AnimGraph_LookAtPractice"

UAnimGraphNode_LookAtPractice::UAnimGraphNode_LookAtPractice(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_LookAtPractice::GetControllerDescription() const
{
	return LOCTEXT("LookAtPracticeNode", "Look At Practice");
}

FText UAnimGraphNode_LookAtPractice::GetTooltipText() const
{
	return LOCTEXT("AnimGraphNode_LookAtPractice_Tooltip", "This node allow a bone to trace or follow another bone");
}

FText UAnimGraphNode_LookAtPractice::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	if ((TitleType == ENodeTitleType::ListView || TitleType == ENodeTitleType::MenuTitle) && (Node.BoneToModify.BoneName == NAME_None))
	{
		return GetControllerDescription();
	}
	// @TODO: the bone can be altered in the property editor, so we have to 
	//        choose to mark this dirty when that happens for this to properly work
	else //if (!CachedNodeTitles.IsTitleCached(TitleType, this))
	{
		FFormatNamedArguments Args;
		Args.Add(TEXT("ControllerDescription"), GetControllerDescription());
		Args.Add(TEXT("BoneName"), FText::FromName(Node.BoneToModify.BoneName));

		// FText::Format() is slow, so we cache this to save on performance
		if (TitleType == ENodeTitleType::ListView || TitleType == ENodeTitleType::MenuTitle)
		{
			CachedNodeTitles.SetCachedTitle(TitleType, FText::Format(LOCTEXT("AnimGraphNode_LookAtPractice_ListTitle", "{ControllerDescription} - Bone: {BoneName}"), Args), this);
		}
		else
		{
			CachedNodeTitles.SetCachedTitle(TitleType, FText::Format(LOCTEXT("AnimGraphNode_LookAt_Title", "{ControllerDescription}\nBone: {BoneName}"), Args), this);
		}
	}
	return CachedNodeTitles[TitleType];		
}

#undef LOCTEXT_NAMESPACE
