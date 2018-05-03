// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "EdGraph/EdGraphNodeUtils.h"
#include "AnimGraphNode_SkeletalControlBase.h"
#include "AnimNode_TwoBoneIKPractice.h"
#include "AnimGraphNode_TwoBoneIKPractice.generated.h"

/**
 * 
 */
UCLASS(MinimalAPI, meta=(Keywords = "Look At, Follow, Trace, Track"))
class UAnimGraphNode_TwoBoneIKPractice : public UAnimGraphNode_SkeletalControlBase
{
	GENERATED_UCLASS_BODY()
	
	UPROPERTY(EditAnywhere, Category=Settings)
	FAnimNode_TwoBoneIKPractice Node;
	
public:
	// UEdGraphNode interface
	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
	virtual FText GetTooltipText() const override;
	// End of UEdGraphNode interface
	
	// UAnimGraphNode_SkeletalControlBase interface
	virtual const FAnimNode_SkeletalControlBase* GetNode() const override { return &Node; }
	// End of UAnimGraphNode_SkeletalControlBase interface

protected:
	// UAnimGraphNode_SkeletalControlBase interface
	virtual FText GetControllerDescription() const override;
	// End of UAnimGraphNode_SkeletalControlBase interface

private:
	/** Constructing FText strings can be costly, so we cache the node's title */
	FNodeTitleTextTable CachedNodeTitles;
};
