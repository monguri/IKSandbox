// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "EdGraph/EdGraphNodeUtils.h"
#include "AnimGraphNode_SkeletalControlBaseLocal.h"
#include "AnimNode_JacobianIKLocal.h"
#include "AnimGraphNode_JacobianIKLocal.generated.h"

/**
 * 
 */
UCLASS(MinimalAPI)
class UAnimGraphNode_JacobianIKLocal : public UAnimGraphNode_SkeletalControlBaseLocal
{
	GENERATED_UCLASS_BODY()
	
	UPROPERTY(EditAnywhere, Category=Settings)
	FAnimNode_JacobianIKLocal Node;
	
public:
	// UEdGraphNode interface
	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
	virtual FText GetTooltipText() const override;
	// End of UEdGraphNode interface
	
	// UAnimGraphNode_SkeletalControlBase interface
	virtual const FAnimNode_SkeletalControlBaseLocal* GetNode() const override { return &Node; }
	// End of UAnimGraphNode_SkeletalControlBase interface

protected:
	// UAnimGraphNode_SkeletalControlBase interface
	virtual FText GetControllerDescription() const override;
	// End of UAnimGraphNode_SkeletalControlBase interface
};
