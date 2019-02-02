// Copyright 1998-2018 Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "BonePose.h"
#include "AnimGraphNode_Base.h"
#include "AnimGraphNode_SkeletalControlBaseLocal.generated.h"

class FCompilerResultsLog;
class USkeletalMeshComponent;
struct FAnimNode_SkeletalControlBaseLocal;
struct HActor;

/**
 * This is the base class for the 'source version' of all skeletal control animation graph nodes
 * (nodes that manipulate the pose rather than playing animations to create a pose or blending between poses)
 *
 * Concrete subclasses should contain a member struct derived from FAnimNode_SkeletalControlBase
 */
UCLASS(Abstract)
class IKPLUGINSANDBOX_API UAnimGraphNode_SkeletalControlBaseLocal : public UAnimGraphNode_Base
{
	GENERATED_UCLASS_BODY()

public:
	// UEdGraphNode interface
	virtual FLinearColor GetNodeTitleColor() const override;
	virtual FText GetTooltipText() const override;
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
	// End of UEdGraphNode interface

	// UAnimGraphNode_Base interface
	virtual FString GetNodeCategory() const override;
	virtual void CreateOutputPins() override;
	virtual void ValidateAnimNodePostCompile(FCompilerResultsLog& MessageLog, UAnimBlueprintGeneratedClass* CompiledClass, int32 CompiledNodeIndex) override;
	virtual void CustomizePinData(UEdGraphPin* Pin, FName SourcePropertyName, int32 ArrayIndex) const override;
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
	// End of UAnimGraphNode_Base interface

	/** Are we currently showing this pin */
	bool IsPinShown(const FName PinName) const;

public:
	// set literal value for FVector
	void SetDefaultValue(const FName InDefaultValueName, const FVector& InValue);
	// get literal value for vector
	void GetDefaultValue(const FName UpdateDefaultValueName, FVector& OutVec);

	void GetDefaultValue(const FName PropName, FRotator& OutValue)
	{
		FVector Value;
		GetDefaultValue(PropName, Value);
		OutValue.Pitch = Value.X;
		OutValue.Yaw = Value.Y;
		OutValue.Roll = Value.Z;
	}

	template<class ValueType>
	ValueType GetNodeValue(const FName PropName, const ValueType& CompileNodeValue)
	{
		if (IsPinShown(PropName))
		{
			ValueType Val;
			GetDefaultValue(PropName, Val);
			return Val;
		}
		return CompileNodeValue;
	}

	void SetDefaultValue(const FName PropName, const FRotator& InValue)
	{
		FVector VecValue(InValue.Pitch, InValue.Yaw, InValue.Roll);
		SetDefaultValue(PropName, VecValue);
	}

	template<class ValueType>
	void SetNodeValue(const FName PropName, ValueType& CompileNodeValue, const ValueType& InValue)
	{
		if (IsPinShown(PropName))
		{
			SetDefaultValue(PropName, InValue);
		}
		CompileNodeValue = InValue;
	}

protected:
	// Returns the short descriptive name of the controller
	virtual FText GetControllerDescription() const;

	virtual const FAnimNode_SkeletalControlBaseLocal* GetNode() const PURE_VIRTUAL(UAnimGraphNode_SkeletalControlBaseLocal::GetNode, return nullptr;);
};
