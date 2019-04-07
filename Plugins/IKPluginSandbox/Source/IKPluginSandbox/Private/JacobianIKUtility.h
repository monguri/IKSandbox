#pragma once

namespace JacobianIKUtility
{
// XYZ��3�����Ɏg���萔
const int32 AXIS_COUNT = 3;
// ��]������3�����Ɏg���萔
const int32 ROTATION_AXIS_COUNT = 3;

// degree�P�ʂł�sin/cos�֐�
FORCEINLINE float SinD(float degree) { return FMath::Sin(FMath::DegreesToRadians(degree)); }
FORCEINLINE float CosD(float degree) { return FMath::Cos(FMath::DegreesToRadians(degree)); }

// �e���̉�]�s��
//TODO: �����̕�����A�s�D���D��ɂ��Ă͂��܂肫����ƍl���͂��Ă��炸�ق�������̃\�[�X���Q�l�ɂ��Ă���
FORCEINLINE FMatrix RotationX(float Roll)
{
	return FMatrix(
		FPlane(1, 0, 0, 0),
		FPlane(0, CosD(Roll), -SinD(Roll), 0),
		FPlane(0, SinD(Roll), CosD(Roll), 0),
		FPlane(0, 0, 0, 1)
	);
}

FORCEINLINE FMatrix RotationY(float Pitch)
{
	return FMatrix(
		FPlane(CosD(Pitch), 0, SinD(Pitch), 0),
		FPlane(0, 1, 0, 0),
		FPlane(-SinD(Pitch), 0, CosD(Pitch), 0),
		FPlane(0, 0, 0, 1)
	);
}

FORCEINLINE FMatrix RotationZ(float Yaw)
{
	return FMatrix(
		FPlane(CosD(Yaw), SinD(Yaw), 0, 0),
		FPlane(-SinD(Yaw), CosD(Yaw), 0, 0),
		FPlane(0, 0, 1, 0),
		FPlane(0, 0, 0, 1)
	);
}

// �e���̉�]�s�����]�p�̕ϐ��Ŕ��������s��
FORCEINLINE FMatrix RotationDifferentialX(float Roll)
{
	return FMatrix(
		FPlane(0, 0, 0, 0),
		FPlane(0, -SinD(Roll), -CosD(Roll), 0),
		FPlane(0, CosD(Roll), -SinD(Roll), 0),
		FPlane(0, 0, 0, 0)
	);
}

FORCEINLINE FMatrix RotationDifferentialY(float Pitch)
{
	return FMatrix(
		FPlane(-SinD(Pitch), 0, CosD(Pitch), 0),
		FPlane(0, 0, 0, 0),
		FPlane(-CosD(Pitch), 0, -SinD(Pitch), 0),
		FPlane(0, 0, 0, 0)
	);
}

FORCEINLINE FMatrix RotationDifferentialZ(float Yaw)
{
	return FMatrix(
		FPlane(-SinD(Yaw), CosD(Yaw), 0, 0),
		FPlane(-CosD(Yaw), -SinD(Yaw), 0, 0),
		FPlane(0, 0, 0, 0),
		FPlane(0, 0, 0, 0)
	);
}

struct AnySizeMatrix
{
	AnySizeMatrix();
	AnySizeMatrix(uint8 _NumRow, uint8 _NumColumn);
	float Get(uint8 Row, uint8 Column) const;
	void Set(uint8 Row, uint8 Column, float Value);
	void ZeroClear();
	static void Transpose(const AnySizeMatrix& InMatrix, AnySizeMatrix& OutMatrix);
	static void Multiply(const AnySizeMatrix& A, const AnySizeMatrix& B, AnySizeMatrix& OutResult);
	static void Add(const AnySizeMatrix& A, const AnySizeMatrix& B, AnySizeMatrix& OutResult);
	static float Inverse3x3(const AnySizeMatrix& InMatrix, AnySizeMatrix& OutMatrix);
	static float InverseNxN(uint8 Size, const AnySizeMatrix& InMatrix, AnySizeMatrix& OutMatrix);
	static void TransformVector(const AnySizeMatrix& InMatrix, const TArray<float>& InVector, TArray<float>& OutVector);

	TArray<float> Elements; // 1-dimensional array for access speed.
	uint8 NumRow;
	uint8 NumColumn;
};

FORCEINLINE AnySizeMatrix::AnySizeMatrix()
{
}

FORCEINLINE AnySizeMatrix::AnySizeMatrix(uint8 _NumRow, uint8 _NumColumn)
{
	Elements.AddZeroed(_NumRow * _NumColumn);
	NumRow = _NumRow;
	NumColumn = _NumColumn;
}

FORCEINLINE float AnySizeMatrix::Get(uint8 Row, uint8 Column) const
{
	return Elements[Row * NumColumn + Column];
}

FORCEINLINE void AnySizeMatrix::Set(uint8 Row, uint8 Column, float Value)
{
	// TODO:operator[][]�̉��Z�q�I�[�o�[���C�h�͍��Ȃ����ȁH
	Elements[Row * NumColumn + Column] = Value;
}

FORCEINLINE void AnySizeMatrix::ZeroClear()
{
	FMemory::Memzero(Elements.GetData(), sizeof(float) * NumRow * NumColumn);
}

FORCEINLINE void AnySizeMatrix::Transpose(const AnySizeMatrix& InMatrix, AnySizeMatrix& OutMatrix)
{
	for (int32 Row = 0; Row < InMatrix.NumRow; ++Row)
	{
		for (int32 Column = 0; Column < InMatrix.NumColumn; ++Column)
		{
			OutMatrix.Set(Column, Row, InMatrix.Get(Row, Column));
		}
	}
}

FORCEINLINE void AnySizeMatrix::Multiply(const AnySizeMatrix& A, const AnySizeMatrix& B, AnySizeMatrix& OutResult)
{
	check(A.NumColumn == B.NumRow);
	check(OutResult.NumRow == A.NumRow);
	check(OutResult.NumColumn == B.NumColumn);

	for (int32 Row = 0; Row < OutResult.NumRow; ++Row)
	{
		for (int32 Column = 0; Column < OutResult.NumColumn; ++Column)
		{
			OutResult.Set(Row, Column, 0);

			for (int32 i = 0; i < A.NumColumn; ++i)
			{
				OutResult.Set(Row, Column, OutResult.Get(Row, Column) + A.Get(Row, i) * B.Get(i, Column));
			}
		}
	}
}

FORCEINLINE void AnySizeMatrix::Add(const AnySizeMatrix& A, const AnySizeMatrix& B, AnySizeMatrix& OutResult)
{
	check(A.NumRow == B.NumRow);
	check(A.NumColumn == B.NumColumn);
	check(OutResult.NumRow == A.NumRow);
	check(OutResult.NumColumn == B.NumColumn);

	for (int32 Row = 0; Row < OutResult.NumRow; ++Row)
	{
		for (int32 Column = 0; Column < OutResult.NumColumn; ++Column)
		{
			OutResult.Set(Row, Column, A.Get(Row, Column) + B.Get(Row, Column));
		}
	}
}

FORCEINLINE float AnySizeMatrix::Inverse3x3(const AnySizeMatrix& InMatrix, AnySizeMatrix& OutMatrix)
{
	float Determinant =
		  InMatrix.Get(0, 0) * InMatrix.Get(1, 1) * InMatrix.Get(2, 2)
		+ InMatrix.Get(1, 0) * InMatrix.Get(2, 1) * InMatrix.Get(0, 2)
		+ InMatrix.Get(2, 0) * InMatrix.Get(0, 1) * InMatrix.Get(1, 2)
		- InMatrix.Get(0, 0) * InMatrix.Get(2, 1) * InMatrix.Get(1, 2)
		- InMatrix.Get(2, 0) * InMatrix.Get(1, 1) * InMatrix.Get(0, 2)
		- InMatrix.Get(1, 0) * InMatrix.Get(0, 1) * InMatrix.Get(2, 2);

	if (Determinant == 0)
	{
		return Determinant;
	}

	OutMatrix.Set(0, 0, (InMatrix.Get(1, 1) * InMatrix.Get(2, 2) - InMatrix.Get(1, 2) * InMatrix.Get(2, 1)) / Determinant);
	OutMatrix.Set(0, 1, (InMatrix.Get(0, 2) * InMatrix.Get(2, 1) - InMatrix.Get(0, 1) * InMatrix.Get(2, 2)) / Determinant);
	OutMatrix.Set(0, 2, (InMatrix.Get(0, 1) * InMatrix.Get(1, 2) - InMatrix.Get(0, 2) * InMatrix.Get(1, 1)) / Determinant);

	OutMatrix.Set(1, 0, (InMatrix.Get(1, 2) * InMatrix.Get(2, 0) - InMatrix.Get(1, 0) * InMatrix.Get(2, 2)) / Determinant);
	OutMatrix.Set(1, 1, (InMatrix.Get(0, 0) * InMatrix.Get(2, 2) - InMatrix.Get(0, 2) * InMatrix.Get(2, 0)) / Determinant);
	OutMatrix.Set(1, 2, (InMatrix.Get(0, 2) * InMatrix.Get(1, 0) - InMatrix.Get(0, 0) * InMatrix.Get(1, 2)) / Determinant);

	OutMatrix.Set(2, 0, (InMatrix.Get(1, 0) * InMatrix.Get(2, 1) - InMatrix.Get(1, 1) * InMatrix.Get(2, 0)) / Determinant);
	OutMatrix.Set(2, 1, (InMatrix.Get(0, 1) * InMatrix.Get(2, 0) - InMatrix.Get(0, 0) * InMatrix.Get(2, 1)) / Determinant);
	OutMatrix.Set(2, 2, (InMatrix.Get(0, 0) * InMatrix.Get(1, 1) - InMatrix.Get(0, 1) * InMatrix.Get(1, 0)) / Determinant);


	// TODO:�ꎞ�I
	Determinant =
		  InMatrix.Get(3, 3) * InMatrix.Get(4, 4) * InMatrix.Get(5, 5)
		+ InMatrix.Get(4, 3) * InMatrix.Get(5, 4) * InMatrix.Get(3, 5)
		+ InMatrix.Get(5, 3) * InMatrix.Get(3, 4) * InMatrix.Get(4, 5)
		- InMatrix.Get(3, 3) * InMatrix.Get(5, 4) * InMatrix.Get(4, 5)
		- InMatrix.Get(5, 3) * InMatrix.Get(4, 4) * InMatrix.Get(3, 5)
		- InMatrix.Get(4, 3) * InMatrix.Get(3, 4) * InMatrix.Get(5, 5);

	if (Determinant == 0)
	{
		return Determinant;
	}

	OutMatrix.Set(3, 3, (InMatrix.Get(4, 4) * InMatrix.Get(5, 5) - InMatrix.Get(4, 5) * InMatrix.Get(5, 4)) / Determinant);
	OutMatrix.Set(3, 4, (InMatrix.Get(3, 5) * InMatrix.Get(5, 4) - InMatrix.Get(3, 4) * InMatrix.Get(5, 5)) / Determinant);
	OutMatrix.Set(3, 5, (InMatrix.Get(3, 4) * InMatrix.Get(4, 5) - InMatrix.Get(3, 5) * InMatrix.Get(4, 4)) / Determinant);

	OutMatrix.Set(4, 3, (InMatrix.Get(4, 5) * InMatrix.Get(5, 3) - InMatrix.Get(4, 3) * InMatrix.Get(5, 5)) / Determinant);
	OutMatrix.Set(4, 4, (InMatrix.Get(3, 3) * InMatrix.Get(5, 5) - InMatrix.Get(3, 5) * InMatrix.Get(5, 3)) / Determinant);
	OutMatrix.Set(4, 5, (InMatrix.Get(3, 5) * InMatrix.Get(4, 3) - InMatrix.Get(3, 3) * InMatrix.Get(4, 5)) / Determinant);

	OutMatrix.Set(5, 3, (InMatrix.Get(4, 3) * InMatrix.Get(5, 4) - InMatrix.Get(4, 4) * InMatrix.Get(5, 3)) / Determinant);
	OutMatrix.Set(5, 4, (InMatrix.Get(3, 4) * InMatrix.Get(5, 3) - InMatrix.Get(3, 3) * InMatrix.Get(5, 4)) / Determinant);
	OutMatrix.Set(5, 5, (InMatrix.Get(3, 3) * InMatrix.Get(4, 4) - InMatrix.Get(3, 4) * InMatrix.Get(4, 3)) / Determinant);

	return Determinant;
}

FORCEINLINE float AnySizeMatrix::InverseNxN(uint8 Size, const AnySizeMatrix& InMatrix, AnySizeMatrix& OutMatrix)
{
	// http://thira.plavox.info/blog/2008/06/_c.html�@�����̂܂܎g���Ă���
	float Determinant = 1.0;

	// �O�p�s����쐬
	AnySizeMatrix WorkMatrix = InMatrix; // �R�s�[

	for (uint8 i = 0; i < Size; ++i)
	{
		for (uint8 j = 0; j < Size; ++j)
		{
			if (i < j)
			{
				float Multiplier = InMatrix.Get(j, i) / InMatrix.Get(i, i);

				for (uint8 k = 0; k < Size; ++k)
				{
					WorkMatrix.Set(j, k, (WorkMatrix.Get(j, k) - WorkMatrix.Get(i, k) * Multiplier));
				}
			}
		}
	}

	// �Ίp�����̐�
	for (uint8 i = 0; i < Size; ++i)
	{
		Determinant *= WorkMatrix.Get(i, i);
	}
	 
	if (FMath::Abs(Determinant) < SMALL_NUMBER)
	{
		return Determinant;
	}

	// �P�ʍs��ɂ��Ă���
	for (uint8 i = 0; i < Size; ++i)
	{
		OutMatrix.Set(i, i, 1.0f);
	}

	WorkMatrix = InMatrix; // �R�s�[

	//�|���o���@
	for (uint8 i = 0; i < Size; ++i)
	{
		float Multiplier = 1.0f / InMatrix.Get(i, i); // Determinant��0�łȂ����0�łȂ�

		for (uint8 j = 0; j < Size; ++j)
		{
			WorkMatrix.Set(i, j, WorkMatrix.Get(i, j) * Multiplier);
			OutMatrix.Set(i, j, OutMatrix.Get(i, j) * Multiplier);
		}

		for (uint8 j = 0; j < Size; ++j)
		{
			if (i != j)
			{
				Multiplier = InMatrix.Get(j, i) / InMatrix.Get(i, i); // Determinant��0�łȂ����0�łȂ�

				for (uint8 k = 0; k < Size; ++k)
				{
					WorkMatrix.Set(j, k, (WorkMatrix.Get(j, k) - WorkMatrix.Get(i, k) * Multiplier));
					OutMatrix.Set(j, k, OutMatrix.Get(j, k) - OutMatrix.Get(i, k) * Multiplier);
				}
			}
		}
	}

	return Determinant;
}

FORCEINLINE void AnySizeMatrix::TransformVector(const AnySizeMatrix& InMatrix, const TArray<float>& InVector, TArray<float>& OutVector)
{
	check((int32)InMatrix.NumRow == InVector.Num());
	check((int32)InMatrix.NumColumn == OutVector.Num());

	for (int32 Column = 0; Column < OutVector.Num(); ++Column)
	{
		OutVector[Column] = 0.0f;
	}

	for (int32 Column = 0; Column < InMatrix.NumColumn; ++Column)
	{
		for (int32 Row = 0; Row < InMatrix.NumRow; ++Row)
		{
			OutVector[Column] += InVector[Row] * InMatrix.Get(Row, Column);
		}
	}
}
} // namespace JacobianIKUtility

