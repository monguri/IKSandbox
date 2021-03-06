#pragma once

namespace JacobianIKUtility
{
// XYZの3成分に使う定数
const int32 AXIS_COUNT = 3;
// 回転方向の3成分に使う定数
const int32 ROTATION_AXIS_COUNT = 3;

// degree単位でのsin/cos関数
FORCEINLINE float SinD(float degree) { return FMath::Sin(FMath::DegreesToRadians(degree)); }
FORCEINLINE float CosD(float degree) { return FMath::Cos(FMath::DegreesToRadians(degree)); }

// 各軸の回転行列
//TODO: 正負の符号や、行優先列優先についてはあまりきちんと考慮はしておらずほげたつさんのソースを参考にしている
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

// 各軸の回転行列を回転角の変数で微分した行列
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
	static float InverseNxN(const AnySizeMatrix& InMatrix, AnySizeMatrix& OutMatrix);
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
	// TODO:operator[][]の演算子オーバーライドは作れないかな？
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

	return Determinant;
}

FORCEINLINE float AnySizeMatrix::InverseNxN(const AnySizeMatrix& InMatrix, AnySizeMatrix& OutMatrix)
{
	// http://thira.plavox.info/blog/2008/06/_c.html　https://seesaawiki.jp/w/pafuhana1213/d/n%BC%A1%A4%CE%B5%D5%B9%D4%CE%F3 をそのまま使っている
	float Determinant = 1.0;

	uint8 Size = InMatrix.NumRow;
	check(Size == InMatrix.NumColumn);
	check(Size == OutMatrix.NumRow);
	check(Size == OutMatrix.NumColumn);

	// 三角行列を作成
	AnySizeMatrix WorkMatrix = InMatrix; // コピー

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

	// 対角部分の積
	for (uint8 i = 0; i < Size; ++i)
	{
		Determinant *= WorkMatrix.Get(i, i);
	}
	 
	if (FMath::Abs(Determinant) < SMALL_NUMBER)
	{
		return Determinant;
	}

	// 単位行列にしておく
	for (uint8 i = 0; i < Size; ++i)
	{
		OutMatrix.Set(i, i, 1.0f);
	}

	WorkMatrix = InMatrix; // コピー

	// ピボット選択を行ったGauss-Jordan法
	for (uint8 i = 0; i < Size; ++i)
	{
#if 0
		//ピボット選択 i行i列目の要素の絶対値が最大に
		uint8 Max = i;
		for (uint8 j = i + 1; j < Size; ++j)
		{
			if (FMath::Abs(WorkMatrix.Get(j, i)) > FMath::Abs(WorkMatrix.Get(Max, i)))
			{
				Max = j;
			}
		}

		// 行の入れ替え
		if (Max != i)
		{
			for (uint8 j = 0; j < Size; ++j)
			{
				float SwapTmp = WorkMatrix.Get(Max, j);
				WorkMatrix.Set(Max, j, WorkMatrix.Get(i, j));
				WorkMatrix.Set(i, j, SwapTmp);

				SwapTmp = OutMatrix.Get(Max, j);
				OutMatrix.Set(Max, j, OutMatrix.Get(i, j));
				OutMatrix.Set(i, j, SwapTmp);
			}
		}
#endif

		float Multiplier = 1.0f / InMatrix.Get(i, i); // Determinantが0でなければ0でない

		// i行i列目の要素が1になるように
		for (uint8 j = 0; j < Size; ++j)
		{
			WorkMatrix.Set(i, j, WorkMatrix.Get(i, j) * Multiplier);
			OutMatrix.Set(i, j, OutMatrix.Get(i, j) * Multiplier);
		}

		// i行目のi列目以外の要素が0になるように
		for (uint8 j = 0; j < Size; ++j)
		{
			if (i != j)
			{
				Multiplier = WorkMatrix.Get(j, i) / WorkMatrix.Get(i, i); // Determinantが0でなければ0でない

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

