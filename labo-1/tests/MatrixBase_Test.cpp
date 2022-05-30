/**
 * @file DenseStorage_Test.cpp
 *
 * @brief Unit tests for the MatrixBase class.
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include <gtest/gtest.h>

#include "../MatrixBase.h"
#include "../Matrix.h"

using namespace gti320;

#pragma region MatrixBase_Static

/*
 * Teste que le constructeur par d�faut d'une MatrixBase statique cr�e correctement l'objet
 */
TEST(TestLabo1, MatrixBase_Static_DefaultConstructor_Ok)
{
	MatrixBase<double, 3, 5> doubleMatrix;
	MatrixBase<float, 1, 3> floatMatrix;
	MatrixBase<int, 4, 4> intMatrix;

	EXPECT_EQ(3 * 5, doubleMatrix.size());
	EXPECT_EQ(3, doubleMatrix.rows());
	EXPECT_EQ(5, doubleMatrix.cols());

	EXPECT_EQ(1 * 3, floatMatrix.size());
	EXPECT_EQ(1, floatMatrix.rows());
	EXPECT_EQ(3, floatMatrix.cols());

	EXPECT_EQ(4 * 4, intMatrix.size());
	EXPECT_EQ(4, intMatrix.rows());
	EXPECT_EQ(4, intMatrix.cols());

	for (auto i = 0; i < 3 * 5; ++i)
	{
		EXPECT_DOUBLE_EQ(0.0, doubleMatrix[i]);
	}

	for (auto i = 0; i < 1 * 3; ++i)
	{
		EXPECT_FLOAT_EQ(0.f, floatMatrix[i]);
	}

	for (auto i = 0; i < 4 * 4; ++i)
	{
		EXPECT_EQ(0, intMatrix[i]);
	}
}

/*
 * Teste que le constructeur par taille d'une MatrixBase statique cr�e correctement l'objet
 */
TEST(TestLabo1, MatrixBase_Static_SizeConstructor_Ok)
{
	MatrixBase<int, 4, 4> intMatrix(4, 4);

	EXPECT_EQ(4 * 4, intMatrix.size());
	EXPECT_EQ(4, intMatrix.rows());
	EXPECT_EQ(4, intMatrix.cols());

	for (auto i = 0; i < 4 * 4; ++i)
	{
		EXPECT_EQ(0, intMatrix[i]);
	}
}

/*
 * Teste que le constructeur par copie d'une MatrixBase statique cr�e correctement l'objet
 */
TEST(TestLabo1, MatrixBase_Static_CopyConstructor_Ok)
{
	Matrix<int, 3, 4, RowStorage> matrixToCopy{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };

	MatrixBase<int, 3, 4> copiedMatrix(matrixToCopy);

	EXPECT_EQ(3 * 4, copiedMatrix.size());
	EXPECT_EQ(3, copiedMatrix.rows());
	EXPECT_EQ(4, copiedMatrix.cols());

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(i, copiedMatrix[i]);
	}
}

/*
 * Teste que l'op�rateur "=" d'une MatrixBase statique copie correctement l'objet
 */
TEST(TestLabo1, MatrixBase_Static_EqualOperator_Ok)
{
	Matrix<int, 3, 4, RowStorage> matrixToCopy{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };

	MatrixBase<int, 3, 4> copiedMatrix;
	copiedMatrix = matrixToCopy;

	EXPECT_EQ(3 * 4, copiedMatrix.size());
	EXPECT_EQ(3, copiedMatrix.rows());
	EXPECT_EQ(4, copiedMatrix.cols());

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(i, copiedMatrix[i]);
	}
}

/*
 * Teste que la m�thode "setZero" d'une MatrixBase statique copie correctement l'objet
 */
TEST(TestLabo1, MatrixBase_Static_setZero_Ok)
{
	MatrixBase<int, 3, 4> matrix;

	matrix.setZero();

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(0, matrix[i]);
	}
}

/*
 * Teste que les accesseurs d'une MatrixBase statique retournent des valeurs consistantes
 */
TEST(TestLabo1, MatrixBase_Static_Get_Ok)
{
	MatrixBase<int, 3, 4> matrix;

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(0, matrix[i]);
		EXPECT_EQ(0, matrix.data()[i]);
		EXPECT_EQ(0, matrix.storage()[i]);
	}
}

#pragma endregion

#pragma region MatrixBase_DynamicCols

/*
 * Teste que le constructeur par d�faut d'une MatrixBase dynamique par colonnes cr�e correctement l'objet vide
 */
TEST(TestLabo1, MatrixBase_DynamicCols_DefaultConstructor_Ok)
{
	MatrixBase<double, 3, Dynamic> doubleMatrix;
	MatrixBase<float, 1, Dynamic> floatMatrix;
	MatrixBase<int, 4, Dynamic> intMatrix;

	EXPECT_EQ(0, doubleMatrix.size());
	EXPECT_EQ(3, doubleMatrix.rows());
	EXPECT_EQ(0, doubleMatrix.cols());

	EXPECT_EQ(0, floatMatrix.size());
	EXPECT_EQ(1, floatMatrix.rows());
	EXPECT_EQ(0, floatMatrix.cols());

	EXPECT_EQ(0, intMatrix.size());
	EXPECT_EQ(4, intMatrix.rows());
	EXPECT_EQ(0, intMatrix.cols());
}

/*
 * Teste que le constructeur par taille d'une MatrixBase dynamique par colonnes cr�e correctement l'objet
 */
TEST(TestLabo1, MatrixBase_DynamicCols_SizeConstructor_Ok)
{
	MatrixBase<int, 4, Dynamic> matrix(4, 10);

	EXPECT_EQ(40, matrix.size());
	EXPECT_EQ(4, matrix.rows());
	EXPECT_EQ(10, matrix.cols());

	for (auto i = 0; i < 4 * 10; ++i)
	{
		EXPECT_EQ(0, matrix[i]);
	}
}

/*
 * Teste que le constructeur par copie d'une MatrixBase dynamique par colonnes cr�e correctement l'objet
 */
TEST(TestLabo1, MatrixBase_DynamicCols_CopyConstructor_Ok)
{
	Matrix<int, 3, Dynamic, RowStorage> matrixToCopy{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };

	MatrixBase<int, 3, Dynamic> copiedMatrix(matrixToCopy);

	EXPECT_EQ(3 * 4, copiedMatrix.size());
	EXPECT_EQ(3, copiedMatrix.rows());
	EXPECT_EQ(4, copiedMatrix.cols());

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(i, copiedMatrix[i]);
	}
}

/*
 * Teste que l'op�rateur "=" d'une MatrixBase dynamique par colonnes copie correctement l'objet
 */
TEST(TestLabo1, MatrixBase_DynamicCols_EqualOperator_Ok)
{
	Matrix<int, 3, Dynamic, RowStorage> matrixToCopy{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };

	MatrixBase<int, 3, Dynamic> copiedMatrix;
	copiedMatrix = matrixToCopy;

	EXPECT_EQ(3 * 4, copiedMatrix.size());
	EXPECT_EQ(3, copiedMatrix.rows());
	EXPECT_EQ(4, copiedMatrix.cols());

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(i, copiedMatrix[i]);
	}
}

/*
 * Teste que le la m�thode "resize" d'une MatrixBase dynamique par colonnes redimensionne correctement l'objet
 */
TEST(TestLabo1, MatrixBase_DynamicCols_resize_Ok)
{
	MatrixBase<int, 4, Dynamic> matrix;
	matrix.resize(4, 10);

	EXPECT_EQ(40, matrix.size());
	EXPECT_EQ(4, matrix.rows());
	EXPECT_EQ(10, matrix.cols());
}

/*
 * Teste que la m�thode "setZero" d'une MatrixBase dynamique par colonnes copie correctement l'objet
 */
TEST(TestLabo1, MatrixBase_DynamicCols_setZero_Ok)
{
	MatrixBase<int, 3, Dynamic> matrix(3, 4);

	matrix.setZero();

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(0, matrix[i]);
	}
}

/*
 * Teste que les accesseurs d'une MatrixBase dynamique par colonnes retournent des valeurs consistantes
 */
TEST(TestLabo1, MatrixBase_DynamicCols_Get_Ok)
{
	MatrixBase<int, 3, Dynamic> matrix(3, 4);

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(0, matrix[i]);
		EXPECT_EQ(0, matrix.data()[i]);
		EXPECT_EQ(0, matrix.storage()[i]);
	}
}

#pragma endregion

#pragma region MatrixBase_DynamicRows

/*
 * Teste que le constructeur par d�faut d'une MatrixBase dynamique par lignes cr�e correctement l'objet vide
 */
TEST(TestLabo1, MatrixBase_DynamicRows_DefaultConstructor_Ok)
{
	MatrixBase<double, Dynamic, 5> doubleMatrix;
	MatrixBase<float, Dynamic, 3> floatMatrix;
	MatrixBase<int, Dynamic, 4> intMatrix;

	EXPECT_EQ(0, doubleMatrix.size());
	EXPECT_EQ(0, doubleMatrix.rows());
	EXPECT_EQ(5, doubleMatrix.cols());

	EXPECT_EQ(0, floatMatrix.size());
	EXPECT_EQ(0, floatMatrix.rows());
	EXPECT_EQ(3, floatMatrix.cols());

	EXPECT_EQ(0, intMatrix.size());
	EXPECT_EQ(0, intMatrix.rows());
	EXPECT_EQ(4, intMatrix.cols());
}

/*
 * Teste que le constructeur par taille d'une MatrixBase dynamique par lignes cr�e correctement l'objet
 */
TEST(TestLabo1, MatrixBase_DynamicRows_SizeConstructor_Ok)
{
	MatrixBase<int, Dynamic, 10> matrix(4, 10);

	EXPECT_EQ(40, matrix.size());
	EXPECT_EQ(4, matrix.rows());
	EXPECT_EQ(10, matrix.cols());

	for (auto i = 0; i < 4 * 10; ++i)
	{
		EXPECT_EQ(0, matrix[i]);
	}
}

/*
 * Teste que le constructeur par copie d'une MatrixBase dynamique par lignes cr�e correctement l'objet
 */
TEST(TestLabo1, MatrixBase_DynamicRows_CopyConstructor_Ok)
{
	Matrix<int, Dynamic, 4, RowStorage> matrixToCopy{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };

	MatrixBase<int, Dynamic, 4> copiedMatrix(matrixToCopy);

	EXPECT_EQ(3 * 4, copiedMatrix.size());
	EXPECT_EQ(3, copiedMatrix.rows());
	EXPECT_EQ(4, copiedMatrix.cols());

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(i, copiedMatrix[i]);
	}
}

/*
 * Teste que l'op�rateur "=" d'une MatrixBase dynamique par lignes copie correctement l'objet
 */
TEST(TestLabo1, MatrixBase_DynamicRows_EqualOperator_Ok)
{
	Matrix<int, Dynamic, 4, RowStorage> matrixToCopy{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };

	MatrixBase<int, Dynamic, 4> copiedMatrix;
	copiedMatrix = matrixToCopy;

	EXPECT_EQ(3 * 4, copiedMatrix.size());
	EXPECT_EQ(3, copiedMatrix.rows());
	EXPECT_EQ(4, copiedMatrix.cols());

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(i, copiedMatrix[i]);
	}
}

/*
 * Teste que le la m�thode "resize" d'une MatrixBase dynamique par lignes redimensionne correctement l'objet
 */
TEST(TestLabo1, MatrixBase_DynamicRows_resize_Ok)
{
	MatrixBase<int, Dynamic, 10> matrix;
	matrix.resize(4, 10);

	EXPECT_EQ(40, matrix.size());
	EXPECT_EQ(4, matrix.rows());
	EXPECT_EQ(10, matrix.cols());
}

/*
 * Teste que la m�thode "setZero" d'une MatrixBase dynamique par lignes copie correctement l'objet
 */
TEST(TestLabo1, MatrixBase_DynamicRows_setZero_Ok)
{
	MatrixBase<int, Dynamic, 4> matrix(3, 4);

	matrix.setZero();

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(0, matrix[i]);
	}
}

/*
 * Teste que les accesseurs d'une MatrixBase dynamique par lignes retournent des valeurs consistantes
 */
TEST(TestLabo1, MatrixBase_DynamicRows_Get_Ok)
{
	MatrixBase<int, Dynamic, 4> matrix(3, 4);

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(0, matrix[i]);
		EXPECT_EQ(0, matrix.data()[i]);
		EXPECT_EQ(0, matrix.storage()[i]);
	}
}

#pragma endregion

#pragma region MatrixBase_FullDynamic

/*
 * Teste que le constructeur par d�faut d'une MatrixBase dynamique cr�e correctement l'objet vide
 */
TEST(TestLabo1, MatrixBase_FullDynamic_DefaultConstructor_Ok)
{
	MatrixBase<double, Dynamic, Dynamic> doubleMatrix;
	MatrixBase<float, Dynamic, Dynamic> floatMatrix;
	MatrixBase<int, Dynamic, Dynamic> intMatrix;

	EXPECT_EQ(0, doubleMatrix.size());
	EXPECT_EQ(0, doubleMatrix.rows());
	EXPECT_EQ(0, doubleMatrix.cols());

	EXPECT_EQ(0, floatMatrix.size());
	EXPECT_EQ(0, floatMatrix.rows());
	EXPECT_EQ(0, floatMatrix.cols());

	EXPECT_EQ(0, intMatrix.size());
	EXPECT_EQ(0, intMatrix.rows());
	EXPECT_EQ(0, intMatrix.cols());
}

/*
 * Teste que le constructeur par taille d'une MatrixBase dynamique cr�e correctement l'objet
 */
TEST(TestLabo1, MatrixBase_FullDynamic_SizeConstructor_Ok)
{
	MatrixBase<int, Dynamic, Dynamic> matrix(4, 10);

	EXPECT_EQ(40, matrix.size());
	EXPECT_EQ(4, matrix.rows());
	EXPECT_EQ(10, matrix.cols());

	for (auto i = 0; i < 4 * 10; ++i)
	{
		EXPECT_EQ(0, matrix[i]);
	}
}

/*
 * Teste que le constructeur par copie d'une MatrixBase dynamique cr�e correctement l'objet
 */
TEST(TestLabo1, MatrixBase_FullDynamic_CopyConstructor_Ok)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> matrixToCopy{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };

	MatrixBase<int, Dynamic, Dynamic> copiedMatrix(matrixToCopy);

	EXPECT_EQ(3 * 4, copiedMatrix.size());
	EXPECT_EQ(3, copiedMatrix.rows());
	EXPECT_EQ(4, copiedMatrix.cols());

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(i, copiedMatrix[i]);
	}
}

/*
 * Teste que l'op�rateur "=" d'une MatrixBase dynamique copie correctement l'objet
 */
TEST(TestLabo1, MatrixBase_FullDynamic_EqualOperator_Ok)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> matrixToCopy{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };

	MatrixBase<int, Dynamic, Dynamic> copiedMatrix;
	copiedMatrix = matrixToCopy;

	EXPECT_EQ(3 * 4, copiedMatrix.size());
	EXPECT_EQ(3, copiedMatrix.rows());
	EXPECT_EQ(4, copiedMatrix.cols());

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(i, copiedMatrix[i]);
	}
}

/*
 * Teste que le la m�thode "resize" d'une MatrixBase dynamique redimensionne correctement l'objet
 */
TEST(TestLabo1, MatrixBase_FullDynamic_resize_Ok)
{
	MatrixBase<int, Dynamic, Dynamic> matrix;
	matrix.resize(4, 10);

	EXPECT_EQ(40, matrix.size());
	EXPECT_EQ(4, matrix.rows());
	EXPECT_EQ(10, matrix.cols());
}

/*
 * Teste que la m�thode "setZero" d'une MatrixBase dynamique copie correctement l'objet
 */
TEST(TestLabo1, MatrixBase_FullDynamic_setZero_Ok)
{
	MatrixBase<int, Dynamic, Dynamic> matrix(3, 4);

	matrix.setZero();

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(0, matrix[i]);
	}
}

/*
 * Teste que les accesseurs d'une MatrixBase dynamique retournent des valeurs consistantes
 */
TEST(TestLabo1, MatrixBase_FullDynamic_Get_Ok)
{
	MatrixBase<int, Dynamic, Dynamic> matrix(3, 4);

	for (auto i = 0; i < 3 * 4; ++i)
	{
		EXPECT_EQ(0, matrix[i]);
		EXPECT_EQ(0, matrix.data()[i]);
		EXPECT_EQ(0, matrix.storage()[i]);
	}
}

#pragma endregion