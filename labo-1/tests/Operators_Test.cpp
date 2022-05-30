/**
 * @file Operators_Test.cpp
 *
 * @brief Unit tests for the Operators file.
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include <gtest/gtest.h>

#include "../Matrix.h"
#include "../Vector.h"
#include "../Operators.h"

using namespace gti320;

/*
 * Teste que la multiplication de matrices g�n�riques fonctionne correctement pour des matrices de m�me taille
 */
TEST(TestLabo1, Operator_Multiplication_MatrixMatrix_Generic_SameSize)
{
	Matrix<int, 3, 3, ColumnStorage> leftMatrix{ {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Matrix<int, 3, 3, ColumnStorage> rightMatrix{ {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };

	auto resultMatrix = leftMatrix * rightMatrix;

	EXPECT_EQ(9, resultMatrix.size());
	EXPECT_EQ(3, resultMatrix.rows());
	EXPECT_EQ(3, resultMatrix.cols());

	int expectedValue = 0;
	for (auto i = 0; i < 3; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(expectedValue, resultMatrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que la multiplication de matrices g�n�riques fonctionne correctement pour des matrices de taille diff�rente
 */
TEST(TestLabo1, Operator_Multiplication_MatrixMatrix_Generic_DifferentSize)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> leftMatrix{ {0, 1, 2, 3}, {4, 5, 6, 7} };
	Matrix<int, Dynamic, Dynamic, RowStorage> rightMatrix{ {0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {9, 10, 11} };

	auto resultMatrix = leftMatrix * rightMatrix;

	EXPECT_EQ(2 * 3, resultMatrix.size());
	EXPECT_EQ(2, resultMatrix.rows());
	EXPECT_EQ(3, resultMatrix.cols());

	// Expected:
	// | 42  48  54|
	// |114 136 158|
	EXPECT_EQ(42, resultMatrix(0, 0));
	EXPECT_EQ(48, resultMatrix(0, 1));
	EXPECT_EQ(54, resultMatrix(0, 2));
	EXPECT_EQ(114, resultMatrix(1, 0));
	EXPECT_EQ(136, resultMatrix(1, 1));
	EXPECT_EQ(158, resultMatrix(1, 2));
}

/*
 * Teste que la multiplication de matrices fonctionne correctement pour des matrices dynamiques ligne x colonne
 */
TEST(TestLabo1, Operator_Multiplication_MatrixMatrix_DynamicLineColumn_SameSize)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> leftMatrix{ {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Matrix<int, Dynamic, Dynamic, ColumnStorage> rightMatrix{ {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };

	auto resultMatrix = leftMatrix * rightMatrix;

	EXPECT_EQ(9, resultMatrix.size());
	EXPECT_EQ(3, resultMatrix.rows());
	EXPECT_EQ(3, resultMatrix.cols());

	int expectedValue = 0;
	for (auto i = 0; i < 3; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(expectedValue, resultMatrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que la multiplication de matrices fonctionne correctement pour des matrices dynamiques ligne x colonne de taille diff�rente
 */
TEST(TestLabo1, Operator_Multiplication_MatrixMatrix_DynamicLineColumn_DifferentSize)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> leftMatrix{ {0, 1, 2, 3}, {4, 5, 6, 7} };
	Matrix<int, Dynamic, Dynamic, ColumnStorage> rightMatrix{ {0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {9, 10, 11} };

	auto resultMatrix = leftMatrix * rightMatrix;

	EXPECT_EQ(2 * 3, resultMatrix.size());
	EXPECT_EQ(2, resultMatrix.rows());
	EXPECT_EQ(3, resultMatrix.cols());

	// Expected:
	// | 42  48  54|
	// |114 136 158|
	EXPECT_EQ(42, resultMatrix(0, 0));
	EXPECT_EQ(48, resultMatrix(0, 1));
	EXPECT_EQ(54, resultMatrix(0, 2));
	EXPECT_EQ(114, resultMatrix(1, 0));
	EXPECT_EQ(136, resultMatrix(1, 1));
	EXPECT_EQ(158, resultMatrix(1, 2));
}

/*
 * Teste que la multiplication de matrices fonctionne correctement pour des matrices dynamiques colonne x ligne
 */
TEST(TestLabo1, Operator_Multiplication_MatrixMatrix_DynamicColumnLine_SameSize)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> leftMatrix{ {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Matrix<int, Dynamic, Dynamic, RowStorage> rightMatrix{ {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };

	auto resultMatrix = leftMatrix * rightMatrix;

	EXPECT_EQ(9, resultMatrix.size());
	EXPECT_EQ(3, resultMatrix.rows());
	EXPECT_EQ(3, resultMatrix.cols());

	int expectedValue = 0;
	for (auto i = 0; i < 3; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(expectedValue, resultMatrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que la multiplication de matrices fonctionne correctement pour des matrices dynamiques colonne x ligne de taille diff�rente
 */
TEST(TestLabo1, Operator_Multiplication_MatrixMatrix_DynamicColumnLine_DifferentSize)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> leftMatrix{ {0, 1, 2, 3}, {4, 5, 6, 7} };
	Matrix<int, Dynamic, Dynamic, RowStorage> rightMatrix{ {0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {9, 10, 11} };

	auto resultMatrix = leftMatrix * rightMatrix;

	EXPECT_EQ(2 * 3, resultMatrix.size());
	EXPECT_EQ(2, resultMatrix.rows());
	EXPECT_EQ(3, resultMatrix.cols());

	// Expected:
	// | 42  48  54|
	// |114 136 158|
	EXPECT_EQ(42, resultMatrix(0, 0));
	EXPECT_EQ(48, resultMatrix(0, 1));
	EXPECT_EQ(54, resultMatrix(0, 2));
	EXPECT_EQ(114, resultMatrix(1, 0));
	EXPECT_EQ(136, resultMatrix(1, 1));
	EXPECT_EQ(158, resultMatrix(1, 2));
}

/*
 * Teste que l'addition de matrices g�n�riques fonctionne correctement pour des matrices carr�s
 */
TEST(TestLabo1, Operator_Addition_MatrixMatrix_Generic_Square)
{
	Matrix<int, 3, 3, ColumnStorage> leftMatrix{ {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Matrix<int, 3, 3, ColumnStorage> rightMatrix{ {5, -6, 5}, {-6, -5, 6}, {5, 6, -5} };

	auto resultMatrix = leftMatrix + rightMatrix;

	EXPECT_EQ(9, resultMatrix.size());
	EXPECT_EQ(3, resultMatrix.rows());
	EXPECT_EQ(3, resultMatrix.cols());

	// Expected:
	// | 5, -5,  7|
	// |-3, -1, 11|
	// |11, 13,  3|
	EXPECT_EQ(5, resultMatrix(0, 0));
	EXPECT_EQ(-5, resultMatrix(0, 1));
	EXPECT_EQ(7, resultMatrix(0, 2));
	EXPECT_EQ(-3, resultMatrix(1, 0));
	EXPECT_EQ(-1, resultMatrix(1, 1));
	EXPECT_EQ(11, resultMatrix(1, 2));
	EXPECT_EQ(11, resultMatrix(2, 0));
	EXPECT_EQ(13, resultMatrix(2, 1));
	EXPECT_EQ(3, resultMatrix(2, 2));
}

/*
 * Teste que l'addition de matrices g�n�riques fonctionne correctement pour des matrices rectangulaires
 */
TEST(TestLabo1, Operator_Addition_MatrixMatrix_Generic_Rectangle)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> leftMatrix{ {0, -1, 2, -4}, {5, -6, 7, -8} };
	Matrix<int, Dynamic, Dynamic, ColumnStorage> rightMatrix{ {1, 2, 3, 4}, {5, 6, 7, 8} };

	auto resultMatrix = leftMatrix + rightMatrix;

	EXPECT_EQ(8, resultMatrix.size());
	EXPECT_EQ(2, resultMatrix.rows());
	EXPECT_EQ(4, resultMatrix.cols());

	// Expected:
	// | 1, 1,  5, 0|
	// |10, 0, 14, 0|
	EXPECT_EQ(1, resultMatrix(0, 0));
	EXPECT_EQ(1, resultMatrix(0, 1));
	EXPECT_EQ(5, resultMatrix(0, 2));
	EXPECT_EQ(0, resultMatrix(0, 3));

	EXPECT_EQ(10, resultMatrix(1, 0));
	EXPECT_EQ(0, resultMatrix(1, 1));
	EXPECT_EQ(14, resultMatrix(1, 2));
	EXPECT_EQ(0, resultMatrix(1, 3));
}

/*
 * Teste que l'addition de matrices dynamiques par colonne fonctionne correctement pour des matrices carr�s
 */
TEST(TestLabo1, Operator_Addition_MatrixMatrix_DynamicColumn_Square)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> leftMatrix{ {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Matrix<int, Dynamic, Dynamic, ColumnStorage> rightMatrix{ {5, -6, 5}, {-6, -5, 6}, {5, 6, -5} };

	auto resultMatrix = leftMatrix + rightMatrix;

	EXPECT_EQ(9, resultMatrix.size());
	EXPECT_EQ(3, resultMatrix.rows());
	EXPECT_EQ(3, resultMatrix.cols());

	// Expected:
	// | 5, -5,  7|
	// |-3, -1, 11|
	// |11, 13,  3|
	EXPECT_EQ(5, resultMatrix(0, 0));
	EXPECT_EQ(-5, resultMatrix(0, 1));
	EXPECT_EQ(7, resultMatrix(0, 2));
	EXPECT_EQ(-3, resultMatrix(1, 0));
	EXPECT_EQ(-1, resultMatrix(1, 1));
	EXPECT_EQ(11, resultMatrix(1, 2));
	EXPECT_EQ(11, resultMatrix(2, 0));
	EXPECT_EQ(13, resultMatrix(2, 1));
	EXPECT_EQ(3, resultMatrix(2, 2));
}

/*
 * Teste que l'addition de matrices dynamiques par colonne fonctionne correctement pour des matrices rectangulaires
 */
TEST(TestLabo1, Operator_Addition_MatrixMatrix_DynamicColumn_Rectangle)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> leftMatrix{ {0, -1, 2, -4}, {5, -6, 7, -8} };
	Matrix<int, Dynamic, Dynamic, ColumnStorage> rightMatrix{ {1, 2, 3, 4}, {5, 6, 7, 8} };

	auto resultMatrix = leftMatrix + rightMatrix;

	EXPECT_EQ(8, resultMatrix.size());
	EXPECT_EQ(2, resultMatrix.rows());
	EXPECT_EQ(4, resultMatrix.cols());

	// Expected:
	// | 1, 1,  5, 0|
	// |10, 0, 14, 0|
	EXPECT_EQ(1, resultMatrix(0, 0));
	EXPECT_EQ(1, resultMatrix(0, 1));
	EXPECT_EQ(5, resultMatrix(0, 2));
	EXPECT_EQ(0, resultMatrix(0, 3));

	EXPECT_EQ(10, resultMatrix(1, 0));
	EXPECT_EQ(0, resultMatrix(1, 1));
	EXPECT_EQ(14, resultMatrix(1, 2));
	EXPECT_EQ(0, resultMatrix(1, 3));
}

/*
 * Teste que l'addition de matrices dynamiques par ligne fonctionne correctement pour des matrices carr�s
 */
TEST(TestLabo1, Operator_Addition_MatrixMatrix_DynamicRow_Square)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> leftMatrix{ {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Matrix<int, Dynamic, Dynamic, RowStorage> rightMatrix{ {5, -6, 5}, {-6, -5, 6}, {5, 6, -5} };

	auto resultMatrix = leftMatrix + rightMatrix;

	EXPECT_EQ(9, resultMatrix.size());
	EXPECT_EQ(3, resultMatrix.rows());
	EXPECT_EQ(3, resultMatrix.cols());

	// Expected:
	// | 5, -5,  7|
	// |-3, -1, 11|
	// |11, 13,  3|
	EXPECT_EQ(5, resultMatrix(0, 0));
	EXPECT_EQ(-5, resultMatrix(0, 1));
	EXPECT_EQ(7, resultMatrix(0, 2));
	EXPECT_EQ(-3, resultMatrix(1, 0));
	EXPECT_EQ(-1, resultMatrix(1, 1));
	EXPECT_EQ(11, resultMatrix(1, 2));
	EXPECT_EQ(11, resultMatrix(2, 0));
	EXPECT_EQ(13, resultMatrix(2, 1));
	EXPECT_EQ(3, resultMatrix(2, 2));
}

/*
 * Teste que l'addition de matrices dynamiques par ligne fonctionne correctement pour des matrices rectangulaires
 */
TEST(TestLabo1, Operator_Addition_MatrixMatrix_DynamicRow_Rectangle)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> leftMatrix{ {0, -1, 2, -4}, {5, -6, 7, -8} };
	Matrix<int, Dynamic, Dynamic, RowStorage> rightMatrix{ {1, 2, 3, 4}, {5, 6, 7, 8} };

	auto resultMatrix = leftMatrix + rightMatrix;

	EXPECT_EQ(8, resultMatrix.size());
	EXPECT_EQ(2, resultMatrix.rows());
	EXPECT_EQ(4, resultMatrix.cols());

	// Expected:
	// | 1, 1,  5, 0|
	// |10, 0, 14, 0|
	EXPECT_EQ(1, resultMatrix(0, 0));
	EXPECT_EQ(1, resultMatrix(0, 1));
	EXPECT_EQ(5, resultMatrix(0, 2));
	EXPECT_EQ(0, resultMatrix(0, 3));

	EXPECT_EQ(10, resultMatrix(1, 0));
	EXPECT_EQ(0, resultMatrix(1, 1));
	EXPECT_EQ(14, resultMatrix(1, 2));
	EXPECT_EQ(0, resultMatrix(1, 3));
}

/*
 * Teste que la multiplication d'une matrice par colonne et d'un scalaire fonctionne correctement
 */
TEST(TestLabo1, Operator_Multiplication_ScalarMatrix_Column_Ok)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7} };

	auto resultMatrix = -2 * matrix;

	int expectedValue = 0;
	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 4; ++j)
		{
			EXPECT_EQ(expectedValue * -2, resultMatrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que la multiplication d'un scalaire et d'une matrice par colonne fonctionne correctement
 */
TEST(TestLabo1, Operator_Multiplication_MatrixScalar_Column_Ok)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7} };

	auto resultMatrix =  matrix * -2;

	int expectedValue = 0;
	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 4; ++j)
		{
			EXPECT_EQ(expectedValue * -2, resultMatrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que la multiplication d'une matrice par ligne et d'un scalaire fonctionne correctement
 */
TEST(TestLabo1, Operator_Multiplication_ScalarMatrix_Row_Ok)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7} };

	auto resultMatrix = -2 * matrix;

	int expectedValue = 0;
	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 4; ++j)
		{
			EXPECT_EQ(expectedValue * -2, resultMatrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que la multiplication d'un scalaire et d'une matrice par ligne fonctionne correctement
 */
TEST(TestLabo1, Operator_Multiplication_MatrixScalar_Row_Ok)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7} };

	auto resultMatrix = matrix * -2;

	int expectedValue = 0;
	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 4; ++j)
		{
			EXPECT_EQ(expectedValue * -2, resultMatrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que la multiplication d'une matrice carr�e par colonne et d'un vecteur fonctionne correctement
 */
TEST(TestLabo1, Operator_Multiplication_MatrixVector_Column_Square)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> matrix{ {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Vector<int, Dynamic> vector{ 0, 1, -2 };

	auto resultVector = matrix * vector;

	EXPECT_EQ(3, resultVector.size());

	// Expected [-3, -6, -9]
	EXPECT_EQ(-3, resultVector(0));
	EXPECT_EQ(-6, resultVector(1));
	EXPECT_EQ(-9, resultVector(2));
}

/*
 * Teste que la multiplication dune matrice rectangle par colonne et d'un vecteur fonctionne correctement
 */
TEST(TestLabo1, Operator_Multiplication_MatrixVector_Column_Rectangle)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };
	Vector<int, Dynamic> vector{ 0, 1, -2, 3 };

	auto resultVector = matrix * vector;

	EXPECT_EQ(3, resultVector.size());

	// Expected [6, 14, 22]
	EXPECT_EQ(6, resultVector(0));
	EXPECT_EQ(14, resultVector(1));
	EXPECT_EQ(22, resultVector(2));
}

/*
 * Teste que la multiplication d'une matrice carr�e par ligne et d'un vecteur fonctionne correctement
 */
TEST(TestLabo1, Operator_Multiplication_MatrixVector_Row_Square)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> matrix{ {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Vector<int, Dynamic> vector{ 0, 1, -2 };

	auto resultVector = matrix * vector;

	EXPECT_EQ(3, resultVector.size());

	// Expected [-3, -6, -9]
	EXPECT_EQ(-3, resultVector(0));
	EXPECT_EQ(-6, resultVector(1));
	EXPECT_EQ(-9, resultVector(2));
}

/*
 * Teste que la multiplication d'une matrice rectangle par ligne et d'un vecteur fonctionne correctement
 */
TEST(TestLabo1, Operator_Multiplication_MatrixVector_Row_Rectangle)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };
	Vector<int, Dynamic> vector{ 0, 1, -2, 3 };

	auto resultVector = matrix * vector;

	EXPECT_EQ(3, resultVector.size());

	// Expected [6, 14, 22]
	EXPECT_EQ(6, resultVector(0));
	EXPECT_EQ(14, resultVector(1));
	EXPECT_EQ(22, resultVector(2));
}

/*
 * Teste que la multiplication d'un vecteur et d'un scalaire fonctionne correctement
 */
TEST(TestLabo1, Operator_Multiplication_VectorScalar_Ok)
{
	Vector<int, Dynamic> vector{ 0, 1, -2, 3 };

	auto resultVector = vector * -2;

	EXPECT_EQ(0, resultVector(0));
	EXPECT_EQ(-2, resultVector(1));
	EXPECT_EQ(4, resultVector(2));
	EXPECT_EQ(-6, resultVector(3));
}

/*
 * Teste que la multiplication d'un scalaire et d'un vecteur fonctionne correctement
 */
TEST(TestLabo1, Operator_Multiplication_ScalarVector_Ok)
{
	Vector<int, Dynamic> vector{ 0, 1, -2, 3 };

	auto resultVector = -2 * vector;

	EXPECT_EQ(0, resultVector(0));
	EXPECT_EQ(-2, resultVector(1));
	EXPECT_EQ(4, resultVector(2));
	EXPECT_EQ(-6, resultVector(3));
}

/*
 * Teste que l'addition de deux vecteurs fonctionne correctement
 */
TEST(TestLabo1, Operator_Addition_VectorVector_Ok)
{
	Vector<int, Dynamic> leftVector{ 0, 1, -2, 3 };
	Vector<int, Dynamic> rightVector{ 5, -4, -2, 1 };

	auto resultVector = leftVector + rightVector;

	EXPECT_EQ(5, resultVector(0));
	EXPECT_EQ(-3, resultVector(1));
	EXPECT_EQ(-4, resultVector(2));
	EXPECT_EQ(4, resultVector(3));
}

/*
 * Teste que la soustractions de deux vecteurs fonctionne correctement
 */
TEST(TestLabo1, Operator_Substraction_VectorVector_Ok)
{
	Vector<int, Dynamic> leftVector{ 0, 1, -2, 3 };
	Vector<int, Dynamic> rightVector{ 5, -4, -2, 1 };

	auto resultVector = leftVector - rightVector;

	EXPECT_EQ(-5, resultVector(0));
	EXPECT_EQ(5, resultVector(1));
	EXPECT_EQ(0, resultVector(2));
	EXPECT_EQ(2, resultVector(3));
}

/*
 * Teste que la multiplication d'un vecteur par - fonctionne correctement
 */
TEST(TestLabo1, Operator_Inverse_Vector_Ok)
{
	Vector<int, Dynamic> vector{ 0, 1, -2, 3 };

	auto resultVector = -vector;

	EXPECT_EQ(0, resultVector(0));
	EXPECT_EQ(-1, resultVector(1));
	EXPECT_EQ(2, resultVector(2));
	EXPECT_EQ(-3, resultVector(3));
}

/*
 * Teste que la multiplication d'une matrice par - fonctionne correctement
 */
TEST(TestLabo1, Operator_Inverse_Matrix_Ok)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> rowMatrix{ {1, -2}, {3, 4} };
	Matrix<double, Dynamic, Dynamic, ColumnStorage> colMatrix{ {1.0, -2.0}, {3.0, 4.0} };

	auto resultRowMatrix = -rowMatrix;
	auto resultColMatrix = -colMatrix;

	EXPECT_EQ(-1, resultRowMatrix(0, 0));
	EXPECT_DOUBLE_EQ(-1.0, resultColMatrix(0, 0));

	EXPECT_EQ(2, resultRowMatrix(0, 1));
	EXPECT_DOUBLE_EQ(2.0, resultColMatrix(0, 1));

	EXPECT_EQ(-3, resultRowMatrix(1, 0));
	EXPECT_DOUBLE_EQ(-3.0, resultColMatrix(1, 0));

	EXPECT_EQ(-4, resultRowMatrix(1, 1));
	EXPECT_DOUBLE_EQ(-4.0, resultColMatrix(1, 1));
}