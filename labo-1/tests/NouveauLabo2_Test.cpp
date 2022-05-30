/**
 * @file Vector_Test.cpp
 *
 * @brief Unit tests for the Vector class.
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include <gtest/gtest.h>

#include "../Vector.h"
#include "../Matrix.h"
#include "../Operators.h"

using namespace gti320;

#pragma region Matrix_row&col
/*
 * Teste que la m�thode row retourne bien une sous matrice repr�sentant une ligne d'une matrice par colonne
 */
TEST(NouveauTestLabo1FromLabo2, Matrix_Column_row_Ok)
{
	Matrix<int, 3, 4, ColumnStorage> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };

	SubMatrix<int, 3, 4, ColumnStorage> subMatrix = matrix.row(1);

	EXPECT_EQ(1, subMatrix.rows());
	EXPECT_EQ(4, subMatrix.cols());

	EXPECT_EQ(4, subMatrix(0, 0));
	EXPECT_EQ(5, subMatrix(0, 1));
	EXPECT_EQ(6, subMatrix(0, 2));
	EXPECT_EQ(7, subMatrix(0, 3));
}

/*
 * Teste que la m�thode col retourne bien une sous matrice repr�sentant une ligne d'une matrice par colonne
 */
TEST(NouveauTestLabo1FromLabo2, Matrix_Column_col_Ok)
{
	Matrix<int, 3, 4, ColumnStorage> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };

	SubMatrix<int, 3, 4, ColumnStorage> subMatrix = matrix.col(2);

	EXPECT_EQ(3, subMatrix.rows());
	EXPECT_EQ(1, subMatrix.cols());

	EXPECT_EQ(2, subMatrix(0, 0));
	EXPECT_EQ(6, subMatrix(1, 0));
	EXPECT_EQ(10, subMatrix(2, 0));
}

/*
 * Teste que la m�thode row retourne bien une sous matrice repr�sentant une ligne d'une matrice par ligne
 */
TEST(NouveauTestLabo1FromLabo2, Matrix_Row_row_Ok)
{
	Matrix<int, 3, 4, RowStorage> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };

	SubMatrix<int, 3, 4, RowStorage> subMatrix = matrix.row(1);

	EXPECT_EQ(1, subMatrix.rows());
	EXPECT_EQ(4, subMatrix.cols());

	EXPECT_EQ(4, subMatrix(0, 0));
	EXPECT_EQ(5, subMatrix(0, 1));
	EXPECT_EQ(6, subMatrix(0, 2));
	EXPECT_EQ(7, subMatrix(0, 3));
}

/*
 * Teste que la m�thode col retourne bien une sous matrice repr�sentant une ligne d'une matrice par ligne
 */
TEST(NouveauTestLabo1FromLabo2, Matrix_Row_col_Ok)
{
	Matrix<int, 3, 4, RowStorage> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };

	SubMatrix<int, 3, 4, RowStorage> subMatrix = matrix.col(2);

	EXPECT_EQ(3, subMatrix.rows());
	EXPECT_EQ(1, subMatrix.cols());

	EXPECT_EQ(2, subMatrix(0, 0));
	EXPECT_EQ(6, subMatrix(1, 0));
	EXPECT_EQ(10, subMatrix(2, 0));
}
#pragma endregion

#pragma region Vector_SubMatrixConstructor
/*
 * Teste que le constructeur par sous matrice d'un Vector cr�e correctement l'objet avec une sous matrice qui contient une seule ligne
 */
TEST(NouveauTestLabo1FromLabo2, Vector_SubMatrixConstructor_Row)
{
	Matrix<int, 3, 4, RowStorage> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };
	SubMatrix<int, 3, 4, RowStorage> subMatrix = matrix.row(1);

	Vector<double> vector = subMatrix;

	EXPECT_EQ(4, vector.size());

	EXPECT_EQ(4, vector(0));
	EXPECT_EQ(5, vector(1));
	EXPECT_EQ(6, vector(2));
	EXPECT_EQ(7, vector(3));
}

/*
 * Teste que le constructeur par sous matrice d'un Vector cr�e correctement l'objet avec une sous matrice qui contient une seule colonne
 */
TEST(NouveauTestLabo1FromLabo2, Vector_SubMatrixConstructor_Col)
{
	Matrix<int, 3, 4, RowStorage> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11} };
	SubMatrix<int, 3, 4, RowStorage> subMatrix = matrix.col(2);

	Vector<double> vector = subMatrix;

	EXPECT_EQ(3, vector.size());

	EXPECT_EQ(2, vector(0));
	EXPECT_EQ(6, vector(1));
	EXPECT_EQ(10, vector(2));
}
#pragma endregion

#pragma region Matrix_MatrixSubstraction
/*
 * Teste que la soustraction de matrices g�n�riques fonctionne correctement pour des matrices carr�s
 */
TEST(TestLabo1, Operator_Substraction_MatrixMatrix_Generic_Square)
{
	Matrix<int, 3, 3, ColumnStorage> leftMatrix{ {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Matrix<int, 3, 3, ColumnStorage> rightMatrix{ {5, -6, 5}, {-6, -5, 6}, {5, 6, -5} };

	auto resultMatrix = leftMatrix - rightMatrix;

	EXPECT_EQ(9, resultMatrix.size());
	EXPECT_EQ(3, resultMatrix.rows());
	EXPECT_EQ(3, resultMatrix.cols());

	// Expected:
	// |-5,  7, -3|
	// | 9,  9, -1|
	// | 1,  1, 13|
	EXPECT_EQ(-5, resultMatrix(0, 0));
	EXPECT_EQ(7, resultMatrix(0, 1));
	EXPECT_EQ(-3, resultMatrix(0, 2));
	EXPECT_EQ(9, resultMatrix(1, 0));
	EXPECT_EQ(9, resultMatrix(1, 1));
	EXPECT_EQ(-1, resultMatrix(1, 2));
	EXPECT_EQ(1, resultMatrix(2, 0));
	EXPECT_EQ(1, resultMatrix(2, 1));
	EXPECT_EQ(13, resultMatrix(2, 2));
}

/*
 * Teste que la soustraction de matrices g�n�riques fonctionne correctement pour des matrices rectangulaires
 */
TEST(TestLabo1, Operator_Substraction_MatrixMatrix_Generic_Rectangle)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> leftMatrix{ {0, -1, 2, -4}, {5, 6, 7, -8} };
	Matrix<int, Dynamic, Dynamic, ColumnStorage> rightMatrix{ {1, 2, 3, 4}, {-5, -6, 7, 8} };

	auto resultMatrix = leftMatrix - rightMatrix;

	EXPECT_EQ(8, resultMatrix.size());
	EXPECT_EQ(2, resultMatrix.rows());
	EXPECT_EQ(4, resultMatrix.cols());

	// Expected:
	// |-1, -3, -1,  -8|
	// |10, 12,  0, -16|
	EXPECT_EQ(-1, resultMatrix(0, 0));
	EXPECT_EQ(-3, resultMatrix(0, 1));
	EXPECT_EQ(-1, resultMatrix(0, 2));
	EXPECT_EQ(-8, resultMatrix(0, 3));

	EXPECT_EQ(10, resultMatrix(1, 0));
	EXPECT_EQ(12, resultMatrix(1, 1));
	EXPECT_EQ(0, resultMatrix(1, 2));
	EXPECT_EQ(-16, resultMatrix(1, 3));
}

/*
 * Teste que la soustraction de matrices dynamiques par colonne fonctionne correctement pour des matrices carr�s
 */
TEST(TestLabo1, Operator_Substraction_MatrixMatrix_DynamicColumn_Square)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> leftMatrix{ {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Matrix<int, Dynamic, Dynamic, ColumnStorage> rightMatrix{ {5, -6, 5}, {-6, -5, 6}, {5, 6, -5} };

	auto resultMatrix = leftMatrix - rightMatrix;

	EXPECT_EQ(9, resultMatrix.size());
	EXPECT_EQ(3, resultMatrix.rows());
	EXPECT_EQ(3, resultMatrix.cols());

	// Expected:
	// |-5,  7, -3|
	// | 9,  9, -1|
	// | 1,  1, 13|
	EXPECT_EQ(-5, resultMatrix(0, 0));
	EXPECT_EQ(7, resultMatrix(0, 1));
	EXPECT_EQ(-3, resultMatrix(0, 2));
	EXPECT_EQ(9, resultMatrix(1, 0));
	EXPECT_EQ(9, resultMatrix(1, 1));
	EXPECT_EQ(-1, resultMatrix(1, 2));
	EXPECT_EQ(1, resultMatrix(2, 0));
	EXPECT_EQ(1, resultMatrix(2, 1));
	EXPECT_EQ(13, resultMatrix(2, 2));
}

/*
 * Teste que la soustraction de matrices dynamiques par colonne fonctionne correctement pour des matrices rectangulaires
 */
TEST(TestLabo1, Operator_Substraction_MatrixMatrix_DynamicColumn_Rectangle)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> leftMatrix{ {0, -1, 2, -4}, {5, 6, 7, -8} };
	Matrix<int, Dynamic, Dynamic, ColumnStorage> rightMatrix{ {1, 2, 3, 4}, {-5, -6, 7, 8} };

	auto resultMatrix = leftMatrix - rightMatrix;

	EXPECT_EQ(8, resultMatrix.size());
	EXPECT_EQ(2, resultMatrix.rows());
	EXPECT_EQ(4, resultMatrix.cols());

	// Expected:
	// |-1, -3, -1,  -8|
	// |10, 12,  0, -16|
	EXPECT_EQ(-1, resultMatrix(0, 0));
	EXPECT_EQ(-3, resultMatrix(0, 1));
	EXPECT_EQ(-1, resultMatrix(0, 2));
	EXPECT_EQ(-8, resultMatrix(0, 3));

	EXPECT_EQ(10, resultMatrix(1, 0));
	EXPECT_EQ(12, resultMatrix(1, 1));
	EXPECT_EQ(0, resultMatrix(1, 2));
	EXPECT_EQ(-16, resultMatrix(1, 3));
}

/*
 * Teste que la soustraction de matrices dynamiques par ligne fonctionne correctement pour des matrices carr�s
 */
TEST(TestLabo1, Operator_Substraction_MatrixMatrix_DynamicRow_Square)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> leftMatrix{ {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Matrix<int, Dynamic, Dynamic, RowStorage> rightMatrix{ {5, -6, 5}, {-6, -5, 6}, {5, 6, -5} };

	auto resultMatrix = leftMatrix - rightMatrix;

	EXPECT_EQ(9, resultMatrix.size());
	EXPECT_EQ(3, resultMatrix.rows());
	EXPECT_EQ(3, resultMatrix.cols());

	// Expected:
	// |-5,  7, -3|
	// | 9,  9, -1|
	// | 1,  1, 13|
	EXPECT_EQ(-5, resultMatrix(0, 0));
	EXPECT_EQ(7, resultMatrix(0, 1));
	EXPECT_EQ(-3, resultMatrix(0, 2));
	EXPECT_EQ(9, resultMatrix(1, 0));
	EXPECT_EQ(9, resultMatrix(1, 1));
	EXPECT_EQ(-1, resultMatrix(1, 2));
	EXPECT_EQ(1, resultMatrix(2, 0));
	EXPECT_EQ(1, resultMatrix(2, 1));
	EXPECT_EQ(13, resultMatrix(2, 2));
}

/*
 * Teste que la soustraction de matrices dynamiques par ligne fonctionne correctement pour des matrices rectangulaires
 */
TEST(TestLabo1, Operator_Substraction_MatrixMatrix_DynamicRow_Rectangle)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> leftMatrix{ {0, -1, 2, -4}, {5, 6, 7, -8} };
	Matrix<int, Dynamic, Dynamic, ColumnStorage> rightMatrix{ {1, 2, 3, 4}, {-5, -6, 7, 8} };

	auto resultMatrix = leftMatrix - rightMatrix;

	EXPECT_EQ(8, resultMatrix.size());
	EXPECT_EQ(2, resultMatrix.rows());
	EXPECT_EQ(4, resultMatrix.cols());

	// Expected:
	// |-1, -3, -1,  -8|
	// |10, 12,  0, -16|
	EXPECT_EQ(-1, resultMatrix(0, 0));
	EXPECT_EQ(-3, resultMatrix(0, 1));
	EXPECT_EQ(-1, resultMatrix(0, 2));
	EXPECT_EQ(-8, resultMatrix(0, 3));

	EXPECT_EQ(10, resultMatrix(1, 0));
	EXPECT_EQ(12, resultMatrix(1, 1));
	EXPECT_EQ(0, resultMatrix(1, 2));
	EXPECT_EQ(-16, resultMatrix(1, 3));
}
#pragma endregion

#pragma region Matrix_meanMatrix
/*
 * Teste que la m�thode meanMatrix retourne bien une matrice o� chaque colonne est la moyenne des toutes les colonnes de la matrice pour une matrice par colonnes
 */
TEST(TestLabo1, Matrix_Column_meanMatrix_Ok)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> matrix{ {0, 1, 4, 6, 7, 0}, {-3, -4, 5, -8, -8, 0}, {10, 12, 16, -16, 18, 2} };

	auto resultMatrix = matrix.meanMatrix();

	EXPECT_EQ(3, resultMatrix.rows());
	EXPECT_EQ(6, resultMatrix.cols());

	// Expected:
	// | 3,  3,  3,  3,  3|
	// |-3, -3, -3, -3, -3|
	// | 7,  7,  7,  7,  7|
	EXPECT_EQ(3, resultMatrix(0, 1));
	EXPECT_EQ(3, resultMatrix(0, 2));
	EXPECT_EQ(3, resultMatrix(0, 3));
	EXPECT_EQ(3, resultMatrix(0, 4));
	EXPECT_EQ(3, resultMatrix(0, 0));

	EXPECT_EQ(-3, resultMatrix(1, 0));
	EXPECT_EQ(-3, resultMatrix(1, 1));
	EXPECT_EQ(-3, resultMatrix(1, 2));
	EXPECT_EQ(-3, resultMatrix(1, 3));
	EXPECT_EQ(-3, resultMatrix(1, 4));

	EXPECT_EQ(7, resultMatrix(2, 0));
	EXPECT_EQ(7, resultMatrix(2, 1));
	EXPECT_EQ(7, resultMatrix(2, 2));
	EXPECT_EQ(7, resultMatrix(2, 3));
	EXPECT_EQ(7, resultMatrix(2, 4));
}

/*
 * Teste que la m�thode meanMatrix retourne bien une matrice o� chaque colonne est la moyenne des toutes les colonnes de la matrice pour une matrice par lignes
 */
TEST(TestLabo1, Matrix_Row_meanMatrix_Ok)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> matrix{ {0, 1, 4, 6, 7, 0}, {-3, -4, 5, -8, -8, 0}, {10, 12, 16, -16, 18, 2} };

	auto resultMatrix = matrix.meanMatrix();

	EXPECT_EQ(3, resultMatrix.rows());
	EXPECT_EQ(6, resultMatrix.cols());

	// Expected:
	// | 3,  3,  3,  3,  3|
	// |-3, -3, -3, -3, -3|
	// | 7,  7,  7,  7,  7|
	EXPECT_EQ(3, resultMatrix(0, 1));
	EXPECT_EQ(3, resultMatrix(0, 2));
	EXPECT_EQ(3, resultMatrix(0, 3));
	EXPECT_EQ(3, resultMatrix(0, 4));
	EXPECT_EQ(3, resultMatrix(0, 0));

	EXPECT_EQ(-3, resultMatrix(1, 0));
	EXPECT_EQ(-3, resultMatrix(1, 1));
	EXPECT_EQ(-3, resultMatrix(1, 2));
	EXPECT_EQ(-3, resultMatrix(1, 3));
	EXPECT_EQ(-3, resultMatrix(1, 4));

	EXPECT_EQ(7, resultMatrix(2, 0));
	EXPECT_EQ(7, resultMatrix(2, 1));
	EXPECT_EQ(7, resultMatrix(2, 2));
	EXPECT_EQ(7, resultMatrix(2, 3));
	EXPECT_EQ(7, resultMatrix(2, 4));
}
#pragma endregion