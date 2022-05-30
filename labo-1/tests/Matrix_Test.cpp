/**
 * @file Matrix_Test.cpp
 *
 * @brief Unit tests for the Matrix class.
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include <gtest/gtest.h>

#include "../Matrix.h"

using namespace gti320;

#pragma region Matrix_ColumnStorage

/*
 * Teste que le constructeur par d�faut d'une Matrix par colonnes cr�e correctement l'objet
 */
TEST(TestLabo1, Matrix_ColumnStorage_DefaultConstructor_Ok)
{
	Matrix<double, 3, 5, ColumnStorage> doubleMatrix;
	Matrix<float, 1, Dynamic, ColumnStorage> floatMatrix;
	Matrix<int, Dynamic, 4, ColumnStorage> intMatrix;
	Matrix<char, Dynamic, Dynamic, ColumnStorage> charMatrix;

	EXPECT_EQ(3 * 5, doubleMatrix.size());
	EXPECT_EQ(3, doubleMatrix.rows());
	EXPECT_EQ(5, doubleMatrix.cols());

	EXPECT_EQ(0, floatMatrix.size());
	EXPECT_EQ(1, floatMatrix.rows());
	EXPECT_EQ(0, floatMatrix.cols());

	EXPECT_EQ(0, intMatrix.size());
	EXPECT_EQ(0, intMatrix.rows());
	EXPECT_EQ(4, intMatrix.cols());

	EXPECT_EQ(0, charMatrix.size());
	EXPECT_EQ(0, charMatrix.rows());
	EXPECT_EQ(0, charMatrix.cols());

	for (auto i = 0; i < 3 * 5; ++i)
	{
		EXPECT_DOUBLE_EQ(0.0, doubleMatrix[i]);
	}
}

/*
 * Teste que le constructeur par copie d'une Matrix par colonnes cr�e correctement l'objet
 */
TEST(TestLabo1, Matrix_ColumnStorage_CopyConstructor_Ok)
{
	Matrix<int, 2, 3, ColumnStorage> matrixToCopy = { {0, 1, 2}, {3, 4, 5} };

	Matrix<int, 2, 3, ColumnStorage> matrix(matrixToCopy);

	EXPECT_EQ(2 * 3, matrix.size());
	EXPECT_EQ(2, matrix.rows());
	EXPECT_EQ(3, matrix.cols());

	int expectedValue = 0;
	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(expectedValue, matrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que le constructeur par taille d'une Matrix par colonnes cr�e correctement l'objet
 */
TEST(TestLabo1, Matrix_ColumnStorage_SizeConstructor_Ok)
{
	Matrix<int, 2, 3, ColumnStorage> matrix(2, 3);

	EXPECT_EQ(2 * 3, matrix.size());
	EXPECT_EQ(2, matrix.rows());
	EXPECT_EQ(3, matrix.cols());

	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(0, matrix(i, j));
		}
	}
}

/*
 * Teste que le constructeur par liste d'initialisation d'une Matrix par colonnes cr�e correctement l'objet
 */
TEST(TestLabo1, Matrix_ColumnStorage_InitializationListConstructor_Ok)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> matrix = { {0, 1, 2}, {3, 4, 5} };

	EXPECT_EQ(2 * 3, matrix.size());
	EXPECT_EQ(2, matrix.rows());
	EXPECT_EQ(3, matrix.cols());

	int expectedValue = 0;
	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(expectedValue, matrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que l'op�rateur "=" d'une matrice par colonnes copie correctement une sous matrice par colonnes
 */
TEST(TestLabo1, Matrix_ColumnStorage_EqualOperator_ColumnStorageSubMatrix_Ok)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> matrixToCopy = { {0, 0, 1, 2}, {0, 3, 4, 5} };
	auto subMatrix = matrixToCopy.block(0, 1, 2, 3);

	Matrix<int, 2, 3, ColumnStorage> matrix;
	matrix = subMatrix;

	EXPECT_EQ(2 * 3, matrix.size());
	EXPECT_EQ(2, matrix.rows());
	EXPECT_EQ(3, matrix.cols());

	int expectedValue = 0;
	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(expectedValue, matrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que l'op�rateur "=" d'une matrice par colonnes copie correctement une sous matrice par lignes
 */
TEST(TestLabo1, Matrix_ColumnStorage_EqualOperator_RowsStorageSubMatrix_Ok)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> matrixToCopy = { {0, 0, 1, 2}, {0, 3, 4, 5} };
	auto subMatrix = matrixToCopy.block(0, 1, 2, 3);

	Matrix<int, 2, 3, ColumnStorage> matrix;
	matrix = subMatrix;

	EXPECT_EQ(2 * 3, matrix.size());
	EXPECT_EQ(2, matrix.rows());
	EXPECT_EQ(3, matrix.cols());

	int expectedValue = 0;
	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(expectedValue, matrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que la m�thode "block" d'une matrice par colonnes retourne correctement la sous matrice correspondante
 */
TEST(TestLabo1, Matrix_ColumnStorage_bloc_Ok)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> matrixToCopy = { {0, 0, 1, 2}, {0, 3, 4, 5} };
	auto subMatrix1 = matrixToCopy.block(0, 1, 2, 3);
	auto subMatrix2 = matrixToCopy.block(1, 1, 1, 1);
	auto subMatrix3 = matrixToCopy.block(0, 0, 2, 4);

	EXPECT_EQ(2, subMatrix1.rows());
	EXPECT_EQ(3, subMatrix1.cols());

	EXPECT_EQ(1, subMatrix2.rows());
	EXPECT_EQ(1, subMatrix2.cols());

	EXPECT_EQ(2, subMatrix3.rows());
	EXPECT_EQ(4, subMatrix3.cols());

	int expectedValue = 0;
	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(expectedValue, subMatrix1(i, j));
			++expectedValue;
		}
	}

	EXPECT_EQ(3, subMatrix2(0, 0));
}

/*
 * Teste que la m�thode "transpose" d'une matrice par colonnes retourne bien une matrice transpos�e avec un retour sp�cialis�
 */
TEST(TestLabo1, Matrix_ColumnStorage_transpose_SpecializedReturn_Ok)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> rectangleMatrix = { {0, 1, 2}, {3, 4, 5} };
	Matrix<int, Dynamic, Dynamic, ColumnStorage> squareMatrix = { {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Matrix<int, Dynamic, Dynamic, ColumnStorage> singleMatrix = { {1} };

	Matrix<int, 3, 2, RowStorage> transposedRectangleMatrix = rectangleMatrix.transpose<int, 3, 2, RowStorage>();
	Matrix<int, Dynamic, Dynamic, ColumnStorage> transposedSquareMatrix = squareMatrix.transpose<int, Dynamic, Dynamic, ColumnStorage>();
	Matrix<int, 1, 1, RowStorage> transposedSingleMatrix = singleMatrix.transpose<int, 1, 1, RowStorage>();

	EXPECT_EQ(3, transposedRectangleMatrix.rows());
	EXPECT_EQ(2, transposedRectangleMatrix.cols());
	EXPECT_EQ(3, transposedSquareMatrix.rows());
	EXPECT_EQ(3, transposedSquareMatrix.cols());
	EXPECT_EQ(1, singleMatrix.rows());
	EXPECT_EQ(1, singleMatrix.cols());

	int expectedValue = 0;
	for (auto j = 0; j < 2; ++j)
	{
		for (auto i = 0; i < 3; ++i)
		{
			EXPECT_EQ(expectedValue, transposedRectangleMatrix(i, j));
			++expectedValue;
		}
	}

	expectedValue = 0;
	for (auto j = 0; j < 3; ++j)
	{
		for (auto i = 0; i < 3; ++i)
		{
			EXPECT_EQ(expectedValue, transposedSquareMatrix(i, j));
			++expectedValue;
		}
	}

	EXPECT_EQ(1, singleMatrix(0, 0));
}

/*
 * Teste que la m�thode "transpose" d'une matrice par colonnes retourne bien une matrice transpos�e avec un retour par d�faut
 */
TEST(TestLabo1, Matrix_ColumnStorage_transpose_DefaultReturn_Ok)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> rectangleMatrix = { {0, 1, 2}, {3, 4, 5} };
	Matrix<int, Dynamic, Dynamic, ColumnStorage> squareMatrix = { {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Matrix<int, Dynamic, Dynamic, ColumnStorage> singleMatrix = { {1} };

	auto transposedRectangleMatrix = rectangleMatrix.transpose();
	auto transposedSquareMatrix = squareMatrix.transpose();
	auto transposedSingleMatrix = singleMatrix.transpose();

	EXPECT_EQ(3, transposedRectangleMatrix.rows());
	EXPECT_EQ(2, transposedRectangleMatrix.cols());
	EXPECT_EQ(3, transposedSquareMatrix.rows());
	EXPECT_EQ(3, transposedSquareMatrix.cols());
	EXPECT_EQ(1, singleMatrix.rows());
	EXPECT_EQ(1, singleMatrix.cols());

	int expectedValue = 0;
	for (auto j = 0; j < 2; ++j)
	{
		for (auto i = 0; i < 3; ++i)
		{
			EXPECT_EQ(expectedValue, transposedRectangleMatrix(i, j));
			++expectedValue;
		}
	}

	expectedValue = 0;
	for (auto j = 0; j < 3; ++j)
	{
		for (auto i = 0; i < 3; ++i)
		{
			EXPECT_EQ(expectedValue, transposedSquareMatrix(i, j));
			++expectedValue;
		}
	}

	EXPECT_EQ(1, singleMatrix(0, 0));
}

/*
 * Teste que la m�thode "setIdentity" d'une matrice par colonnes change bien la matrice pour une matrice identit�
 */
TEST(TestLabo1, Matrix_ColumnStorage_transpose_Ok)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> squareMatrix = { {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Matrix<int, Dynamic, Dynamic, ColumnStorage> rectangleMatrix = { {0, 1}, {3, 4}, {6, 7} };

	squareMatrix.setIdentity();
	rectangleMatrix.setIdentity();

	for (auto i = 0; i < squareMatrix.rows(); ++i)
	{
		for (auto j = 0; j < squareMatrix.cols(); ++j)
		{
			if (i == j)
			{
				EXPECT_EQ(1, squareMatrix(i, j));
			}
			else
			{
				EXPECT_EQ(0, squareMatrix(i, j));
			}
		}
	}

	for (auto i = 0; i < rectangleMatrix.rows(); ++i)
	{
		for (auto j = 0; j < rectangleMatrix.cols(); ++j)
		{
			if (i == j)
			{
				EXPECT_EQ(1, rectangleMatrix(i, j));
			}
			else
			{
				EXPECT_EQ(0, rectangleMatrix(i, j));
			}
		}
	}
}

#pragma endregion

#pragma region Matrix_RowStorage

/*
 * Teste que le constructeur par d�faut d'une Matrix par lignes cr�e correctement l'objet
 */
TEST(TestLabo1, Matrix_RowStorage_DefaultConstructor_Ok)
{
	Matrix<double, 3, 5, RowStorage> doubleMatrix;
	Matrix<float, 1, Dynamic, RowStorage> floatMatrix;
	Matrix<int, Dynamic, 4, RowStorage> intMatrix;
	Matrix<char, Dynamic, Dynamic, RowStorage> charMatrix;

	EXPECT_EQ(3 * 5, doubleMatrix.size());
	EXPECT_EQ(3, doubleMatrix.rows());
	EXPECT_EQ(5, doubleMatrix.cols());

	EXPECT_EQ(0, floatMatrix.size());
	EXPECT_EQ(1, floatMatrix.rows());
	EXPECT_EQ(0, floatMatrix.cols());

	EXPECT_EQ(0, intMatrix.size());
	EXPECT_EQ(0, intMatrix.rows());
	EXPECT_EQ(4, intMatrix.cols());

	EXPECT_EQ(0, charMatrix.size());
	EXPECT_EQ(0, charMatrix.rows());
	EXPECT_EQ(0, charMatrix.cols());

	for (auto i = 0; i < 3 * 5; ++i)
	{
		EXPECT_DOUBLE_EQ(0.0, doubleMatrix[i]);
	}
}

/*
 * Teste que le constructeur par copie d'une Matrix par lignes cr�e correctement l'objet
 */
TEST(TestLabo1, Matrix_RowStorage_CopyConstructor_Ok)
{
	Matrix<int, 2, 3, RowStorage> matrixToCopy = { {0, 1, 2}, {3, 4, 5} };

	Matrix<int, 2, 3, RowStorage> matrix(matrixToCopy);

	EXPECT_EQ(2 * 3, matrix.size());
	EXPECT_EQ(2, matrix.rows());
	EXPECT_EQ(3, matrix.cols());

	int expectedValue = 0;
	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(expectedValue, matrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que le constructeur par taille d'une Matrix par lignes cr�e correctement l'objet
 */
TEST(TestLabo1, Matrix_RowStorage_SizeConstructor_Ok)
{
	Matrix<int, 2, 3, RowStorage> matrix(2, 3);

	EXPECT_EQ(2 * 3, matrix.size());
	EXPECT_EQ(2, matrix.rows());
	EXPECT_EQ(3, matrix.cols());

	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(0, matrix(i, j));
		}
	}
}

/*
 * Teste que le constructeur par liste d'initialisation d'une Matrix par lignes cr�e correctement l'objet
 */
TEST(TestLabo1, Matrix_RowStorage_InitializationListConstructor_Ok)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> matrix = { {0, 1, 2}, {3, 4, 5} };

	EXPECT_EQ(2 * 3, matrix.size());
	EXPECT_EQ(2, matrix.rows());
	EXPECT_EQ(3, matrix.cols());

	int expectedValue = 0;
	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(expectedValue, matrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que l'op�rateur "=" d'une matrice par lignes copie correctement une sous matrice par colonnes
 */
TEST(TestLabo1, Matrix_RowStorage_EqualOperator_ColumnStorageSubMatrix_Ok)
{
	Matrix<int, Dynamic, Dynamic, ColumnStorage> matrixToCopy = { {0, 0, 1, 2}, {0, 3, 4, 5} };
	auto subMatrix = matrixToCopy.block(0, 1, 2, 3);

	Matrix<int, 2, 3, RowStorage> matrix;
	matrix = subMatrix;

	EXPECT_EQ(2 * 3, matrix.size());
	EXPECT_EQ(2, matrix.rows());
	EXPECT_EQ(3, matrix.cols());

	int expectedValue = 0;
	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(expectedValue, matrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que l'op�rateur "=" d'une matrice par lignes copie correctement une sous matrice par lignes
 */
TEST(TestLabo1, Matrix_RowStorage_EqualOperator_RowsStorageSubMatrix_Ok)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> matrixToCopy = { {0, 0, 1, 2}, {0, 3, 4, 5} };
	auto subMatrix = matrixToCopy.block(0, 1, 2, 3);

	Matrix<int, 2, 3, RowStorage> matrix;
	matrix = subMatrix;

	EXPECT_EQ(2 * 3, matrix.size());
	EXPECT_EQ(2, matrix.rows());
	EXPECT_EQ(3, matrix.cols());

	int expectedValue = 0;
	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(expectedValue, matrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que la m�thode "block" d'une matrice par lignes retourne correctement la sous matrice correspondante
 */
TEST(TestLabo1, Matrix_RowStorage_bloc_Ok)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> matrixToCopy = { {0, 0, 1, 2}, {0, 3, 4, 5} };
	auto subMatrix1 = matrixToCopy.block(0, 1, 2, 3);
	auto subMatrix2 = matrixToCopy.block(1, 1, 1, 1);
	auto subMatrix3 = matrixToCopy.block(0, 0, 2, 4);

	EXPECT_EQ(2, subMatrix1.rows());
	EXPECT_EQ(3, subMatrix1.cols());

	EXPECT_EQ(1, subMatrix2.rows());
	EXPECT_EQ(1, subMatrix2.cols());

	EXPECT_EQ(2, subMatrix3.rows());
	EXPECT_EQ(4, subMatrix3.cols());

	int expectedValue = 0;
	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(expectedValue, subMatrix1(i, j));
			++expectedValue;
		}
	}

	EXPECT_EQ(3, subMatrix2(0, 0));
}

/*
 * Teste que la m�thode "transpose" d'une matrice par lignes retourne bien une matrice transpos�e avec un retour sp�cialis�
 */
TEST(TestLabo1, Matrix_RowStorage_transpose_SpecializedReturn_Ok)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> rectangleMatrix = { {0, 1, 2}, {3, 4, 5} };
	Matrix<int, Dynamic, Dynamic, RowStorage> squareMatrix = { {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Matrix<int, Dynamic, Dynamic, RowStorage> singleMatrix = { {1} };

	Matrix<int, 3, 2, RowStorage> transposedRectangleMatrix = rectangleMatrix.transpose<int, 3, 2, RowStorage>();
	Matrix<int, Dynamic, Dynamic, ColumnStorage> transposedSquareMatrix = squareMatrix.transpose<int, Dynamic, Dynamic, ColumnStorage>();
	Matrix<int, 1, 1, RowStorage> transposedSingleMatrix = singleMatrix.transpose<int, 1, 1, RowStorage>();

	EXPECT_EQ(3, transposedRectangleMatrix.rows());
	EXPECT_EQ(2, transposedRectangleMatrix.cols());
	EXPECT_EQ(3, transposedSquareMatrix.rows());
	EXPECT_EQ(3, transposedSquareMatrix.cols());
	EXPECT_EQ(1, singleMatrix.rows());
	EXPECT_EQ(1, singleMatrix.cols());

	int expectedValue = 0;
	for (auto j = 0; j < 2; ++j)
	{
		for (auto i = 0; i < 3; ++i)
		{
			EXPECT_EQ(expectedValue, transposedRectangleMatrix(i, j));
			++expectedValue;
		}
	}

	expectedValue = 0;
	for (auto j = 0; j < 3; ++j)
	{
		for (auto i = 0; i < 3; ++i)
		{
			EXPECT_EQ(expectedValue, transposedSquareMatrix(i, j));
			++expectedValue;
		}
	}

	EXPECT_EQ(1, singleMatrix(0, 0));
}

/*
 * Teste que la m�thode "transpose" d'une matrice par lignes retourne bien une matrice transpos�e avec un retour par d�faut
 */
TEST(TestLabo1, Matrix_RowStorage_transpose_DefaultReturn_Ok)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> rectangleMatrix = { {0, 1, 2}, {3, 4, 5} };
	Matrix<int, Dynamic, Dynamic, RowStorage> squareMatrix = { {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Matrix<int, Dynamic, Dynamic, RowStorage> singleMatrix = { {1} };

	auto transposedRectangleMatrix = rectangleMatrix.transpose();
	auto transposedSquareMatrix = squareMatrix.transpose();
	auto transposedSingleMatrix = singleMatrix.transpose();

	EXPECT_EQ(3, transposedRectangleMatrix.rows());
	EXPECT_EQ(2, transposedRectangleMatrix.cols());
	EXPECT_EQ(3, transposedSquareMatrix.rows());
	EXPECT_EQ(3, transposedSquareMatrix.cols());
	EXPECT_EQ(1, singleMatrix.rows());
	EXPECT_EQ(1, singleMatrix.cols());

	int expectedValue = 0;
	for (auto j = 0; j < 2; ++j)
	{
		for (auto i = 0; i < 3; ++i)
		{
			EXPECT_EQ(expectedValue, transposedRectangleMatrix(i, j));
			++expectedValue;
		}
	}

	expectedValue = 0;
	for (auto j = 0; j < 3; ++j)
	{
		for (auto i = 0; i < 3; ++i)
		{
			EXPECT_EQ(expectedValue, transposedSquareMatrix(i, j));
			++expectedValue;
		}
	}

	EXPECT_EQ(1, singleMatrix(0, 0));
}

/*
 * Teste que la m�thode "setIdentity" d'une matrice par lignes change bien la matrice pour une matrice identit�
 */
TEST(TestLabo1, Matrix_RowStorage_transpose_Ok)
{
	Matrix<int, Dynamic, Dynamic, RowStorage> squareMatrix = { {0, 1, 2}, {3, 4, 5}, {6, 7, 8} };
	Matrix<int, Dynamic, Dynamic, RowStorage> rectangleMatrix = { {0, 1}, {3, 4}, {6, 7} };

	squareMatrix.setIdentity();
	rectangleMatrix.setIdentity();

	for (auto i = 0; i < squareMatrix.rows(); ++i)
	{
		for (auto j = 0; j < squareMatrix.cols(); ++j)
		{
			if (i == j)
			{
				EXPECT_EQ(1, squareMatrix(i, j));
			}
			else
			{
				EXPECT_EQ(0, squareMatrix(i, j));
			}
		}
	}

	for (auto i = 0; i < rectangleMatrix.rows(); ++i)
	{
		for (auto j = 0; j < rectangleMatrix.cols(); ++j)
		{
			if (i == j)
			{
				EXPECT_EQ(1, rectangleMatrix(i, j));
			}
			else
			{
				EXPECT_EQ(0, rectangleMatrix(i, j));
			}
		}
	}
}

#pragma endregion

#pragma region Submatrix

/*
 * Teste que le constructeur avec une matrice d'une SubMatrix cr�e correctement l'objet
 */
TEST(TestLabo1, SubMatrix_MatrixConstructor_Ok)
{
	Matrix<int, 4, 4> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11}, {12, 13, 14, 15} };

	SubMatrix<int, 4, 4> subMatrix(matrix, 2, 1, 2, 3);

	EXPECT_EQ(2, subMatrix.rows());
	EXPECT_EQ(3, subMatrix.cols());

	// Expected:
	// | 9, 10, 11|
	// |13, 14, 15|
	EXPECT_EQ(9, subMatrix(0, 0));
	EXPECT_EQ(13, subMatrix(1, 0));

	EXPECT_EQ(10, subMatrix(0, 1));
	EXPECT_EQ(14, subMatrix(1, 1));

	EXPECT_EQ(11, subMatrix(0, 2));
	EXPECT_EQ(15, subMatrix(1, 2));

	// On regarde si on est en mesure de changer une valeur dans la matrice originale
	subMatrix(0, 0) = 40;
	EXPECT_EQ(40, matrix(2, 1));
}

/*
 * Teste que le constructeur avec une matrice d'une SubMatrix cr�e correctement l'objet si on demande de prendre toute la matrice
 */
TEST(TestLabo1, SubMatrix_MatrixConstructor_AllMatrix_Ok)
{
	Matrix<int, 2, 3> matrix{ {0, 1, 2}, {3, 4, 5} };

	SubMatrix<int, 2, 3> subMatrix(matrix, 0, 0, 2, 3);

	EXPECT_EQ(2, subMatrix.rows());
	EXPECT_EQ(3, subMatrix.cols());

	int expectedValue = 0;
	for (auto i = 0; i < 2; ++i)
	{
		for (auto j = 0; j < 3; ++j)
		{
			EXPECT_EQ(expectedValue, subMatrix(i, j));
			++expectedValue;
		}
	}
}

/*
 * Teste que le constructeur avec une matrice d'une SubMatrix cr�e correctement l'objet si on demande de prendre une matrix de taille 1x1
 */
TEST(TestLabo1, SubMatrix_MatrixConstructor_OneByOne_Ok)
{
	Matrix<int, 2, 3> matrix{ {0, 1, 2}, {3, 4, 5} };

	SubMatrix<int, 2, 3> subMatrix(matrix, 0, 1, 1, 1);

	EXPECT_EQ(1, subMatrix.rows());
	EXPECT_EQ(1, subMatrix.cols());
	EXPECT_EQ(1, subMatrix(0, 0));
}

/*
 * Teste que le constructeur avec une matrice constante d'une SubMatrix cr�e correctement l'objet
 */
TEST(TestLabo1, SubMatrix_ConstMatrixConstructor_Ok)
{
	const Matrix<int, 4, 4> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11}, {12, 13, 14, 15} };

	SubMatrix<int, 4, 4> subMatrix(matrix, 2, 1, 2, 3);

	EXPECT_EQ(2, subMatrix.rows());
	EXPECT_EQ(3, subMatrix.cols());

	// Expected:
	// | 9, 10, 11|
	// |13, 14, 15|
	EXPECT_EQ(9, subMatrix(0, 0));
	EXPECT_EQ(13, subMatrix(1, 0));

	EXPECT_EQ(10, subMatrix(0, 1));
	EXPECT_EQ(14, subMatrix(1, 1));

	EXPECT_EQ(11, subMatrix(0, 2));
	EXPECT_EQ(15, subMatrix(1, 2));

	// On regarde si on est en mesure de changer une valeur dans la matrice originale
	subMatrix(0, 0) = 40;
	EXPECT_EQ(40, matrix(2, 1));
}

/*
 * Teste que le constructeur de copie d'une SubMatrix cr�e correctement l'objet
 */
TEST(TestLabo1, SubMatrix_CopyConstructor_Ok)
{
	Matrix<int, 4, 4> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11}, {12, 13, 14, 15} };
	SubMatrix<int, 4, 4> subMatrixToCopy(matrix, 2, 1, 2, 3);

	SubMatrix<int, 4, 4> subMatrix(subMatrixToCopy);

	EXPECT_EQ(2, subMatrix.rows());
	EXPECT_EQ(3, subMatrix.cols());

	// Expected:
	// | 9, 10, 11|
	// |13, 14, 15|
	EXPECT_EQ(9, subMatrix(0, 0));
	EXPECT_EQ(13, subMatrix(1, 0));

	EXPECT_EQ(10, subMatrix(0, 1));
	EXPECT_EQ(14, subMatrix(1, 1));

	EXPECT_EQ(11, subMatrix(0, 2));
	EXPECT_EQ(15, subMatrix(1, 2));

	// On regarde si on est en mesure de changer une valeur dans la matrice originale
	subMatrix(0, 0) = 40;
	EXPECT_EQ(40, matrix(2, 1));
}

/*
 * Teste que l'op�rateur "=" d'une sous matrice avec une matrice par colonnes copie correctement les donn�es
 */
TEST(TestLabo1, SubMatrix_OperatorEqual_ColumnStorage_Ok)
{
	Matrix<int, 4, 4> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11}, {12, 13, 14, 15} };
	SubMatrix<int, 4, 4> subMatrix(matrix, 2, 1, 2, 3);

	Matrix<int, 2, 3, ColumnStorage> matrixToCopy{ {100, 101, 102}, {103, 104, 105} };

	subMatrix = matrixToCopy;

	EXPECT_EQ(100, subMatrix(0, 0));
	EXPECT_EQ(101, subMatrix(0, 1));
	EXPECT_EQ(102, subMatrix(0, 2));
	EXPECT_EQ(103, subMatrix(1, 0));
	EXPECT_EQ(104, subMatrix(1, 1));
	EXPECT_EQ(105, subMatrix(1, 2));

	// Il faut �galement que la matrice originale soit chang�e
	EXPECT_EQ(100, matrix(2, 1));
	EXPECT_EQ(101, matrix(2, 2));
	EXPECT_EQ(102, matrix(2, 3));
	EXPECT_EQ(103, matrix(3, 1));
	EXPECT_EQ(104, matrix(3, 2));
	EXPECT_EQ(105, matrix(3, 3));
}

/*
 * Teste que l'op�rateur "=" d'une sous matrice avec une matrice par lignes copie correctement les donn�es
 */
TEST(TestLabo1, SubMatrix_OperatorEqual_RowStorage_Ok)
{
	Matrix<int, 4, 4> matrix{ {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11}, {12, 13, 14, 15} };
	SubMatrix<int, 4, 4> subMatrix(matrix, 2, 1, 2, 3);

	Matrix<int, 2, 3, RowStorage> matrixToCopy{ {100, 101, 102}, {103, 104, 105} };

	subMatrix = matrixToCopy;

	EXPECT_EQ(100, subMatrix(0, 0));
	EXPECT_EQ(101, subMatrix(0, 1));
	EXPECT_EQ(102, subMatrix(0, 2));
	EXPECT_EQ(103, subMatrix(1, 0));
	EXPECT_EQ(104, subMatrix(1, 1));
	EXPECT_EQ(105, subMatrix(1, 2));

	// Il faut �galement que la matrice originale soit chang�e
	EXPECT_EQ(100, matrix(2, 1));
	EXPECT_EQ(101, matrix(2, 2));
	EXPECT_EQ(102, matrix(2, 3));
	EXPECT_EQ(103, matrix(3, 1));
	EXPECT_EQ(104, matrix(3, 2));
	EXPECT_EQ(105, matrix(3, 3));
}

/*
 * Teste que la fonction "transpose" d'une sous matrice donne correctement la matrice transpos�e lorsque l'on souhaite avoir une matrice par colonnes
 */
TEST(TestLabo1, SubMatrix_Transpose_ColumnStorage_Ok)
{
	Matrix<int, 4, 4> matrix{ {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 1, 2, 3}, {0, 4, 5, 6} };
	SubMatrix<int, 4, 4> subMatrix(matrix, 2, 1, 2, 3);

	Matrix<int, 3, 2, ColumnStorage> transposedMatrix = subMatrix.transpose<int, 3, 2, ColumnStorage>();

	// Original:	Expected:
	// |1, 2, 3|	|1, 4|
	// |4, 5, 6|	|2, 5|
	//				|3, 6|
	EXPECT_EQ(1, transposedMatrix(0, 0));
	EXPECT_EQ(2, transposedMatrix(1, 0));
	EXPECT_EQ(3, transposedMatrix(2, 0));
	EXPECT_EQ(4, transposedMatrix(0, 1));
	EXPECT_EQ(5, transposedMatrix(1, 1));
	EXPECT_EQ(6, transposedMatrix(2, 1));
}

/*
 * Teste que la fonction "transpose" d'une sous matrice donne correctement la matrice transpos�e lorsque l'on souhaite avoir une matrice par lignes
 */
TEST(TestLabo1, SubMatrix_Transpose_RowStorage_Ok)
{
	Matrix<int, 4, 4, RowStorage> matrix{ {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 1, 2, 3}, {0, 4, 5, 6} };
	SubMatrix<int, 4, 4, RowStorage> subMatrix(matrix, 2, 1, 2, 3);

	Matrix<int, 3, 2, RowStorage> transposedMatrix = subMatrix.transpose<int, 3, 2, RowStorage>();

	// Original:	Expected:
	// |1, 2, 3|	|1, 4|
	// |4, 5, 6|	|2, 5|
	//				|3, 6|
	EXPECT_EQ(1, transposedMatrix(0, 0));
	EXPECT_EQ(2, transposedMatrix(1, 0));
	EXPECT_EQ(3, transposedMatrix(2, 0));
	EXPECT_EQ(4, transposedMatrix(0, 1));
	EXPECT_EQ(5, transposedMatrix(1, 1));
	EXPECT_EQ(6, transposedMatrix(2, 1));
}

#pragma endregion