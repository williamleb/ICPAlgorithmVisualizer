#pragma once

/**
 * @file Operators.h
 *
 * @brief Opérateurs arithmétiques pour les matrices et les vecteurs.
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include "Matrix.h"
#include "Vector.h"
#include "GTIAssert.h"

/**
 * Implantation de divers opérateurs arithmétiques pour les matrices et les
 * vecteurs.
 */
namespace gti320
{
	/**
	 * Multiplication : Matrix * Matrix (générique)
	 */
	template <typename Scalar, int RowsA, int ColsA, int StorageA, int RowsB, int ColsB, int StorageB>
	Matrix<Scalar, RowsA, ColsB> operator*(const Matrix<Scalar, RowsA, ColsA, StorageA>& left, const Matrix<Scalar, RowsB, ColsB, StorageB>& right)
	{
		ASSERT(left.cols() == right.rows(), "Trying to multiply two matrices that are not compatible together");

		// Puisqu'on ne peut pas inférer la méthode de stockage de la matrice, on ne peut pas faire d'optimisations et
		// on doit utiliser la méthode générique
		Matrix<Scalar, RowsA, ColsB> result(left.rows(), right.cols());
		for (auto j = 0; j < right.cols(); ++j)
		{
			for (auto i = 0; i < left.rows(); ++i)
			{
				for (auto k = 0; k < left.cols(); ++k)
				{
					result(i, j) += left(i, k) * right(k, j);
				}
			}
		}

		return result;
	}

	/**
	 * Multiplication : Matrix(colonnes) * Matrix(lignes)
	 *
	 * Spécialisation de l'opérateur de multiplication pour le cas où la matrice
	 * de gauche utilise un stockage par colonnes.
	 */
	template <typename Scalar>
	Matrix<Scalar, Dynamic, Dynamic> operator*(const Matrix<Scalar, Dynamic, Dynamic, ColumnStorage>& left, const Matrix<Scalar, Dynamic, Dynamic, RowStorage>& right)
	{
		ASSERT(left.cols() == right.rows(), "Trying to multiply two matrices that are not compatible together");

		// Le stockage des matrices est incompatible avec la multiplication de matrice traditionnel. On doit plutôt
		// itérer sur les lignes et les colonnes avant d'itérer sur k.
		Matrix<Scalar, Dynamic, Dynamic> result(left.rows(), right.cols());
		for (auto k = 0; k < left.cols(); ++k)
		{
			for (auto j = 0; j < right.cols(); ++j)
			{
				for (auto i = 0; i < left.rows(); ++i)
				{
					result(i, j) += left(i, k) * right(k, j);
				}
			}
		}

		return result;
	}

	/**
	 * Multiplication : Matrix(ligne) * Matrix(colonne)
	 *
	 * Spécialisation de l'opérateur de multiplication pour le cas où la matrice
	 * de gauche utilise un stockage par lignes et celle de gauche un stockage
	 * par colonnes.
	 */
	template <typename Scalar>
	Matrix<Scalar, Dynamic, Dynamic> operator*(const Matrix<Scalar, Dynamic, Dynamic, RowStorage>& left, const Matrix<Scalar, Dynamic, Dynamic, ColumnStorage>& right)
	{
		ASSERT(left.cols() == right.rows(), "Trying to multiply two matrices that are not compatible together");

		// La multiplication ligne x colonne est la plus simple: la façon dont l'information est stockée nous permet de faire une multiplication "normale"
		// On itère sur k avant de changer les lignes et les colonnes
		Matrix<Scalar, Dynamic, Dynamic> result(left.rows(), right.cols());
		for (auto j = 0; j < right.cols(); ++j)
		{
			for (auto i = 0; i < left.rows(); ++i)
			{
				for (auto k = 0; k < left.cols(); ++k)
				{
					result(i, j) += left(i, k) * right(k, j);
				}
			}
		}

		return result;
	}


	/**
	 * Addition : Matrix + Matrix (générique)
	 */
	template <typename Scalar, int Rows, int Cols, int StorageA, int StorageB>
	Matrix<Scalar, Rows, Cols> operator+(const Matrix<Scalar, Rows, Cols, StorageA>& left, const Matrix<Scalar, Rows, Cols, StorageB>& right)
	{
		ASSERT(left.rows() == right.rows(), "Trying to add two matrices of different height");
		ASSERT(left.cols() == right.cols(), "Trying to add two matrices of different width");

		// Puisqu'on ne peut pas inférer la méthode de stockage de la matrice, on ne peut pas faire d'optimisations et
		// on doit utiliser la méthode générique
		Matrix<Scalar, Rows, Cols, ColumnStorage> result(left.rows(), left.cols());
		for (auto i = 0; i < result.rows(); ++i)
		{
			for (auto j = 0; j < result.cols(); ++j)
			{
				result(i, j) = left(i, j) + right(i, j);
			}
		}

		return result;
	}

	/**
	 * Addition : Matrix(colonne) + Matrix(colonne)
	 *
	 * Spécialisation de l'opérateur d'addition pour le cas où les deux matrices
	 * sont stockées par colonnes.
	 */
	template <typename Scalar>
	Matrix<Scalar, Dynamic, Dynamic> operator+(const Matrix<Scalar, Dynamic, Dynamic, ColumnStorage>& left, const Matrix<Scalar, Dynamic, Dynamic, ColumnStorage>& right)
	{
		ASSERT(left.rows() == right.rows(), "Trying to add two matrices of different height");
		ASSERT(left.cols() == right.cols(), "Trying to add two matrices of different width");

		// Puisque les deux matrices sont par colonne, on itère sur les colonnes
		Matrix<Scalar, Dynamic, Dynamic, ColumnStorage> result(left);
		for (auto j = 0; j < result.cols(); ++j)
		{
			for (auto i = 0; i < result.rows(); ++i)
			{
				result(i, j) += right(i, j);
			}
		}

		return result;
	}

	/**
	 * Addition : Matrix(ligne) + Matrix(ligne)
	 *
	 * Spécialisation de l'opérateur d'addition pour le cas où les deux matrices
	 * sont stockées par lignes.
	 */
	template <typename Scalar>
	Matrix<Scalar, Dynamic, Dynamic, RowStorage> operator+(const Matrix<Scalar, Dynamic, Dynamic, RowStorage>& left, const Matrix<Scalar, Dynamic, Dynamic, RowStorage>& right)
	{
		ASSERT(left.rows() == right.rows(), "Trying to add two matrices of different height");
		ASSERT(left.cols() == right.cols(), "Trying to add two matrices of different width");

		// Puisque les deux matrices sont par ligne, on itère sur les lignes
		Matrix<Scalar, Dynamic, Dynamic, RowStorage> result(left);
		for (auto i = 0; i < result.rows(); ++i)
		{
			for (auto j = 0; j < result.cols(); ++j)
			{
				result(i, j) += right(i, j);
			}
		}

		return result;
	}

	// NOUVEAU
	/**
	 * Addition : Matrix - Matrix (générique)
	 */
	template <typename Scalar, int Rows, int Cols, int StorageA, int StorageB>
	Matrix<Scalar, Rows, Cols> operator-(const Matrix<Scalar, Rows, Cols, StorageA>& left, const Matrix<Scalar, Rows, Cols, StorageB>& right)
	{
		ASSERT(left.rows() == right.rows(), "Trying to substract two matrices of different height");
		ASSERT(left.cols() == right.cols(), "Trying to substract two matrices of different width");

		// Puisqu'on ne peut pas inférer la méthode de stockage de la matrice, on ne peut pas faire d'optimisations et
		// on doit utiliser la méthode générique
		Matrix<Scalar, Rows, Cols, ColumnStorage> result(left.rows(), left.cols());
		for (auto i = 0; i < result.rows(); ++i)
		{
			for (auto j = 0; j < result.cols(); ++j)
			{
				result(i, j) = left(i, j) - right(i, j);
			}
		}

		return result;
	}

	/**
	 * Addition : Matrix(colonne) - Matrix(colonne)
	 *
	 * Spécialisation de l'opérateur d'addition pour le cas où les deux matrices
	 * sont stockées par colonnes.
	 */
	template <typename Scalar>
	Matrix<Scalar, Dynamic, Dynamic> operator-(const Matrix<Scalar, Dynamic, Dynamic, ColumnStorage>& left, const Matrix<Scalar, Dynamic, Dynamic, ColumnStorage>& right)
	{
		ASSERT(left.rows() == right.rows(), "Trying to substract two matrices of different height");
		ASSERT(left.cols() == right.cols(), "Trying to substract two matrices of different width");

		// Puisque les deux matrices sont par colonne, on itère sur les colonnes
		Matrix<Scalar, Dynamic, Dynamic, ColumnStorage> result(left);
		for (auto j = 0; j < result.cols(); ++j)
		{
			for (auto i = 0; i < result.rows(); ++i)
			{
				result(i, j) -= right(i, j);
			}
		}

		return result;
	}

	/**
	 * Addition : Matrix(ligne) - Matrix(ligne)
	 *
	 * Spécialisation de l'opérateur d'addition pour le cas où les deux matrices
	 * sont stockées par lignes.
	 */
	template <typename Scalar>
	Matrix<Scalar, Dynamic, Dynamic, RowStorage> operator-(const Matrix<Scalar, Dynamic, Dynamic, RowStorage>& left, const Matrix<Scalar, Dynamic, Dynamic, RowStorage>& right)
	{
		ASSERT(left.rows() == right.rows(), "Trying to substract two matrices of different height");
		ASSERT(left.cols() == right.cols(), "Trying to substract two matrices of different width");

		// Puisque les deux matrices sont par ligne, on itère sur les lignes
		Matrix<Scalar, Dynamic, Dynamic, RowStorage> result(left);
		for (auto i = 0; i < result.rows(); ++i)
		{
			for (auto j = 0; j < result.cols(); ++j)
			{
				result(i, j) -= right(i, j);
			}
		}

		return result;
	}
	// END_NOUVEAU

	/**
	 * Multiplication  : Scalaire * Matrix(colonne)
	 *
	 * Spécialisation de l'opérateur de multiplication par un scalaire pour le
	 * cas d'une matrice stockée par colonnes.
	 */
	template <typename Scalar, int Rows, int Cols>
	Matrix<Scalar, Rows, Cols, ColumnStorage> operator*(const Scalar& scalar, const Matrix<Scalar, Rows, Cols, ColumnStorage>& matrix)
	{
		// On itère la matrice par colonne puisque c'est de cette façon qu'elle est stockée
		Matrix<Scalar, Rows, Cols, ColumnStorage> result(matrix);
		for (auto j = 0; j < matrix.cols(); ++j)
		{
			for (auto i = 0; i < matrix.rows(); ++i)
			{
				result(i, j) *= scalar;
			}
		}

		return result;
	}

	/**
	 * Multiplication  :  Matrix(colonne) * Scalaire
	 *
	 * Spécialisation de l'opérateur de multiplication par un scalaire pour le
	 * cas d'une matrice stockée par colonnes.
	 */
	template <typename Scalar, int Rows, int Cols>
	Matrix<Scalar, Rows, Cols, ColumnStorage> operator*(const Matrix<Scalar, Rows, Cols, ColumnStorage>& matrix, const Scalar& scalar)
	{
		return scalar * matrix;
	}

	/**
	 * Multiplication  : Scalaire * Matrix(ligne)
	 *
	 * Spécialisation de l'opérateur de multiplication par un scalaire pour le
	 * cas d'une matrice stockée par lignes.
	 */
	template <typename Scalar, int Rows, int Cols>
	Matrix<Scalar, Rows, Cols, RowStorage> operator*(const Scalar& scalar, const Matrix<Scalar, Rows, Cols, RowStorage>& matrix)
	{
		// On itère la matrice par lignes puisque c'est de cette façon qu'elle est stockée
		Matrix<Scalar, Rows, Cols, RowStorage> result(matrix);
		for (auto i = 0; i < matrix.rows(); ++i)
		{
			for (auto j = 0; j < matrix.cols(); ++j)
			{
				result(i, j) *= scalar;
			}
		}

		return result;
	}

	/**
	 * Multiplication  :  Matrix(ligne) * Scalaire
	 *
	 * Spécialisation de l'opérateur de multiplication par un scalaire pour le
	 * cas d'une matrice stockée par lignes.
	 */
	template <typename Scalar, int Rows, int Cols>
	Matrix<Scalar, Rows, Cols, RowStorage> operator*(const Matrix<Scalar, Rows, Cols, RowStorage>& matrix, const Scalar& scalar)
	{
		return scalar * matrix;
	}

	/*
	 *	Mupltiplication d'une matrice par -1
	 */
	template <typename Scalar, int Rows, int Cols, int StorageType>
	Matrix<Scalar, Rows, Cols, StorageType> operator-(const Matrix<Scalar, Rows, Cols, StorageType>& matrix)
	{
		return static_cast<Scalar>(-1) * matrix;
	}

	/**
	 * Multiplication : Matrice(ligne) * Vecteur
	 *
	 * Spécialisation de l'opérateur de multiplication matrice*vecteur pour le
	 * cas où la matrice est représentée par lignes.
	 */
	template <typename Scalar, int Rows, int Cols>
	Vector<Scalar, Rows> operator*(const Matrix<Scalar, Rows, Cols, RowStorage>& matrix, const Vector<Scalar, Cols>& vector)
	{
		ASSERT(vector.size() == matrix.cols(), "Trying to multiply a vector with a matrix of invalid size");

		// On itère la matrice par lignes puisque c'est de cette façon qu'elle est stockée
		Vector<Scalar, Rows> result(matrix.rows());
		for (auto i = 0; i < matrix.rows(); ++i)
		{
			for (auto j = 0; j < matrix.cols(); ++j)
			{
				result(i) += vector(j) * matrix(i, j);
			}
		}

		return result;
	}

	/**
	 * Multiplication : Matrice(colonne) * Vecteur
	 *
	 * Spécialisation de l'opérateur de multiplication matrice*vecteur pour le
	 * cas où la matrice est représentée par colonnes.
	 */
	template <typename Scalar, int Rows, int Cols>
	Vector<Scalar, Rows> operator*(const Matrix<Scalar, Rows, Cols, ColumnStorage>& matrix, const Vector<Scalar, Cols>& vector)
	{
		ASSERT(vector.size() == matrix.cols(), "Trying to multiply a vector with a matrix of invalid size");

		// On itère la matrice par colonne puisque c'est de cette façon qu'elle est stockée
		Vector<Scalar, Rows> result(matrix.rows());
		for (auto j = 0; j < matrix.cols(); ++j)
		{
			for (auto i = 0; i < matrix.rows(); ++i)
			{
				result(i) += vector(j) * matrix(i, j);
			}
		}

		return result;
	}

	/**
	 * Multiplication : Scalaire * Vecteur
	 */
	template <typename Scalar, int Rows>
	Vector<Scalar, Rows> operator*(const Scalar& scalar, const Vector<Scalar, Rows>& vector)
	{
		Vector<Scalar, Rows> result(vector);
		for (auto i = 0; i < vector.size(); ++i)
		{
			result(i) *= scalar;
		}
		return result;
	}

	/**
	 * Multiplication : Vecteur * Scalaire 
	 */
	template <typename Scalar, int Rows>
	Vector<Scalar, Rows> operator*(const Vector<Scalar, Rows>& vector, const Scalar& scalar)
	{
		return scalar * vector;
	}


	/**
	 * Addition : Vecteur + Vecteur
	 */
	template <typename Scalar, int RowsA, int RowsB>
	Vector<Scalar, RowsA> operator+(const Vector<Scalar, RowsA>& left, const Vector<Scalar, RowsB>& right)
	{
		auto minSize = std::min(left.size(), right.size());

		Vector<Scalar, RowsA> result(left);
		for (auto i = 0; i < minSize; ++i)
		{
			result(i) += right(i);
		}
		return result;
	}

	/**
	 * Soustraction : Vecteur - Vecteur
	 */
	template <typename Scalar, int RowsA, int RowsB>
	Vector<Scalar, RowsA> operator-(const Vector<Scalar, RowsA>& left, const Vector<Scalar, RowsB>& right)
	{
		auto minSize = std::min(left.size(), right.size());

		Vector<Scalar, RowsA> result(left);
		for (auto i = 0; i < minSize; ++i)
		{
			result(i) -= right(i);
		}
		return result;
	}

	/*
	 *	Mupltiplication d'un vecteur
	 */
	template <typename Scalar, int Rows>
	Vector<Scalar, Rows> operator-(const Vector<Scalar, Rows>& vector)
	{
		return static_cast<Scalar>(-1) * vector;
	}
}
