#pragma once

/**
 * @file Matrix.h
 *
 * @brief Impl�mentation de matrices simples.
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include <ostream>
#include <algorithm>

#include "MatrixBase.h"
#include "GTIAssert.h"
#include "Vector.h"

namespace gti320
{
	enum StorageType
	{
		ColumnStorage = 0,
		RowStorage = 1
	};

	template <typename Scalar, int RowsAtCompile, int ColsAtCompile, int StorageType>
	class SubMatrix;

	template <typename Scalar, int RowsAtCompile, int ColsAtCompile, int StorageType>
	class Matrix;

	template<typename Scalar, int Rows, int Cols, int StorageType>
	void fillMatrixFromInitializerList(Matrix<Scalar, Rows, Cols, StorageType>& matrix, const std::initializer_list<std::initializer_list<Scalar>>& initializerList);

	// NOUVEAU
	template <typename Scalar = double, int RowsAtCompile = Dynamic, int ColsAtCompile = Dynamic>
	class GenericMatrix : public MatrixBase<Scalar, RowsAtCompile, ColsAtCompile>
	{
	public:
		/**
		 * Constructeur par défaut
		 */
		GenericMatrix() : MatrixBase<Scalar, RowsAtCompile, ColsAtCompile>()
		{
		}

		/**
		 * Constructeur de copie
		 */
		GenericMatrix(const GenericMatrix& other) : MatrixBase<Scalar, RowsAtCompile, ColsAtCompile>(other)
		{
		}

		/**
		 * Constructeur avec specification du nombre de ligne et de colonnes
		 */
		explicit GenericMatrix(int rows, int cols) : MatrixBase<Scalar, RowsAtCompile, ColsAtCompile>(rows, cols)
		{
		}

		/**
		 * Destructeur
		 */
		virtual ~GenericMatrix()
		{
		}

		/**
		 * Opérateur de copie a partir d'une sous-matrice.
		 */
		template <typename OtherScalar, int OtherRows, int OtherCols, int OtherStorage = ColumnStorage>
		GenericMatrix& operator=(const SubMatrix<OtherScalar, OtherRows, OtherCols, OtherStorage> & submatrix)
		{
			ASSERT(RowsAtCompile == Dynamic || submatrix.rows() == this->rows(), "Trying to assign a matrix with a submatrix with inconsistent rows");
			ASSERT(ColsAtCompile == Dynamic || submatrix.cols() == this->cols(), "Trying to assign a matrix with a submatrix with inconsistent cols");

			this->resize(submatrix.rows(), submatrix.cols());

			// On parcours dans la direction des colonnes puisque la sous matrice à copier devrat être une sous matrice par colonnes
			for (auto j = 0; j < submatrix.cols(); ++j)
			{
				for (auto i = 0; i < submatrix.rows(); ++i)
				{
					(*this)(i, j) = submatrix(i, j);
				}
			}

			return *this;
		}

		/**
		 * Opérateur de copie a partir d'une sous-matrice.
		 */
		template <typename OtherScalar, int OtherRows, int OtherCols>
		GenericMatrix& operator=(const SubMatrix<OtherScalar, OtherRows, OtherCols, RowStorage>& submatrix)
		{
			ASSERT(RowsAtCompile == Dynamic || submatrix.rows() == this->rows(), "Trying to assign a matrix with a submatrix with inconsistent rows");
			ASSERT(ColsAtCompile == Dynamic || submatrix.cols() == this->cols(), "Trying to assign a matrix with a submatrix with inconsistent cols");

			this->resize(submatrix.rows(), submatrix.cols());

			// On parcours dans la direction des lignes puisque la sous matrice à copier est une sous matrice par lignes
			for (auto i = 0; i < submatrix.rows(); ++i)
			{
				for (auto j = 0; j < submatrix.cols(); ++j)
				{
					(*this)(i, j) = submatrix(i, j);
				}
			}

			return *this;
		}

		/**
		 * Accesseur a une entrée de la matrice (lecture seule)
		 */
		virtual Scalar operator()(int i, int j) const = 0;

		/**
		 * Accesseur a une entrée de la matrice (lecture ou écriture)
		 */
		virtual Scalar& operator()(int i, int j) = 0;

		// Ami avec l'opérateur `<<` pour pouvoir être en mesure d'afficher la matrice à la console
		template<typename OtherScalar, int OtherRows, int OtherCols>
		friend std::ostream& operator<< (std::ostream& out, const GenericMatrix<OtherScalar, OtherRows, OtherCols>& matrix);
	};
	// END_NOUVEAU

	/**
	 * Classe Matrix spécialisée pour le cas d'un stockage par colonnes.
	 *
	 * (le cas d'un stockage par ligne fait l'objet d'une spécialisation de
	 * patron, voir plus bas)
	 */
	template <typename Scalar = double, int RowsAtCompile = Dynamic, int ColsAtCompile = Dynamic, int StorageType = ColumnStorage>
	class Matrix : public GenericMatrix<Scalar, RowsAtCompile, ColsAtCompile>
	{
	public:
		/**
		 * Constructeur par défaut
		 */
		Matrix() : GenericMatrix<Scalar, RowsAtCompile, ColsAtCompile>()
		{
		}

		/**
		 * Constructeur de copie
		 */
		Matrix(const Matrix& other) : GenericMatrix<Scalar, RowsAtCompile, ColsAtCompile>(other)
		{
		}

		/**
		 * Constructeur avec specification du nombre de ligne et de colonnes
		 */
		explicit Matrix(int rows, int cols) : GenericMatrix<Scalar, RowsAtCompile, ColsAtCompile>(rows, cols)
		{
		}

		/**
		 * Constructeur avec une liste d'initialistation
		 *
		 * Ce constructeur est plus lent que les autres et ne devraient être utilisé qu'en cas de nécessité ou de test.
		 */
		Matrix(const std::initializer_list<std::initializer_list<Scalar>>& initializerList) : GenericMatrix<Scalar, RowsAtCompile, ColsAtCompile>()
		{
			fillMatrixFromInitializerList(*this, initializerList);
		}

		/**
		 * Destructeur
		 */
		~Matrix()
		{
		}

		/**
		 * Opérateur de copie a partir d'une sous-matrice.
		 */
		template <typename OtherScalar, int OtherRows, int OtherCols, int OtherStorage>
		Matrix & operator=(const SubMatrix<OtherScalar, OtherRows, OtherCols, OtherStorage> & submatrix)
		{
			GenericMatrix<Scalar, RowsAtCompile, ColsAtCompile>::operator=(submatrix);
			return  *this;
		}

		/**
		 * Accesseur a une entrée de la matrice (lecture seule)
		 */
		Scalar operator()(int i, int j) const override
		{
			ASSERTF(i >= 0 && i < this->rows(), "Trying to access out of matrix range (i = %d; rows = %d)", i, this->rows());
			ASSERTF(j >= 0 && j < this->cols(), "Trying to access out of matrix range (j = %d; rows = %d)", j, this->cols());

			return this->m_storage[j * this->rows() + i];
		}

		/**
		 * Accesseur a une entrée de la matrice (lecture ou écriture)
		 */
		Scalar& operator()(int i, int j) override
		{
			ASSERTF(i >= 0 && i < this->rows(), "Trying to access out of matrix range (i = %d; rows = %d)", i, this->rows());
			ASSERTF(j >= 0 && j < this->cols(), "Trying to access out of matrix range (j = %d; rows = %d)", j, this->cols());

			return this->m_storage[j * this->rows() + i];
		}

		/**
		 * Crée une sous-matrice pour un block de taille (rows, cols) à partir
		 * de l'indice (i,j).
		 */
		SubMatrix<Scalar, RowsAtCompile, ColsAtCompile, StorageType> block(int i, int j, int rows, int cols) const
		{
			return SubMatrix<Scalar, RowsAtCompile, ColsAtCompile, StorageType>(*this, i, j, rows, cols);
		}

		// NOUVEAU
		/*
		 * Retourne une ligne de la matrice sous la forme d'une sous-matrice
		 */
		SubMatrix<Scalar, RowsAtCompile, ColsAtCompile, StorageType> row(int rowIndex) const
		{
			ASSERTF(rowIndex >= 0 && rowIndex < this->rows(), "Trying to get a row out of bound (rowIndex = %d; rows = %d)", rowIndex, this->rows());
			return SubMatrix<Scalar, RowsAtCompile, ColsAtCompile, StorageType>(*this, rowIndex, 0, 1, this->cols());
		}

		/*
		 * Retourne une colonne de la matrice sous la forme d'une sous-matrice
		 */
		SubMatrix<Scalar, RowsAtCompile, ColsAtCompile, StorageType> col(int colIndex) const
		{
			ASSERTF(colIndex >= 0 && colIndex < this->cols(), "Trying to get a row out of bound (colIndex = %d; colIndex = %d)", colIndex, this->rows());
			return SubMatrix<Scalar, RowsAtCompile, ColsAtCompile, StorageType>(*this, 0, colIndex, this->rows(), 1);
		}
		// END_NOUVEAU

		// NOUVEAU
		template<int NewColsAtCompile, int NewRowsAtCompile>
		Matrix<Scalar, NewColsAtCompile, NewRowsAtCompile, StorageType> convert() const
		{
			ASSERT(NewColsAtCompile == Dynamic || NewColsAtCompile == this->cols(), "Trying to convert a matrix with column mismatch");
			ASSERT(NewRowsAtCompile == Dynamic || NewRowsAtCompile == this->rows(), "Trying to convert a matrix with row mismatch");

			Matrix<Scalar, NewColsAtCompile, NewRowsAtCompile, StorageType> convertedMatrix(this->rows(), this->cols());
			for (auto j = 0; j < convertedMatrix.cols(); ++j)
			{
				for (auto i = 0; i < convertedMatrix.rows(); ++i)
				{
					convertedMatrix(i, j) = (*this)(i, j);
				}
			}

			return convertedMatrix;
		}
		// END_NOUVEAU

		// NOUVEAU
		/*
		 * Construit une matrice ou chaque colonne représente la colonne moyenne de toute la matrice
		 */
		Matrix<Scalar, RowsAtCompile, ColsAtCompile, StorageType> meanMatrix() const
		{
			Vector<Scalar, RowsAtCompile> meanVector(this->rows());
			for (auto j = 0; j < this->cols(); ++j)
			{
				for (auto i = 0; i < this->rows(); ++i)
				{
					meanVector(i) += (*this)(i, j);
				}
			}

			for (auto i = 0; i < meanVector.size(); ++i)
			{
				meanVector(i) = meanVector(i) / this->cols();
			}

			Matrix<Scalar, RowsAtCompile, ColsAtCompile, StorageType> meanMatrix(this->rows(), this->cols());
			for (auto j = 0; j < this->cols(); ++j)
			{
				for (auto i = 0; i < this->rows(); ++i)
				{
					meanMatrix(i, j) += meanVector(i);
				}
			}

			return meanMatrix;
		}
		// END_NOUVEAU

		/**
		 * Calcule l'inverse de la matrice
		 */
		Matrix inverse() const
		{
			// Do nothing.
			return *this;
		}

		/**
		 * Retourne la transposée de la matrice
		 */
		template <typename OtherScalar, int OtherRows, int OtherCols, int OtherStorage>
		Matrix<OtherScalar, OtherRows, OtherCols, OtherStorage> transpose() const
		{
			Matrix<OtherScalar, OtherRows, OtherCols, OtherStorage> transposedMatrix(this->cols(), this->rows());

			// On parcours dans la direction des colonnes puisque la matrice devrait être une matrice par colonnes
			for (auto j = 0; j < this->cols(); ++j)
			{
				for (auto i = 0; i < this->rows(); ++i)
				{
					transposedMatrix(j, i) = (*this)(i, j);
				}
			}

			return transposedMatrix;
		}

		/**
		 * Retourne la transposée de la matrice pour une matrice de type de stockage inverse
		 */
		Matrix<Scalar, ColsAtCompile, RowsAtCompile, RowStorage> transpose() const
		{
			// On ne veut pas que notre optimisation casse la méthode `transpose` si on ajoute un autre type de matrice
			if (StorageType != ColumnStorage)
			{
				return this->transpose<Scalar, ColsAtCompile, RowsAtCompile, RowStorage>();
			}

			Matrix<Scalar, ColsAtCompile, RowsAtCompile, RowStorage> transposedMatrix(this->cols(), this->rows());
			std::memcpy(transposedMatrix.m_storage.data(), this->m_storage.data(), sizeof(Scalar) * this->size());

			return transposedMatrix;
		}

		/**
		 * Assigne l'identité a la matrice
		 */
		void setIdentity()
		{
			this->setZero();

			int diagonalLength = std::min(this->rows(), this->cols());
			for (auto i = 0; i < diagonalLength; ++i)
			{
				(*this)(i, i) = 1;
			}
		}

		// Ami avec sa transposée pour pouvoir créer une transposée rapidement à partir des données directement
		friend Matrix<Scalar, ColsAtCompile, RowsAtCompile, RowStorage>;
	};

	/**
	 * Classe Matrix specialisée pour un stockage par lignes
	 */
	template <typename Scalar, int RowsAtCompile, int ColsAtCompile>
	class Matrix<Scalar, RowsAtCompile, ColsAtCompile, RowStorage> : public GenericMatrix<Scalar, RowsAtCompile, ColsAtCompile>
	{
	public:
		/**
		 * Constructeur par défaut
		 */
		Matrix() : GenericMatrix<Scalar, RowsAtCompile, ColsAtCompile>()
		{
		}

		/**
		 * Constructeur de copie
		 */
		Matrix(const Matrix& other) : GenericMatrix<Scalar, RowsAtCompile, ColsAtCompile>(other)
		{
		}

		/**
		 * Constructeur avec specification du nombre de ligne et de colonnes
		 */
		explicit Matrix(int rows, int cols) : GenericMatrix<Scalar, RowsAtCompile, ColsAtCompile>(rows, cols)
		{
		}

		/**
		 * Constructeur avec une liste d'initialistation
		 *
		 * Ce constructeur est plus lent que les autres et ne devraient être utilisé qu'en cas de nécessité ou de test.
		 */
		Matrix(const std::initializer_list<std::initializer_list<Scalar>>& initializerList) : GenericMatrix<Scalar, RowsAtCompile, ColsAtCompile>()
		{
			fillMatrixFromInitializerList(*this, initializerList);
		}

		/**
		 * Destructeur
		 */
		~Matrix()
		{
		}

		/**
		 * Opérateur de copie a partir d'une sous-matrice.
		 */
		template <typename OtherScalar, int OtherRows, int OtherCols, int OtherStorage>
		Matrix& operator=(const SubMatrix<OtherScalar, OtherRows, OtherCols, OtherStorage>& submatrix)
		{
			GenericMatrix<Scalar, RowsAtCompile, ColsAtCompile>::operator=(submatrix);
			return *this;
		}

		/**
		 * Accesseur a une entrée de la matrice (lecture seule)
		 */
		Scalar operator()(int i, int j) const override
		{
			ASSERTF(i >= 0 && i < this->rows(), "Trying to access out of matrix range (i = %d; rows = %d)", i, this->rows());
			ASSERTF(j >= 0 && j < this->cols(), "Trying to access out of matrix range (j = %d; rows = %d)", j, this->cols());

			return this->m_storage[i * this->cols() + j];
		}

		/**
		 * Accesseur a une entrée de la matrice (lecture ou écriture)
		 */
		Scalar& operator()(int i, int j) override	
		{
			ASSERTF(i >= 0 && i < this->rows(), "Trying to access out of matrix range (i = %d; rows = %d)", i, this->rows());
			ASSERTF(j >= 0 && j < this->cols(), "Trying to access out of matrix range (j = %d; rows = %d)", j, this->cols());

			return this->m_storage[i * this->cols() + j];
		}

		/**
		 * Crée une sous-matrice pour un block de taille (rows, cols) a partir
		 * de l'index (i,j).
		 */
		SubMatrix<Scalar, RowsAtCompile, ColsAtCompile, RowStorage> block(int i, int j, int rows, int cols) const
		{
			return SubMatrix<Scalar, RowsAtCompile, ColsAtCompile, RowStorage>(*this, i, j, rows, cols);
		}

		// NOUVEAU
		/*
		 * Retourne une ligne de la matrice sous la forme d'une sous-matrice
		 */
		SubMatrix<Scalar, RowsAtCompile, ColsAtCompile, RowStorage> row(int rowIndex) const
		{
			ASSERTF(rowIndex >= 0 && rowIndex < this->rows(), "Trying to get a row out of bound (rowIndex = %d; rows = %d)", rowIndex, this->rows());
			return SubMatrix<Scalar, RowsAtCompile, ColsAtCompile, RowStorage>(*this, rowIndex, 0, 1, this->cols());
		}

		/*
		 * Retourne une colonne de la matrice sous la forme d'une sous-matrice
		 */
		SubMatrix<Scalar, RowsAtCompile, ColsAtCompile, RowStorage> col(int colIndex) const
		{
			ASSERTF(colIndex >= 0 && colIndex < this->rows(), "Trying to get a row out of bound (colIndex = %d; colIndex = %d)", colIndex, this->rows());
			return SubMatrix<Scalar, RowsAtCompile, ColsAtCompile, RowStorage>(*this, 0, colIndex, this->rows(), 1);
		}
		// END_NOUVEAU

		// NOUVEAU
		/*
		 * Construit une matrice ou chaque colonne représente la colonne moyenne de toute la matrice
		 */
		Matrix<Scalar, RowsAtCompile, ColsAtCompile, RowStorage> meanMatrix() const
		{
			Vector<Scalar, RowsAtCompile> meanVector(this->rows());
			for (auto i = 0; i < this->rows(); ++i)
			{
				for (auto j = 0; j < this->cols(); ++j)
				{
					meanVector(i) += (*this)(i, j);
				}
			}

			for (auto i = 0; i < meanVector.size(); ++i)
			{
				meanVector(i) = meanVector(i) / this->cols();
			}

			Matrix<Scalar, RowsAtCompile, ColsAtCompile, RowStorage> meanMatrix(this->rows(), this->cols());
			for (auto i = 0; i < this->rows(); ++i)
			{
				for (auto j = 0; j < this->cols(); ++j)
				{
					meanMatrix(i, j) += meanVector(i);
				}
			}

			return meanMatrix;
		}
		// END_NOUVEAU

		// NOUVEAU
		template<int NewColsAtCompile, int NewRowsAtCompile>
		Matrix<Scalar, NewColsAtCompile, NewRowsAtCompile, RowStorage> convert() const
		{
			ASSERT(NewColsAtCompile == Dynamic || NewColsAtCompile == this->cols(), "Trying to convert a matrix with column mismatch");
			ASSERT(NewRowsAtCompile == Dynamic || NewRowsAtCompile == this->rows(), "Trying to convert a matrix with row mismatch");

			Matrix<Scalar, NewColsAtCompile, NewRowsAtCompile, RowStorage> convertedMatrix(this->rows(), this->cols());
			for (auto i = 0; i < convertedMatrix.rows(); ++i)
			{
				for (auto j = 0; j < convertedMatrix.cols(); ++j)
				{
					convertedMatrix(i, j) = (*this)(i, j);
				}
			}

			return convertedMatrix;
		}
		// END_NOUVEAU

		/**
		 * Calcule l'inverse de la matrice
		 */
		Matrix inverse() const
		{
			// Do nothing.
			return *this;
		}

		/**
		 * Retourne la transposée de la matrice pour une matrice de type de stockage inverse
		 */
		Matrix<Scalar, ColsAtCompile, RowsAtCompile, ColumnStorage> transpose() const
		{
			Matrix<Scalar, ColsAtCompile, RowsAtCompile, ColumnStorage> transposedMatrix(this->cols(), this->rows());
			std::memcpy(transposedMatrix.m_storage.data(), this->m_storage.data(), sizeof(Scalar) * this->size());

			return transposedMatrix;
		}

		/**
		 * Retourne la transposée de la matrice
		 */
		template <typename OtherScalar, int OtherRows, int OtherCols, int OtherStorage>
		Matrix<OtherScalar, OtherRows, OtherCols, OtherStorage> transpose() const
		{
			Matrix<OtherScalar, OtherRows, OtherCols, OtherStorage> transposedMatrix(this->cols(), this->rows());

			// On parcours dans la direction des lignes puisque la matrice est une matrice par lignes
			for (auto i = 0; i < this->rows(); ++i)
			{
				for (auto j = 0; j < this->cols(); ++j)
				{

					transposedMatrix(j, i) = (*this)(i, j);
				}
			}

			return transposedMatrix;
		}

		/**
		 * Affecte l'identité à la matrice
		 */
		void setIdentity()
		{
			this->setZero();

			int diagonalLength = std::min(this->rows(), this->cols());
			for (auto i = 0; i < diagonalLength; ++i)
			{
				(*this)(i, i) = 1;
			}
		}

		// Ami avec sa transposée pour pouvoir créer une transposée rapidement à partir des données directement
		friend Matrix<Scalar, ColsAtCompile, RowsAtCompile, ColumnStorage>;
	};

	/*
	 * Cet opérateur n'est pas optimisé pour les matrices en colonnes puisqu'il est obligatoire d'insérer dans
	 * le flux la matrice ligne par ligne.
	 *
	 * Ce n'est pas un problème, vu que cet opérateur n'est qu'utile pour afficher une matrice. Les matrices trop
	 * grosses où l'ordre du stockage a une importance pour la performance ne devraient donc pas utiliser
	 * cet opérateur, puisqu'il ne nous est pas utile de les afficher.
	 */
	template<typename Scalar, int Rows, int Cols>
	std::ostream& operator<< (std::ostream& out, const GenericMatrix<Scalar, Rows, Cols>& matrix)
	{
		for (int i = 0; i < matrix.rows(); ++i)
		{
			out << "|";
			for (int j = 0; j < matrix.cols(); ++j)
			{
				out << matrix(i, j);

				if (j != matrix.cols() - 1)
				{
					out << ", ";
				}
			}
			out << "|" << std::endl;
		}

		return out;
	}

	/*
	 * Cette fonction utilise une liste d'initialisation afin de créer une matrice.
	 *
	 * Elle a été déplacée hors des classes de matrice afin de permettre les deux spécialisations d'en bénéficier.
	 *
	 * Cette fonction n'est pas optimisée pour les patrices en colonnes. Ce n'est pas un problème, puisque l'initialisation avec une
	 * liste d'initialisation ne devrait se faire en majorité qu'en cas de test ou pour des petites matrices.
	 */
	template<typename Scalar, int Rows, int Cols, int StorageType>
	void fillMatrixFromInitializerList(Matrix<Scalar, Rows, Cols, StorageType>& matrix, const std::initializer_list<std::initializer_list<Scalar>>& initializerList)
	{
		int rows = initializerList.size();
		int cols = 0;

		for (auto row = initializerList.begin(); row != initializerList.end(); ++row)
		{
			if (row == initializerList.begin())
			{
				cols = row->size();
			}

			ASSERT(row->size() == cols, "Attempting to create a Matrix with two different number of cols.");
		}

#ifdef _DEBUG
		if (Rows != Dynamic)
		{
			ASSERT(rows == Rows, "Attempting to create a Matrix with two different number of rows.");
		}
		if (Cols != Dynamic)
		{
			ASSERT(cols == Cols, "Attempting to create a Matrix with two different number of cols.");
		}
#endif

		matrix.resize(rows, cols);

		auto i = 0;
		for (auto row = initializerList.begin(); row != initializerList.end(); ++row)
		{
			auto j = 0;
			for (auto col = row->begin(); col != row->end(); ++col)
			{
				matrix(i, j) = *col;
				j++;
			}

			++i;
		}
	}

	/**
	 * Classe pour accéder à une sous-matrice.
	 *
	 * Un sous-matrice ne copie pas les données. Au lieu de cela, elle conserve une
	 * référence a la matrice originale.
	 */
	template <typename Scalar, int RowsAtCompile, int ColsAtCompile, int StorageType = ColumnStorage>
	class SubMatrix
	{
	private:
		// Référence a la matrice originale
		Matrix<Scalar, RowsAtCompile, ColsAtCompile, StorageType>& m_matrix;

		// Constructeur par défaut (privé)
		SubMatrix()
		{
		}

		// (i,j) est le coin supérieur gauche de la sous-matrice
		int m_i; // Décalage en ligne 
		int m_j; // Décalage en colonne

		// la sous-matrice est de dimension : m_rows x m_cols
		int m_rows; // Height of the sub matrix (m_rows)
		int m_cols; // Width of the sub matrix (columns)

	public:
		/**
		 * Constructeur a partir d'une référence en lecture seule à une matrice.
		 */
		SubMatrix(const Matrix<Scalar, RowsAtCompile, ColsAtCompile, StorageType>& matrix, int i, int j, int rows, int cols) :
			m_matrix(const_cast<Matrix<Scalar, RowsAtCompile, ColsAtCompile, StorageType>&>(matrix)),
			m_i(i), m_j(j), m_rows(rows), m_cols(cols)
		{
			ASSERTF(rows > 0, "Trying to create a submatrix with invalid number of rows");
			ASSERTF(cols > 0, "Trying to create a submatrix with invalid number of cols");
			ASSERTF(i >= 0 && i + rows <= matrix.rows(), "Trying to create out of bound submatrix. Matrix height: %d (max index: %d). Indexes of sub matrix rows: %d - %d", matrix.rows(), matrix.rows() - 1, i, i + rows - 1);
			ASSERTF(j >= 0 && j + cols <= matrix.cols(), "Trying to create out of bound submatrix. Matrix width: %d (max index: %d). Indexes of sub matrix cols: %d - %d", matrix.cols(), matrix.cols() - 1, j, j + cols - 1);
		}

		/**
		 * Constructeur a partir d'une référence en lecture et écriture à une matrice.
		 */
		explicit SubMatrix(Matrix<Scalar, RowsAtCompile, ColsAtCompile, StorageType>& matrix, int i, int j, int rows, int cols) :
			m_matrix(matrix),
			m_i(i), m_j(j), m_rows(rows), m_cols(cols)
		{
			ASSERTF(rows > 0, "Trying to create a submatrix with invalid number of rows");
			ASSERTF(cols > 0, "Trying to create a submatrix with invalid number of cols");
			ASSERTF(i >= 0 && i + rows <= matrix.rows(), "Trying to create out of bound submatrix. Matrix height: %d (max index: %d). Indexes of sub matrix rows: %d - %d", matrix.rows(), matrix.rows() - 1, i, i + rows - 1);
			ASSERTF(j >= 0 && j + cols <= matrix.cols(), "Trying to create out of bound submatrix. Matrix width: %d (max index: %d). Indexes of sub matrix cols: %d - %d", matrix.cols(), matrix.cols() - 1, j, j + cols - 1);
		}

		/**
		 * Constructeur de copie
		 */
		SubMatrix(const SubMatrix& other) :
			m_matrix(other.m_matrix),
			m_i(other.m_i), m_j(other.m_j), m_rows(other.m_rows), m_cols(other.m_cols)
		{
		}

		/**
		 * Destructeur
		 */
		~SubMatrix()
		{
		}

		/**
		 * Opérateur de copie (à partir d'une matrice)
		 *
		 * Copies toutes les entrees de la matrice dans la sous-matrice.
		 */
		template <typename OtherScalar, int OtherRows, int OtherCols, int OtherStorage = ColumnStorage>
		SubMatrix & operator=(const Matrix<OtherScalar, OtherRows, OtherCols, OtherStorage> & matrix)
		{
			ASSERT(m_cols == matrix.cols(), "Trying to copy a matrix to a submatrix of different width");
			ASSERT(m_rows == matrix.rows(), "Trying to copy a matrix to a submatrix of different height");

			// On parcours les matrices en colonne puisqu'on sait que l'autre matrice devrait être en colonnes
			for (auto j = 0; j < matrix.cols(); ++j)
			{
				for (auto i = 0; i < matrix.rows(); ++i)
				{
					(*this)(i, j) = matrix(i, j);
				}
			}

			return *this;
		}

		/**
		 * Opérateur de copie (à partir d'une matrice)
		 *
		 * Copies toutes les entrees de la matrice dans la sous-matrice.
		 *
		 * Note : la taille de la matrice doit correspondre a la taille de la
		 * sous-matrice.
		 */
		template <typename OtherScalar, int OtherRows, int OtherCols>
		SubMatrix& operator=(const Matrix<OtherScalar, OtherRows, OtherCols, RowStorage>& matrix)
		{
			ASSERT(m_cols == matrix.cols(), "Trying to copy a matrix to a submatrix of different width");
			ASSERT(m_rows == matrix.rows(), "Trying to copy a matrix to a submatrix of different height");

			// On parcours les matrices en lignes puisqu'on sait que l'autre matrice est en lignes
			for (auto i = 0; i < matrix.rows(); ++i)
			{
				for (auto j = 0; j < matrix.cols(); ++j)
				{
					(*this)(i, j) = matrix(i, j);
				}
			}

			return *this;
		}

		/**
		 * Accesseur aux entrées de la sous-matrice (lecture seule)
		 */
		Scalar operator()(int i, int j) const
		{
			ASSERT(i >= 0 && i < m_rows, "Index out of bound while trying to access submatrix");
			ASSERT(j >= 0 && j < m_cols, "Index out of bound while trying to access submatrix");

			return m_matrix(m_i + i, m_j + j);
		}

		/**
		 * Accesseur aux entrées de la sous-matrice (lecture et écriture)
		 */
		Scalar& operator()(int i, int j)
		{
			ASSERT(i >= 0 && i < m_rows, "Index out of bound while trying to access submatrix");
			ASSERT(j >= 0 && j < m_cols, "Index out of bound while trying to access submatrix");

			return m_matrix(m_i + i, m_j + j);
		}

		/**
		 * Retourne la transposée de la sous-matrice sous la forme d'une matrice.
		 */
		template <typename OtherScalar, int OtherRows, int OtherCols, int OtherStorage>
		Matrix<OtherScalar, OtherRows, OtherCols, OtherStorage> transpose() const
		{
			ASSERT(OtherRows == Dynamic || OtherRows == m_cols, "Trying to transpose a submatrix to a matrix with an invalid number of rows");
			ASSERT(OtherCols == Dynamic || OtherCols == m_rows, "Trying to transpose a submatrix to a matrix with an invalid number of cols");

			Matrix<OtherScalar, OtherRows, OtherCols, OtherStorage> transposedMatrix(m_cols, m_rows);

			// On ne sait pas si le type de la matrice sortante va être par ligne ou par colonne. Cependant, puisque les matrices par colonne
			// sont les matrices par défaut, il est possible d'assumer que cette méthode sera appelée davantage pour les matrices par colonne, et
			// donc on parcours les matrices par colonnes.
			for (auto j = 0; j < transposedMatrix.cols(); ++j)
			{
				for (auto i = 0; i < transposedMatrix.rows(); ++i)
				{
					transposedMatrix(i, j) = (*this)(j, i);
				}
			}

			return transposedMatrix;
		}

		/*
		 * Retourne le nombre de lignes de la sous matrice
		 */
		inline int rows() const { return m_rows; }

		/*
		 * Retourne le nombre de colonnes de la sous matrice
		 */
		inline int cols() const { return m_cols; }
	};
}
