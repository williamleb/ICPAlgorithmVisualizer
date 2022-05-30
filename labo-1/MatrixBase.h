#pragma once

/**
 * @file MatrixBase.h
 *
 * @brief Classe contenant les éléments de base des matrices et des vecteurs.
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include "DenseStorage.h"
#include "GTIAssert.h"

namespace gti320
{
	/**
	 * Classe de base pour les matrices et vecteurs à taille fixe.
	 */
	template <typename Scalar, int Rows, int Cols>
	class MatrixBase
	{
	protected:
		DenseStorage<Scalar, Rows * Cols> m_storage;

	public:
		typedef Scalar Scalar;

		/**
		 * Constructeur par défaut
		 */
		MatrixBase() : m_storage()
		{
		}

		/**
		 * Constructeur de copie
		 */
		MatrixBase(const MatrixBase& other) : m_storage(other.m_storage)
		{
		}

		explicit MatrixBase(int rows, int cols) : m_storage()
		{
			ASSERT(Rows == rows, "Attempting to create a MatrixBase with two different number of rows.");
			ASSERT(Cols == cols, "Attempting to create a MatrixBase with two different number of cols.");
		}

		/**
		 * Destructeur
		 */
		~MatrixBase()
		{
		}


		/**
		 * Redimensionne la matrice
		 */
		void resize(int rows, int cols)
		{
			ASSERT(rows == Rows && cols == Cols, "Attempting to resize a static MatrixBase. Use a dynamic MatrixBase instead.");
		}

		/**
		 * Opérateur de copie
		 */
		MatrixBase& operator=(const MatrixBase& other)
		{
			if (this != &other)
			{
				m_storage = other.m_storage;
			}
			return *this;
		}

		inline void setZero() { m_storage.setZero(); }
		static inline int cols() { return Cols; }
		static inline int rows() { return Rows; }

		/**
		 * Accès à la donnée membre de stockage (en lecture seule)
		 */
		const DenseStorage<Scalar, Rows * Cols>& storage() const
		{
			return m_storage;
		}

		/**
		 * Nombre d'éléments stockés dans le tampon.
		 */
		inline int size() const
		{
			return m_storage.size();
		}

		/**
		 * Accès au tampon de données ((lecture seule))
		 */
		const Scalar* data() const
		{
			return m_storage.data();
		}

		/*
		 * Retourne l'élément du tampon à l'indice `i` en lecture
		 */
		Scalar operator [](int i) const { return m_storage[i]; }
	};

	/**
	 * Classe de base pour les matrices avec un nombre de lignes dynamique et un
	 * nombre fixe de colonnes.
	 */
	template <typename Scalar, int Cols>
	class MatrixBase<Scalar, Dynamic, Cols>
	{
	protected:
		DenseStorage<Scalar, Dynamic> m_storage;
		int m_rows;

	public:
		typedef Scalar Scalar;

		/**
		 * Constructeur par défaut
		 */
		MatrixBase() : m_storage(), m_rows(0)
		{
		}

		/*
		 * Constructeur avec taille spécifiée
		 */
		explicit MatrixBase(int rows, int cols) : m_storage(rows * Cols), m_rows(rows)
		{
			ASSERT(rows >= 0, "Attempting to create a MatrixBase with a negative number of rows.");
			ASSERT(Cols == cols, "Attempting to create a MatrixBase with two different number of cols.");
		}

		/**
		 * Constructeur de copie
		 */
		MatrixBase(const MatrixBase& other) : m_storage(other.m_storage), m_rows(other.m_rows)
		{
		}

		/**
		 * Destructeur
		 */
		~MatrixBase()
		{
		}

		/**
		 * Opérateur de copie
		 */
		MatrixBase& operator=(const MatrixBase& other)
		{
			if (this != &other)
			{
				m_storage = other.m_storage;
				m_rows = other.m_rows;
			}
			return *this;
		}

		/**
		 * Redimensionne la matrice
		 */
		void resize(int rows, int cols)
		{
			ASSERT(Cols == cols, "Attempting to resize the number of cols of a MatrixBase with a static number of cols.");

			m_storage.resize(rows * Cols);
			m_rows = rows;
		}

		inline void setZero() { m_storage.setZero(); }
		static inline int cols() { return Cols; }
		inline int rows() const { return m_rows; }

		/**
		 * Accès à la donnée membre de stockage (en lecture seule)
		 */
		const DenseStorage<Scalar, Dynamic>& storage() const
		{
			return m_storage;
		}

		/**
		 * Nombre d'éléments stockés dans le tampon.
		 */
		inline int size() const
		{
			return m_storage.size();
		}

		/**
		 * Accès au tampon de données ((lecture seule))
		 */
		const Scalar* data() const
		{
			return m_storage.data();
		}

		/*
		 * Retourne l'élément du tampon à l'indice `i` en lecture
		 */
		Scalar operator [](int i) const { return m_storage[i]; }
	};

	/**
	 * Classe de base pour les matrices avec un nombre fixe de lignes et un
	 * nombre de colonnes dynamique.
	 */
	template <typename Scalar, int Rows>
	class MatrixBase<Scalar, Rows, Dynamic>
	{
	protected:
		DenseStorage<Scalar, Dynamic> m_storage;
		int m_cols;

	public:
		typedef Scalar Scalar;

		/**
		 * Constructeur par défaut
		 */
		MatrixBase() : m_storage(), m_cols(0)
		{
		}

		explicit MatrixBase(int rows, int cols) : m_storage(Rows * cols), m_cols(cols)
		{
			ASSERT(Rows == rows, "Attempting to create a MatrixBase with two different number of rows.");
			ASSERT(cols >= 0, "Attempting to create a MatrixBase with a negative number of cols.");
		}

		/**
		 * Constructeur de copie
		 */
		MatrixBase(const MatrixBase& other) : m_storage(other.m_storage), m_cols(other.m_cols)
		{
		}

		/**
		 * Destructeur
		 */
		~MatrixBase()
		{
		}

		/**
		 * Opérateur de copie
		 */
		MatrixBase& operator=(const MatrixBase& other)
		{
			if (this != &other)
			{
				m_storage = other.m_storage;
				m_cols = other.m_cols;
			}

			return *this;
		}

		/**
		 * Redimensionne la matrice
		 */
		void resize(int rows, int cols)
		{
			ASSERT(Rows == rows, "Attempting to resize the number of rows of a MatrixBase with a static number of rows.");
			m_storage.resize(Rows * cols);
			m_cols = cols;
		}

		inline void setZero() { m_storage.setZero(); }
		inline int cols() const { return m_cols; }
		static inline int rows() { return Rows; }

		/**
		 * Accès à la donnée membre de stockage (en lecture seule)
		 */
		const DenseStorage<Scalar, Dynamic>& storage() const
		{
			return m_storage;
		}

		/**
		 * Nombre d'éléments stockés dans le tampon.
		 */
		inline int size() const
		{
			return m_storage.size();
		}

		/**
		 * Accès au tampon de données ((lecture seule))
		 */
		const Scalar* data() const
		{
			return m_storage.data();
		}

		/*
		 * Retourne l'élément du tampon à l'indice `i` en lecture
		 */
		Scalar operator [](int i) const { return m_storage[i]; }
	};

	/**
	 * Classe de base pour les matrices avec un nombre de lignes et de colonnes
	 * dynamiques.
	 */
	template <typename Scalar>
	class MatrixBase<Scalar, Dynamic, Dynamic>
	{
	protected:
		DenseStorage<Scalar, Dynamic> m_storage;
		int m_cols;
		int m_rows;

	public:
		typedef Scalar Scalar;

		/**
		 * Constructeur par défaut
		 */
		MatrixBase() : m_storage(), m_cols(0), m_rows(0)
		{
		}

		explicit MatrixBase(int rows, int cols) : m_storage(rows * cols), m_cols(cols), m_rows(rows)
		{
			ASSERT(rows >= 0, "Attempting to create a MatrixBase with a negative number of rows.");
			ASSERT(cols >= 0, "Attempting to create a MatrixBase with a negative number of cols.");
		}

		/**
		 * Constructeur de copie
		 */
		MatrixBase(const MatrixBase& other) : m_storage(other.m_storage), m_cols(other.m_cols), m_rows(other.m_rows)
		{
		}

		/**
		 * Destructeur
		 */
		~MatrixBase()
		{
		}

		/**
		 * Opérateur de copie
		 */
		MatrixBase& operator=(const MatrixBase& other)
		{
			if (this != &other)
			{
				resize(other.m_rows, other.m_cols);
				m_storage = other.m_storage;
			}
			return *this;
		}

		/**
		 * Redimensionne la matrice
		 */
		void resize(int rows, int cols)
		{
			m_storage.resize(rows * cols);
			m_rows = rows;
			m_cols = cols;
		}

		inline void setZero() { m_storage.setZero(); }
		inline int cols() const { return m_cols; }
		inline int rows() const { return m_rows; }

		/**
		 * Accès à la donnée membre de stockage (en lecture seule)
		 */
		const DenseStorage<Scalar, Dynamic>& storage() const
		{
			return m_storage;
		}

		/**
		 * Nombre d'éléments stockés dans le tampon.
		 */
		inline int size() const
		{
			return m_storage.size();
		}

		/**
		 * Accès au tampon de données ((lecture seule))
		 */
		const Scalar* data() const
		{
			return m_storage.data();
		}

		/*
		 * Retourne l'élément du tampon à l'indice `i` en lecture
		 */
		Scalar operator [](int i) const { return m_storage[i]; }
	};
}
