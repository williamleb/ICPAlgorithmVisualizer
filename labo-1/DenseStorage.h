#pragma once

/**
 * @file DenseStorage.h
 *
 * @brief Stockage dense pour des données à taille fixe ou dynamique.
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include <cstring>
#include <initializer_list>

#include "GTIAssert.h"

namespace gti320
{
	enum SizeType
	{
		Dynamic = -1
	};

	/**
	 * Stockage à taille fixe.
	 *
	 * Le nombre de données à stocker est connu au moment de la compilation.
	 * Ce nombre est donné par le paramètre de patron : Size
	 *
	 */
	template <typename Scalar, int Size>
	class DenseStorage
	{
	private:
		Scalar m_data[Size];

	public:
		/**
		 * Constructeur par défaut
		 *
		 * Initialise l'ensemble des éléments du stockage à 0.
		 */
		DenseStorage()
		{
			memset(m_data, 0, sizeof(Scalar) * Size);
		}

		/**
		 * Constructeur de copie
		 *
		 * Copie les éléments d'un stockage vers le stockage courant.
		 */
		DenseStorage(const DenseStorage& other)
		{
			memcpy(m_data, other.m_data, sizeof(Scalar) * Size);
		}


		/**
		 * Constructeur avec une liste d'initialisation
		 *
		 * Copie les éléments d'un stockage vers le stockage courant.
		 * Si la taille de la liste est plus grande que celle du stockage actuel, seuls les premiers éléments sont copiés.
		 * Si la taille du stockage est plus grande que celle de la liste, les éléments en trop ne sont pas initialisés.
		 */
		DenseStorage(std::initializer_list<Scalar> initializerList)
		{
			int minSize = Size < initializerList.size() ? Size : static_cast<int>(initializerList.size());
			memcpy(m_data, initializerList.begin(), sizeof(Scalar) * minSize);
		}

		/**
		 * Constructeur avec taille spécifiée
		 *
		 * (doit être la même que la taille spécifiée dans le patron)
		 */
		explicit DenseStorage(int size) : DenseStorage()
		{
			ASSERT(Size == size, "Attempting to create a DenseStorage with two different sizes.");
		}

		/**
		 * Constructor avec taille (size) et données initiales (data).
		 */
		explicit DenseStorage(const Scalar* data, int size = Size)
		{
			ASSERT(Size == size, "Attempting to create a DenseStorage with two different sizes.");
			ASSERT(data != nullptr, "Attempting to create a dense storage with no data");

			memcpy(m_data, data, sizeof(Scalar) * size);
		}

		/**
		 * Opérateur de copie
		 */
		DenseStorage& operator=(const DenseStorage& other)
		{
			if (this != &other)
			{
				memcpy(m_data, other.m_data, sizeof(Scalar) * Size);
			}
			return *this;
		}

		/**
		 * Retourne la taille du tampon
		 */
		static inline int size() { return Size; }

		/**
		 * Redimensionne le stockage pour qu'il contienne `size` élément(s).
		 */
		void resize(int size)
		{
			ASSERT(size == Size, "Attempting to resize a static DenseStorage. Use a dynamic DenseStorage instead.");
		}

		/**
		 * Met tous les éléments à zéro.
		 */
		void setZero()
		{
			memset(m_data, 0, sizeof(Scalar) * Size);
		}

		/**
		 * Accès au tampon de données (en lecture seulement)
		 */
		const Scalar* data() const
		{
			return m_data;
		}

		/**
		 * Accès au tampon de données (pour lecture et écriture)
		 */
		Scalar* data()
		{
			return m_data;
		}

		/*
		 * Retourne l'élément du tampon à l'indice `i` en lecture
		 */
		Scalar operator [](int i) const { return m_data[i]; }

		/*
		 * Retourne l'élément du tampon à l'indice `i` en lecture et en écriture
		 */
		Scalar& operator [](int i) { return m_data[i]; }
	};


	/**
	 * Stockage à taille dynamique.
	 *
	 * Le nombre de données à stocker est déterminé à l'exécution.
	 */
	template <typename Scalar>
	class DenseStorage<Scalar, Dynamic>
	{
	private:
		Scalar* m_data;
		int m_size;

	public:
		/**
		 * Constructeur par défaut
		 */
		DenseStorage() : m_data(nullptr), m_size(0)
		{
		}

		/**
		 * Constructeur avec taille spécifiée
		 */
		explicit DenseStorage(int size) : m_data(nullptr), m_size(size)
		{
			ASSERT(size >= 0, "Attempting to create a dense storage with a negative size");

			m_data = new Scalar[size];
			memset(m_data, 0, sizeof(Scalar) * m_size);
		}

		/**
		 * Constructor avec taille (size) et données initiales (data).
		 */
		explicit DenseStorage(const Scalar* data, int size)
		{
			ASSERT(size >= 0, "Attempting to create a dense storage with a negative size");
			ASSERT(data != nullptr, "Attempting to create a dense storage with no data");

			m_data = new Scalar[size];
			memcpy(m_data, data, sizeof(Scalar) * size);
		}

		/**
		 * Constructeur de copie
		 */
		DenseStorage(const DenseStorage& other) : m_data(nullptr), m_size(other.m_size)
		{
			m_data = new Scalar[m_size];
			memcpy(m_data, other.m_data, sizeof(Scalar) * m_size);
		}

		/*
		 * Constructeur par liste d'initialisation
		 */
		DenseStorage(std::initializer_list<Scalar> initializerList) : m_data(nullptr), m_size(initializerList.size())
		{
			m_data = new Scalar[m_size];
			memcpy(m_data, initializerList.begin(), sizeof(Scalar) * m_size);
		}

		/**
		 * Opérateur de copie
		 */
		DenseStorage& operator=(const DenseStorage& other)
		{
			if (this == &other)
			{
				return *this;
			}

			// On alloue un nouveau tampon de la bonne taille avant de copier les éléments
			if (m_size != other.m_size)
			{
				delete[] m_data;

				m_size = other.m_size;
				m_data = new Scalar[m_size];
			}

			memcpy(m_data, other.m_data, sizeof(Scalar) * m_size);

			return *this;
		}

		/**
		 * Destructeur
		 */
		~DenseStorage()
		{
			delete[] m_data;
		}

		/**
		 * Retourne la taille du tampon 
		 */
		inline int size() const { return m_size; }

		/**
		 * Redimensionne le tampon alloué pour le stockage.
		 * La mémoire qui n'est plus utilisée doit être libérée.
		 */
		void resize(int size)
		{
			ASSERT(size >= 0, "Attempting to resize a dense storage with a negative size");

			auto* newData = new Scalar[size];

			delete[] m_data;

			m_data = newData;
			m_size = size;
		}

		/**
		 * Met tous les éléments à zéro.
		 */
		void setZero()
		{
			memset(m_data, 0, sizeof(Scalar) * m_size);
		}

		/**
		 * Accès au tampon de données (en lecteur seulement)
		 */
		const Scalar* data() const { return m_data; }

		/**
		 * Accès au tampon de données (pour lecture et écriture)
		 */
		Scalar* data() { return m_data; }

		/*
		 * Retourne l'élément du tampon à l'indice `i` en lecture
		 */
		Scalar operator [](int i) const { return m_data[i]; }

		/*
		 * Retourne l'élément du tampon à l'indice `i` en lecture et en écriture
		 */
		Scalar& operator [](int i) { return m_data[i]; }
	};
}
