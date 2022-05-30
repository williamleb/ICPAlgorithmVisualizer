/**
 * @file DenseStorage_Test.cpp
 *
 * @brief Unit tests for the DenseStorage class.
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include <gtest/gtest.h>

#include "../DenseStorage.h"

using namespace gti320;

#pragma region DenseStorage_Static

/*
 * Teste que le constructeur par d�faut du DenseStorage statique cr�e correctement l'objet
 */
TEST(TestLabo1, DenseStorage_Static_DefaultConstructor_Ok)
{
	DenseStorage<double, 5> doubleDenseStorage;
	DenseStorage<float, 1> floatDenseStorage;
	DenseStorage<int, 9> intDenseStorage;

	EXPECT_EQ(5, doubleDenseStorage.size());
	EXPECT_EQ(1, floatDenseStorage.size());
	EXPECT_EQ(9, intDenseStorage.size());

	for (auto i = 0; i < 5; ++i)
	{
		EXPECT_DOUBLE_EQ(0.0, doubleDenseStorage.data()[i]);
	}

	EXPECT_FLOAT_EQ(0.f, doubleDenseStorage.data()[0]);

	for (auto i = 0; i < 9; ++i)
	{
		EXPECT_EQ(0, intDenseStorage.data()[i]);
	}
}

/*
 * Teste que le constructeur de copie du DenseStorage statique cr�e correctement l'objet
 */
TEST(TestLabo1, DenseStorage_Static_CopyConstructor_Ok)
{
	DenseStorage<int, 5> storageToCopy{ 1, 4, 2, 1, -5 };

	DenseStorage<int, 5> copiedStorage(storageToCopy);

	EXPECT_EQ(1, copiedStorage[0]);
	EXPECT_EQ(4, copiedStorage[1]);
	EXPECT_EQ(2, copiedStorage[2]);
	EXPECT_EQ(1, copiedStorage[3]);
	EXPECT_EQ(-5, copiedStorage[4]);
}

/*
 * Teste que le constructeur avec une liste d'initialisation du DenseStorage statique cr�e correctement l'objet
 */
TEST(TestLabo1, DenseStorage_Static_InitializerListConstructor_Ok)
{
	DenseStorage<int, 5> intStorage{ 1, 4, 2, 1, -5 };
	DenseStorage<float, 2> floatStorage{ 1.2f, 4.f };

	EXPECT_EQ(1, intStorage[0]);
	EXPECT_EQ(4, intStorage[1]);
	EXPECT_EQ(2, intStorage[2]);
	EXPECT_EQ(1, intStorage[3]);
	EXPECT_EQ(-5, intStorage[4]);

	EXPECT_FLOAT_EQ(1.2f, floatStorage[0]);
	EXPECT_FLOAT_EQ(4.f, floatStorage[1]);
}

/*
 * Teste que le constructeur avec une liste d'initialisation du DenseStorage statique cr�e correctement l'objet
 * si la liste d'initialisation n'est pas de la m�me taille que le stockage
 */
TEST(TestLabo1, DenseStorage_Static_InitializerListConstructor_WrongSize)
{
	DenseStorage<int, 2> denseStorageSmaller{ 1 };
	DenseStorage<int, 2> denseStorageLarger{ 1, 4, 2 };

	EXPECT_EQ(1, denseStorageSmaller[0]);

	EXPECT_EQ(1, denseStorageLarger[0]);
	EXPECT_EQ(4, denseStorageLarger[1]);
}

/*
 * Teste que le constructeur avec une taille du DenseStorage statique cr�e correctement l'objet
 */
TEST(TestLabo1, DenseStorage_Static_SizeConstructor_Ok)
{
	DenseStorage<int, 2> denseStorage(2);

	EXPECT_EQ(2, denseStorage.size());
	EXPECT_EQ(0, denseStorage[0]);
	EXPECT_EQ(0, denseStorage[1]);
}

/*
 * Teste que le constructeur par donn�es du DenseStorage statique cr�e correctement l'objet
 */
TEST(TestLabo1, DenseStorage_Static_DataConstructor_Ok)
{
	int intData[]{ 1, 4, 2, 1, -5 };
	float floatData[]{ 1.2f, 4.f };

	DenseStorage<int, 5> intStorage(intData);
	DenseStorage<float, 2> floatStorage(floatData);

	EXPECT_EQ(1, intStorage[0]);
	EXPECT_EQ(4, intStorage[1]);
	EXPECT_EQ(2, intStorage[2]);
	EXPECT_EQ(1, intStorage[3]);
	EXPECT_EQ(-5, intStorage[4]);

	EXPECT_FLOAT_EQ(1.2f, floatStorage[0]);
	EXPECT_FLOAT_EQ(4.f, floatStorage[1]);
}

/*
 * Teste que le l'op�rateur "=" du DenseStorage statique copie correctement les valeurs
 */
TEST(TestLabo1, DenseStorage_Static_EqualOperator_Ok)
{
	DenseStorage<int, 5> storageToCopy{ 1, 4, 2, 1, -5 };

	DenseStorage<int, 5> copiedStorage;
	copiedStorage = storageToCopy;

	EXPECT_EQ(1, copiedStorage[0]);
	EXPECT_EQ(4, copiedStorage[1]);
	EXPECT_EQ(2, copiedStorage[2]);
	EXPECT_EQ(1, copiedStorage[3]);
	EXPECT_EQ(-5, copiedStorage[4]);
}

/*
 * Teste que le l'op�rateur "=" du DenseStorage fonctionne avec une auto-assignation
 */
TEST(TestLabo1, DenseStorage_Static_EqualOperator_SameStorage)
{
	DenseStorage<int, 5> storageToCopy{ 1, 4, 2, 1, -5 };
	storageToCopy = storageToCopy;

	EXPECT_EQ(1, storageToCopy[0]);
	EXPECT_EQ(4, storageToCopy[1]);
	EXPECT_EQ(2, storageToCopy[2]);
	EXPECT_EQ(1, storageToCopy[3]);
	EXPECT_EQ(-5, storageToCopy[4]);
}

/*
 * Teste que la m�thode "setZero" du DenseStorage assigne bien la valeur 0 � tous les �l�ments
 */
TEST(TestLabo1, DenseStorage_Static_setZero_Ok)
{
	DenseStorage<int, 5> denseStorage{ 1, 4, 2, 1, -5 };
	denseStorage.setZero();

	EXPECT_EQ(0, denseStorage[0]);
	EXPECT_EQ(0, denseStorage[1]);
	EXPECT_EQ(0, denseStorage[2]);
	EXPECT_EQ(0, denseStorage[3]);
	EXPECT_EQ(0, denseStorage[4]);
}

/*
 * Teste que la m�thode "data" du DenseStorage permet bien l'assignation et la lecture de valeurs
 */
TEST(TestLabo1, DenseStorage_Static_data_Ok)
{
	DenseStorage<int, 5> denseStorage{ 1, 4, 2, 1, -5 };
	denseStorage.data()[2] = 10;
	denseStorage.data()[4] = -60;

	EXPECT_EQ(1, denseStorage[0]);
	EXPECT_EQ(4, denseStorage[1]);
	EXPECT_EQ(10, denseStorage[2]);
	EXPECT_EQ(1, denseStorage[3]);
	EXPECT_EQ(-60, denseStorage[4]);
}

/*
 * Teste que l'op�rateur "[]" du DenseStorage permet bien l'assignation et la lecture de valeurs
 */
TEST(TestLabo1, DenseStorage_Static_BracketOperator_Ok)
{
	DenseStorage<int, 5> denseStorage{ 1, 4, 2, 1, -5 };
	denseStorage[2] = 10;
	denseStorage[4] = -60;

	EXPECT_EQ(1, denseStorage[0]);
	EXPECT_EQ(4, denseStorage[1]);
	EXPECT_EQ(10, denseStorage[2]);
	EXPECT_EQ(1, denseStorage[3]);
	EXPECT_EQ(-60, denseStorage[4]);
}

#pragma endregion

#pragma region DenseStorage_Dynamic
/*
 * Teste que le constructeur par d�faut du DenseStorage dynamique cr�e correctement l'objet vide
 */
TEST(TestLabo1, DenseStorage_Dynamic_DefaultConstructor_Ok)
{
	DenseStorage<double, Dynamic> doubleDenseStorage;
	DenseStorage<float, Dynamic> floatDenseStorage;
	DenseStorage<int, Dynamic> intDenseStorage;

	EXPECT_EQ(0, doubleDenseStorage.size());
	EXPECT_EQ(0, floatDenseStorage.size());
	EXPECT_EQ(0, intDenseStorage.size());
}

/*
 * Teste que le constructeur par copie du DenseStorage dynamique cr�e correctement l'objet
 */
TEST(TestLabo1, DenseStorage_Dynamic_CopyConstructor_Ok)
{
	DenseStorage<int, Dynamic> storageToCopy{ 1, 4, 2, 1, -5 };

	DenseStorage<int, Dynamic> copiedStorage(storageToCopy);

	EXPECT_EQ(5, copiedStorage.size());
	EXPECT_EQ(1, copiedStorage[0]);
	EXPECT_EQ(4, copiedStorage[1]);
	EXPECT_EQ(2, copiedStorage[2]);
	EXPECT_EQ(1, copiedStorage[3]);
	EXPECT_EQ(-5, copiedStorage[4]);
}

/*
 * Teste que le constructeur par d�faut du DenseStorage dynamique cr�e correctement l'objet vide
 */
TEST(TestLabo1, DenseStorage_Dynamic_InitializerListConstructor_Ok)
{
	DenseStorage<int, Dynamic> intStorage{ 1, 4, 2, 1, -5 };
	DenseStorage<float, Dynamic> floatStorage{ 1.2f, 4.f };

	EXPECT_EQ(5, intStorage.size());
	EXPECT_EQ(1, intStorage[0]);
	EXPECT_EQ(4, intStorage[1]);
	EXPECT_EQ(2, intStorage[2]);
	EXPECT_EQ(1, intStorage[3]);
	EXPECT_EQ(-5, intStorage[4]);

	EXPECT_EQ(2, floatStorage.size());
	EXPECT_FLOAT_EQ(1.2f, floatStorage[0]);
	EXPECT_FLOAT_EQ(4.f, floatStorage[1]);
}

/*
 * Teste que le constructeur par d�faut du DenseStorage dynamique cr�e correctement l'objet avec
 * la bonne taille avec les �l�ments initialis�s � 0
 */
TEST(TestLabo1, DenseStorage_Dynamic_SizeConstructor_Ok)
{
	DenseStorage<int, Dynamic> denseStorage(2);

	EXPECT_EQ(2, denseStorage.size());
	EXPECT_EQ(0, denseStorage[0]);
	EXPECT_EQ(0, denseStorage[1]);
}

/*
 * Teste que le constructeur par donn�es du DenseStorage dynamique cr�e correctement l'objet
 */
TEST(TestLabo1, DenseStorage_Dynamic_DataConstructor_Ok)
{
	int intData[]{ 1, 4, 2, 1, -5 };
	float floatData[]{ 1.2f, 4.f };

	DenseStorage<int, Dynamic> intStorage(intData, 5);
	DenseStorage<float, Dynamic> floatStorage(floatData, 2);

	EXPECT_EQ(1, intStorage[0]);
	EXPECT_EQ(4, intStorage[1]);
	EXPECT_EQ(2, intStorage[2]);
	EXPECT_EQ(1, intStorage[3]);
	EXPECT_EQ(-5, intStorage[4]);

	EXPECT_FLOAT_EQ(1.2f, floatStorage[0]);
	EXPECT_FLOAT_EQ(4.f, floatStorage[1]);
}

/*
 * Teste que l'op�rateur "=" du DenseStorage dynamique copie correctement l'autre stockage
 */
TEST(TestLabo1, DenseStorage_Dynamic_EqualOperator_Ok)
{
	DenseStorage<int, Dynamic> storageToCopy{ 1, 4, 2, 1, -5 };

	DenseStorage<int, Dynamic> copiedStorage;
	copiedStorage = storageToCopy;

	EXPECT_EQ(5, copiedStorage.size());
	EXPECT_EQ(1, copiedStorage[0]);
	EXPECT_EQ(4, copiedStorage[1]);
	EXPECT_EQ(2, copiedStorage[2]);
	EXPECT_EQ(1, copiedStorage[3]);
	EXPECT_EQ(-5, copiedStorage[4]);
}

/*
 * Teste que l'op�rateur "=" du DenseStorage dynamique copie correctement l'autre stockage si
 * le premier stockage existe et a la m�me taille
 */
TEST(TestLabo1, DenseStorage_Dynamic_EqualOperator_SameSize)
{
	DenseStorage<int, Dynamic> storageToCopy{ 1, 4, 2, 1, -5 };

	DenseStorage<int, Dynamic> copiedStorage{ 11, 12, 13, 14, 15 };
	copiedStorage = storageToCopy;

	EXPECT_EQ(5, copiedStorage.size());
	EXPECT_EQ(1, copiedStorage[0]);
	EXPECT_EQ(4, copiedStorage[1]);
	EXPECT_EQ(2, copiedStorage[2]);
	EXPECT_EQ(1, copiedStorage[3]);
	EXPECT_EQ(-5, copiedStorage[4]);
}

/*
 * Teste que l'op�rateur "=" du DenseStorage dynamique copie correctement l'autre stockage si
 * le premier stockage existe et a une taille diff�rente
 */
TEST(TestLabo1, DenseStorage_Dynamic_EqualOperator_DifferentSize)
{
	DenseStorage<int, Dynamic> storageToCopy{ 1, 4, 2, 1, -5 };

	DenseStorage<int, Dynamic> copiedStorage{ 11, 12 };
	copiedStorage = storageToCopy;

	EXPECT_EQ(5, copiedStorage.size());
	EXPECT_EQ(1, copiedStorage[0]);
	EXPECT_EQ(4, copiedStorage[1]);
	EXPECT_EQ(2, copiedStorage[2]);
	EXPECT_EQ(1, copiedStorage[3]);
	EXPECT_EQ(-5, copiedStorage[4]);
}

/*
 * Teste que l'op�rateur "=" du DenseStorage g�re l'auto assignation correctement
 */
TEST(TestLabo1, DenseStorage_Dynamic_EqualOperator_SameStorage)
{
	DenseStorage<int, Dynamic> storageToCopy{ 1, 4, 2, 1, -5 };
	storageToCopy = storageToCopy;

	EXPECT_EQ(5, storageToCopy.size());
	EXPECT_EQ(1, storageToCopy[0]);
	EXPECT_EQ(4, storageToCopy[1]);
	EXPECT_EQ(2, storageToCopy[2]);
	EXPECT_EQ(1, storageToCopy[3]);
	EXPECT_EQ(-5, storageToCopy[4]);
}

/*
 * Teste que la m�thode "resize" du DenseStorage change correctement la taille du stockage
 */
TEST(TestLabo1, DenseStorage_Dynamic_resize_Ok)
{
	DenseStorage<int, Dynamic> denseStorage;

	denseStorage.resize(5);

	EXPECT_EQ(5, denseStorage.size());
}

/*
 * Teste que la m�thode "resize" du DenseStorage change correctement la taille du stockage vers le bas
 * si le stockage existe d�j�
 */
TEST(TestLabo1, DenseStorage_Dynamic_resize_Smaller)
{
	DenseStorage<int, Dynamic> denseStorage = { 1, 2, 3, 4 };

	denseStorage.resize(2);

	EXPECT_EQ(2, denseStorage.size());
}

/*
 * Teste que la m�thode "resize" du DenseStorage change correctement la taille du stockage vers le haut
 * si le stockage existe d�j�
 */
TEST(TestLabo1, DenseStorage_Dynamic_resize_Larger)
{
	DenseStorage<int, Dynamic> denseStorage = { 1, 2 };

	denseStorage.resize(4);

	EXPECT_EQ(4, denseStorage.size());
}

/*
 * Teste que la m�thode "setZero" du DenseStorage change les valeurs du stockage � 0
 */
TEST(TestLabo1, DenseStorage_Dynamic_setZero_Ok)
{
	DenseStorage<int, Dynamic> denseStorage{ 1, 4, 2, 1, -5 };
	denseStorage.setZero();

	EXPECT_EQ(0, denseStorage[0]);
	EXPECT_EQ(0, denseStorage[1]);
	EXPECT_EQ(0, denseStorage[2]);
	EXPECT_EQ(0, denseStorage[3]);
	EXPECT_EQ(0, denseStorage[4]);
}

/*
 * Teste que la m�thode "data" du DenseStorage permet bien l'assignation et la lecture de valeurs
 */
TEST(TestLabo1, DenseStorage_Dynamic_data_Ok)
{
	DenseStorage<int, Dynamic> denseStorage{ 1, 4, 2, 1, -5 };
	denseStorage.data()[2] = 10;
	denseStorage.data()[4] = -60;

	EXPECT_EQ(1, denseStorage[0]);
	EXPECT_EQ(4, denseStorage[1]);
	EXPECT_EQ(10, denseStorage[2]);
	EXPECT_EQ(1, denseStorage[3]);
	EXPECT_EQ(-60, denseStorage[4]);
}

/*
 * Teste que l'op�rateur "[]" du DenseStorage permet bien l"assignation et la lecture de valeurs
 */
TEST(TestLabo1, DenseStorage_Dynamic_BracketOperator_Ok)
{
	DenseStorage<int, Dynamic> denseStorage{ 1, 4, 2, 1, -5 };
	denseStorage[2] = 10;
	denseStorage[4] = -60;

	EXPECT_EQ(1, denseStorage[0]);
	EXPECT_EQ(4, denseStorage[1]);
	EXPECT_EQ(10, denseStorage[2]);
	EXPECT_EQ(1, denseStorage[3]);
	EXPECT_EQ(-60, denseStorage[4]);
}

#pragma endregion