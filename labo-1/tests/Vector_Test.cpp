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

using namespace gti320;

/*
 * Teste que le constructeur par d�faut d'un Vector cr�e correctement l'objet
 */
TEST(TestLabo1, Vector_DefaultConstructor_Ok)
{
	Vector<double, 4> doubleVector;
	Vector<float, 1> floatVector;
	Vector<int, Dynamic> intVector;

	EXPECT_EQ(4, doubleVector.size());
	EXPECT_EQ(1, floatVector.size());
	EXPECT_EQ(0, intVector.size());

	for (auto i = 0; i < 4; ++i)
	{
		EXPECT_DOUBLE_EQ(0.0, doubleVector(i));
	}

	EXPECT_FLOAT_EQ(0.f, floatVector(0));
}

/*
 * Teste que le constructeur par liste d'initialization d'un Vector cr�e correctement l'objet
 */
TEST(TestLabo1, Vector_InitializerListConstructor_Ok)
{
	Vector<int, 3> staticVector = { 0, 1, 2 };
	Vector<int, Dynamic> dynamicVector = { 0, 1, 2 };

	EXPECT_EQ(3, staticVector.size());
	EXPECT_EQ(3, dynamicVector.size());

	for (auto i = 0; i < 3; ++i)
	{
		EXPECT_EQ(i, staticVector(i));
		EXPECT_EQ(i, dynamicVector(i));
	}
}

/*
 * Teste que le constructeur par taille d'un Vector cr�e correctement l'objet
 */
TEST(TestLabo1, Vector_SizeConstructor_Ok)
{
	Vector<int, 3> staticVector(3);
	Vector<int, Dynamic> dynamicVector(3);

	EXPECT_EQ(3, staticVector.size());
	EXPECT_EQ(3, dynamicVector.size());

	for (auto i = 0; i < 3; ++i)
	{
		EXPECT_EQ(0, staticVector(i));
		EXPECT_EQ(0, dynamicVector(i));
	}
}

/*
 * Teste que le constructeur par copie d'un Vector cr�e correctement l'objet
 */
TEST(TestLabo1, Vector_CopyConstructor_Ok)
{
	Vector<int, Dynamic> vectorToCopy {0, 1, 2, 3};

	Vector<int, Dynamic> vector(vectorToCopy);

	EXPECT_EQ(4, vector.size());

	for (auto i = 0; i < 4; ++i)
	{
		EXPECT_EQ(i, vector(i));
	}
}

/*
 * Teste que l'op�rateur "=" d'un Vector copie correctement l'objet
 */
TEST(TestLabo1, Vector_EqualOperator_Ok)
{
	Vector<int, Dynamic> vectorToCopy{ 0, 1, 2, 3 };

	Vector<int, Dynamic> vector;
	vector = vectorToCopy;

	EXPECT_EQ(4, vector.size());

	for (auto i = 0; i < 4; ++i)
	{
		EXPECT_EQ(i, vector(i));
	}
}

/*
 * Teste que la m�thode "resize" d'un Vector redimensionne correctement l'objet
 */
TEST(TestLabo1, Vector_resize_Ok)
{
	Vector<int, Dynamic> vector(6);
	vector.resize(1);

	EXPECT_EQ(1, vector.size());

	vector.resize(7);

	EXPECT_EQ(7, vector.size());
}

/*
 * Teste que la m�thode "dot" d'un Vector donne le bon produit scalaire pour deux vecteurs parall�les
 */
TEST(TestLabo1, Vector_dot_ParalelVectors)
{
	Vector<double> vector1 {1, 2};
	Vector<double> vector2 {2, 4};

	EXPECT_DOUBLE_EQ(10, vector1.dot(vector2));
}

/*
 * Teste que la m�thode "dot" d'un Vector donne le bon produit scalaire pour deux vecteurs perpendiculaires
 */
TEST(TestLabo1, Vector_dot_PerpendicularVectors)
{
	Vector<double> vector1{ 1, 2 };
	Vector<double> vector2{ 6, -3 };

	EXPECT_DOUBLE_EQ(0, vector1.dot(vector2));
}

/*
 * Teste que la m�thode "dot" d'un Vector donne le bon produit scalaire pour deux vecteurs quelquonques
 */
TEST(TestLabo1, Vector_dot_NormalVectors)
{
	Vector<double> vector1{ 1, 6, 12, 3 };
	Vector<double> vector2{ 5, -4, 2, -20 };

	EXPECT_DOUBLE_EQ(-55, vector1.dot(vector2));
}

/*
 * Teste que la m�thode "squaredNorm" d'un Vector donne la bonne norme �lev�e � la 2
 */
TEST(TestLabo1, Vector_squaredNorm_Ok)
{
	Vector<double> vector1{ 4 };
	Vector<double> vector2{ 3, -4 };
	Vector<double> vector3{ -1, 2, 3 };
	Vector<double> vector4{ 10, 5, 1, 2 };

	EXPECT_DOUBLE_EQ(16, vector1.squaredNorm());
	EXPECT_DOUBLE_EQ(25, vector2.squaredNorm());
	EXPECT_DOUBLE_EQ(14, vector3.squaredNorm());
	EXPECT_DOUBLE_EQ(130, vector4.squaredNorm());
}

/*
 * Teste que la m�thode "norm" d'un Vector donne la bonne norme
 */
TEST(TestLabo1, Vector_norm_Ok)
{
	Vector<double> vector1{ 4 };
	Vector<double> vector2{ 3, -4 };
	Vector<double> vector3{ -1, 2, 3 };
	Vector<double> vector4{ 10, 5, 1, 2 };

	EXPECT_DOUBLE_EQ(4, vector1.norm());
	EXPECT_DOUBLE_EQ(5, vector2.norm());
	EXPECT_DOUBLE_EQ(sqrt(14), vector3.norm());
	EXPECT_DOUBLE_EQ(sqrt(130), vector4.norm());
}