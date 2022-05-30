/**
 * @file Math3D_Test.cpp
 *
 * @brief Unit tests for the Math3D file.
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include <gtest/gtest.h>

#include "../Math3D.h"

using namespace gti320;

const double DOUBLE_THRESHOLD = 1e-3;

static Matrix4d makeMatrix4dFromRotationAndTranslation(const Vector3d& rotation, const Vector3d& translation)
{
	Matrix4d homogenousMatrix;
	homogenousMatrix.block(0, 0, 3, 3) = makeRotation(rotation.x(), rotation.y(), rotation.z());

	homogenousMatrix(0, 3) = translation.x();
	homogenousMatrix(1, 3) = translation.y();
	homogenousMatrix(2, 3) = translation.z();
	homogenousMatrix(3, 3) = 1;

	return homogenousMatrix;
}

/*
 * Teste que la m�thode "inverse" d'une matrice 4d en coordonn�es homog�nes (matrice identit� et aucune translation) retourne le bon r�sultat
 */
TEST(TestLabo1, Math3D_Matrice4d_inverse_IdentityNoTranslation)
{
	auto matrix = makeMatrix4dFromRotationAndTranslation({ 0, 0, 0 }, { 0, 0, 0 });

	auto resultMatrix = matrix.inverse();

	// Pour les tests d'une matrice inverse suivants, j'ai utilis� l'outil en ligne https://matrix.reshish.com/inverCalculation.php pour pr�dire les r�ponses
	// Expected
	// |1, 0, 0, 0|
	// |0, 1, 0, 0|
	// |0, 0, 1, 0|
	// |0, 0, 0, 1|
	EXPECT_DOUBLE_EQ(1.0, resultMatrix(0, 0));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(0, 1));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(0, 2));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(0, 3));

	EXPECT_DOUBLE_EQ(0.0, resultMatrix(1, 0));
	EXPECT_DOUBLE_EQ(1.0, resultMatrix(1, 1));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(1, 2));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(1, 3));

	EXPECT_DOUBLE_EQ(0.0, resultMatrix(2, 0));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(2, 1));
	EXPECT_DOUBLE_EQ(1.0, resultMatrix(2, 2));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(2, 3));

	EXPECT_DOUBLE_EQ(0.0, resultMatrix(3, 0));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(3, 1));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(3, 2));
	EXPECT_DOUBLE_EQ(1.0, resultMatrix(3, 3));
}

/*
 * Teste que la m�thode "inverse" d'une matrice 4d en coordonn�es homog�nes (matrice identit� et translation quelconque) retourne le bon r�sultat
 */
TEST(TestLabo1, Math3D_Matrice4d_inverse_IdentityTranslation)
{
	auto matrix = makeMatrix4dFromRotationAndTranslation({ 0, 0, 0 }, { 13, 90, -15 });

	auto resultMatrix = matrix.inverse();

	// Expected
	// |1 0 0 -13|
	// |0 1 0 -90|
	// |0 0 1  15|
	// |0 0 0  1 |
	EXPECT_DOUBLE_EQ(1.0, resultMatrix(0, 0));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(0, 1));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(0, 2));
	EXPECT_DOUBLE_EQ(-13.0, resultMatrix(0, 3));

	EXPECT_DOUBLE_EQ(0.0, resultMatrix(1, 0));
	EXPECT_DOUBLE_EQ(1.0, resultMatrix(1, 1));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(1, 2));
	EXPECT_DOUBLE_EQ(-90.0, resultMatrix(1, 3));

	EXPECT_DOUBLE_EQ(0.0, resultMatrix(2, 0));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(2, 1));
	EXPECT_DOUBLE_EQ(1.0, resultMatrix(2, 2));
	EXPECT_DOUBLE_EQ(15.0, resultMatrix(2, 3));

	EXPECT_DOUBLE_EQ(0.0, resultMatrix(3, 0));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(3, 1));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(3, 2));
	EXPECT_DOUBLE_EQ(1.0, resultMatrix(3, 3));
}

/*
 * Teste que la m�thode "inverse" d'une matrice 4d en coordonn�es homog�nes (matrice de rotation quelconque et aucune translation) retourne le bon r�sultat
 */
TEST(TestLabo1, Math3D_Matrice4d_inverse_RotationNoTranslation)
{
	// 45 degr�s en x, -45 degr�s en y et 270 degr�s en z
	auto matrix = makeMatrix4dFromRotationAndTranslation({ 0.785398, -0.785398, 4.71239 }, { 0, 0, 0 });

	auto resultMatrix = matrix.inverse();

	// Expected
	// | 0      -0.7071 0.7071 0|
	// | 0.7071  0.5    0.5    0|
	// |-0.7071  0.5    0.5    0|
	// | 0       0      0      1|
	EXPECT_NEAR(0.0, resultMatrix(0, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-0.7071, resultMatrix(0, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.7071, resultMatrix(0, 2), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(0, 3), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.7071, resultMatrix(1, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5, resultMatrix(1, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5, resultMatrix(1, 2), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(1, 3), DOUBLE_THRESHOLD);

	EXPECT_NEAR(-0.7071, resultMatrix(2, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5, resultMatrix(2, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5, resultMatrix(2, 2), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(2, 3), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.0, resultMatrix(3, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(3, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(3, 2), DOUBLE_THRESHOLD);
	EXPECT_NEAR(1.0, resultMatrix(3, 3), DOUBLE_THRESHOLD);
}

/*
 * Teste que la m�thode "inverse" d'une matrice 4d en coordonn�es homog�nes (matrice de rotation quelconque et translation quelconque) retourne le bon r�sultat
 */
TEST(TestLabo1, Math3D_Matrice4d_inverse_RotationTranslation)
{
	// 45 degr�s en x, -45 degr�s en y et 270 degr�s en z
	auto matrix = makeMatrix4dFromRotationAndTranslation({ 0.785398, -0.785398, 4.71239 }, { -10, 20, 30 });

	auto resultMatrix = matrix.inverse();

	// Expected
	// | 0      -0.7071 0.7071 -7.0711 |
	// | 0.7071  0.5    0.5    -17.9288|
	// |-0.7071  0.5    0.5    -32.0711|
	// | 0       0      0      1       |
	EXPECT_NEAR(0.0, resultMatrix(0, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-0.7071, resultMatrix(0, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.7071, resultMatrix(0, 2), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-7.0711, resultMatrix(0, 3), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.7071, resultMatrix(1, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5, resultMatrix(1, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5, resultMatrix(1, 2), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-17.9288, resultMatrix(1, 3), DOUBLE_THRESHOLD);

	EXPECT_NEAR(-0.7071, resultMatrix(2, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5, resultMatrix(2, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5, resultMatrix(2, 2), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-32.0711, resultMatrix(2, 3), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.0, resultMatrix(3, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(3, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(3, 2), DOUBLE_THRESHOLD);
	EXPECT_NEAR(1.0, resultMatrix(3, 3), DOUBLE_THRESHOLD);
}


/*
 * Teste que la m�thode "inverse" d'une matrice 3d de rotation identit� retourne la bonne valeur
 */
TEST(TestLabo1, Math3D_Matrice3d_inverse_Identity)
{
	auto matrix = makeRotation(0, 0, 0);

	auto resultMatrix = matrix.inverse();

	// Expected
	// |1 0 0|
	// |0 1 0|
	// |0 0 1|
	EXPECT_DOUBLE_EQ(1.0, resultMatrix(0, 0));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(0, 1));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(0, 2));

	EXPECT_DOUBLE_EQ(0.0, resultMatrix(1, 0));
	EXPECT_DOUBLE_EQ(1.0, resultMatrix(1, 1));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(1, 2));

	EXPECT_DOUBLE_EQ(0.0, resultMatrix(2, 0));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(2, 1));
	EXPECT_DOUBLE_EQ(1.0, resultMatrix(2, 2));
}

/*
 * Teste que la m�thode "inverse" d'une matrice 3d de rotation quelconque retourne la bonne valeur
 */
TEST(TestLabo1, Math3D_Matrice3d_inverse_Ok)
{
	// 45 degr�s en x, -45 degr�s en y et 270 degr�s en z
	auto matrix = makeRotation(0.785398, -0.785398, 4.71239);

	auto resultMatrix = matrix.inverse();

	// Expected
	// | 0      -0.7071 0.7071|
	// | 0.7071  0.5    0.5   |
	// |-0.7071  0.5    0.5   |
	EXPECT_NEAR(0.0, resultMatrix(0, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-0.7071, resultMatrix(0, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.7071, resultMatrix(0, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.7071, resultMatrix(1, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5, resultMatrix(1, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5, resultMatrix(1, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(-0.7071, resultMatrix(2, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5, resultMatrix(2, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5, resultMatrix(2, 2), DOUBLE_THRESHOLD);
}


/*
 * Teste que la multiplication d'une matrice 4d en coordonn�es homog�nes (matrice identit� et aucune translation) et d'un vecteur 3d retourne la bonne valeur
 */
TEST(TestLabo1, Math3D_Multiplication_Matrice4DVector3D_IdentityNoTranslation)
{
	auto matrix = makeMatrix4dFromRotationAndTranslation({ 0, 0, 0 }, { 0, 0, 0 });

	Vector3d vector{1.0, -2.0, 4.0};

	auto resultVector = matrix * vector;

	// Expected: [1.0, -2.0, 4.0]
	EXPECT_DOUBLE_EQ(1.0, resultVector(0));
	EXPECT_DOUBLE_EQ(-2.0, resultVector(1));
	EXPECT_DOUBLE_EQ(4.0, resultVector(2));
}

/*
 * Teste que la multiplication d'une matrice 4d en coordonn�es homog�nes (matrice identit� et translation quelconque) et d'un vecteur 3d retourne la bonne valeur
 */
TEST(TestLabo1, Math3D_Multiplication_Matrice4DVector3D_IdentityTranslation)
{
	auto matrix = makeMatrix4dFromRotationAndTranslation({ 0, 0, 0 }, { -4, 2, 13 });

	Vector3d vector{ 1.0, -2.0, 4.0 };

	auto resultVector = matrix * vector;

	// Expected: [-3.0, 0.0, 17.0]
	EXPECT_DOUBLE_EQ(-3.0, resultVector(0));
	EXPECT_DOUBLE_EQ(0.0, resultVector(1));
	EXPECT_DOUBLE_EQ(17.0, resultVector(2));
}

/*
 * Teste que la multiplication d'une matrice 4d en coordonn�es homog�nes (matrice de rotation et aucune translation) et d'un vecteur 3d retourne la bonne valeur
 */
TEST(TestLabo1, Math3D_Multiplication_Matrice4DVector3D_RotationNoTranslation)
{
	// 45 degr�s en x, -45 degr�s en y et 270 degr�s en z
	auto matrix = makeMatrix4dFromRotationAndTranslation({ 0.785398, -0.785398, 4.71239 }, { 0, 0, 0 });

	Vector3d vector{ 1.0, -2.0, 4.0 };

	auto resultVector = matrix * vector;

	// Expected: [-4.2426, 0.2929, 1.7071]
	EXPECT_NEAR(-4.2426, resultVector(0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.2929, resultVector(1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(1.7071, resultVector(2), DOUBLE_THRESHOLD);
}

/*
 * Teste que la multiplication d'une matrice 4d en coordonn�es homog�nes (matrice de rotation et translation quelconque) et d'un vecteur 3d retourne la bonne valeur
 */
TEST(TestLabo1, Math3D_Multiplication_Matrice4DVector3D_RotationTranslation)
{
	// 45 degr�s en x, -45 degr�s en y et 270 degr�s en z
	auto matrix = makeMatrix4dFromRotationAndTranslation({ 0.785398, -0.785398, 4.71239 }, { -13, 6, 2 });

	Vector3d vector{ 1.0, -2.0, 4.0 };

	auto resultVector = matrix * vector;

	// Expected: [-17.2426, 6.2929, 3.7071]
	EXPECT_NEAR(-17.2426, resultVector(0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(6.2929, resultVector(1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(3.7071, resultVector(2), DOUBLE_THRESHOLD);
}

/*
 * Teste que la fonction "makeRotation" retourne bien la bonne matrice 3D de rotation lorsqu'on ne sp�cifie pas de rotation
 */
TEST(TestLabo1, Math3D_makeRotation_NoRotation)
{
	auto resultMatrix = makeRotation(0.0, 0.0, 0.0);

	// Expected
	// |1 0 0|
	// |0 1 0|
	// |0 0 1|
	EXPECT_DOUBLE_EQ(1.0, resultMatrix(0, 0));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(0, 1));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(0, 2));

	EXPECT_DOUBLE_EQ(0.0, resultMatrix(1, 0));
	EXPECT_DOUBLE_EQ(1.0, resultMatrix(1, 1));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(1, 2));

	EXPECT_DOUBLE_EQ(0.0, resultMatrix(2, 0));
	EXPECT_DOUBLE_EQ(0.0, resultMatrix(2, 1));
	EXPECT_DOUBLE_EQ(1.0, resultMatrix(2, 2));
}

/*
 * Teste que la fonction "makeRotation" retourne bien la bonne matrice 3D de rotation lors d'une rotation sur l'axe des x seulement
 */
TEST(TestLabo1, Math3D_makeRotation_XRotation)
{
	auto resultMatrix = makeRotation(1.0, 0.0, 0.0);

	// Pour calcul� le r�sultat qu'on attends de tous les tests de rotation suivants, j'ai utilis� l'outil en ligne https://www.andre-gaschler.com/rotationconverter/
	// Expected
	// |1 0       0     |
	// |0 0.5403 -0.8414|
	// |0 0.8414  0.5403|
	EXPECT_NEAR(1.0, resultMatrix(0, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(0, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(0, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.0, resultMatrix(1, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5403, resultMatrix(1, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-0.8414, resultMatrix(1, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.0, resultMatrix(2, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.8414, resultMatrix(2, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5403, resultMatrix(2, 2), DOUBLE_THRESHOLD);
}

/*
 * Teste que la fonction "makeRotation" retourne bien la bonne matrice 3D de rotation lors d'une rotation sur l'axe des y seulement
 */
TEST(TestLabo1, Math3D_makeRotation_YRotation)
{
	auto resultMatrix = makeRotation(0.0, 1.0, 0.0);

	// Expected
	// | 0.5403 0 0.8414|
	// | 0      1 0     |
	// |-0.8414 0 0.5403|
	EXPECT_NEAR(0.5403, resultMatrix(0, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(0, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.8414, resultMatrix(0, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.0, resultMatrix(1, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(1.0, resultMatrix(1, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(1, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(-0.8414, resultMatrix(2, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(2, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5403, resultMatrix(2, 2), DOUBLE_THRESHOLD);
}

/*
 * Teste que la fonction "makeRotation" retourne bien la bonne matrice 3D de rotation lors d'une rotation sur l'axe des z seulement
 */
TEST(TestLabo1, Math3D_makeRotation_ZRotation)
{
	auto resultMatrix = makeRotation(0.0, 0.0, 1.0);

	// Expected
	// |0.5403 -0.8414 0|
	// |0.8414  0.5403 0|
	// |0       0      1|
	EXPECT_NEAR(0.5403, resultMatrix(0, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-0.8414, resultMatrix(0, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(0, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.8414, resultMatrix(1, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5403, resultMatrix(1, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(1, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.0, resultMatrix(2, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(2, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(1.0, resultMatrix(2, 2), DOUBLE_THRESHOLD);
}

/*
 * Teste que la fonction "makeRotation" retourne bien la bonne matrice 3D de rotation lors d'une rotation sur les axes xy
 */
TEST(TestLabo1, Math3D_makeRotation_XYRotation)
{
	auto resultMatrix = makeRotation(1.0, 1.0, 0.0);

	// Expected
	// | 0.5403 0.7080  0.4546|
	// | 0      0.5403 -0.8414|
	// |-0.8414 0.4546  0.2919|
	EXPECT_NEAR(0.5403, resultMatrix(0, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.7080, resultMatrix(0, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.4546, resultMatrix(0, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.0, resultMatrix(1, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5403, resultMatrix(1, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-0.8414, resultMatrix(1, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(-0.8414, resultMatrix(2, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.4546, resultMatrix(2, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.2919, resultMatrix(2, 2), DOUBLE_THRESHOLD);
}

/*
 * Teste que la fonction "makeRotation" retourne bien la bonne matrice 3D de rotation lors d'une rotation sur les axes xz
 */
TEST(TestLabo1, Math3D_makeRotation_XZRotation)
{
	auto resultMatrix = makeRotation(1.0, 0.0, 1.0);

	// Expected
	// |0.5403 -0.4546  0.7080|
	// |0.8414  0.2919 -0.4546|
	// |0       0.8414  0.5403|
	EXPECT_NEAR(0.5403, resultMatrix(0, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-0.4546, resultMatrix(0, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.7080, resultMatrix(0, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.8414, resultMatrix(1, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.2919, resultMatrix(1, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-0.4546, resultMatrix(1, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0, resultMatrix(2, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.8414, resultMatrix(2, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5403, resultMatrix(2, 2), DOUBLE_THRESHOLD);
}

/*
 * Teste que la fonction "makeRotation" retourne bien la bonne matrice 3D de rotation lors d'une rotation sur les axes yz
 */
TEST(TestLabo1, Math3D_makeRotation_YZRotation)
{
	auto resultMatrix = makeRotation(0.0, 1.0, 1.0);

	// Expected
	// | 0.2919 -0.8414 0.4546|
	// | 0.4546  0.5403 0.7080|
	// |-0.8414  0      0.5403|
	EXPECT_NEAR(0.2919, resultMatrix(0, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-0.8414, resultMatrix(0, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.4546, resultMatrix(0, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.4546, resultMatrix(1, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5403, resultMatrix(1, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.7080, resultMatrix(1, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(-0.8414, resultMatrix(2, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.0, resultMatrix(2, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5403, resultMatrix(2, 2), DOUBLE_THRESHOLD);
}

/*
 * Teste que la fonction "makeRotation" retourne bien la bonne matrice 3D de rotation lors d'une rotation sur les axes xyz
 */
TEST(TestLabo1, Math3D_makeRotation_XZYRotation)
{
	auto resultMatrix = makeRotation(1.0, 1.0, 1.0);

	// Expected
	// | 0.2919 -0.0720  0.9537|
	// | 0.4546  0.8877 -0.0720|
	// |-0.8414  0.4546  0.2919|
	EXPECT_NEAR(0.2919, resultMatrix(0, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-0.0720, resultMatrix(0, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.9537, resultMatrix(0, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.4546, resultMatrix(1, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.8877, resultMatrix(1, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-0.0720, resultMatrix(1, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(-0.8414, resultMatrix(2, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.4546, resultMatrix(2, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.2919, resultMatrix(2, 2), DOUBLE_THRESHOLD);
}

/*
 * Teste que la fonction "makeRotation" retourne bien la bonne matrice 3D de rotation lors d'une rotation avec des angles qui n'entrent pas dans un cercle
 */
TEST(TestLabo1, Math3D_makeRotation_OutisdeCircleRotation)
{
	auto resultMatrix = makeRotation(-1.0, 4.0, -1000.0);

	// Expected
	// |-0.3675  0.8049  0.4658|
	// | 0.5404 -0.2227  0.8113|
	// | 0.7568  0.5500 -0.3531|
	EXPECT_NEAR(-0.3675, resultMatrix(0, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.8049, resultMatrix(0, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.4658, resultMatrix(0, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.5404, resultMatrix(1, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-0.2227, resultMatrix(1, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.8113, resultMatrix(1, 2), DOUBLE_THRESHOLD);

	EXPECT_NEAR(0.7568, resultMatrix(2, 0), DOUBLE_THRESHOLD);
	EXPECT_NEAR(0.5500, resultMatrix(2, 1), DOUBLE_THRESHOLD);
	EXPECT_NEAR(-0.3531, resultMatrix(2, 2), DOUBLE_THRESHOLD);
}


/*
 * Teste que la m�thode "setIdentity" d'une matrice 4d en coordonn�es homog�nes retourne la bonne valeur
 */
TEST(TestLabo1, Math3D_Matrice4d_setIdentity_Ok)
{
	Matrix4d matrix{{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10 , 11, 12}, {13, 14, 15, 16}};
	matrix.setIdentity();

	for (auto i = 0; i < 4; ++i)
	{
		for (auto j = 0; j < 4; ++j)
		{
			if (i == j)
			{
				EXPECT_DOUBLE_EQ(1.0,  matrix(i, j));
			}
			else
			{
				EXPECT_DOUBLE_EQ(0.0, matrix(i, j));
			}
		}
	}
}