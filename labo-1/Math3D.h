#pragma once

/**
 * @file Math3D.h
 *
 * @brief Fonctions pour l'intinialisation et la manipulation de matrices de
 * rotation, des matrices de transformations en coordonnées homogènes et les
 * vecteurs 3D.
 * 
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include "Matrix.h"
#include "Vector.h"
#include "Operators.h"

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <math.h>
#include <iostream>


namespace gti320
{
	typedef Vector<double, 3> Vector3d;
	typedef Vector<float, 3> Vector3f;

	// Matrices en coordonnées homogènes :
	typedef Matrix<double, 4, 4, ColumnStorage> Matrix4d;
	typedef Matrix<float, 4, 4, ColumnStorage> Matrix4f;

	// Matrices pour les rotations
	typedef Matrix<double, 3, 3, ColumnStorage> Matrix3d;
	typedef Matrix<float, 3, 3, ColumnStorage> Matrix3f;


	/**
	 * Calcul de la matrice inverse, spécialisé pour le cas d'une matrice de
	 * transformation en coordonnées homogènes qui contient une matrice orthogonale en son centre.
	 */
	template <>
	inline Matrix4d Matrix4d::inverse() const
	{
		// L'inverse d'une matrice en coordonnées homogène est celle-ci:
		// M^(-1) = |R^(-1)  -R^(-1) * t|
		//          |   0          1    |
		//

		Matrix4d inversedMatrix(*this);

		// Ce vecteur correspond à t
		Vector<Scalar, 3> translationVector;
		translationVector.x() = inversedMatrix(0, 3);
		translationVector.y() = inversedMatrix(1, 3);
		translationVector.z() = inversedMatrix(2, 3);

		// Cette matrice correspond à R
		auto rotationSubmatrix = inversedMatrix.block(0, 0, 3, 3);

		// On calcule R^(-1) et on l'assigne à la bonne place dans la matrice
		Matrix<Scalar, 3, 3, ColumnStorage> transposedRotationMatrix = rotationSubmatrix.transpose<Scalar, 3, 3, ColumnStorage>();
		rotationSubmatrix = transposedRotationMatrix;

		// On calcule le vecteur -R^(-1) * t et on l'assigne à la bonne place dans la matrice
		auto newTranslationVector = -transposedRotationMatrix * translationVector;
		inversedMatrix(0, 3) = newTranslationVector.x();
		inversedMatrix(1, 3) = newTranslationVector.y();
		inversedMatrix(2, 3) = newTranslationVector.z();

		return inversedMatrix;
	}

	/**
	 * Calcul de la matrice inverse, spécialisé pour le cas d'une matrice de
	 * rotation.
	 */
	template <>
	inline Matrix3d Matrix3d::inverse() const
	{
		// L'inverse d'une matrice orthogonale, type de matrice dont les matrices de rotation font partie, est sa transposée.
		return this->transpose<Scalar, 3, 3, ColumnStorage>();
	}

	/**
	 * Multiplication d'une matrice 4x4 avec un vecteur 3D où la matrice
	 * représente une transformation en coordonnées homogène.
	 */
	template <typename Scalar>
	Vector<Scalar, 3> operator*(const Matrix<Scalar, 4, 4, ColumnStorage>& matrix, const Vector<Scalar, 3>& vector)
	{
		// On crée le vecteur en coordonnés homogènes afin de pouvoir le multiplier avec la matrice
		Vector<Scalar, 4> homogeneousVector;
		homogeneousVector.x() = vector.x();
		homogeneousVector.y() = vector.y();
		homogeneousVector.z() = vector.z();
		homogeneousVector.w() = 1;

		auto resultHomogeneousVector = matrix * homogeneousVector;

		// On retransforme le vecteur homogène en vecteur 3D
		Vector<Scalar, 3> resultVector;
		resultVector.x() = resultHomogeneousVector.x();
		resultVector.y() = resultHomogeneousVector.y();
		resultVector.z() = resultHomogeneousVector.z();

		return resultVector;
	}


	/**
	 * Initialise et retourne la matrice de rotation définie par les angles
	 * d'Euler XYZ exprimés en radians.
	 *
	 * La matrice correspond au produit : Rz*Ry*Rx.
	 */
	template <typename Scalar>
	static Matrix<Scalar, 3, 3> makeRotation(Scalar x, Scalar y, Scalar z)
	{
		Scalar cosTheta = cos(x);
		Scalar sinTheta = sin(x);

		Scalar cosPhi = cos(y);
		Scalar sinPhi = sin(y);

		Scalar cosPsi = cos(z);
		Scalar sinPsi = sin(z);

		Scalar sinThetaSinPhi = sinTheta * sinPhi;
		Scalar cosThetaSinPhi = cosTheta * sinPhi;

		// Matrice de rotation:
		// |cos(phi) * cos(psi) , -cos(theta) * sin(psi) + sin(theta) * sin(phi) * cos(psi) ,  sin(theta) * sin(psi) + cos(theta) * sin(phi) * cos(psi)|
		// |cos(phi) * sin(psi) ,  cos(theta) * cos(psi) + sin(theta) * sin(phi) * sin(psi) , -sin(theta) * cos(psi) + cos(theta) * sin(phi) * sin(psi)|
		// |     -sin(phi)      ,             sin(theta) * cos(phi)                         ,             cos(theta) * cos(phi)                        |
		Matrix<Scalar, 3, 3> rotationMatrix(3, 3);

		rotationMatrix(0, 0) = cosPhi * cosPsi;
		rotationMatrix(1, 0) = cosPhi * sinPsi;
		rotationMatrix(2, 0) = -sinPhi;

		rotationMatrix(0, 1) = -cosTheta * sinPsi + sinThetaSinPhi * cosPsi;
		rotationMatrix(1, 1) = cosTheta * cosPsi + sinThetaSinPhi * sinPsi;
		rotationMatrix(2, 1) = sinTheta * cosPhi;

		rotationMatrix(0, 2) = sinTheta * sinPsi + cosThetaSinPhi * cosPsi;
		rotationMatrix(1, 2) = -sinTheta * cosPsi + cosThetaSinPhi * sinPsi;
		rotationMatrix(2, 2) = cosTheta * cosPhi;
		
		return rotationMatrix;
	}

	/**
	 * Initialise et retourne la matrice identité
	 */
	template <>
	void Matrix4d::setIdentity()
	{
		this->setZero();
		this->m_storage[0] = 1;
		this->m_storage[5] = 1;
		this->m_storage[10] = 1;
		this->m_storage[15] = 1;
	}
}
