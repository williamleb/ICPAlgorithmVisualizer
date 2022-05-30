/**
 * @file Icp.cpp
 *
 * @brief Fonctions utilisées par l'algorithme ICP.
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include "Icp.h"

#include "Matrix.h"
#include "Vector.h"
#include "Operators.h"
#include "SVD.h"

using namespace gti320;

/**
 * Calcul du déterminant 3x3
 */
double gti320::determinant(const Matrix3d& matrix)
{
	// Pour une matrice
	// |a, b, c|	  |(0,0) (0,1) (0,2)|
	// |d, e, f|  =>  |(1,0) (1,1) (1,2)|
	// |g, h, i|	  |(2,0) (2,1) (2,2)|
	// son déterminant va être: aei + bfg + cdh - ceg - bdi - afh

	return matrix(0, 0) * matrix(1, 1) * matrix(2, 2) // aei
		+ matrix(0, 1) * matrix(1, 2) * matrix(2, 0)  // bfg
		+ matrix(0, 2) * matrix(1, 0) * matrix(2, 1)  // cdh
		- matrix(0, 2) * matrix(1, 1) * matrix(2, 0)  // ceg
		- matrix(0, 1) * matrix(1, 0) * matrix(2, 2)  // bdi
		- matrix(0, 0) * matrix(1, 2) * matrix(2, 1); // afh
}

/**
 * Calcul de l'erreur entre deux nuages de points
 */
double Icp::error(const Points3d& left, const Points3d& right)
{
	ASSERT(left.cols() == right.cols(),
	       "We are trying to compute the error between two point clouds with a different number of points, which is cannot be done with this version of the algorithm")
	;

	Points3d differenceMatrix = right - left;

	double error = 0.0;
	for (auto j = 0; j < differenceMatrix.cols(); ++j)
	{
		// Here, we don't convert every column of the matrix to vectors in order to call the "magnitude" function because it would require to copy the memory
		// of every column into a new object, which would take more time.
		double magnitudeSquared = 0.0;
		for (auto i = 0; i < Points3d::rows(); ++i)
		{
			magnitudeSquared += differenceMatrix(i, j) * differenceMatrix(i, j);
		}
		error += sqrt(magnitudeSquared);
	}

	return error;
}


/**
 * Index du point de points qui minimise la distance à target
 */
int Icp::nearestNeighbor(const Vector3d& target, const Points3d& points)
{
	int nearestNeighbor = 0;
	double minSquaredDistance = std::numeric_limits<double>::max();

	for (auto j = 0; j < points.cols(); ++j)
	{
		Vector3d point = points.col(j);

		double squaredDistance = point.squaredDistance(target);
		if (squaredDistance < minSquaredDistance)
		{
			nearestNeighbor = j;
			minSquaredDistance = squaredDistance;
		}
	}

	return nearestNeighbor;
}

/**
 * Meilleure transformation rigide pour amener les points de source sur ceux destination
 */
Matrix4d Icp::bestFitTransform(const Points3d& destination, const Points3d& source)
{
	// Matrices center(A) et center(B)
	Points3d meanSourceMatrix = source.meanMatrix();
	Points3d meanDestinationMatrix = destination.meanMatrix();

	// AA = A - center(A)
	// BB = B - center(B)
	Points3d centeredSourceMatrix = source - meanSourceMatrix;
	Points3d centeredDestinationMatrix = destination - meanDestinationMatrix;

	// U,V = svd(AA*BB.transpose());
	SVD svd(centeredSourceMatrix * centeredDestinationMatrix.transpose());
	svd.decompose();
	Matrix3d svdUComponent = svd.getU();
	Matrix3d svdVComponent = svd.getV();

	// R = V * U.transpose()
	Matrix3d rotationMatrix = svdVComponent * svdUComponent.transpose();

	// Si le déterminant est -1, alors on inverse la troisième colonne 
	if (determinant(rotationMatrix) < 0)
	{
		svdVComponent(0, 2) = -svdVComponent(0, 2);
		svdVComponent(1, 2) = -svdVComponent(1, 2);
		svdVComponent(2, 2) = -svdVComponent(2, 2);

		rotationMatrix = svdVComponent * svdUComponent.transpose();
	}

	// t = center(B) - R * center(A)
	Vector3d meanSourceVector = meanSourceMatrix.col(0);
	Vector3d meanDestinationVector = meanDestinationMatrix.col(0);
	Vector3d translationVector = meanDestinationVector - rotationMatrix * meanSourceVector;

	// On crée la matrice homogène avec la rotation et la translation
	Matrix4d bestFitTransform;
	bestFitTransform.setIdentity();

	bestFitTransform.block(0, 0, 3, 3) = rotationMatrix;

	bestFitTransform(0, 3) = translationVector.x();
	bestFitTransform(1, 3) = translationVector.y();
	bestFitTransform(2, 3) = translationVector.z();

	return bestFitTransform;
}
