#pragma once

/**
 * @file Icp.h
 *
 * @brief Fonction utilisée par l'algorithme ICP.
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include "Math3D.h"

namespace gti320 
{

  /**
   * Type spécialisé pour les nuages de points.
   *
   * On stocke n points 3D dans une matrice de taille 3xn où chaque colonne
   * contient un point 3D.
   */
  typedef Matrix<double, 3, Dynamic, ColumnStorage> Points3d;

  /**
   * Calcul le déterminant d'une matrice 3x3.
   */
  static double determinant(const gti320::Matrix3d& matrix);

  /**
   * Classe Icp
   *
   * Conteient les fonction nécessaires à l'exécution de l'algorithme ICP.
   */
  class Icp
    {
  public:

    /**
     * Retourne l'index du point de points qui est le plus proche du point target.
     *
     * @param target un point
     * @param points un nuage de points
     * @return l'index du point de A le plus près de p
     */
    static int nearestNeighbor(const Vector3d& target, const Points3d& points);

    /**
     * Étant donné deux nuages de points source et destination, calcule la meilleure
     * transformation rigide pour aligner les deux nuages.
     *
     * @param source le nuage de points de référence
     * @param destination le nuage de points à aligner avec celui de référence
     * @return matrice de transformation rigide en coordonnées homogènes
     */
    static Matrix4d bestFitTransform(const Points3d& destination, const Points3d& source);


    /**
     * Calcul l'erreur entre deux nuages de points.
     *
     * L'erreur est la somme des distances entre le i-ième point de A et le
     * i-ième point de B.
     *
     * @param left nuage de points
     * @param right nuage de points
     * @return la somme des distances entre les points de A et ceux de B
     */
    static double error(const gti320::Points3d& left, const gti320::Points3d& right);

    };

}
