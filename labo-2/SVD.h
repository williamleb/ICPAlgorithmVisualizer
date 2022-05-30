#pragma once

/**
 * @file SVD.h
 *
 * @brief Singular value decomposition.
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include "Matrix.h"
#include <cassert>

#include <algorithm>
#include <cstdlib>
#include <cmath>

namespace gti320
{

    //
    //  An implementation of SVD from "Numerical Recipes in C". 
    //  http://numerical.recipes/webnotes/nr3web2.pdf
    // 
    template<typename Scalar>
    static inline Scalar sign(const Scalar a, const Scalar b)
    {
        return (b > 0.0) ? std::abs(a) : -std::abs(a);
    }

    template<typename Scalar>
    static inline Scalar sq(const Scalar a)
    {
        return (a * a);
    }

    // calculates sqrt( a^2 + b^2 ) 
    template<typename Scalar>
    static inline Scalar pythag(Scalar a, Scalar b)
    {
        return std::sqrt(a*a + b * b);
    }

    template<typename Scalar, int Rows = Dynamic, int Cols = Dynamic, int Storage = ColumnStorage>
    class SVD
    {
    public:

        explicit SVD(const Matrix<Scalar, Rows, Cols, Storage>& matrix) :
            uComponent(matrix), vComponent(matrix.cols(), matrix.cols()), sigmaComponent(matrix.cols())
        {
        }

        const Matrix<Scalar, Rows, Cols>& getU() const { return uComponent; }

        const Matrix<Scalar, Cols, Cols>& getV() const { return vComponent; }

        const Vector<Scalar, Cols>& getSigma() const { return sigmaComponent; }

        void decompose()
        {
            bool flag;
            int i, its, j, jj, k, l, nm;
            Scalar anorm, c, f, g, h, s, scale, x, y, z;

            const int ncols = uComponent.cols();
            const int nrows = uComponent.rows();
            Vector<Scalar> rv1(ncols);

            // Householder reduction to bidiagonal form
            //
            g = scale = anorm = 0.0;
            for (i = 0; i < ncols; ++i)
            {
                l = i + 1;
                rv1(i) = scale * g;
                g = s = scale = 0.0;
                if (i < nrows)
                {
                    for (k = i; k < nrows; ++k)
                    {
                        scale += std::abs(uComponent(k, i));
                    }

                    if (scale != 0.0)
                    {
                        for (k = i; k < nrows; ++k)
                        {
                            uComponent(k, i) /= scale;
                            s += uComponent(k, i) * uComponent(k, i);
                        }
                        f = uComponent(i, i);
                        g = -sign(std::sqrt(s), f);
                        h = f * g - s;
                        uComponent(i, i) = f - g;
                        for (j = l; j < ncols; ++j)
                        {
                            for (s = 0.0, k = i; k < nrows; ++k)
                                s += uComponent(k, i) * uComponent(k, j);
                            f = s / h;
                            for (k = i; k < nrows; ++k)
                                uComponent(k, j) += f * uComponent(k, i);
                        }
                        for (k = i; k < nrows; ++k)
                            uComponent(k, i) *= scale;
                    }
                }
                sigmaComponent(i) = scale * g;
                g = s = scale = 0.0;
                if (i < nrows && i != ncols - 1)
                {
                    for (k = l; k < ncols; ++k)
                        scale += std::abs(uComponent(i, k));

                    if (scale)
                    {
                        for (k = l; k < ncols; ++k)
                        {
                            uComponent(i, k) /= scale;
                            s += uComponent(i, k) * uComponent(i, k);
                        }
                        f = uComponent(i, l);
                        g = -sign(std::sqrt(s), f);
                        h = f * g - s;
                        uComponent(i, l) = f - g;
                        for (k = l; k < ncols; ++k)
                        {
                            rv1(k) = uComponent(i, k) / h;
                        }
                        for (j = l; j < nrows; ++j)
                        {
                            for (s = 0.0, k = l; k < ncols; ++k)
                                s += uComponent(j, k) * uComponent(i, k);
                            for (k = l; k < ncols; ++k)
                                uComponent(j, k) += s * rv1(k);
                        }
                        for (k = l; k < ncols; ++k)
                            uComponent(i, k) *= scale;
                    }
                }
                const Scalar tmp = (std::abs(sigmaComponent(i)) + std::abs(rv1(i)));
                anorm = std::max(anorm, tmp);

                //printf(".");
                //fflush(stdout);
            }

            // Accumulation of right-hand transformations
            //
            for (i = ncols - 1; i >= 0; --i)
            {
                if (i < ncols - 1)
                {
                    if (g) {
                        for (j = l; j < ncols; ++j)
                            vComponent(j, i) = (uComponent(i, j) / uComponent(i, l)) / g;  // double division to avoid possible underflow

                        for (j = l; j < ncols; ++j)
                        {
                            for (s = 0.0, k = l; k < ncols; ++k)
                                s += uComponent(i, k) * vComponent(k, j);
                            for (k = l; k < ncols; ++k)
                                vComponent(k, j) += s * vComponent(k, i);
                        }
                    }
                    for (j = l; j < ncols; ++j)
                        vComponent(i, j) = vComponent(j, i) = 0.0;
                }
                vComponent(i, i) = 1.0;
                g = rv1(i);
                l = i;
                //printf(".");
                //fflush(stdout);
            }

            // Accumulation of left-hand transformations
            //
            for (i = std::min(nrows, ncols) - 1; i >= 0; --i)
            {
                l = i + 1;
                g = sigmaComponent(i);
                for (j = l; j < ncols; ++j)
                    uComponent(i, j) = 0.0;
                if (g)
                {
                    g = 1.0 / g;
                    for (j = l; j < ncols; ++j)
                    {
                        for (s = 0.0, k = l; k < nrows; ++k)
                            s += uComponent(k, i) * uComponent(k, j);
                        f = (s / uComponent(i, i)) * g;
                        for (k = i; k < nrows; ++k)
                            uComponent(k, j) += f * uComponent(k, i);
                    }
                    for (j = i; j < nrows; ++j)
                        uComponent(j, i) *= g;
                }
                else
                    for (j = i; j < nrows; ++j)
                        uComponent(j, i) = 0.0;
                uComponent(i, i) = uComponent(i, i) + 1.0;
                //printf(".");
                //fflush(stdout);
            }

            // Diagonalization of the bidiagonal form.
            // Loops over singular values using iterations.
            //
            for (k = ncols - 1; k >= 0; --k)
            {
                // Max iterations: 30
                for (its = 0; its < 30; ++its)
                {
                    flag = true;
                    for (l = k; l >= 0; --l)     // Test for splitting
                    {
                        nm = l - 1;
                        if ((std::abs(rv1(l)) + anorm) == anorm)
                        {
                            flag = false;
                            break;
                        }
                        if (std::abs(sigmaComponent(nm) + anorm) == anorm)
                            break;
                    }

                    if (flag)
                    {                           // Cancellation of rv1(l) if l > 0
                        c = 0.0;
                        s = 1.0;
                        for (i = l; i <= k; ++i)
                        {
                            f = s * rv1(i);
                            rv1(i) = c * rv1(i);
                            if ((std::abs(f) + anorm) == anorm)
                                break;

                            g = sigmaComponent(i);
                            h = pythag(f, g);
                            sigmaComponent(i) = h;
                            h = 1.0 / h;
                            c = g * h;
                            s = -f * h;
                            for (j = 0; j < nrows; ++j)
                            {
                                y = uComponent(j, nm);
                                z = uComponent(j, i);
                                uComponent(j, nm) = y * c + z * s;
                                uComponent(j, i) = z * c - y * s;
                            }
                        }
                    }
                    z = sigmaComponent(k);
                    if (l == k)         // Convergence.
                    {
                        if (z < 0.0)                // Compute non-negative singular values
                        {
                            sigmaComponent(k) = -z;
                            for (j = 0; j < ncols; ++j)
                            {
                                vComponent(j, k) = -vComponent(j, k); // Reverse bases direction 
                            }
                        }
                        break;
                    }

                    // Assertion when max iterations reached without convergence
                    assert(its < 29);

                    x = sigmaComponent(l);                     // Shift from bottom 2-by-2 minor
                    nm = k - 1;
                    y = sigmaComponent(nm);
                    g = rv1(nm);
                    h = rv1(k);
                    f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
                    g = pythag(f, 1.0);
                    f = ((x - z) * (x + z) + h * ((y / (f + sign(g, f))) - h)) / x;
                    c = s = 1.0;                    // Next QR transformation
                    for (j = l; j <= nm; ++j)
                    {
                        i = j + 1;
                        g = rv1(i);
                        y = sigmaComponent(i);
                        h = s * g;
                        g = c * g;
                        z = pythag(f, h);
                        rv1(j) = z;
                        c = f / z;
                        s = h / z;
                        f = x * c + g * s;
                        g = g * c - x * s;
                        h = y * s;
                        y *= c;
                        for (jj = 0; jj < ncols; ++jj)
                        {
                            x = vComponent(jj, j);
                            z = vComponent(jj, i);
                            vComponent(jj, j) = x * c + z * s;
                            vComponent(jj, i) = z * c - x * s;
                        }
                        z = pythag(f, h);
                        sigmaComponent(j) = z;                 // Rotation can be arbitrary if z == 0
                        if (z != 0.0)
                        {
                            z = 1.0 / z;
                            c = f * z;
                            s = h * z;
                        }
                        f = c * g + s * y;
                        x = c * y - s * g;
                        for (jj = 0; jj < nrows; ++jj)
                        {
                            y = uComponent(jj, j);
                            z = uComponent(jj, i);
                            uComponent(jj, j) = y * c + z * s;
                            uComponent(jj, i) = z * c - y * s;
                        }
                    }
                    rv1(l) = 0.0;
                    rv1(k) = f;
                    sigmaComponent(k) = x;
                }
                //printf(".");
                //fflush(stdout);
            }
            //printf("\n");

            reorder();
        }

    private:

        void reorder() {
            int s, inc = 1;
            Scalar sw;

            const int ncols = uComponent.cols();
            const int nrows = uComponent.rows();
            Vector<Scalar> su(nrows), sv(ncols);
            do { inc *= 3; inc++; } while (inc <= ncols);   // Sort using Shell�s sort.

            do {
                inc /= 3;
                for (int i = inc; i < ncols; ++i) {
                    sw = sigmaComponent(i);
                    for (int k = 0; k < nrows; ++k) su(k) = uComponent(k, i);
                    for (int k = 0; k < ncols; ++k) sv(k) = vComponent(k, i);
                    int j = i;
                    while (sigmaComponent(j - inc) < sw) {
                        sigmaComponent(j) = sigmaComponent(j - inc);
                        for (int k = 0; k < nrows; ++k) uComponent(k, j) = uComponent(k, j - inc);
                        for (int k = 0; k < ncols; ++k) vComponent(k, j) = vComponent(k, j - inc);
                        j -= inc;
                        if (j < inc) break;
                    }
                    sigmaComponent(j) = sw;
                    for (int k = 0; k < nrows; ++k) uComponent(k, j) = su(k);
                    for (int k = 0; k < ncols; ++k) vComponent(k, j) = sv(k);
                }
            } while (inc > 1);

            for (int k = 0; k < ncols; ++k)
            {
                // Flip signs.
                s = 0;
                for (int i = 0; i < nrows; ++i) if (uComponent(i, k) < 0.0) ++s;
                for (int j = 0; j < ncols; ++j) if (vComponent(j, k) < 0.0) ++s;
                if (s > (nrows + ncols) / 2)
                {
                    for (int i = 0; i < nrows; ++i) uComponent(i, k) = -uComponent(i, k);
                    for (int j = 0; j < ncols; ++j) vComponent(j, k) = -vComponent(j, k);
                }
            }
        }


        Matrix<Scalar, Rows, Cols, Storage> uComponent;
        Matrix<Scalar, Cols, Cols> vComponent;
        Vector<Scalar, Cols> sigmaComponent;
    };

}