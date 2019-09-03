//
// Created by 顾涵彬 on 2019-08-28.
//

#include <cstdint>
#include "Matrix.h"
#include "SquareMatrix.h"
#include "MatrixSolver.h"
#include "Quaternion.h"
#ifndef MATRIX_CTAIN_H
#define MATRIX_CTAIN_H


namespace Ctain {
    typedef SMatrix<double> Matrixd;
    typedef Matrix<double> MatrixXd;
    typedef Matrix<double> Matrix23d;
    typedef SMatrix<double> Matrix33d;

    typedef SMatrix<float> Matrixf;
    typedef Matrixf Matrix2f;
    typedef Matrixf Matrix3f;

    typedef Matrix<float> Vectorf;
    typedef Vectorf Vector2f;
    typedef Vectorf Vector3f;

    typedef Matrix<double> Vectord;
    typedef Matrix<double> Vector2d;
    typedef Matrix<double> Vector3d;
    typedef Matrix<double> MatrixXcd;

    typedef Quaternion<double> Quaterniond;
} // end namespace Ctain

#endif // Ctain_CtainBASE_H


