//
// Created by 顾涵彬 on 2019-08-29.
//

#ifndef MATRIX_SQUAREMATRIX_H
#define MATRIX_SQUAREMATRIX_H
#include "Matrix.h"
namespace Ctain {
#define Matrix Matrix<_Scalar>
    template<typename _Scalar>
    class SMatrix: public Matrix{
    public:
        SMatrix(int D) : Matrix(D, D) {}
        SMatrix() : Matrix(0, 0) {}
        SMatrix(_Scalar _data[], int D) :
                Matrix(_data, D, D) {}
        SMatrix(_Scalar **_data, int D) :
                Matrix(_data, D, D) {}
        SMatrix(Matrix m) :
                Matrix(m) {}
     //   void operator =(const Matrix &m){
   //     }
        _Scalar determinant();
        _Scalar M(int m, int n);
        SMatrix<_Scalar> inverse() {
            SMatrix<_Scalar> res(Matrix::_Rows);
            _Scalar d = determinant();
            for(int i = 0; i < Matrix::_Rows; i++) {
                for (int j = 0; j < Matrix::_Cols; j++) {
                    res.Data(j, i) = 1.0*M(i, j)/d;
                }
            }
            return res;

        }


    };//class Matrix end

    template<typename _Scalar>
    _Scalar SMatrix<_Scalar>::determinant() {
        int r, c, m;
        int lop = 0;
        int n = Matrix::_Rows;
        _Scalar result = 0;
        _Scalar mid = 1;
        if (n != 1) {
            lop = (n == 2) ? 1 : n;
            for (m = 0; m < lop; m++) {
                mid = 1;
                for (r = 0, c = m; r < n; r++, c++) {
                    mid = mid * (*(Matrix::data+r*n+c%n));
                }
                result += mid;
            }
            for (m = 0; m < lop; m++) {
                mid = 1;
                for (r = 0, c = n-1-m+n; r < n; r++, c--) {
                    mid = mid * (*(Matrix::data+r*n+c%n));
                }
                result -= mid;
            }
        }
        else
            result = Matrix::data[0];
        return result;
    }

    template<typename _Scalar>
    _Scalar SMatrix<_Scalar>::M(int m, int n) {
        float mid_result = 0;
        int sign = 1;
        int k = Matrix::_Rows;
        SMatrix mid(k-1);
        int c = 0;
        for (int i = 0; i < k; i++) {
            for (int j = 0; j < k; j++) {
                if (i != m && j != n)
                {
                    mid.Data(c++) = Matrix::cData(i,j);
                }
            }
        }
        sign = (m+n)%2 == 0 ? 1 : -1;
        mid_result = (float)sign*mid.determinant();
        return mid_result;
    }
#undef Matrix

}//namespace Ctain end
#endif //MATRIX_SQUAREMATRIX_H
