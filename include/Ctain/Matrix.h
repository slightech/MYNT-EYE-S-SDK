//
// Created by 顾涵彬 on 2019-08-28.
//

#ifndef MATRIX_MATRIX_H
#define MATRIX_MATRIX_H

#include <cstring>
#include <iostream>
#include <cmath>

namespace Ctain {
    template<typename _Scalar>
    class Matrix {
    public:
        Matrix(int Rows, int Cols) :
        _Rows(Rows), _Cols(Cols), _isSub(0), input_id(0) {
            _startRow = 0;
            _startCol = 0;
            _Rows_raw = Rows;
            _Cols_raw = Cols;
            data = new _Scalar [_Rows * _Cols];
            memset(data, 0, _Rows * _Cols * sizeof(_Scalar));
        }
        Matrix() :
        _Rows(0), _Cols(0), _isSub(0), input_id(0) {
            _startRow = 0;
            _startCol = 0;
            _Rows_raw = 0;
            _Cols_raw = 0;
        }
        Matrix(_Scalar _data[], int Rows, int Cols) :
        _Rows(Rows), _Cols(Cols), _isSub(0), input_id(0) {
            _startRow = 0;
            _startCol = 0;
            _Rows_raw = Rows;
            _Cols_raw = Cols;
            data = new _Scalar [_Rows * _Cols];
            memcpy(data, _data, _Rows * _Cols * sizeof(_Scalar));
        }

        Matrix(_Scalar **_data, int Rows, int Cols) :
        _Rows(Rows), _Cols(Cols), _isSub(0), input_id(0) {
            _startRow = 0;
            _startCol = 0;
            _Rows_raw = Rows;
            _Cols_raw = Cols;
            data = new _Scalar [_Rows * _Cols];
            for (int i = 0; i < _Rows; ++i) {
                memcpy(data + i * _Cols, *(_data + i), _Cols * sizeof(_Scalar));
            }
        }

        template<typename T>
        Matrix<T> cast() {
            Matrix<T> res(_Rows, _Cols);
            for (int i = 0; i < _Rows; i++) {
                for (int j = 0; j < _Cols; j++) {
                    res(i,j) = cData(i, j);
                }
            }
            return res;
        }

        void setIdentity() {
            for (int i = 0; i < _Rows; i++) {
                for (int j = 0; j < _Cols; j++) {
                    if(i == j) {
                        Data(i,j) = 1;
                    }else {
                        Data(i,j) = 0;
                    }
                }
            }
        }
        void setZero() {
            for (int i = 0; i < _Rows; i++) {
                for (int j = 0; j < _Cols; j++) {
                    Data(i, j) = 0;
                }
            }
        }
        inline int cols() const { return _Cols; }

        inline int rows() const { return _Rows; }

        inline int size() const { return cols() * rows(); }
        _Scalar * addr() {
            return data;
        }

        friend Matrix<_Scalar> &operator <<(Matrix<_Scalar> &m,_Scalar val) {
            m.Data(m.input_id++) = val;
            return m;
        }


        friend std::ostream &operator <<(std::ostream &os,const Matrix<_Scalar> &m) {
//            os << std::endl;
            for (int i = 0; i < m._Rows; i++) {
                for (int j = 0; j < m._Cols - 1; j++) {
                    std::cout.width(10);
                    std::cout.setf(std::ios::left);
                    std::cout.precision(6);
                    os << m.cData(i,j) ;
                }
                std::cout.width(9);
                std::cout.setf(std::ios::left);
                std::cout.precision(8);
                os << m.cData(i,m._Cols - 1) << std::endl;
            }
            return os;
        }

        friend Matrix<_Scalar> operator *(
            double a, const Matrix<_Scalar> &m) {
            Matrix<_Scalar> res;
            res = m;
            for(int i = 0; i < m._Rows; i++) {
                for(int j = 0; j < m._Cols; j++) {
                    res.Data(i,j) *= a;
                }
            }
            return res;
        }

        friend Matrix<_Scalar> operator -(
            const Matrix<_Scalar> &m) {
            Matrix<_Scalar> res;
            res = m;
            for(int i = 0; i < m._Rows; i++) {
                for(int j = 0; j < m._Cols; j++) {
                    res.Data(i,j) *= -1;
                }
            }
            return res;
        }

        friend Matrix<_Scalar> operator +(
            double a, const Matrix<_Scalar> &m) {
            Matrix<_Scalar> res;
            res = m;
            for(int i = 0; i < m._Rows; i++) {
                for(int j = 0; j < m._Cols; j++) {
                    res.Data(i,j) += a;
                }
            }
            return res;
        }        

        void operator =(Matrix<_Scalar> m);
//        void operator =(Matrix<_Scalar> &m);
        Matrix<_Scalar> operator +(const Matrix<_Scalar> &m) const;
        Matrix<_Scalar> operator -(const Matrix<_Scalar> &m) const;
        Matrix<_Scalar> operator *(const Matrix<_Scalar> &m) const;
        
        Matrix<_Scalar> operator /(double m) const;
        _Scalar &operator()(int i, int j) {
            return Data(i,j);
        }
        _Scalar &operator()(int id) {
            return Data(id);
        }
        _Scalar operator()(int id) const {
            return cData(id);
        }
        Matrix<_Scalar> transpose() const;

        Matrix<_Scalar> col(int Col) {
            return block(0, Col, _Rows, 1);
        }
        Matrix<_Scalar> row(int Row) {
            return block(Row, 0, 1, _Cols);
        }
        Matrix<_Scalar> block(int sRow, int sCol, int Rows, int Cols) {
            Matrix<_Scalar> sub;
            sub = *this;
            sub.setSub(sRow, sCol, Rows, Cols, data);
            return sub;
        }

        template<int Rows, int Cols>
        Matrix<_Scalar> topLeftCorner() const {
            Matrix<_Scalar> sub;
            sub = *this;
            sub.setSub(0, 0, Rows, Cols, data);
            return sub;
        }

        template<int Rows, int Cols>
        Matrix<_Scalar> topRightCorner() const {
            Matrix<_Scalar> sub;
            sub = *this;
            sub.setSub(_Rows-1-Rows, _Cols-1-Cols, Rows, Cols, data);
            return sub;
        }

        void setSub(int sRow, int sCol, int Rows, int Cols, _Scalar *Data) {
            _isSub = true;
            _Rows_raw = _Rows;
            _Cols_raw = _Cols;
            _Rows = Rows;
            _Cols = Cols;
            _startRow = sRow;
            _startCol = sCol;
            data = Data;
        }

        void normalize();
        double norm() const;

        virtual ~Matrix() {
            if(!data)
                delete[] data;
        }

//        template<int _Rows, int _Cols>
//        inline Matrix<_Scalar, _Rows, _Cols> block<_Rows, _Cols>(Index i,Index j) {
//
//        }
        inline _Scalar *Data() {
            return data;
        }
    protected:
        _Scalar *data;
        int _Rows;
        int _Cols;
        bool _isSub;
        int _startRow;
        int _startCol;
        int _Rows_raw;
        int _Cols_raw;
        int input_id ;
        inline int id(int i, int j) const {
            if(_isSub)
                return (i + _startRow) * _Cols_raw + j + _startCol;
            else
                return i * _Cols + j;
        }
        inline _Scalar &Data(int i,int j) {
            return data[id(i,j)];
        }
        inline _Scalar &Data(int id) {
            int i = id / _Cols;
            int j = id % _Cols;
            int index;
            if(_isSub)
                index = (i + _startRow) * _Cols_raw + j + _startCol;
            else
                index = i * _Cols + j;
            return data[index];
        }
        inline _Scalar cData(int id) const{
            int i = id / _Cols;
            int j = id % _Cols;
            int index;
            if(_isSub)
                index = (i + _startRow) * _Cols_raw + j + _startCol;
            else
                index = i * _Cols + j;
            return data[index];
        }
        inline _Scalar cData (int i,int j) const{
            return data[id(i,j)];
        }

    }; //class Matrix end

    template<typename _Scalar>
    void Matrix<_Scalar>::operator =(Matrix<_Scalar> m) {
        if(m._isSub) {
            _isSub = true;
            _Rows = m._Rows;
            _Cols = m._Cols;
            _Rows_raw = m._Rows_raw;
            _Cols_raw = m._Cols_raw;
            _startRow = m._startRow;
            _startCol = m._startCol;
            data = m.Data();
            return;
        }
        if(!_isSub) {
            if(size() != m.size()) {
                if(size() > 0) {
                    delete[] data;
                }
                _Rows = m._Rows;
                _Cols = m._Cols;
                data = new _Scalar[_Rows * _Cols];
            }else {
                _Rows = m._Rows;
                _Cols = m._Cols;
            }
        }

        for (int i = 0; i < m._Rows; i++) {
            for (int j = 0; j < m._Cols; j++) {
                Data(i,j) = m.cData(i, j);
            }
        }
    }

    template<typename _Scalar>
    Matrix<_Scalar> Matrix<_Scalar>::operator +(const Matrix<_Scalar> &m) const{
        Matrix<_Scalar> sum;
        sum = *this;
        for(int i = 0; i < _Rows * _Cols; i++) {
            sum.data[i] += m.data[i];
        }
        return sum;
    }

    template<typename _Scalar>
    Matrix<_Scalar> Matrix<_Scalar>::operator -(const Matrix<_Scalar> &m) const{
        Matrix<_Scalar> sum;
        sum = *this;
        for(int i = 0; i < _Rows * _Cols; i++) {
            sum.data[i] -= m.data[i];
        }
        return sum;
    }

    template<typename _Scalar>
    Matrix<_Scalar> Matrix<_Scalar>::transpose() const
    {
        Matrix<_Scalar> res(_Cols, _Rows);
        for(int i = 0; i < _Rows; i++) {
            for (int j = 0; j < _Cols; j++) {
                res.Data(j, i) = cData(i, j);
            }
        }
        return res;
    }

    template<typename _Scalar>
    Matrix<_Scalar> Matrix<_Scalar>::operator *(const Matrix<_Scalar> &m) const {
        if(_Cols != m._Rows) {
            //todo:output err
            return m;
        }
        Matrix<_Scalar> res(_Rows, m._Cols);
        for(int i = 0; i < _Rows; i++) {
            for(int j = 0; j < m._Cols; j++) {
                int sum = 0;
                for(int k = 0; k < _Cols; k++) {
                    sum += cData(i, k) * m.cData(k, j);
                }
                res.Data(i,j) = sum;
            }
        }
        return res;
    }

    template<typename _Scalar>
    Matrix<_Scalar> Matrix<_Scalar>::operator /(double m) const {
        Matrix<_Scalar> res(_Rows, _Cols);
        for(int i = 0; i < _Rows; i++) {
            for(int j = 0; j < _Cols; j++) {
                res.Data(i,j) /= m;
            }
        }
        return res;
    }


    template<typename _Scalar>
    void Matrix<_Scalar>::normalize() {
        double sum = 0;
        for(int i = 0; i < _Rows; i++) {
            for(int j = 0; j < _Cols; j++) {
                sum += Matrix::cData(i, j);
            }
        }
        sum = sqrt(sum);
        for(int i = 0; i < _Rows; i++) {
            for(int j = 0; j < _Cols; j++) {
                Matrix::Data(i, j) /= sum;
            }
        }
    }

    template<typename _Scalar>
    double Matrix<_Scalar>::norm() const{
        double sum = 0;
        for(int i = 0; i < _Rows; i++) {
            for(int j = 0; j < _Cols; j++) {
                sum += Matrix::cData(i, j);
            }
        }
        sum = sqrt(sum);

        return sum;
    }
} //namespace Ctain end
#endif //MATRIX_MATRIX_H