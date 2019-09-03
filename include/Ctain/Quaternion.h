//
// Created by 顾涵彬 on 2019-08-30.
//

#ifndef MATRIX_QUATERNION_H
#define MATRIX_QUATERNION_H

#include "SquareMatrix.h"
#include <cmath>
namespace Ctain {
//    using SMatrix<double>;
    template<typename T>
    class Quaternion {
    public:
        Quaternion(){}
        Quaternion(SMatrix<double> m){
            //double f = 1e-10;
            _w = sqrt(m(0)+m(4)+m(8)+1)/2;
            _x = (m(5)-m(7))/(4*_w);
            _y = (m(6)-m(2))/(4*_w);
            _z = (m(1)-m(3))/(4*_w);
        }
        Quaternion(T X,T Y,T Z,T W) : _x(X),_y(Y),_z(Z),_w(W){}
        void normalize() {
            double len;
            len = sqrt(_x*_x+_y*_y+_z*_z+_w*_w);
            _x = _x / len;
            _y = _y / len;
            _z = _z / len;
            _w = _w / len;
        }
        T x(){return _x;}
        T y(){return _y;}
        T z(){return _z;}
        T w(){return _w;}
        SMatrix<double> toRotationMatrix() const {
            SMatrix<double> r(3);
            double q0=_w,q1=_x,q2=_y,q3=_z;
            r(0) = 1 - 2*q2*q2-2*q3*q3;
            r(1) = 2*q1*q2+2*q0*q3;
            r(2) = 2*q1*q3-2*q0*q2;
            r(3) = 2*q1*q2 - 2*q0*q3;
            r(4) = 1-2*q1*q1 - 2*q3*q3;
            r(5) = 2*q2*q3 +2*q0*q1;
            r(6) = 2*q1*q3+2*q0*q2;
            r(7) = 2*q2*q3 - 2*q0*q1;
            r(8) = 1-2*q1*q1 -2*q2*q2;
            return r;
        }
    private:
        T _x;
        T _y;
        T _z;
        T _w;
    };
}
#endif //MATRIX_QUATERNION_H
