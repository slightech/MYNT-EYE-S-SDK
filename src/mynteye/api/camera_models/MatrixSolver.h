//
// Created by 顾涵彬 on 2019-08-30.
//

#ifndef SRC_MYNTEYE_API_CAMERA_MODELS_MATRIXSOLVER_H_
#define SRC_MYNTEYE_API_CAMERA_MODELS_MATRIXSOLVER_H_
#include <cmath>
#include <complex>

static bool Matrix_EigenValue(double *K1, int n,
    int LoopNumber, double Error1, double *Ret);
static void Matrix_Hessenberg(double *A1, int n, double *ret);

namespace Ctain {
class EigenSolver {
 public:
  explicit EigenSolver(SMatrix<double> s) {
    double *A = new double[s.rows()*2];
    double *B = new double[s.size()];
    for (int i = 0; i < s.size(); i++)
      B[i] = s(i);
    memset(A, 0, sizeof(s.rows()*2));
    Matrix_EigenValue(B, s.rows(), 1000, 1e-10, A);
    Matrix<double> tt(A, s.rows(), 2);
    t = tt;
    SMatrix<double> s2(A, s.rows());
    delete []A;
    delete []B;
}
  Matrix<double> eigenvalues() {
    return t;
  }

 private:
  Matrix<double> t;
};

}  // namespace Ctain

static void Matrix_Hessenberg(double *A1, int n, double *ret) {
  int MaxNumber;
  double temp, *A;
  A = new double[n*n];
  memset(A, 0, sizeof(double)*n*n);
  for (int i = 0; i < n; i++) {
    int k = i * n;
    for (int j = 0; j < n; j++) {
      A[k + j] = A1[k + j];
    }
  }
  for (int k = 1; k < n-1; k++) {
    int i = k-1;
    MaxNumber = k;
    temp = fabs(A[k*n+i]);
    for (int j = k+1; j < n; j++) {
      if (fabs(A[j * n + i]) > temp) {
        temp = fabs(A[j * n + i]);
        MaxNumber = j;
      }
    }
    ret[0] = A[MaxNumber * n + i];
    if (ret[0] != 0) {
      if (MaxNumber != k) {
        for (int j = k-1; j < n; j++) {
          temp = A[i * n + j];
          A[i * n + j] = A[k * n + j];
          A[k * n + j] = temp;
        }
        for (int j = 0; j < n; j++) {
          temp = A[j * n + i];
          A[j * n + i] = A[j * n + k];
          A[j * n + k] = temp;
        }
      }
      for (int i = k + 1; i < n; i++) {
        temp = A[i * n + k - 1] / ret[0];
        A[i * n + k - 1] = 0;
        for (int j = k; j < n; j++) {
          A[i * n + j] -= temp * A[k * n + j];
        }
        for (int j = 0; j < n; j++) {
          A[j * n + k] += temp * A[j * n + i];
        }
      }
    }
  }
  for (int i = 0; i < n; i++) {
    int k = i * n;
    for (int j = 0; j < n; j++) {
      ret[k + j] = A[k + j];
    }
  }
  delete []A;
}

static bool Matrix_EigenValue(double *K1, int n,
    int LoopNumber, double Error1, double *Ret) {
  int i, j, k, t, m, Loop1;
  double b, c, d, g, xy, p, q, r, x, s, e, f, z, y, temp, *A;
  A = new double[n * n];
  memset(A, 0, sizeof(double) * n * n);
  Matrix_Hessenberg(K1, n, A);
  m = n;
  Loop1 = LoopNumber;
  while (m != 0) {
    t = m - 1;
    while (t > 0) {
      temp = fabs(A[(t - 1) * n + t - 1]);
      temp += fabs(A[t * n + t]);
      temp = temp * Error1;
      if (fabs(A[t * n + t - 1]) > temp) {
        t--;
      } else {
        break;
      }
    }
    if (t == m-1) {
      Ret[(m - 1) * 2] = A[(m - 1) * n + m - 1];
      Ret[(m - 1) * 2 + 1] = 0;
      m -= 1;
      Loop1 = LoopNumber;
    } else if (t == m - 2) {
      b = -A[(m - 1) * n + m - 1] - A[(m - 2) * n + m - 2];
      c = A[(m - 1) * n + m - 1] * A[(m - 2) * n + m - 2]
          - A[(m - 1) * n + m - 2] * A[(m - 2) * n + m - 1];
      d = b * b - 4 * c;
      y = sqrt(fabs(d));
      if (d > 0) {
        xy = 1;
        if (b < 0) {
          xy = -1;
        }
        Ret[(m - 1) * 2] = -(b + xy * y) / 2;
        Ret[(m - 1) * 2 + 1] = 0;
        Ret[(m - 2) * 2] = c / Ret[(m - 1) * 2];
        Ret[(m - 2) * 2 + 1] = 0;
      } else {
        Ret[(m - 1) * 2] = -b / 2;
        Ret[(m - 2) * 2] = -b / 2;
        Ret[(m - 1) * 2 + 1] = y / 2;
        Ret[(m - 2) * 2 + 1] = -y / 2;
      }
      m -= 2;
      Loop1 = LoopNumber;
    } else {
      if (Loop1 < 1) {
        delete []A;
        return false;
      }
      Loop1--;
      j = t + 2;
      while (j < m) {
        A[j * n + j - 2] = 0;
        j++;
      }
      j = t + 3;
      while (j < m) {
        A[j * n + j - 3] = 0;
        j++;
      }
      k = t;
      while (k < m - 1) {
        if (k != t) {
          p = A[k * n + k - 1];
          q = A[(k + 1) * n + k - 1];
          if (k != m - 2) {
            r = A[(k + 2) * n + k - 1];
          } else {
            r = 0;
          }
        } else {
          b = A[(m - 1) * n + m - 1];
          c = A[(m - 2) * n + m - 2];
          x = b + c;
          y = b * c - A[(m - 2) * n + m - 1] * A[(m - 1) * n + m - 2];
          p = A[t * n + t] * (A[t * n + t] - x) +
              A[t * n + t + 1] * A[(t + 1) * n + t] + y;
          q = A[(t + 1) * n + t] * (A[t * n + t] + A[(t + 1) * n + t + 1] - x);
          r = A[(t + 1) * n + t] * A[(t + 2) * n + t + 1];
        }
        if (p != 0 || q != 0 || r != 0) {
          if (p < 0) {
            xy = -1;
          } else {
            xy = 1;
          }
          s = xy * sqrt(p * p + q * q + r * r);
          if (k != t) {
            A[k * n + k - 1]= -s;
          }
          e = -q / s;
          f = -r / s;
          x = -p / s;
          y = -x - f * r / (p + s);
          g = e * r / (p + s);
          z = -x - e * q / (p + s);
          for (j = k; j < m; j++) {
            b = A[k * n + j];
            c = A[(k + 1) * n + j];
            p = x * b + e * c;
            q = e * b + y * c;
            r = f * b + g * c;
            if (k != m - 2) {
              b = A[(k + 2) * n + j];
              p += f * b;
              q += g * b;
              r += z * b;
              A[(k + 2) * n + j] = r;
            }
            A[(k + 1) * n + j] = q;
            A[k * n + j] = p;
          }
          j = k + 3;
          if (j > m - 2) {
            j = m - 1;
          }
          for (i = t; i < j + 1; i++) {
            b = A[i * n + k];
            c = A[i * n + k + 1];
            p = x * b + e * c;
            q = e * b + y * c;
            r = f * b + g * c;
            if (k != m - 2) {
              b = A[i * n + k + 2];
              p += f * b;
              q += g * b;
              r += z * b;
              A[i * n + k + 2] = r;
            }
            A[i * n + k + 1] = q;
            A[i * n + k] = p;
          }
        }
        k++;
      }
    }
  }
  delete []A;
  return true;
}

#endif  // SRC_MYNTEYE_API_CAMERA_MODELS_MATRIXSOLVER_H_
