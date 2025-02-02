#ifndef MATRIX2
#define MATRIX2
#include <cmath>
#pragma once
#include "Vector2.h"
using real = float;
namespace Mathutils {
class Matrix2 {
private:
    real data[2][2];

public:
    // Constructors
    Matrix2()
    {
        data[0][0] = data[1][1] = 1;
        data[0][1] = data[1][0] = 0;
    }
    Matrix2(real value) { data[0][0] = data[1][1]  = value; data[0][1] = data[1][0] = 0;}
    Matrix2(real a00, real a01, real a10, real a11)
    {
        data[0][0] = a00;
        data[0][1] = a01;
        data[1][0] = a10;
        data[1][1] = a11;
    }

    // Array access operators
    real* operator[](int i) { return data[i]; }
    const real* operator[](int i) const { return data[i]; }

    // Arithmetic operators
    Matrix2 operator+(const Matrix2& m) const
    {
        return Matrix2(data[0][0] + m[0][0], data[0][1] + m[0][1],
            data[1][0] + m[1][0], data[1][1] + m[1][1]);
    }

    Matrix2 operator-(const Matrix2& m) const
    {
        return Matrix2(data[0][0] - m[0][0], data[0][1] - m[0][1],
            data[1][0] - m[1][0], data[1][1] - m[1][1]);
    }

    Matrix2 operator*(const Matrix2& m) const
    {
        return Matrix2(
            data[0][0] * m[0][0] + data[0][1] * m[1][0],
            data[0][0] * m[0][1] + data[0][1] * m[1][1],
            data[1][0] * m[0][0] + data[1][1] * m[1][0],
            data[1][0] * m[0][1] + data[1][1] * m[1][1]);
    }

    Vector2 operator*(const Vector2& v) const
    {
        return Vector2(data[0][0] * v.x + data[0][1] * v.y,
            data[1][0] * v.x + data[1][1] * v.y);
    }

    Matrix2 operator*(real s) const
    {
        return Matrix2(data[0][0] * s, data[0][1] * s,
            data[1][0] * s, data[1][1] * s);
    }

    Matrix2 operator/(real s) const
    {
        real inv_s = 1.0f / s;
        return Matrix2(data[0][0] * inv_s, data[0][1] * inv_s,
            data[1][0] * inv_s, data[1][1] * inv_s);
    }

    // Compound assignment operators
    Matrix2& operator+=(const Matrix2& m)
    {
        data[0][0] += m[0][0];
        data[0][1] += m[0][1];
        data[1][0] += m[1][0];
        data[1][1] += m[1][1];
        return *this;
    }

    Matrix2& operator*=(real s)
    {
        data[0][0] *= s;
        data[0][1] *= s;
        data[1][0] *= s;
        data[1][1] *= s;
        return *this;
    }

    // Utility functions
    static Matrix2 outer_product(const Vector2& v1, const Vector2& v2)
    {
        return Matrix2(v1.x * v2.x, v1.y * v2.x,
            v1.x * v2.y, v1.y * v2.y);
    }

    real determinant() const
    {
        return data[0][0] * data[1][1] - data[0][1] * data[1][0];
    }

    // 添加转置操作
    Matrix2 transpose() const
    {
        return Matrix2(data[0][0], data[1][0],
            data[0][1], data[1][1]);
    }

    // 简写方法
    Matrix2 t() const
    {
        return transpose();
    }

    // 修正后的polar decomposition实现
    static void polar_decomp(const Matrix2& m, Matrix2& R, Matrix2& S)
    {
        auto x = m[0][0] + m[1][1];
        auto y = m[0][1] - m[1][0];
        auto scale = 1.0f / std::sqrt(x * x + y * y);
        auto c = x * scale, s = y * scale;
        R[0][0] = c;
        R[1][0] = -s;
        R[0][1] = s;
        R[1][1] = c;
        S = R.t() * m;
    };
};
}
#endif
