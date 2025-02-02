#pragma once
#include "Vector2.h"
#include "Matrix2.h"
#include "Vector3.h"

namespace Mathutils {

// Element-wise square for Vector2
inline Vector2 sqr(const Vector2& v) {
    return Vector2(v.x * v.x, v.y * v.y);
}

// Forward declarations of other math utilities
inline void polar_decomp(const Matrix2& m, Matrix2& R, Matrix2& S){
    auto x = m[0][0] + m[1][1];
    auto y = m[0][1] - m[1][0];
    auto scale = 1.0f / std::sqrt(x * x + y * y);
    auto c = x * scale, s = y * scale;
    R[0][0] = c;
    R[1][0] = -s;
    R[0][1] = s;
    R[1][1] = c;
    S = R.t() * m;
}

} // namespace Mathutils