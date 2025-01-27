#include "Mathutils.h"
#ifdef __cplusplus
extern "C" {
#endif
Vec Mat2_times_Vec(const Mat& mat, const Vec& vec)
{
    return Vec(mat(0,0)*vec.x + mat(0,1)*vec.y, mat(1,0)*vec.x + mat(1,1)*vec.y);
}

void polar_decomp(Mat m, Mat &R, Mat &S) {
    auto x = m(0, 0) + m(1, 1);
    auto y = m(1, 0) - m(0, 1);
    auto scale = 1.0F / std::sqrt(x * x + y * y);
    auto c = x * scale, s = y * scale;
    R(0, 0) = c;
    R(0, 1) = -s;
    R(1, 0) = s;
    R(1, 1) = c;
    S = R.t() * m;
}

Mat outer_product(const Vec& vec1, const Vec& vec2){
    Mat res(2,2);
    res(0,0) = vec1.x * vec2.x;
    res(0,1) = vec1.x * vec2.y;
    res(1,0) = vec1.y * vec2.x;
    res(1,1) = vec1.y * vec2.y;
    return res;
}

Vec generate_random_float(float min, float max)
{
    float randX = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    float randY = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    return Vec(randX, randY);
}
#ifdef __cplusplus
}
#endif