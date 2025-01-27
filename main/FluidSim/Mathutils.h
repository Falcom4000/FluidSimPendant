#include "Vector2.h"
#include "mat.h"
#include <cstdlib>
#include <ctime>
using dspm::Mat;
using real = float;
using Vec = Vector2;
#ifdef __cplusplus
extern "C" {
#endif
Vec Mat2_times_Vec(const Mat& mat, const Vec& vec);

void polar_decomp(Mat m, Mat &R, Mat &S);

Mat outer_product(const Vec& vec1, const Vec& vec2);

Vec generate_random_float(float min, float max);
#ifdef __cplusplus
}
#endif