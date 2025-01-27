#include "Vector2.h"
#include "mat.h"
using Vec = Vector2;
using real = float;
using dspm::Mat;
struct Particle {
    Vec x, v;
    Mat F;
    Mat C;
    real Jp;
    Particle(Vec x, Vec v = Vec(0))
        : x(x)
        , v(v)
        , F(dspm::Mat::eye(2)) // 
        , C(Mat(2,2)) // 浅复制？
        , Jp(1)
    {
    }
};