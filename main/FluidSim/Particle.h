#include "Mathutils.h"
using Vec = Mathutils::Vector2;
using real = float;
using Vector3 = Mathutils::Vector3;
using Mat = Mathutils::Matrix2;

struct Particle {
    Vec x, v;
    Mat F, C;
    real Jp;
    Particle(Vec x, Vec v = Vec(0))
        : x(x)
        , v(v)
        , F(1)
        , C(0)
        , Jp(1)
    {
    }
};