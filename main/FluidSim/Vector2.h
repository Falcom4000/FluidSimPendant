#ifndef VECTOR2H
#define VECTOR2H
#include <cmath>
#include <cstdlib>
namespace Mathutils {
    using real = float;
    class Vector2
{
public:
    float x, y;
public:
    Vector2()
        : x(0.0f)
        , y(0.0f)
    {
    }

    Vector2(const float x_, const float y_)
        : x(x_)
        , y(y_)
    {
    }

    Vector2(const float x_)
        : x(x_)
        , y(x_)
    {
    }
    void operator+=(const Vector2 &v)
    {
        x += v.x;
        y += v.y;
    }

    void operator-=(const Vector2 &v)
    {
        x -= v.x;
        y -= v.y;
    }

    void operator*=(const Vector2 &v)
    {
        x *= v.x;
        y *= v.y;
    }

    Vector2 operator+(const float &k) const { return Vector2(x + k, y + k); }

    Vector2 operator-(const float &k) const { return Vector2(x - k, y - k); }

    Vector2 operator*(const float &k) const { return Vector2(x * k, y * k); }

    Vector2 operator+(const Vector2 &v) const { return Vector2(x + v.x, y + v.y); }

    Vector2 operator-(const Vector2 &v) const { return Vector2(x - v.x, y - v.y); }

    Vector2 operator*(const Vector2 &v) const { return Vector2(x * v.x, y * v.y); }

    Vector2 operator/(const Vector2 &v) const { return Vector2(x / v.x, y / v.y); }

    float dot(const Vector2 &v) const { return x * v.x + y * v.y; }

    float cross(const Vector2 &v) const { return x * v.y - y * v.x; }

    Vector2 rotate90() const { return Vector2(-y, x); }

    float lengthSqr() const { return x * x + y * y; }

    float length() const { return std::sqrt(lengthSqr()); }

public:

    static Vector2 crossProd(const Vector2 &v) { return Vector2(-v.y, v.x); }
    static Vector2 RAND() { const float x_  = real(rand())/RAND_MAX ; const float y_  = real(rand())/RAND_MAX ;return Vector2(x_, y_);}

};
}


// Vector2 operator*(const float &k, const Vector2 &v){
//     return Vector2(k * v.x, k * v.y);
// }
#endif