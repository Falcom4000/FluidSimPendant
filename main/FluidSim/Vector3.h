#ifndef VECTOR3_H
#define VECTOR3_H
#include <cmath>
// TODO: Add support for dsp library
namespace Mathutils {
class Vector3 {
public:
    float x, y, z;

public:
    Vector3()
        : x(0)
        , y(0)
        , z(0)
    {
    }

    Vector3(const float x, const float y, const float z)
        : x(x)
        , y(y)
        , z(z)
    {
    }

    Vector3(const float x)
        : x(x)
        , y(x)
        , z(x)
    {
    }

    void operator*=(const float k)
    {
        x *= k;
        y *= k;
        z *= k;
    }

    void operator+=(const Vector3& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    void operator-=(const Vector3& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    void operator*=(const Vector3& v)
    {
        x *= v.x;
        y *= v.y;
        z *= v.z;
    }

    void operator/=(const Vector3& v)
    {
        x /= v.x;
        y /= v.y;
        z /= v.z;
    }
    Vector3 operator+(const float& k) const { return Vector3(x + k, y + k, z + k); }

    Vector3 operator-(const float& k) const { return Vector3(x - k, y - k, z - k); }

    Vector3 operator*(const float& k) const { return Vector3(x * k, y * k, z * k); }

    Vector3 operator+(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }

    Vector3 operator-(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }

    Vector3 operator*(const Vector3& v) const { return Vector3(x * v.x, y * v.y, z * v.z); }

    Vector3 operator/(const Vector3& v) const { return Vector3(x / v.x, y / v.y, z / v.z); }
};

}

#endif
