#ifndef VECTOR_H
#define VECTOR_H

#include <cmath>

class Vector
{
public:
    Vector(float x, float y) : mag(magFromCart(x, y)), angle(angleFromCart(x, y)), x(x), y(y) { }

    Vector(float mag_, float angle_, float x_, float y_) : mag(mag_), angle(angle_), x(x_), y(y_) {}

    static Vector fromPol(float mag, float angle);

    static float dot(const Vector& A, const Vector& B);

    static float magFromCart(float x_, float y_);

    static float angleFromCart(float x_, float y_);

    void cartUpdate(float x_, float y_);

    void polUpdate(float mag_, float angle_);

    Vector& perp();

    Vector operator+(const Vector& other) const { return {x + other.x, y + other.y}; }
    
    Vector operator-(const Vector& other) const { return {x - other.x, y - other.y}; }
    
    Vector operator*(const float& scale) const { return {x * scale, y * scale}; }
    
    Vector operator/(const float& scale) const { return {x / scale, y / scale}; }

    Vector& operator+=(const Vector& other);

    Vector& operator*=(const Vector& other);

    Vector& operator*=(const float& scale);

public:
    float mag = 0, angle = 0, x = 0, y = 0;
};

inline Vector operator*(float scale, const Vector& vec)
{
    return Vector(vec.x * scale, vec.y * scale);
}
inline Vector operator/(float scale, const Vector& vec)
{
    return Vector(vec.x / scale, vec.y / scale);
}

#endif