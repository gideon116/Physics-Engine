#include "vector.h"
#include <cmath>


Vector Vector::fromPol(float mag, float angle)
{
    return Vector(mag, angle, std::cos(angle) * mag, std::sin(angle) * mag);
}

float Vector::dot(const Vector& A, const Vector& B)
{
    return A.x * B.x + A.y * B.y;
}

float Vector::magFromCart(float x_, float y_)
{
    return std::sqrt(std::pow(x_, 2) + std::pow(y_, 2));
}

float Vector::angleFromCart(float x_, float y_)
{
    return std::atan2(y_, x_);
}

void Vector::cartUpdate(float x_, float y_)
{
    mag = magFromCart(x, y);
    angle = angleFromCart(x, y);
    x = x_;
    y = y_;
}

void Vector::polUpdate(float mag_, float angle_)
{
    mag = mag_;
    angle = angle_;
    x = std::cos(angle) * mag;
    y = std::sin(angle) * mag;
}


Vector& Vector::perp()
{
    float old_x = x;
    x = -y;
    y = old_x;
    mag = magFromCart(x, y);
    angle = angleFromCart(x, y);
    return *this;
}

Vector& Vector::operator+=(const Vector& other)
{
    this->x += other.x;
    this->y += other.y;
    this->mag = magFromCart(this->x, this->y);
    this->angle = angleFromCart(this->x, this->y);
    return *this;
}
