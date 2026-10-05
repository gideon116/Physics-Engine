#ifndef PLAYER_H
#define PLAYER_H

#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include <SFML/Graphics.hpp>

constexpr float Pi = 3.14159265358979323846264338327950288419716939937510;

class Vector
{
public:
    static Vector fromCart(float x, float y)
    {
        return Vector(std::sqrt(std::pow(x, 2) + std::pow(y, 2)), std::atan2(y, x), x, y);

    }
    static Vector fromPol(float mag, float angle)
    {
        return Vector(mag, angle, std::cos(angle) * mag, std::sin(angle) * mag);
    }
    
    float mag = 0, angle = 0, x = 0, y = 0;
    void cartUpdate(float x_, float y_)
    {
        mag = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        angle = std::atan2(y, x);
        x = x_;
        y = y_;
    }
    void polUpdate(float mag_, float angle_)
    {
        mag = mag_;
        angle = angle_;
        x = std::cos(angle) * mag;
        y = std::sin(angle) * mag;
    }
    
private:
    Vector(float mag_, float angle_, float x_, float y_) : mag(mag_), angle(angle_), x(x_), y(y_) {}
};

struct Position
{
    float x = 0, y = 0;
};

class Player
{
public:
    Player(
        const std::string &name, const float x, const float y,
        const std::string &color, const float mass = 2)
    {
        m_name = new std::string(name);
        m_pos = {x, y};
        m_time = std::chrono::steady_clock::now();
        m_mass = mass;

        m_points_local.reserve(3);
        m_points_world.reserve(3);
        m_points_local.push_back({0.f, 0.f});
        m_points_local.push_back({32.f, 32.f * std::sqrt(3.f)});
        m_points_local.push_back({64.f, 0.f});

        m_I = m_mass * 64;

        float A = 0, Cx = 0, Cy = 0;
        for (int i = 0; i < m_points_local.size(); i++)
        {
            const sf::Vector2f& point_i = m_points_local[i];
            const sf::Vector2f& point_i_p1 = (i == m_points_local.size() - 1)
                                             ? m_points_local[0]
                                             : m_points_local[i + 1];
            A += point_i.x * point_i_p1.y - point_i_p1.x * point_i.y;

            float temp_var = point_i.x * point_i_p1.y - point_i_p1.x * point_i.y;
            Cx += (point_i.x + point_i_p1.x) * temp_var;
            Cy += (point_i.y + point_i_p1.y) * temp_var;
        }
        A *= 0.5;
        Cx *= 1/(6 * A);
        Cy *= 1/(6 * A);
        m_centroid = Vector::fromCart(Cx, Cy);

        for (int i = 0; i < m_points_local.size(); i++)
            m_points_local[i] = {m_points_local[i].x - Cx, m_points_local[i].y - Cy};
    };
    
    void sustainedForce(const float angle, const float force);
    void pulse(const float angle, const float mag);
    void pulse(const Vector impulse);

    void updatePos();
    void updateVel();
    void updateVel(const float vx, const float vy) { m_vel.cartUpdate(vx, vy); }
    void updateTime();


    const Position getPos() const { return m_pos; }
    // TODO: return vel as constant
    Vector& getVel() { return m_vel; }
    const float& getMass() { return m_mass; }
    const float& getDt() { return m_dt; }
    const std::string* getName() const { return m_name; }

    void setPos(Position new_pos) { m_pos = new_pos; };
    std::vector<sf::Vector2f> &getPoints() { return m_points_world; }
    void draw(sf::RenderWindow &window);

    // rule of 5
    ~Player() { delete m_name; }
    Player(const Player &other);
    Player(Player &&other) noexcept;
    Player& operator=(const Player &other);
    Player& operator=(Player &&other) noexcept;

private:
    std::string* m_name = nullptr;
    Position m_pos;
    Vector m_acc = Vector::fromCart(0, 0);
    Vector m_ang_acc = Vector::fromCart(0, 0);
    Vector m_vel = Vector::fromCart(0, 0);
    Vector m_centroid = Vector::fromCart(0, 0);
    std::chrono::steady_clock::time_point m_time;
    float m_dt = 0;
    float m_mass = 2;
    float m_angle = 0;
    float m_I = 0;

    std::vector<sf::Vector2f> m_points_local;
    std::vector<sf::Vector2f> m_points_world;

};

#endif
