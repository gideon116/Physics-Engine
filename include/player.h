#ifndef PLAYER_H
#define PLAYER_H

#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include <SFML/Graphics.hpp>
#include "vector.h"

constexpr float Pi = 3.14159265358979323846264338327950288419716939937510;

class Player
{
public:
    Player(
        const std::string& name, const float x, const float y,
        const std::string& shape = "triangle", const float mass = 2)
    {
        m_name = new std::string(name);
        m_pos = {x, y};
        m_time = std::chrono::steady_clock::now();
        m_mass = mass;

        if (shape == "triangle")
        {
            m_points_world.reserve(3);
            m_points_local.reserve(3);
            m_points_local.push_back({0.f, 0.f});
            m_points_local.push_back({32.f, 32.f * std::sqrt(3.f)});
            m_points_local.push_back({64.f, 0.f});
            m_I = m_mass * std::pow(64, 2) / 12;
        }
        else if (shape == "rectangle")
        {
            m_points_world.reserve(4);
            m_points_local.reserve(4);
            m_points_local.push_back({0.f, 0.f});
            m_points_local.push_back({800.f, 0.f});
            m_points_local.push_back({800.f, 50.f});
            m_points_local.push_back({0.f, 50.f});
            m_I = m_mass * (std::pow(800, 2) + std::pow(50, 2)) / 12;
        }

        normToCentroid();
    };
    
    void applyForce(const float angle, const float force);
    void pulse(const float angle, const float mag);
    void pulse(const Vector impulse_norm, const Vector contact_point);

    void updatePos();
    void updateVel();
    void updateVel(const float vx, const float vy) { m_vel.cartUpdate(vx, vy); }
    void updateTime();

    const Vector getPos() const { return m_pos; }
    const float getI() const { return m_I; }
    const float getW() const { return m_ang_vel; }
    // TODO: return vel as constant
    Vector& getVel() { return m_vel; }
    const float& getMass() { return m_mass; }
    const float& getDt() { return m_dt; }
    const std::string* getName() const { return m_name; }

    void setPos(Vector new_pos) { m_pos = new_pos; };
    std::vector<Vector> &getPoints() { return m_points_world; }
    void draw(sf::RenderWindow& window, const sf::Color& color = sf::Color(255, 255, 0));
    void normToCentroid();

    // rule of 5
    ~Player() { delete m_name; }
    Player(const Player &other);
    Player(Player &&other) noexcept;
    Player& operator=(const Player &other);
    Player& operator=(Player &&other) noexcept;

private:
    std::string* m_name = nullptr;
    Vector m_pos = {0, 0};
    Vector m_acc = {0, 0};
    float m_ang_acc = 0;
    Vector m_vel = {0, 0};
    float m_ang_vel = 0;
    std::chrono::steady_clock::time_point m_time;
    float m_dt = 0;
    float m_mass = 2;
    float m_angle = 0;
    float m_I = 0;

    std::vector<Vector> m_points_local;
    std::vector<Vector> m_points_world;
};

#endif
