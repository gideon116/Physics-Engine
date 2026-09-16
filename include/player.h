#ifndef PLAYER_H
#define PLAYER_H

#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include <SFML/Graphics.hpp>

struct Vector
{
    float mag = 0, angle = 0;
    float x(){ return std::cos(angle) * mag; }
    float y(){ return std::sin(angle) * mag; }
};

struct Position
{
    float x = 0, y = 0;
};

class Player
{
public:
    Player(const std::string &name, const float x, const float y, const std::string &color, const float mass = 2)
    {
        m_name = new std::string(name);
        m_pos = {x, y};
        m_time = std::chrono::steady_clock::now();
        m_mass = mass;

        m_points_local.reserve(5);
        m_points_world.reserve(5);
        m_points_local.push_back({0.f, 40.f});
        m_points_local.push_back({0.f, 15.f});
        m_points_local.push_back({20.f, 0.f});
        m_points_local.push_back({40.f, 15.f});
        m_points_local.push_back({40.f, 40.f});

        float A = 0, Cx = 0, Cy = 0;;
        for (int i = 0; i < m_points_local.size(); i++)
        {
            const sf::Vector2f& point_i = m_points_local[i];
            const sf::Vector2f& point_i_p1 = (i == m_points_local.size() - 1) ? m_points_local[0] : m_points_local[i + 1];
            A += point_i.x * point_i_p1.y - point_i_p1.x * point_i.y;

            float temp_var = point_i.x * point_i_p1.y - point_i_p1.x * point_i.y;
            Cx += (point_i.x + point_i_p1.x) * temp_var;
            Cy += (point_i.y + point_i_p1.y) * temp_var;
        }
        A *= 0.5;
        Cx *= 1/(6 * A);
        Cy *= 1/(6 * A);

        for (int i = 0; i < m_points_local.size(); i++)
            m_points_local[i] = {m_points_local[i].x - Cx, m_points_local[i].y - Cy};
    };
    
    void sustainedForce(const float angle, const float force);
    void pulse(const float angle, const float impulse);

    void updatePos();
    void updateVel();
    void updateVel(const float vx, const float vy);
    void updateTime();


    const Position getPos() const;
    // TODO: return vel as constant
    Vector& getVel() { return m_vel; }
    const float& getMass() { return m_mass; }
    const float& getDt() { return m_dt; }
    const std::string* getName() const;

    void setPos(Position new_pos);

    void updatePoints()
    {
        m_points_world.clear();
        for (int i = 0; i < m_points_local.size(); i++)
        {
            sf::Vector2f& point = m_points_local[i];
            float new_x = point.x * std::cos(m_angle) - point.y * std::sin(m_angle) + m_pos.x;
            float new_y = point.x * std::sin(m_angle) + point.y * std::cos(m_angle) + m_pos.y;
            m_points_world.push_back({new_x, new_y});
        }
    }

    std::vector<sf::Vector2f> &getPoints()
    {
        updatePoints();
        return m_points_world;
    }
    void draw(sf::RenderWindow &window)
    {
        sf::ConvexShape m_shape;
        m_shape.setPointCount(5);
        updatePoints();

        for (int i = 0; i < m_points_world.size(); i++)
        {
            sf::Vector2f& point = m_points_world[i];
            m_shape.setPoint(i, {point.x, point.y});
        }

        m_shape.setFillColor(sf::Color(255, 255, 0));
        m_shape.setOutlineThickness(1.f);
        m_shape.setOutlineColor(sf::Color(0, 0, 0));
        window.draw(m_shape);
    }

    // rule of 5
    ~Player();
    Player(const Player &other);
    Player(Player &&other) noexcept;
    Player& operator=(const Player &other);
    Player& operator=(Player &&other) noexcept;

private:
    std::string* m_name = nullptr;
    Position m_pos = {0, 0};
    Vector m_acc = {0, 0};
    Vector m_vel = {0, 0};
    std::chrono::steady_clock::time_point m_time;
    float m_dt = 0;
    float m_mass = 2;
    float m_angle = 0;

    
    std::vector<sf::Vector2f> m_points_local;
    std::vector<sf::Vector2f> m_points_world;
    


};

#endif
