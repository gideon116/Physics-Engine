#include "player.h"
#include <cmath>


void Player::applyForce(const float angle, const float force)
{
    Vector da = Vector::fromPol(force / m_mass, angle);
    m_acc.cartUpdate(da.x, da.y);
}

void Player::pulse(const float angle, const float mag)
{
    Vector dv = Vector::fromPol(mag / m_mass, angle);
    float vx = m_vel.x + dv.x;
    float vy = m_vel.y + dv.y;
    m_vel.cartUpdate(vx, vy);
}

void Player::pulse(const Vector impulse_norm, const Vector contact_point)
{
    float vx = m_vel.x + impulse_norm.x / m_mass;
    float vy = m_vel.y + impulse_norm.y / m_mass;
    m_vel.cartUpdate(vx, vy);
    
    float num = (-contact_point.y + m_pos.y) * impulse_norm.x +
                (contact_point.x - m_pos.x) * impulse_norm.y;
    m_ang_vel += num / m_I;
}

void Player::updateTime()
{
    auto curr_time = std::chrono::steady_clock::now();
    m_dt = std::chrono::duration_cast<std::chrono::duration<double>>(curr_time - m_time).count();
    m_time = curr_time;
}

void Player::updateVel() // this just applies acceleration
{
    // apply linear acceleration
    float vx = m_vel.x + m_acc.x * m_dt;
    float vy = m_vel.y + m_acc.y * m_dt;
    m_vel.cartUpdate(vx, vy);

    // apply angular acceleration
    m_ang_vel += m_ang_acc * m_dt;
}

void Player::updatePos()
{
    updateTime();

    m_pos.x += m_vel.x * m_dt + 0.5 * m_acc.x * m_dt * m_dt;
    m_pos.y += m_vel.y * m_dt + 0.5 * m_acc.y * m_dt * m_dt;

    // TODO: update the angle here
    m_angle += m_ang_vel * m_dt + 0.5 * m_ang_acc * m_dt * m_dt;

    updateVel(); // again this is just acceleration

    m_points_world.clear();
    for (int i = 0; i < m_points_local.size(); i++)
    {
        sf::Vector2f& point = m_points_local[i];
        float new_x = point.x * std::cos(m_angle) - point.y * std::sin(m_angle) + m_pos.x;
        float new_y = point.x * std::sin(m_angle) + point.y * std::cos(m_angle) + m_pos.y;
        m_points_world.push_back({new_x, new_y});
    }
}

void Player::draw(sf::RenderWindow &window)
{
    sf::ConvexShape m_shape;
    m_shape.setPointCount(3);

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

Player::Player(const Player &other)
{
    delete m_name;

    m_name = new std::string(*(other.m_name));
    m_pos = other.m_pos;
    
}

Player::Player(Player &&other) noexcept
{
    delete m_name;

    m_name = other.m_name;
    m_pos = other.m_pos;

    other.m_name = nullptr;
    other.m_pos = {0, 0};
}

Player& Player::operator=(const Player &other)
{
    delete m_name;

    m_name = new std::string(*(other.m_name));
    m_pos = other.m_pos;

    return *this;
}

Player& Player::operator=(Player &&other) noexcept
{
    delete m_name;
    m_name = other.m_name;
    m_pos = other.m_pos;
    
    other.m_name = nullptr;
    other.m_pos = {0, 0};

    return *this;
}