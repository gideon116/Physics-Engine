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
    m_vel += dv;
}

void Player::pulse(const Vector impulse_norm, const Vector contact_point)
{
    m_vel += impulse_norm / m_mass;

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
    m_vel += m_acc * m_dt;

    // apply angular acceleration
    m_ang_vel += m_ang_acc * m_dt;
}

void Player::updatePos()
{
    updateTime();

    m_pos += m_vel * m_dt + 0.5 * m_acc * m_dt * m_dt;


    // TODO: update the angle here
    m_angle += m_ang_vel * m_dt + 0.5 * m_ang_acc * m_dt * m_dt;

    updateVel(); // again this is just acceleration

    m_points_world.clear();
    for (int i = 0; i < m_points_local.size(); i++)
    {
        Vector& point = m_points_local[i];
        float new_x = point.x * std::cos(m_angle) - point.y * std::sin(m_angle) + m_pos.x;
        float new_y = point.x * std::sin(m_angle) + point.y * std::cos(m_angle) + m_pos.y;
        m_points_world.push_back({new_x, new_y});
    }
}

void Player::draw(sf::RenderWindow& window, const sf::Color& color)
{
    

    sf::ConvexShape m_shape;
    m_shape.setPointCount(m_points_world.size());
    for (int i = 0; i < m_points_world.size(); i++)
    {
        Vector& point = m_points_world[i];
        m_shape.setPoint(i, {point.x, point.y});
    }

    m_shape.setFillColor(color);
    m_shape.setOutlineThickness(1.f);
    m_shape.setOutlineColor(sf::Color(0, 0, 0));
    window.draw(m_shape);
}

void Player::normToCentroid()
{
    float A = 0, Cx = 0, Cy = 0;
    for (int i = 0; i < m_points_local.size(); i++)
    {
        const Vector& point_i = m_points_local[i];
        const Vector& point_i_p1 = (i == m_points_local.size() - 1)
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
    for (int i = 0; i < m_points_local.size(); i++)
        m_points_local[i] = {m_points_local[i].x - Cx, m_points_local[i].y - Cy};
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