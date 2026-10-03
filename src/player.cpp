#include "player.h"
#include <cmath>



void Player::sustainedForce(const float angle, const float force)
{
    Vector da = {force / m_mass, angle};
    float ax = m_acc.x() + da.x();
    float ay = m_acc.y() + da.y();
    m_acc.mag = std::sqrt(std::pow(ax, 2) + std::pow(ay, 2));
    m_acc.angle = std::atan2(ay, ax);
}

void Player::pulse(const float angle, const float impulse)
{
    Vector dv = {impulse / m_mass, angle};
    float vx = m_vel.x() + dv.x();
    float vy = m_vel.y() + dv.y();
    m_vel.mag = std::sqrt(std::pow(vx, 2) + std::pow(vy, 2));
    m_vel.angle = std::atan2(vy, vx);
}

void Player::updateTime()
{
    auto curr_time = std::chrono::steady_clock::now();
    m_dt = std::chrono::duration_cast<std::chrono::duration<double>>(curr_time - m_time).count();
    m_time = curr_time;
}

void Player::updateVel()
{
    float vx = m_vel.x() + m_acc.x() * m_dt;
    float vy = m_vel.y() + m_acc.y() * m_dt;
    m_vel.mag = std::sqrt(std::pow(vx, 2) + std::pow(vy, 2));
    m_vel.angle = std::atan2(vy, vx);
}

void Player::updateVel(const float vx, const float vy)
{
    m_vel.mag = std::sqrt(std::pow(vx, 2) + std::pow(vy, 2));
    m_vel.angle = std::atan2(vy, vx);
}


void Player::updatePos()
{
    updateTime();
    
    m_pos.x += m_vel.x() * m_dt + 0.5 * m_acc.x() * m_dt * m_dt;
    m_pos.y += m_vel.y() * m_dt + 0.5 * m_acc.y() * m_dt * m_dt;

    updateVel();
}

void Player::updatePoints()
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

const Position Player::getPos() const
{
    return m_pos;
}

void Player::setPos(Position new_pos)
{
    m_pos = new_pos;
}

const std::string* Player::getName() const
{
    return m_name;
}

Player::~Player()
{
    delete m_name;
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