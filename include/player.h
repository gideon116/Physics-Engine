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
        const float* vertices, const float num_vertices, const float mass = 2
    )
    {
        m_name = new std::string(name);
        m_pos = {x, y};
        m_time = std::chrono::steady_clock::now();
        m_mass = mass;
        m_num_vertices = num_vertices;
        m_vertex_array.reserve(m_num_vertices);

        // TODO: we no longer need m_points_world and m_points_local to be vectors
        m_points_world = new Vector[m_num_vertices * 2];
        m_points_local = new Vector[m_num_vertices * 2];
        for (int i = 0; i < m_num_vertices; i++)
            m_points_local[i] = {vertices[i * 2], vertices[i * 2 + 1]};

        Vector CxCy = {0, 0};
        for (int i = 0; i < m_num_vertices / 3; i++)
        {
            Vector* ptr = m_points_local + i * 3;
            float mass_i = m_mass/(m_num_vertices/3);
            CxCy += Vector::calcCentroid(ptr, 3) * mass_i;
        }
        CxCy *= (1/m_mass);
        for (int i = 0; i < m_num_vertices/3; i++)
        {
            Vector* ptr = m_points_local + i * 3;
            float mass_i = m_mass/(m_num_vertices/3);
            Vector centroid_i = Vector::calcCentroid(ptr, 3);

            float inner = 0;
            for (int j = 0; j < 3; j++)
                inner += std::pow(Vector::calcDistance(ptr[(j+1) % 3], ptr[j]), 2);
            m_I += (mass_i * inner / 36) + mass_i * std::pow((centroid_i - CxCy).mag, 2);
        }
        for (int i = 0; i < m_num_vertices; i++)
            m_points_local[i] = {m_points_local[i] - CxCy};
    };

    void applyForce(const float angle, const float force);
    void pulse(const float angle, const float mag);
    void pulse(const Vector impulse_norm, const Vector contact_point);

    void updateWorldPoints();
    void updatePos();
    void updateVel();
    void updateVel(const float vx, const float vy) { m_vel.cartUpdate(vx, vy); }
    void updateTime();

    const Vector getPos() const { return m_pos; }
    const float getI() const { return m_I; }
    const float getW() const { return m_ang_vel; }
    const Vector& getVel() const { return m_vel; }
    const float getMass() const { return m_mass; }
    const float getDt() const { return m_dt; }
    const int getNumVertices() const { return m_num_vertices; }
    const std::string* getName() const { return m_name; }
    Vector* getPoints() { return m_points_world; }

    void setPos(Vector new_pos) { m_pos = new_pos; updateWorldPoints(); };
    void draw(sf::RenderWindow& window, const sf::Color& color = sf::Color(255, 255, 0));

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
    int m_num_vertices = 1;

    Vector* m_points_local;
    Vector* m_points_world;
    std::vector<sf::Vertex> m_vertex_array;
};

#endif
