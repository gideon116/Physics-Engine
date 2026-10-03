#include <iostream>
#include "player.h"
#include <SFML/Graphics.hpp>

float Pi = 3.14159265358979323846264338327950288419716939937510;


struct Collision
{
    sf::Vector2f normal;
    sf::Vector2f contact;
    float overlap;
    bool collide = false;
};

sf::Vector2f getContactPoints(const std::vector<sf::Vector2f>& A_vertices, const std::vector<sf::Vector2f>& B_vertices)
{
    sf::Vector2f contact;
    float min_dist = 1e6;
    for (const auto& c : A_vertices)
    {
        for (int i = 0; i < B_vertices.size(); i++)
        {
            sf::Vector2f p1 = B_vertices[i], p2 = B_vertices[(i + 1) % B_vertices.size()];
            sf::Vector2f v = {p2.x - p1.x, p2.y - p1.y};
            sf::Vector2f w = {c.x - p1.x, c.y - p1.y};

            float t = (w.x * v.x + w.y * v.y) / (v.x * v.x + v.y * v.y);
            sf::Vector2f w_p = {v.x * t, v.y * t};
            sf::Vector2f w_t = {w.x - w_p.x, w.y - w_p.y};
            float d = std::sqrt(w_t.x * w_t.x + w_t.y * w_t.y);
            if (d < min_dist)
            {
                min_dist = d;
                contact = c;
            }
        }
    }
    return contact;
}


std::vector<sf::Vector2f> getAxis(const std::vector<sf::Vector2f>& vertices)
{
    std::vector<sf::Vector2f> axis;
    axis.reserve(vertices.size());
    for (int i = 0; i < vertices.size(); i++)
    {
        int next_i = (i == vertices.size() - 1) ? 0 : i + 1;
        sf::Vector2f edge = {vertices[i].y - vertices[next_i].y, - (vertices[i].x - vertices[next_i].x)};
        axis.push_back(edge / std::sqrt(edge.x * edge.x + edge.y * edge.y));
    }
    return axis;
}

std::array<float, 2> project(const sf::Vector2f& axis, const std::vector<sf::Vector2f>& vertices)
{
    float min = vertices[0].x * axis.x + vertices[0].y * axis.y;
    float max = min;
    for (int i = 1; i < vertices.size(); i++)
    {
        float val = vertices[i].x * axis.x + vertices[i].y * axis.y;
        min = std::min(min, val);
        max = std::max(max, val);
    }
    std::array<float, 2> min_max = {min, max};
    return min_max;
}

Collision checkOverlap(Player& A, Player& B)
{
    std::vector<sf::Vector2f> &A_vertices = A.getPoints();
    std::vector<sf::Vector2f> &B_vertices = B.getPoints();

    std::vector<sf::Vector2f> axis_1 = getAxis(A_vertices);
    std::vector<sf::Vector2f> axis_2 = getAxis(B_vertices);

    Collision c = {{}, {}, 1e9, false};

    for (int i = 0; i < A_vertices.size(); i++)
    {
        std::array<float, 2> pa = project(axis_1[i], A_vertices);
        std::array<float, 2> pb = project(axis_1[i], B_vertices);
        if (pa[0] > pb[1] || pa[1] < pb[0])
            // no overlap
            return c;
        float top = std::min(pa[1], pb[1]);
        float bottom = std::max(pa[0], pb[0]);
        float curr_overlap = std::abs(top - bottom);
        // containment
        if ((pa[1] < pb[1] && pa[0] > pb[0]) || (pa[1] > pb[1] && pa[0] < pb[0]))
            curr_overlap += std::min(std::abs(pa[0] - pb[0]), std::abs(pa[1] - pb[1]));

        if (c.overlap > curr_overlap)
        {
            c.overlap = curr_overlap;
            c.normal = axis_1[i];

        }
    }
    for (int i = 0; i < B_vertices.size(); i++)
    {
        std::array<float, 2> pa = project(axis_2[i], A_vertices);
        std::array<float, 2> pb = project(axis_2[i], B_vertices);

        if (pa[0] > pb[1] || pa[1] < pb[0])
            // no overlap
            return c;

        float top = std::min(pa[1], pb[1]);
        float bottom = std::max(pa[0], pb[0]);
        float curr_overlap = std::abs(top - bottom);

        // containment
        if ((pa[1] < pb[1] && pa[0] > pb[0]) || (pa[1] > pb[1] && pa[0] < pb[0]))
            curr_overlap += std::min(std::abs(pa[0] - pb[0]), std::abs(pa[1] - pb[1]));

        if (c.overlap > curr_overlap)
        {
            c.overlap = curr_overlap;
            c.normal = axis_2[i];
        }
    }
    c.collide = true;
    c.contact = getContactPoints(A_vertices, B_vertices);
    return c;
}

void elasticCollision(Player& A, Player& B)
{
    float m_a = A.getMass();
    float m_b = B.getMass();
    float v_a1_x = A.getVel().x();
    float v_b1_x = B.getVel().x();
    float v_a1_y = A.getVel().y();
    float v_b1_y = B.getVel().y();

    float v_a2_x = ((m_a - m_b) * v_a1_x + 2 * m_b * v_b1_x) / (m_a + m_b);
    float v_b2_x = (2 * m_a * v_a1_x + (m_b - m_a) * v_b1_x) / (m_a + m_b);

    float v_a2_y = ((m_a - m_b) * v_a1_y + 2 * m_b * v_b1_y) / (m_a + m_b);
    float v_b2_y = (2 * m_a * v_a1_y + (m_b - m_a) * v_b1_y) / (m_a + m_b);

    A.updateVel(v_a2_x, v_a2_y);
    B.updateVel(v_b2_x, v_b2_y);
}

void semiElasticCollision(Player& A, Player& B, float e = 0.9)
{
    float m_a = A.getMass();
    float m_b = B.getMass();
    float v_a1_x = A.getVel().x();
    float v_b1_x = B.getVel().x();
    float v_a1_y = A.getVel().y();
    float v_b1_y = B.getVel().y();

    float dx = B.getPos().x - A.getPos().x + 400 - 25;
    float dy = B.getPos().y - A.getPos().y + 25 - 25;

    float dist = std::max(std::sqrt(dx * dx + dy * dy), 0.01f);
    float nx = dx/dist;
    float ny = dy/dist;

    float v_a1_n = v_a1_x * nx + v_a1_y * ny;
    float v_b1_n = v_b1_x * nx + v_b1_y * ny;

    float v_a2_n = (m_a * v_a1_n + m_b * v_b1_n + m_b * e * (v_b1_n - v_a1_n)) / (m_a + m_b);
    float v_b2_n = (m_a * v_a1_n + m_b * v_b1_n + m_a * e * (v_a1_n - v_b1_n)) / (m_a + m_b);

    float dKE = -0.5 * (1 - std::pow(e, 2)) * std::pow((v_a1_n - v_b1_n), 2) * m_a * m_b / (m_a + m_b);
    // do somthing with dKE

    float v_a2_x = v_a1_x + (v_a2_n - v_a1_n) * nx;
    float v_a2_y = v_a1_y + (v_a2_n - v_a1_n) * ny;
    float v_b2_x = v_b1_x + (v_b2_n - v_b1_n) * nx;
    float v_b2_y = v_b1_y + (v_b2_n - v_b1_n) * ny;

    A.updateVel(v_a2_x, v_a2_y);
    B.updateVel(v_b2_x, v_b2_y);
}


void inBounds(Player& A, const float boundRight,
              const float boundLeft, const float boundBottom, const float boundTop, const float e)
{

    std::vector<sf::Vector2f> &points = A.getPoints();

    for (const auto& point : points)
    {
        float diff = point.x - boundRight;
        if (diff > 0)
        {
            A.setPos({(A.getPos().x - diff), A.getPos().y});
            A.updateVel(-A.getVel().x() * e, A.getVel().y());
        }
        diff = boundLeft - point.x;
        if (diff > 0)
        {
            A.setPos({(A.getPos().x + diff), A.getPos().y});
            A.updateVel(-A.getVel().x() * e, A.getVel().y());
        }
        diff = boundTop - point.y;
        if (diff > 0)
        {
            A.setPos({A.getPos().x, (A.getPos().y + diff)});
            A.updateVel(A.getVel().x(), -A.getVel().y() * e);
        }
        diff = point.y - boundBottom;
        if (diff > 0)
        {
            A.setPos({A.getPos().x, (A.getPos().y - diff)});
            A.updateVel(A.getVel().x(), -A.getVel().y() * e);

            // friction
            // sBob.pulse(Pi + sBob.getVel().angle, frictionCoe * gravity * sBob.getDt());
        }
    }
}


int main()
{
    int frame = 0;
    int delay = 0;
    float frictionCoe = 0.1;
    float gravity = 1000;

    sf::RenderWindow window(sf::VideoMode({800, 600}), "Physics yay");
    window.setFramerateLimit(60);

    float boundRight = (float)window.getSize().x;
    float boundLeft = 0;
    float boundBottom = 550;
    float boundTop = 50;

    float e = 0.9;

    // set the scene
    sf::RectangleShape groundS({800, 50});
    groundS.setFillColor(sf::Color(196, 164, 132));
    groundS.setPosition({0, boundBottom});

    sf::RectangleShape skyS({800, 50});
    skyS.setFillColor(sf::Color(135, 206, 235));
    skyS.setPosition({0, 0});

    Player sBob = {"sBob", 50, 100, "red"};
    Player sPat = {"sPat", 400, 100, "green"};
    
    // sBob.sustainedForce(-3*Pi/2, gravity);
    // sPat.sustainedForce(-3*Pi/2, gravity);

    while (window.isOpen())
    {
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        window.clear(sf::Color(180, 255, 180));

        // -------------------
        frame++;

        if ((!delay || (frame - delay) > 10))
        {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left))
            { 
                sBob.pulse(Pi, 100);
                delay = frame;
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right))
            { 
                sBob.pulse(0, 100);
                delay = frame;
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up))
            { 
                sBob.pulse(-Pi/2, 1000);
                delay = frame;
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Down))
            { 
                sBob.pulse(-3*Pi/2, 100);
                delay = frame;
            }
        }

        {
            sBob.updatePos();
            sPat.updatePos();

            inBounds(sBob, boundRight, boundLeft, boundBottom, boundTop, e);
            inBounds(sPat, boundRight, boundLeft, boundBottom, boundTop, e);

            Collision c = checkOverlap(sBob, sPat);
            if (c.collide)
            {
                float inv_sBob_mass = 1/sBob.getMass(), inv_sPat_mass = 1/sPat.getMass();
                float inv_comb_mass = inv_sBob_mass + inv_sPat_mass;
                float norm_sBob_mass = c.overlap * inv_sBob_mass / inv_comb_mass;
                float norm_sPat_mass = c.overlap * inv_sPat_mass / inv_comb_mass;

                sBob.setPos({sBob.getPos().x - (c.normal.x * norm_sBob_mass), sBob.getPos().y - (c.normal.y * norm_sBob_mass)});
                sPat.setPos({sPat.getPos().x - (c.normal.x * norm_sBob_mass), sPat.getPos().y - (c.normal.y * norm_sPat_mass)});

                float mag = (sPat.getVel().x() - sBob.getVel().x()) * c.normal.x + (sPat.getVel().y() - sBob.getVel().y()) * c.normal.y;
                if (mag < 0)
                {
                    sf::Vector2f impulse = {-(1 + e) * mag / (inv_comb_mass) * c.normal.x, -(1 + e) * mag / (inv_comb_mass) * c.normal.y};
                    sBob.updateVel(-impulse.x * inv_sBob_mass, -impulse.y * inv_sBob_mass);
                    sPat.updateVel(impulse.x * inv_sPat_mass, impulse.y * inv_sPat_mass);
                }
            }

            std::cout << c.collide << "\n";
            std::cout << c.contact.x << " " << c.contact.y << "\n";
        }

        sBob.draw(window);
        sPat.draw(window);

        window.draw(groundS);
        window.draw(skyS);

        // -------------------

        window.display();
    }
}