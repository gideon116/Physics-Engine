#include <iostream>
#include "player.h"
#include <SFML/Graphics.hpp>



struct Collision
{
    Vector normal;
    Vector contact;
    float overlap;
    bool collide = false;
};

void getContactPoints(
    const std::vector<Vector>& A_vertices,  const std::vector<Vector>& B_vertices,
    float& min_dist, Vector& contact)
{
    float d = min_dist;
    for (const auto& c : A_vertices)
    {
        for (int i = 0; i < B_vertices.size(); i++)
        {
            Vector p1 = B_vertices[i], p2 = B_vertices[(i + 1) % B_vertices.size()];
            Vector v = {p2.x - p1.x, p2.y - p1.y};
            Vector w = {c.x - p1.x, c.y - p1.y};

            float t = (w.x * v.x + w.y * v.y) / (v.x * v.x + v.y * v.y); // w . v / v . v
            if (t >= 0 && t <= 1)
            {
                Vector w_p = {v.x * t, v.y * t}; // v * t
                Vector w_t = {w.x - w_p.x, w.y - w_p.y}; // w - v * t
                d = std::sqrt(w_t.x * w_t.x + w_t.y * w_t.y);
            }
            if (d < min_dist)
            {
                min_dist = d;
                contact = c;
            }
        }
    }
}

std::vector<Vector> getAxis(const std::vector<Vector>& vertices)
{
    std::vector<Vector> axis;
    axis.reserve(vertices.size());
    for (int i = 0; i < vertices.size(); i++)
    {
        int next_i = (i == vertices.size() - 1) ? 0 : i + 1;
        Vector edge = {
            vertices[i].y - vertices[next_i].y, - (vertices[i].x - vertices[next_i].x)};
        axis.push_back(edge / std::sqrt(edge.x * edge.x + edge.y * edge.y));
    }
    return axis;
}

std::array<float, 2> project(const Vector& axis, const std::vector<Vector>& vertices)
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
    std::vector<Vector> &A_vertices = A.getPoints();
    std::vector<Vector> &B_vertices = B.getPoints();

    std::vector<Vector> axis_1 = getAxis(A_vertices);
    std::vector<Vector> axis_2 = getAxis(B_vertices);

    Collision c = {{0, 0}, {0, 0}, 1e9, false};

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
    float d = (B.getPos().x - A.getPos().x) * c.normal.x + (B.getPos().y - A.getPos().y) * c.normal.y;
    if (d < 0)
    {
        c.normal.x *= -1;
        c.normal.y *= -1;
    }
    c.collide = true;
    float min_dist = 1e6;
    getContactPoints(A_vertices, B_vertices, min_dist, c.contact);
    getContactPoints(B_vertices, A_vertices, min_dist, c.contact);
    return c;
}

void collide(Player& A, Player& B, const Collision& c, float e)
{
    float inv_A_mass = 1/A.getMass(), inv_B_mass = 1/B.getMass();
    float inv_comb_mass = inv_A_mass + inv_B_mass;
    float norm_A_mass = c.overlap * inv_A_mass / inv_comb_mass;
    float norm_B_mass = c.overlap * inv_B_mass / inv_comb_mass;

    A.setPos(A.getPos() - (c.normal * norm_A_mass));
    B.setPos(B.getPos() + (c.normal * norm_B_mass));

    Vector r_ap = (c.contact - A.getPos()).perp();
    Vector r_bp = (c.contact - B.getPos()).perp(); 
    Vector v_ap = A.getVel() + A.getW() * r_ap;
    Vector v_bp = B.getVel() + B.getW() * r_bp;

    float mag = Vector::dot((v_bp - v_ap), c.normal);

    if (mag < 0) // > 0 would mean they are already seperating
    {
        
        Vector r_ac_prep = (A.getPos() - c.contact).perp();
        Vector r_bc_prep = (B.getPos() - c.contact).perp();
        float r_ac_prep_n = std::pow(Vector::dot(r_ac_prep, c.normal), 2) / A.getI();
        float r_bc_prep_n = std::pow(Vector::dot(r_bc_prep, c.normal), 2) / B.getI();

        float denom = Vector::dot(c.normal, c.normal * inv_comb_mass) + r_ac_prep_n + r_bc_prep_n;

        float impulse = -(1 + e) * mag / denom;
        Vector impulse_norm = impulse * c.normal;

        A.pulse(impulse_norm * -1, c.contact);
        B.pulse(impulse_norm, c.contact);
    }
}

void elasticCollision(Player& A, Player& B)
{
    float m_a = A.getMass();
    float m_b = B.getMass();
    float v_a1_x = A.getVel().x;
    float v_b1_x = B.getVel().x;
    float v_a1_y = A.getVel().y;
    float v_b1_y = B.getVel().y;

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
    float v_a1_x = A.getVel().x;
    float v_b1_x = B.getVel().x;
    float v_a1_y = A.getVel().y;
    float v_b1_y = B.getVel().y;

    float dx = B.getPos().x - A.getPos().x + 400 - 25;
    float dy = B.getPos().y - A.getPos().y + 25 - 25;

    float dist = std::max(std::sqrt(dx * dx + dy * dy), 0.01f);
    float nx = dx/dist;
    float ny = dy/dist;

    float v_a1_n = v_a1_x * nx + v_a1_y * ny;
    float v_b1_n = v_b1_x * nx + v_b1_y * ny;

    float v_a2_n = (m_a * v_a1_n + m_b * v_b1_n + m_b * e * (v_b1_n - v_a1_n)) / (m_a + m_b);
    float v_b2_n = (m_a * v_a1_n + m_b * v_b1_n + m_a * e * (v_a1_n - v_b1_n)) / (m_a + m_b);

    float dKE = -0.5 * (1 - std::pow(e, 2)) *
                std::pow((v_a1_n - v_b1_n), 2) * m_a * m_b / (m_a + m_b);
    // do somthing with dKE

    float v_a2_x = v_a1_x + (v_a2_n - v_a1_n) * nx;
    float v_a2_y = v_a1_y + (v_a2_n - v_a1_n) * ny;
    float v_b2_x = v_b1_x + (v_b2_n - v_b1_n) * nx;
    float v_b2_y = v_b1_y + (v_b2_n - v_b1_n) * ny;

    A.updateVel(v_a2_x, v_a2_y);
    B.updateVel(v_b2_x, v_b2_y);
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
    Player groundS = {"groundS", 400, 550 + 25, "rectangle", 2e9};
    Player skyS = {"skyS", 400, 25, "rectangle", 2e9};

    Player sBob = {"sBob", 50, 100, "triangle"};
    Player sPat = {"sPat", 400, 100, "triangle"};
    
    // sBob.applyForce(-3*Pi/2, gravity);
    // sPat.applyForce(-3*Pi/2, gravity);

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
                sBob.pulse(-Pi/2, 100);
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
            groundS.updatePos();
            skyS.updatePos();

            Collision c = checkOverlap(sBob, sPat);
            if (c.collide)
                collide(sBob, sPat, c, e);

            c = checkOverlap(sBob, groundS);
            if (c.collide)
                collide(sBob, groundS, c, e);
            
            c = checkOverlap(sPat, groundS);
            if (c.collide)
                collide(sPat, groundS, c, e);
            
            c = checkOverlap(sBob, skyS);
            if (c.collide)
                collide(sBob, skyS, c, e);
            
            c = checkOverlap(sPat, skyS);
            if (c.collide)
                collide(sPat, skyS, c, e);
        }

        sBob.draw(window);
        sPat.draw(window);

        groundS.draw(window, sf::Color(196, 164, 132));
        skyS.draw(window, sf::Color(135, 206, 235));

        // -------------------

        window.display();
    }
}