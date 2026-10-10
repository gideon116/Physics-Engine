#include "physics.h"

namespace Physics
{
    void getContactPoints(
        const std::vector<Vector>& A_vertices,  const std::vector<Vector>& B_vertices,
        float& min_dist, Vector& contact
    )
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

    void checkOverlap(Player& A, Player& B, const float e)
    {
        Vector normal = {0, 0};
        Vector contact = {0, 0};
        float overlap = 1e9;
        bool collide = false;

        const std::vector<Vector>& A_vertices = A.getPoints();
        const std::vector<Vector>& B_vertices = B.getPoints();

        std::vector<Vector> axis_1 = getAxis(A_vertices);
        std::vector<Vector> axis_2 = getAxis(B_vertices);

        for (int i = 0; i < A_vertices.size(); i++)
        {
            std::array<float, 2> pa = project(axis_1[i], A_vertices);
            std::array<float, 2> pb = project(axis_1[i], B_vertices);
            if (pa[0] > pb[1] || pa[1] < pb[0])
                // no overlap
                return;
            float top = std::min(pa[1], pb[1]);
            float bottom = std::max(pa[0], pb[0]);
            float curr_overlap = std::abs(top - bottom);
            // containment
            if ((pa[1] < pb[1] && pa[0] > pb[0]) || (pa[1] > pb[1] && pa[0] < pb[0]))
                curr_overlap += std::min(std::abs(pa[0] - pb[0]), std::abs(pa[1] - pb[1]));

            if (overlap > curr_overlap)
            {
                overlap = curr_overlap;
                normal = axis_1[i];
            }
        }
        for (int i = 0; i < B_vertices.size(); i++)
        {
            std::array<float, 2> pa = project(axis_2[i], A_vertices);
            std::array<float, 2> pb = project(axis_2[i], B_vertices);

            if (pa[0] > pb[1] || pa[1] < pb[0])
                // no overlap
                return;

            float top = std::min(pa[1], pb[1]);
            float bottom = std::max(pa[0], pb[0]);
            float curr_overlap = std::abs(top - bottom);

            // containment
            if ((pa[1] < pb[1] && pa[0] > pb[0]) || (pa[1] > pb[1] && pa[0] < pb[0]))
                curr_overlap += std::min(std::abs(pa[0] - pb[0]), std::abs(pa[1] - pb[1]));

            if (overlap > curr_overlap)
            {
                overlap = curr_overlap;
                normal = axis_2[i];
            }
        }
        float d = Vector::dot((B.getPos() - A.getPos()), normal);
        if (d < 0)
            normal *= -1;
        collide = true;
        float min_dist = 1e6;
        getContactPoints(A_vertices, B_vertices, min_dist, contact);
        getContactPoints(B_vertices, A_vertices, min_dist, contact);

        if (collide)
            collidePlayers(A, B, normal, contact, overlap, e);
    }

    void collidePlayers(
        Player& A, Player& B, const Vector& normal, Vector& contact, float& overlap, float e
    )
    {
        float inv_A_mass = 1/A.getMass(), inv_B_mass = 1/B.getMass();
        float inv_comb_mass = inv_A_mass + inv_B_mass;
        float norm_A_mass = overlap * inv_A_mass / inv_comb_mass;
        float norm_B_mass = overlap * inv_B_mass / inv_comb_mass;

        A.setPos(A.getPos() - (normal * norm_A_mass));
        B.setPos(B.getPos() + (normal * norm_B_mass));

        Vector r_ap = (contact - A.getPos()).perp();
        Vector r_bp = (contact - B.getPos()).perp(); 
        Vector v_ap = A.getVel() + A.getW() * r_ap;
        Vector v_bp = B.getVel() + B.getW() * r_bp;

        float mag = Vector::dot((v_bp - v_ap), normal);

        if (mag < 0) // > 0 would mean they are already seperating
        {
            Vector r_ac_prep = (A.getPos() - contact).perp();
            Vector r_bc_prep = (B.getPos() - contact).perp();
            float r_ac_prep_n = std::pow(Vector::dot(r_ac_prep, normal), 2) / A.getI();
            float r_bc_prep_n = std::pow(Vector::dot(r_bc_prep, normal), 2) / B.getI();

            float denom = Vector::dot(normal, normal * inv_comb_mass) + r_ac_prep_n + r_bc_prep_n;

            float impulse = -(1 + e) * mag / denom;
            Vector impulse_norm = impulse * normal;

            A.pulse(impulse_norm * -1, contact);
            B.pulse(impulse_norm, contact);
        }
    }
}