#ifndef PHYSICS_H
#define PHYSICS_H

#include <iostream>
#include "vector.h"
#include "player.h"

namespace Physics
{
    struct Collision
    {
        Vector normal = {0, 0};
        Vector contact = {0, 0};
        float overlap = 1e9;
        bool collide = false;
    };

    void getContactPoints(
        const std::vector<Vector>& A_vertices,  const std::vector<Vector>& B_vertices,
        float& min_dist, Vector& contact
    );
    std::vector<Vector> getAxis(const std::vector<Vector>& vertices);
    std::array<float, 2> project(const Vector& axis, const std::vector<Vector>& vertices);
    Collision checkOverlap(Player& A, Player& B);
    void collide(Player& A, Player& B, const Collision& c, float e);
}

#endif