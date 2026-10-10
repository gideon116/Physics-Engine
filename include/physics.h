#ifndef PHYSICS_H
#define PHYSICS_H

#include <iostream>
#include "vector.h"
#include "player.h"

namespace Physics
{
    void getContactPoints(
        const std::vector<Vector>& A_vertices,  const std::vector<Vector>& B_vertices,
        float& min_dist, Vector& contact
    );
    std::vector<Vector> getAxis(const std::vector<Vector>& vertices);
    std::array<float, 2> project(const Vector& axis, const std::vector<Vector>& vertices);
    void checkOverlap(Player& A, Player& B, const float e);
    void collidePlayers(
        Player& A, Player& B, const Vector& normal, Vector& contact, float& overlap, float e
    );
}

#endif