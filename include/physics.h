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
    void getAxis(const Vector* vertices, Vector* axis, const int num_vertices);
    std::array<float, 2> project(const Vector& axis, const Vector* vertices, const int num_vertices);
    void checkOverlap(Player& A, Player& B, const float e);
    void collidePlayers(
        Player& A, Player& B, const Vector& normal, Vector& contact, float& overlap, float e
    );

    void checkOverlap(Player& A, Player& B, Vector* A_vertices_raw, Vector* B_vertices_raw, const float e);
}

#endif