#include "SphereSystem.h"
#include "FrameData.h"

#define CELL(name, x, y, z, numCells) {\
    int ind = index(x, y, z, numCells);\
    if (ind < 0 || ind >= cells.size()) {\
        std::cout << __LINE__ << ": index out of range " << ind << " : " << cells.size() << "\n";\
        __debugbreak();\
    }}\
    GridCell& name = cells[index(x, y, z, numCells)]

SphereSystem::SphereSystem(std::unique_ptr<Accelerator> a)
    : accelerator(std::move(a)) {
    data.mass = 1;
    data.damping = 1;
    data.forceScaling = 500;
    data.kernelId = Quadratic;
    data.damping = 5;
    data.gravity = Vec3(0, -9.81f, 0);
    data.gravityScale = 1;
}

void SphereSystem::SimulateTimestep(FrameData& data) {
    for (Sphere& s : spheres) {
        Vec3 v_12 = s.vel;
        s.vel += s.ai * data.time;
        s.pos += v_12 * data.time;
        s.ai = data.gravity * data.gravityScale; // gravity
        s.ai -= s.vel * data.damping / data.mass;
    }

    // calculate 
    accelerator->CalculateCollisions(*this, data);
}

double randDouble() {
    return (double)rand() / (double)RAND_MAX;
}

double randDouble(double min, double max) {
    return randDouble() * (max - min) + min;
}

void SphereSystem::addRandomBall() {
    Sphere s{Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0)};
    int tries = 0;
    do {
        s.pos.x = randDouble(-0.5 + sphereRadius, 0.5 - sphereRadius);
        s.pos.y = randDouble(-0.5 + sphereRadius, 0.5 - sphereRadius);
        s.pos.z = randDouble(-0.5 + sphereRadius, 0.5 - sphereRadius);
        tries++;
    } while (collision(s) && tries < 100);

    if (tries >= 100) {
        return;
    }

    spheres.push_back(s);
}

void SphereSystem::removeLastBall() {
    spheres.pop_back();
}

bool SphereSystem::collision(Sphere& a) {
    for (Sphere& b : spheres) {
        if (&a == &b)
            continue;

        float d = a.pos.squaredDistanceTo(b.pos);
        float x = d / (sphereRadius * sphereRadius * 4); // distance / 2*radius

        if (x>1)
            continue;

        return true;
    }

    return false;
}

void BruteForceAccelerator::CalculateCollisions(SphereSystem& system, FrameData& data) {
    float radius = sphereRadius;
    float forceScaling = data.forceScaling;
    float m = data.mass;

    for (Sphere& a : system.spheres) {
        // wall collision
        Vec3 d1 = a.pos + Vec3(0.5, 0.5, 0.5) - radius;
        Vec3 d2 = -a.pos + Vec3(0.5, 0.5, 0.5) - radius;

        if (d1.x < 0) {
            float d = d1.x + 2 * radius;
            float x = d / (radius * 2);
            float f = forceScaling * data.kernel(x);
            Vec3 force(f, 0, 0);
            a.ai += force / m;
        }
        if (d1.y < 0) {
            float d = d1.y + 2 * radius;
            float x = d / (radius * 2);
            float f = forceScaling * data.kernel(x);
            Vec3 force(0, f, 0);
            a.ai += force / m;
        }
        if (d1.z < 0) {
            float d = d1.z + 2 * radius;
            float x = d / (radius * 2);
            float f = forceScaling * data.kernel(x);
            Vec3 force(0, 0, f);
            a.ai += force / m;
        }
        if (d2.x < 0) {
            float d = d2.x + 2 * radius;
            float x = d / (radius * 2);
            float f = forceScaling * data.kernel(x);
            Vec3 force(f, 0, 0);
            a.ai -= force / m;
        }
        if (d2.y < 0) {
            float d = d2.y + 2 * radius;
            float x = d / (radius * 2);
            float f = forceScaling * data.kernel(x);
            Vec3 force(0, f, 0);
            a.ai -= force / m;
        }
        if (d2.z < 0) {
            float d = d2.z + 2 * radius;
            float x = d / (radius * 2);
            float f = forceScaling * data.kernel(x);
            Vec3 force(0, 0, f);
            a.ai -= force / m;
        }

        for (Sphere& b : system.spheres) {
            if (&a == &b)
                continue;
            
            float d = a.pos.squaredDistanceTo(b.pos);
            float x = d / (radius * radius * 4); // distance / 2*radius

            if(x>1)
                continue;
            
            Vec3 d_norm = b.pos - a.pos;
            normalize(d_norm);

            float f = forceScaling * data.kernel(x);

            Vec3 force = f * d_norm;

            a.ai -= force / m;
            b.ai += force / m;
        }
    }
}

GridAccelerator::GridAccelerator() {
    float gridSpacing = 2 * sphereRadius;
    int numCellsRow = (int) (1 / gridSpacing);
    int numCells = numCellsRow * numCellsRow * numCellsRow;
    cells.reserve(numCells);
    for (int i = 0; i < numCells; i++) {
        cells.emplace_back();
        cells.back().spheres.reserve(10);
    }
}

int index(int x, int y, int z, int gridSize) {
    return x + y * gridSize + z * gridSize * gridSize;
}

void CalculateCollisionOtherCells(Sphere* a, GridCell& c, FrameData& data)
{
    for (Sphere* b : c.spheres)
    {

        if (a == b)
            continue;

        float d = a->pos.squaredDistanceTo(b->pos);
        float x = d / (sphereRadius * sphereRadius * 4); // distance / 2*radius

        if (x>1)
            continue;

        Vec3 d_norm = b->pos - a->pos;
        normalize(d_norm);

        float f = data.forceScaling * data.kernel(x);

        Vec3 force = f * d_norm;

        a->ai -= force / data.mass;
        b->ai += force / data.mass;


    }
}

void GridAccelerator::CalculateCollisions(SphereSystem& system, FrameData& data) {
    float radius = sphereRadius;
    float forceScaling = data.forceScaling;
    float m = data.mass;

    float gridSpacing = 2 * sphereRadius;
    int numCells = (int)(1 / gridSpacing);

    for (GridCell& c : cells) {
        c.spheres.clear();
    }

    for (Sphere& a : system.spheres) {
        Vec3 idx = (a.pos + 0.5) * numCells;

        int x = (int)idx.x;
        int y = (int)idx.y;
        int z = (int)idx.z;

        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (z < 0) z = 0;

        if (x >= numCells) x = numCells - 1;
        if (y >= numCells) y = numCells - 1;
        if (z >= numCells) z = numCells - 1;


        CELL(c, x, y, z, numCells);

        c.spheres.push_back(&a);
        c.x = x;
        c.y = y;
        c.z = z;



        // wall collision
        Vec3 d1 = a.pos + Vec3(0.5, 0.5, 0.5) - radius;
        Vec3 d2 = -a.pos + Vec3(0.5, 0.5, 0.5) - radius;

        if (d1.x < 0) {
            float d = d1.x + 2 * radius;
            float x = d / (radius * 2);
            float f = forceScaling * data.kernel(x);
            Vec3 force(f, 0, 0);
            a.ai += force / m;
        }
        if (d1.y < 0) {
            float d = d1.y + 2 * radius;
            float x = d / (radius * 2);
            float f = forceScaling * data.kernel(x);
            Vec3 force(0, f, 0);
            a.ai += force / m;
        }
        if (d1.z < 0) {
            float d = d1.z + 2 * radius;
            float x = d / (radius * 2);
            float f = forceScaling * data.kernel(x);
            Vec3 force(0, 0, f);
            a.ai += force / m;
        }
        if (d2.x < 0) {
            float d = d2.x + 2 * radius;
            float x = d / (radius * 2);
            float f = forceScaling * data.kernel(x);
            Vec3 force(f, 0, 0);
            a.ai -= force / m;
        }
        if (d2.y < 0) {
            float d = d2.y + 2 * radius;
            float x = d / (radius * 2);
            float f = forceScaling * data.kernel(x);
            Vec3 force(0, f, 0);
            a.ai -= force / m;
        }
        if (d2.z < 0) {
            float d = d2.z + 2 * radius;
            float x = d / (radius * 2);
            float f = forceScaling * data.kernel(x);
            Vec3 force(0, 0, f);
            a.ai -= force / m;
        }
    }


    //for (GridCell& c : cells) {
    //    std::cout << c.spheres.size() << ", ";
    //}
    //std::cout << "\n";

    for (GridCell& c : cells)
    {
        for (Sphere* a : c.spheres)
        {
            for (int i = c.x-1; i < c.x+2; i++)
            {
                if (i < 0)
                    continue;
                else if (i >= numCells)
                    break;
                for (int j = c.y - 1; j < c.y + 2; j++)
                {
                    if (j < 0)
                        continue;
                    else if (j >= numCells)
                        break;
                    for (int k = c.z - 1; k < c.z + 2; k++)
                    {
                        if (k < 0)
                            continue;
                        else if (k >= numCells)
                            break;

                        CELL(cell, i, j, k, numCells);

                        CalculateCollisionOtherCells(a, cell, data);
                    }
                }
            }
        }
    }


}
