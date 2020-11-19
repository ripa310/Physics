#pragma once

#include "Simulator.h"
#include "FrameData.h"

struct Sphere {
    Vec3 pos;
    Vec3 vel;
    Vec3 ai;
};

struct Accelerator;

struct SphereSystem {
    std::vector<Sphere> spheres;
    Vec3 offset;
    Vec3 color;
    std::unique_ptr<Accelerator> accelerator;
    FrameData data;

    void SimulateTimestep(FrameData& data);

    SphereSystem(std::unique_ptr<Accelerator> a);
    void addRandomBall();
    void removeLastBall();
    bool collision(Sphere& s);
};

struct Accelerator {
    virtual void CalculateCollisions(SphereSystem& system, FrameData& data) = 0;
};

struct BruteForceAccelerator : public Accelerator {
    void CalculateCollisions(SphereSystem& system, FrameData& data) override;
};

struct GridCell {
    std::vector<Sphere*> spheres;
    int x;
    int y;
    int z;
};

struct GridAccelerator : public Accelerator {

    std::vector<GridCell> cells;

    GridAccelerator();

    void CalculateCollisions(SphereSystem& system, FrameData& data) override;
};
