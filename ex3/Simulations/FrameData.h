#pragma once

#include <functional>

enum Kernel {
    Constant = 0,
    Linear = 1,
    Quadratic = 2,
    WeakElectricCharge = 3,
    ElectricCharge = 4
};

struct FrameData {
    float time;
    float mass;
    float forceScaling;
    float damping;
    Kernel kernelId;
    std::function<float(float)> kernel;
    Vec3 gravity;
    float gravityScale;
};

extern float sphereRadius;
