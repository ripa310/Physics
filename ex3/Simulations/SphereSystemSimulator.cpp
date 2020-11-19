#include "SphereSystemSimulator.h"

float sphereRadius = 0.1f;

std::function<float(float)> SphereSystemSimulator::m_Kernels[5] = {
    [](float x) {return 1.0f; },              // Constant, m_iKernel = 0
    [](float x) {return 1.0f - x; },          // Linear, m_iKernel = 1, as given in the exercise Sheet, x = d/2r
    [](float x) {return (1.0f - x)*(1.0f - x); }, // Quadratic, m_iKernel = 2
    [](float x) {return 1.0f / (x)-1.0f; },     // Weak Electric Charge, m_iKernel = 3
    [](float x) {return 1.0f / (x*x) - 1.0f; },   // Electric Charge, m_iKernel = 4
};

enum TestCase {
    Test,
    Demo1,
    Demo2,
    Demo3
};

// SphereSystemSimulator member functions
SphereSystemSimulator::SphereSystemSimulator() {
    data.mass = 1;
    data.damping = 1;
    data.forceScaling = 500;
    data.kernelId = Quadratic;
    data.damping = 5;
    data.gravity = Vec3(0, -9.81f, 0);
    data.gravityScale = 1;
    m_iNumSpheres = 2;
}
const char* SphereSystemSimulator::getTestCasesStr() {
    return "Test,Demo 1, Demo 2, Demo 3";
}

void SphereSystemSimulator::initUI(DrawingUtilitiesClass * DUC) {
    this->DUC = DUC;

    TwAddVarCB(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32, [](const void *value, void *clientData) {
        int v = *(int*)value;
        auto& sim = *(SphereSystemSimulator*)clientData;
        sim.m_iNumSpheres = v;
    }, [](void *value, void *clientData) {
        *(int*)value = ((SphereSystemSimulator*)clientData)->m_iNumSpheres;
    }, this, "min=0");
        
    TwType TW_TYPE_TESTCASE = TwDefineEnumFromString("Kernel", "Constant,Linear,Quadratic,WeakElectricCharge,ElectricCharge");
    TwAddVarRW(DUC->g_pTweakBar, "Kernel", TW_TYPE_TESTCASE, &data.kernelId, "");
    TwAddVarRW(DUC->g_pTweakBar, "Force Scaling", TW_TYPE_FLOAT, &data.forceScaling, "min=0.0 step=1.0");
    TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &data.damping, "min=0.0 step=0.1");
    TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &data.mass, "min=0.0 step=0.1");
    TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &data.gravityScale, "min=0.0 step=0.1");
}

void SphereSystemSimulator::reset() {

}

void SphereSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
    for (SphereSystem& sys : systems) {
        DUC->setUpLighting(Vec3(0.0), Vec3(0.0), 1, sys.color);
        for (Sphere& sph : sys.spheres) {
            DUC->drawSphere(sph.pos + sys.offset, Vec3(sphereRadius));
        }
    }
}

void SphereSystemSimulator::notifyCaseChanged(int testCase) {
    m_iTestCase = testCase;
    switch (m_iTestCase) {
    case TestCase::Test: {
        systems.clear();

        auto acc = std::make_unique<GridAccelerator>();
        systems.emplace_back(std::move(acc));

        auto& sys = systems[0];
        sys.color = Vec3(1, 0, 0);
        //sys.spheres.push_back({ Vec3(0, 0.05, 0), Vec3(1, 0.5, 0.7), Vec3(0, 0, 0) });
        //sys.spheres.push_back({ Vec3(0.5, 0, 0), Vec3(-1, 0, 0), Vec3(0, 0, 0) });


        //auto acc2 = std::make_unique<BruteForceAccelerator>();
        //systems.emplace_back(std::move(acc2), m_fRadius);

        //auto& sys2 = systems[1];
        //sys2.color = Vec3(0, 1, 0);
        //sys2.spheres.push_back({ Vec3(-0.5, 0, 0), Vec3(0, -1, 0), Vec3(0, 0, 0) });
        //sys2.spheres.push_back({ Vec3(0, -0.5, 0), Vec3(0, 1, 0), Vec3(0, 0, 0) });
        break;
    }
    
    case TestCase::Demo1: {
        systems.clear();
        break;
    }
    
    case TestCase::Demo2: {
        systems.clear();
        break;
    }
    

    case TestCase::Demo3: {
        sphereRadius = 0.05f;
        
        systems.clear();

        auto bruteForceAcc = std::make_unique<BruteForceAccelerator>();
        systems.push_back(SphereSystem(std::move(bruteForceAcc)));

        auto gridAcc = std::make_unique<GridAccelerator>();
        systems.push_back(SphereSystem(std::move(gridAcc)));

        auto& sys1 = systems[0];
        sys1.color = Vec3(1, 0, 0);

        auto& sys2 = systems[1];
        sys2.color = Vec3(0, 1, 0);


        float x = -0.5f + sphereRadius;
        float y = 0.0f;
        float z = -0.5f + sphereRadius;


        for (int l = 0; l < 4; l++) {
            y += 2.1f * sphereRadius;

            x = -0.5f + sphereRadius + l * 0.01f;
            for (int i = 0; i < 5; i++) {
                x += 2.1f * sphereRadius;

                z = -0.5f + sphereRadius + (l + i) * 0.005f;
                for (int k = 0; k < 5; k++) {
                    z += 2.1f * sphereRadius;

                    sys1.spheres.push_back({
                        Vec3(x, y, z),
                        Vec3(0, 0, 0),
                        Vec3(0, 0, 0)
                    });
                    sys2.spheres.push_back({
                        Vec3(x, y, z),
                        Vec3(0, 0, 0),
                        Vec3(0, 0, 0)
                    });
                }
            }
        }
        break;
    }
    }
}

void SphereSystemSimulator::externalForcesCalculations(float timeElapsed) {

}

void SphereSystemSimulator::simulateTimestep(float timeStep) {

    switch (m_iTestCase) {
    case TestCase::Test:
        for (SphereSystem& s : systems) {
            while (s.spheres.size() > m_iNumSpheres) {
                s.removeLastBall();
            }

            while (s.spheres.size() < m_iNumSpheres) {
                s.addRandomBall();
            }
        }
        break;
    }

    data.time = timeStep;
    data.kernel = m_Kernels[data.kernelId];

    for (SphereSystem& s : systems) {
        s.SimulateTimestep(data);
    }
}

void SphereSystemSimulator::onClick(int x, int y) {

}

void SphereSystemSimulator::onMouse(int x, int y) {

}

void SphereSystemSimulator::addRandomBall() {
    for (SphereSystem& s : systems) {
        s.addRandomBall();
    }
}

void SphereSystemSimulator::removeLastBall() {
    for (SphereSystem& s : systems) {
        s.removeLastBall();
    }
}
