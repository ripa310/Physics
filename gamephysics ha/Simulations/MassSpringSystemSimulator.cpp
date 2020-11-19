#include "MassSpringSystemSimulator.h"

extern float g_fTimestep;

enum TestCase {
    Demo1,
    Demo2,
    Demo3,
    Demo4,
};

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
    m_gravity = Vec3(0, 0, 0);
    m_gravity_str =0;
}

const char * MassSpringSystemSimulator::getTestCasesStr()
{
    return "Demo 1,Demo 2, Demo 3,Demo 4";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
    this->DUC = DUC;
    switch (m_iTestCase)
    {
    case TestCase::Demo1: break;
    case TestCase::Demo4:
        TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1 step=1");
        TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.01 step=0.01");
        TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0.01 step=0.1");
        TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INT32, &m_iIntegrator, "min=0");
        TwAddVarRW(DUC->g_pTweakBar, "Gravity Dir", TW_TYPE_DIR3D, &m_gravity, "");
        TwAddVarRW(DUC->g_pTweakBar, "Gravity Strength", TW_TYPE_FLOAT, &m_gravity_str, "step=0.1");
        break;
    }
}

void MassSpringSystemSimulator::reset()
{
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext * pd3dImmediateContext)
{
    switch (m_iTestCase)
    {
    case TestCase::Demo1: break;
    case TestCase::Demo2:
    case TestCase::Demo3:
    case TestCase::Demo4:
        for (auto& p : m_massPoints) {
            float scale = 0.01;
            this->DUC->drawSphere(p.pos, Vec3(scale));
        }
        this->DUC->beginLine();
        for (auto& e : m_springs) {
            Vec3& p1 = getPositionOfMassPoint(e.point1);
            Vec3& p2 = getPositionOfMassPoint(e.point2);
            this->DUC->drawLine(p1, { 1, 1, 1 }, p2, { 1, 1, 1 });
        }
        this->DUC->endLine();
        break;
    }
}



void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
    switch (m_iTestCase)
    {
    case TestCase::Demo1:
        cout << "Test !\n";
        m_massPoints.clear();
        m_springs.clear();

        addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
        addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

        addSpring(0, 1, 1);

        setIntegrator(EULER);
        setStiffness(40);
        setMass(1);
        setDampingFactor(0);
        m_gravity_str = 0;
        g_fTimestep = 0.1f;

        break;

    case TestCase::Demo2:
        cout << "Demo 2!\n";
        m_massPoints.clear();
        m_springs.clear();

        addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
        addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

        addSpring(0, 1, 1);

        setIntegrator(EULER);
        setStiffness(40);
        setMass(1);
        setDampingFactor(0);
        m_gravity_str = 0;
        g_fTimestep = 0.005f;

        break;

    case TestCase::Demo3:
        cout << "Demo 3!\n";
        m_massPoints.clear();
        m_springs.clear();

        addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
        addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

        addSpring(0, 1, 1);

        setIntegrator(MIDPOINT);
        setStiffness(40);
        setMass(1);
        setDampingFactor(0);
        m_gravity_str = 0;
        g_fTimestep = 0.005f;

        break;


    case TestCase::Demo4: {
        cout << "Demo 4!\n";
        m_massPoints.clear();
        m_springs.clear();

        setIntegrator(MIDPOINT);
        setStiffness(200);
        setMass(1);
        setDampingFactor(1);
        m_gravity = Vec3(0, -1, 0);
        m_gravity_str = 1;
        g_fTimestep = 0.001f;

        float size = 0.125f;
        m_massPoints.push_back(MassPoint(-size,  -size * 2, -size));
        m_massPoints.push_back(MassPoint(size,   -size * 2, -size));
        m_massPoints.push_back(MassPoint(size,   -size * 2, size));
        m_massPoints.push_back(MassPoint(-size,  -size * 2, size));

        m_massPoints.push_back(MassPoint(-size, 0, -size));
        m_massPoints.push_back(MassPoint(size,  0, -size));
        m_massPoints.push_back(MassPoint(size,  0, size));
        m_massPoints.push_back(MassPoint(-size, 0, size));

        m_massPoints.push_back(MassPoint(-size, size * 2, -size));
        m_massPoints.push_back(MassPoint(size,  size * 2, -size));
        m_massPoints.push_back(MassPoint(size,  size * 2, size));
        m_massPoints.push_back(MassPoint(-size, size * 2, size));

        float f = 1.0f;
        // horizontal
        addSpringRel(0, 1, f);
        addSpringRel(1, 2, f);
        addSpringRel(2, 3, f);
        addSpringRel(3, 0, f);

        addSpringRel(4, 5, f);
        addSpringRel(5, 6, f);
        addSpringRel(6, 7, f);
        addSpringRel(7, 4, f);

        addSpringRel(8, 9, f);
        addSpringRel(9, 10, f);
        addSpringRel(10, 11, f);
        addSpringRel(11, 8, f);

        // vertical
        addSpringRel(0, 4, f);
        addSpringRel(1, 5, f);
        addSpringRel(2, 6, f);
        addSpringRel(3, 7, f);

        addSpringRel(4, 8, f);
        addSpringRel(5, 9, f);
        addSpringRel(6, 10, f);
        addSpringRel(7, 11, f);

        addSpringRel(0, 8, f);
        addSpringRel(1, 9, f);
        addSpringRel(2, 10, f);
        addSpringRel(3, 11, f);

        // diagonal
        addSpringRel(0, 5, f);
        addSpringRel(1, 4, f);

        addSpringRel(1, 6, f);
        addSpringRel(2, 5, f);

        addSpringRel(2, 7, f);
        addSpringRel(3, 6, f);

        addSpringRel(3, 4, f);
        addSpringRel(0, 7, f);

        //
        addSpringRel(4, 9, f);
        addSpringRel(5, 8, f);

        addSpringRel(5, 10, f);
        addSpringRel(6, 9, f);

        addSpringRel(6, 11, f);
        addSpringRel(7, 10, f);

        addSpringRel(7, 8, f);
        addSpringRel(4, 11, f);

        // diagonal horizontal
        addSpringRel(0, 2, f);
        addSpringRel(1, 3, f);

        addSpringRel(4, 6, f);
        addSpringRel(5, 7, f);

        addSpringRel(8, 10, f);
        addSpringRel(9, 11, f);

        // x
        addSpringRel(0, 6, f);
        addSpringRel(1, 7, f);
        addSpringRel(2, 4, f);
        addSpringRel(3, 5, f);

        addSpringRel(4, 10, f);
        addSpringRel(5, 11, f);
        addSpringRel(6, 8, f);
        addSpringRel(7, 9, f);

        addSpringRel(0, 10, f);
        addSpringRel(1, 11, f);
        addSpringRel(2, 8, f);
        addSpringRel(3, 9, f);

        break;
    }
    default:
        cout << "Empty Test!\n";
        break;
    }
}

float MassSpringSystemSimulator::getDistanceBetween(int a, int b) {
    Vec3 p0 = getPositionOfMassPoint(a);
    Vec3 p1 = getPositionOfMassPoint(b);
    float d = p0.squaredDistanceTo(p1);
    return sqrtf(d);
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
    // damping
    for (MassPoint& mp : m_massPoints) {
        mp.force += m_gravity * m_gravity_str * m_fMass;
        mp.force -= mp.vel * m_fDamping;
    }
}

void MassSpringSystemSimulator::calculateInternalForces()
{
    for (Spring& s : m_springs) {
        MassPoint& p1 = m_massPoints[s.point1];
        MassPoint& p2 = m_massPoints[s.point2];

        // nico
        Vec3 dist = p1.pos - p2.pos;
        Vec3 normal = dist;
        normalize(normal);
        normal *= s.len;
        Vec3 x = dist - normal;
        Vec3 force = x * m_fStiffness;
        p1.force -= force;
        p2.force += force;

        // minh
        //Vec3 distV = p1.pos - p2.pos;
        //Vec3 normal = distV;
        //normalize(normal);
        //float dist = (sqrtf(distV.x * distV.x + distV.y * distV.y + distV.z * distV.z) - s.len) * m_fStiffness;
        //Vec3 force = dist * normal;
        //p1.force -= force;
        //p2.force += force;
    }
}

template <typename T>
void Clip(T& f, T& v, T min, T max) {
    T off = 0.001;
    if (f < min) {
        f = min + off;
        v = 0;
    }
    if (f > max) {
        f = max - off;
        v = 0;
    }
}

void MassSpringSystemSimulator::eulerIntegration(float timeStep) {
    calculateInternalForces();

    for (MassPoint& p : m_massPoints) {
        if (p.fixed) continue;

        p.pos += p.vel * timeStep;
        Vec3 a = p.force / m_fMass;
        p.vel += a * timeStep;

        p.force = 0;
    }
}

void MassSpringSystemSimulator::midpointIntegration(float timeStep) {
    calculateInternalForces();

    // integrate positions
    for (MassPoint& p : m_massPoints) {
        if (p.fixed) continue;

        Vec3 a = p.force / m_fMass;

        p.xtmp = p.pos;
        p.vtmp = p.vel;
        
        p.pos = p.pos + 0.5f * timeStep * p.vel;
        p.vel = p.vel + 0.5f * timeStep * a;

        p.force = 0;
    }

    calculateInternalForces();
    externalForcesCalculations(timeStep);

    // integrate velocities
    for (MassPoint& p : m_massPoints) {
        if (p.fixed) continue;

        p.pos = p.xtmp + timeStep * p.vel;

        Vec3 a = p.force / m_fMass;
        p.vel = p.vtmp + timeStep * a;

        p.force = 0;
    }
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
    switch (m_iIntegrator) {
    case EULER:
        eulerIntegration(timeStep);
        break;
    case MIDPOINT:
        midpointIntegration(timeStep);
        break;
    }


    if (m_iTestCase == TestCase::Demo4) {
        // collision detection
        for (auto& p : m_massPoints) {
            double boxBounds = 0.5;
            Clip(p.pos.x, p.vel.x, -boxBounds, boxBounds);
            Clip(p.pos.y, p.vel.y, -boxBounds, boxBounds);
            Clip(p.pos.z, p.vel.z, -boxBounds, boxBounds);
            p.force = 0;
        }
    }

    if (m_iTestCase == TestCase::Demo1) {
        int i = 0;
        for (auto& p : m_massPoints) {
            cout << "P" << i << "( pos = " << m_massPoints[i].pos << ", vel = " << m_massPoints[i].vel << ")\n";
            i++;
        }
    }
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass)
{
    m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
    m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
    m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
    m_massPoints.push_back({ position, Velocity, isFixed });
    return m_massPoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
    m_springs.push_back({ masspoint1, masspoint2, initialLength });
}

void MassSpringSystemSimulator::addSpringRel(int masspoint1, int masspoint2, float rel)
{
    addSpring(masspoint1, masspoint2, getDistanceBetween(masspoint1, masspoint2) * rel);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
    return m_massPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
    return m_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
    return m_massPoints[index].pos;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
    return m_massPoints[index].vel;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
    m_externalForce += force;
}
