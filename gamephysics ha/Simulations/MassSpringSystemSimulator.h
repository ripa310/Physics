#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

struct MassPoint {
    Vec3 pos;
    Vec3 xtmp;
    Vec3 vel;
    Vec3 vtmp;
    bool fixed;
    Vec3 force;

    MassPoint() {}

    MassPoint(Vec3 pos, Vec3 vel, bool fixed)
        : pos(pos), vel(vel), fixed(fixed), force({ 0, 0, 0 }) {}

    MassPoint(float x, float y, float z)
        : pos({ x, y, z }), vel({ 0, 0, 0 }), fixed(false), force({ 0, 0, 0 }) {}
};

struct Spring {
    int point1;
    int point2;
    float len;
};

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
    void addSpringRel(int masspoint1, int masspoint2, float rel = 1);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);

    float getDistanceBetween(int a, int b);

    void eulerIntegration(float time);
    void midpointIntegration(float time);
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

    void calculateInternalForces();
private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

    Vec3 m_gravity;
    float m_gravity_str;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

private:
    std::vector<MassPoint> m_massPoints;
    std::vector<Spring> m_springs;
};
#endif