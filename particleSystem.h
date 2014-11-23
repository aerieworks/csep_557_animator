/***********************
 * ParticleSystem class
 ***********************/

/**
 * The particle system class simply "manages" a collection of particles.
 * Its primary responsibility is to run the simulation, evolving particles
 * over time according to the applied forces using Euler's method.
 * This header file contains the functions that you are required to implement.
 * (i.e. the rest of the code relies on this interface)
 * In addition, there are a few suggested state variables included.
 * You should add to this class (and probably create new classes to model
 * particles and forces) to build your system.
 */

#ifndef __PARTICLE_SYSTEM_H__
#define __PARTICLE_SYSTEM_H__

#pragma warning(disable : 4018)
#pragma warning(disable : 4267)
#pragma warning(disable : 4311)
#pragma warning(disable : 4312)
#pragma warning(disable : 4244)
#pragma warning(disable : 4305)

#include <vector>
#include "vec.h"

class Particle {
public:
    Particle(const float mass, const Vec3f position) : mass(mass), position(position) {}
    float mass;
    Vec3f position;
    Vec3f velocity;
};

typedef std::vector<Particle>::iterator PARTICLE_ITER;
typedef std::vector<Particle>::const_iterator PARTICLE_CITER;

class Force {
public:
    virtual Vec3f computeForce(const Particle& particle, const float time) const = 0;
};

typedef std::vector<Force*>::const_iterator FORCE_PTR_CITER;

class ConstantForce : public Force {
    const Vec3f acceleration;
public:
    ConstantForce(const Vec3f acceleration) : acceleration(acceleration) {}
    Vec3f computeForce(const Particle& particle, const float time) const {
        return particle.mass * acceleration;
    }
};

class ViscousDrag : public Force {
    const float dragCoefficient;
public:
    ViscousDrag(const float dragCoefficient) : dragCoefficient(dragCoefficient) {}
    Vec3f computeForce(const Particle& particle, const float time) const {
        return particle.velocity * -dragCoefficient;
    }
};

class ParticleCollection {
public:
    std::vector<Particle> particles;
    std::vector<Force*> forces;
    
    virtual void addForce(Force& force) { forces.push_back(&force); }
    virtual void updateParticles(const float time, const float deltaT);
    virtual void drawParticles(const float time);
};

typedef std::vector<ParticleCollection*>::iterator PARTICLE_COLLECTION_PTR_ITER;

class ParticleEmitter : public ParticleCollection {
    Vec3f position;
    const float particleMass;
    const float emissionRate;
    float lastEmissionTime;
public:
    ParticleEmitter(const float particleMass, const float emissionRate) : particleMass(particleMass), emissionRate(emissionRate), lastEmissionTime(0) {}
    
    void setPosition(const Vec3f position) { this->position = position; }
    
    virtual void updateParticles(const float time, const float deltaT) {
        if (time - lastEmissionTime > (1.0 / emissionRate)) {
            particles.push_back(Particle(particleMass, position));
            lastEmissionTime = time;
        }
        
        ParticleCollection::updateParticles(time, deltaT);
    }
};

class ParticleSystem {
    std::vector<ParticleCollection*> particleCollections;
    
public:
	/** Constructor **/
	ParticleSystem();


	/** Destructor **/
	virtual ~ParticleSystem();
    
    void addParticleCollection(ParticleCollection* pc) { particleCollections.push_back(pc); }
    
	/** Simulation fxns **/
	// This fxn should render all particles in the system,
	// at current time t.
	virtual void drawParticles(float t);

	// This fxn should save the configuration of all particles
	// at current time t.
	virtual void bakeParticles(float t);

	// This function should compute forces acting on all particles
	// and update their state (pos and vel) appropriately.
	virtual void computeForcesAndUpdateParticles(float t);

	// This function should reset the system to its initial state.
	// When you need to reset your simulation, PLEASE USE THIS FXN.
	// It sets some state variables that the UI requires to properly
	// update the display.  Ditto for the following two functions.
	virtual void resetSimulation(float t);

	// This function should start the simulation
	virtual void startSimulation(float t);

	// This function should stop the simulation
	virtual void stopSimulation(float t);

	// This function should clear out your data structure
	// of baked particles (without leaking memory).
	virtual void clearBaked();	



	// These accessor fxns are implemented for you
	float getBakeStartTime() { return bake_start_time; }
	float getBakeEndTime() { return bake_end_time; }
	float getBakeFps() { return bake_fps; }
	bool isSimulate() { return simulate; }
	bool isDirty() { return dirty; }
	void setDirty(bool d) { dirty = d; }



protected:
	


	/** Some baking-related state **/
	float bake_fps;						// frame rate at which simulation was baked
	float bake_start_time;				// time at which baking started 
										// These 2 variables are used by the UI for
										// updating the grey indicator 
	float bake_end_time;				// time at which baking ended

	/** General state variables **/
	bool simulate;						// flag for simulation mode
	bool dirty;							// flag for updating ui (don't worry about this)

};


#endif	// __PARTICLE_SYSTEM_H__
