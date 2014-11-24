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
#include "force.h"
#include "particle.h"
#include "collidable.h"
#include "vec.h"

class ParticleCollection {
    float maxParticleAge;
    
    virtual void updatePosition(const float deltaT, Particle& particle);
public:
    std::vector<Particle> particles;
    std::vector<Force*> forces;
    
    ParticleCollection() : maxParticleAge(-1) {}
    
    virtual void addParticle(Particle particle) { particles.push_back(particle); }
    virtual void addForce(Force& force) { forces.push_back(&force); }
    virtual void updateParticles(const float time, const float deltaT);
    virtual void drawParticles(const float time);
    
    void setMaxParticleAge(const float maxParticleAge) { this->maxParticleAge = maxParticleAge; }
};

class ParticleEmitter : public ParticleCollection {
    Vec3f position;
    float particleMass;
    float particleRadius;
    Vec3f particleColor;
    float emissionRate;
    Vec3f jitter;
    float lastEmissionTime;
    
    Vec3f jitterVelocity() {
        Vec3f velocity;
        for (int i = 0; i < 3; i++) {
            const float factor = (float)(rand() % 100) / 100.0;
            velocity[i] = factor * jitter[i] - jitter[i] / 2.0;
        }
        
        return velocity;
    }
    
public:
    ParticleEmitter(const float particleMass, const float particleRadius, const Vec3f particleColor) :
        ParticleCollection(),
        particleMass(particleMass),
        particleRadius(particleRadius),
        particleColor(particleColor) {}
    
    virtual void updateParticles(const float time, const float deltaT);

    void setPosition(const Vec3f position) { this->position = position; }
    void setEmissionRate(const float emissionRate) { this->emissionRate = emissionRate; }
    void setJitter(const Vec3f jitter) { this->jitter = jitter; }
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
