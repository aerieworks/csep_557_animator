#pragma warning(disable : 4786)

#include "particleSystem.h"
#include "modelerdraw.h"

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
//#include <limits>

using namespace std;

static float prevT;

#define TIME_EPSILON (0.01)

/***************
 * Constructors
 ***************/

ParticleSystem::ParticleSystem() 
{
	// TODO

}





/*************
 * Destructor
 *************/

ParticleSystem::~ParticleSystem() 
{
	// TODO

}


/******************
 * Simulation fxns
 ******************/

void ParticleSystem::spawnParticle(Vec3f position)
{
    particles.push_back(Particle(0.25, position));
}

void ParticleSystem::addForce(Force& force)
{
    forces.push_back(&force);
}

/** Start the simulation */
void ParticleSystem::startSimulation(float t)
{
	// These values are used by the UI ...
	// negative bake_end_time indicates that simulation
	// is still progressing, and allows the
	// indicator window above the time slider
	// to correctly show the "baked" region
	// in grey.
	bake_end_time = -1;
	simulate = true;
	dirty = true;

    printf("Starting simulation.\n");
    while (prevT < t)
    {
        const float nextT = fmin(t, prevT + TIME_EPSILON);
        computeForcesAndUpdateParticles(nextT);
        drawParticles(nextT);
    }
}

/** Stop the simulation */
void ParticleSystem::stopSimulation(float t)
{
    
	// TODO
    printf("Stopping simulation.\n");
	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Reset the simulation */
void ParticleSystem::resetSimulation(float t)
{
    printf("Resetting simulation.\n");
    lastEmissionTime = 0;
    prevT = 0;

	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Compute forces and update particles **/
void ParticleSystem::computeForcesAndUpdateParticles(float t)
{
    const float deltaT = t - prevT;
    if (deltaT < 0)
    {
        resetSimulation(t);
    }
    
    for (PARTICLE_ITER particle = particles.begin(); particle != particles.end(); ++particle)
    {
        Vec3f deltaV;
        for (FORCE_PTR_CITER force = forces.cbegin(); force != forces.cend(); ++force)
        {
            deltaV += (*force)->computeForce(*particle, t) * deltaT;
        }
        particle->position += particle->velocity * deltaT;
        particle->velocity += deltaV;
    }
    
    if (t - lastEmissionTime >= 1.0)
    {
        spawnParticle(Vec3f(0, 5.0, 0));
        lastEmissionTime = t;
    }
    
	// Debugging info
	if( t - prevT > .04 )
		printf("(!!) Dropped Frame %lf (!!)\n", t-prevT);
	prevT = t;
}


/** Render particles */
void ParticleSystem::drawParticles(float t)
{
    if (!simulate)
    {
        return;
    }
    
    for (PARTICLE_CITER p_iter = particles.cbegin(); p_iter != particles.cend(); ++p_iter)
    {
        glPushMatrix();
        setDiffuseColor(0.43, 0.26, 0.09);
        glTranslatef(p_iter->position[0], p_iter->position[1], p_iter->position[2]);
        drawSphere(0.1);
        glPopMatrix();
    }
}





/** Adds the current configuration of particles to
  * your data structure for storing baked particles **/
void ParticleSystem::bakeParticles(float t) 
{

	// TODO (baking is extra credit)
}

/** Clears out your data structure of baked particles */
void ParticleSystem::clearBaked()
{

	// TODO (baking is extra credit)
}





