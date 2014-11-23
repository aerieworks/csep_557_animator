#pragma warning(disable : 4786)

#include "particleSystem.h"
#include "modelerdraw.h"

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>

using namespace std;

static float prevT;

#define TIME_EPSILON (0.01)

void ParticleCollection::updateParticles(const float time, const float deltaT)
{
    // Default implementation assumes all forces in collection apply to all particles.
    for (auto particle = particles.begin(); particle != particles.end(); )
    {
        if (time - particle->getCreationTime() >= maxParticleAge)
        {
            // Remove particles that have expired.
            particles.erase(particle);
        }
        else
        {
            Vec3f deltaV;
            for (auto force = forces.cbegin(); force != forces.cend(); ++force)
            {
                deltaV += (*force)->computeForce(*particle, time) * deltaT;
            }
            particle->position += particle->velocity * deltaT;
            particle->velocity += deltaV;
            ++particle;
        }
    }
}

void ParticleCollection::drawParticles(const float time)
{
    // Default implementation renders all particles as simple brown spheres.
    for (PARTICLE_CITER p_iter = particles.cbegin(); p_iter != particles.cend(); ++p_iter)
    {
        glPushMatrix();
        setDiffuseColor(0.43, 0.26, 0.09);
        glTranslatef(p_iter->position[0], p_iter->position[1], p_iter->position[2]);
        drawSphere(0.01);
        glPopMatrix();
    }
}

void ParticleEmitter::updateParticles(const float time, const float deltaT)
{
    if (time - lastEmissionTime > (1.0 / emissionRate)) {
        Particle p(time, particleMass, position);
        p.velocity = jitterVelocity();
        particles.push_back(p);
        lastEmissionTime = time;
    }
    
    ParticleCollection::updateParticles(time, deltaT);
}

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
    int iterations = 0;
    while (prevT < t)
    {
        iterations += 1;
        const float nextT = fmin(t, prevT + TIME_EPSILON);
        computeForcesAndUpdateParticles(nextT);
        drawParticles(nextT);
    }
    printf("Computed %d iterationss.\n", iterations);
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
    prevT = 0;

	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Compute forces and update particles **/
void ParticleSystem::computeForcesAndUpdateParticles(float t)
{
    if (!simulate)
    {
        return;
    }
    
    const float deltaT = t - prevT;
    for (PARTICLE_COLLECTION_PTR_ITER pc_iter = particleCollections.begin(); pc_iter != particleCollections.end(); ++pc_iter)
    {
        ParticleCollection* pc = *pc_iter;
        pc->updateParticles(t, deltaT);
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
    
    for (PARTICLE_COLLECTION_PTR_ITER pc_iter = particleCollections.begin(); pc_iter != particleCollections.end(); ++pc_iter)
    {
        (*pc_iter)->drawParticles(t);
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





