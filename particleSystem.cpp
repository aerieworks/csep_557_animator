#pragma warning(disable : 4786)

#include "modelerapp.h"
#include "modelerdraw.h"
#include "particleSystem.h"

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <iostream>

using namespace std;

static float prevT;

#define TIME_EPSILON (0.01)

void ParticleCollection::updatePosition(const float deltaT, Particle& particle)
{
    auto collidables = ModelerApplication::Instance()->GetCollidables();
    
    float remainingT = deltaT;
    for (auto collidable = collidables.cbegin(); collidable != collidables.cend(); ++collidable)
    {
        Collision collision;
        if ((*collidable)->testCollision(particle, deltaT, collision))
        {
            ModelerApplication::Instance()->AddCollision(collision);
            particle.position += collision.time * particle.velocity + particle.radius * collision.normal;
            Vec3f vNormal = (collision.normal * particle.velocity) * collision.normal;
            Vec3f vTangent = particle.velocity - vNormal;
            particle.velocity = vTangent - 0.5 * vNormal;
            remainingT -= collision.time;
            break;
        }
    }

    particle.position += particle.velocity * remainingT;
}

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
                deltaV += (*force)->computeForce(*particle, time) / particle->mass * deltaT;
            }
            updatePosition(deltaT, *particle);
            particle->velocity += deltaV;
            ++particle;
        }
    }
}

void ParticleCollection::drawParticles(const float time)
{
    // Default implementation renders all particles as simple brown spheres.
    for (auto p_iter = particles.cbegin(); p_iter != particles.cend(); ++p_iter)
    {
        glPushMatrix();
        setDiffuseColor(p_iter->color[0], p_iter->color[1], p_iter->color[2]);
        glTranslatef(p_iter->position[0], p_iter->position[1], p_iter->position[2]);
        drawSphere(p_iter->radius);
        glPopMatrix();
    }
}

void ParticleEmitter::updateParticles(const float time, const float deltaT)
{
    if (time - lastEmissionTime > (1.0 / emissionRate)) {
        Particle p(time, particleMass, position, particleRadius, particleColor);
        p.velocity = jitterVelocity();
        p.velocity[0] -= 3.0;
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
    for (auto pc_iter = particleCollections.begin(); pc_iter != particleCollections.end(); ++pc_iter)
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
    
    for (auto pc_iter = particleCollections.begin(); pc_iter != particleCollections.end(); ++pc_iter)
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





