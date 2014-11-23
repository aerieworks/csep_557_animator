//
//  particle.h
//  Animator
//
//  Created by Annabelle Richard on 11/23/14.
//  Copyright (c) 2014 Annabelle Richard. All rights reserved.
//

#ifndef Animator_particle_h
#define Animator_particle_h

#include "vec.h"

class Particle {
    float creationTime;
    
public:
    float mass;
    Vec3f position;
    Vec3f velocity;
    
    Particle(const Particle& p) : Particle(p.creationTime, p.mass, p.position) { velocity = p.velocity; }
    Particle(const float creationTime, const float mass, const Vec3f position) : creationTime(creationTime), mass(mass), position(position) {}
    
    float getCreationTime() const { return creationTime; }
};

#endif
