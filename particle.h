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
    bool isFixed;
    
public:
    float mass;
    Vec3f position;
    Vec3f velocity;

    float radius;
    Vec3f color;

    Particle(const Particle& p) : Particle(p.creationTime, p.mass, p.position, p.radius, p.color) { velocity = p.velocity; }
    Particle(const float creationTime, const float mass, const Vec3f position, const float radius, const Vec3f color) :
        creationTime(creationTime),
        mass(mass),
        position(position),
        radius(radius),
        isFixed(false),
        color(color) {}
    
    float getCreationTime() const { return creationTime; }
    bool fixed() const { return isFixed; }
    void fixed(const bool isFixed) { this->isFixed = isFixed; }
};

#endif
