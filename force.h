//
//  force.h
//  Animator
//
//  Created by Annabelle Richard on 11/23/14.
//  Copyright (c) 2014 Annabelle Richard. All rights reserved.
//

#ifndef Animator_force_h
#define Animator_force_h

#include "particle.h"
#include "vec.h"

class Force {
public:
    virtual Vec3f computeForce(const Particle& particle, const float time) const = 0;
};

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

#endif
