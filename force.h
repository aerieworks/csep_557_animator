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
        return acceleration;
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

class Spring : public Force {
    const float ks, kd, restLength;
    const Particle& p1;
    const Particle& p2;
    
public:
    Spring(const float ks, const float kd, const float restLength, const Particle& p1, const Particle& p2) : ks(ks), kd(kd), restLength(restLength), p1(p1), p2(p2) {}
    Vec3f computeForce(const Particle& particle, const float time) const {
        if (&particle != &p1 && &particle != &p2) {
            return Vec3f();
        }
        
        const Vec3f deltaX = p1.position - p2.position;
        const float deltaXLength = deltaX.length();
        const Vec3f normDeltaX = deltaX / deltaXLength;
        const Vec3f deltaV = p1.velocity - p2.velocity;
        const Vec3f force = -(ks * (deltaXLength - restLength) + kd * (deltaV * normDeltaX)) * normDeltaX;
        return (&particle == &p1) ? force : -force;
    }
};

#endif
