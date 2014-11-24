//
//  surface.h
//  Animator
//
//  Created by Annabelle Richard on 11/23/14.
//  Copyright (c) 2014 Annabelle Richard. All rights reserved.
//

#ifndef Animator_surface_h
#define Animator_surface_h

#include <iostream>
#include "particle.h"
#include "vec.h"

using namespace std;

#define COLLISION_EPSILON (M_PI / 32)

struct Collision {
    Vec3f location;
    Vec3f normal;
    float time;
};

class Collidable {
public:
    virtual bool testCollision(const Particle& particle, const float deltaT, Collision& collision) const = 0;
};

class Surface : public Collidable {
protected:
    Vec3f normal, a, b, c;
    float barycentricDenominator;
    
    virtual bool testCollision(const Particle& particle, const float deltaT, Collision& collision) const {
        if ((particle.position - a) * normal < COLLISION_EPSILON && particle.velocity * normal <= 0) {
            collision.normal = normal;
            collision.time = deltaT;
            return true;
        }
        
        return false;
    }
    
public:
    Surface(const Vec3f a, const Vec3f b, const Vec3f c) : a(a), b(b), c(c) {
        normal = (b - a) ^ (c - a);
        normal.normalize();
    }
};

class QuadSurface : public Surface {
    Vec3f d;
    float surfaceD;
public:
    QuadSurface(const Vec3f a, const Vec3f b, const Vec3f c, const Vec3f d) : Surface(a, b, c), d(d) {
        surfaceD = normal * a;
    }
    
    virtual bool testCollision(const Particle& particle, const float deltaT, Collision& collision) const {
        if (Surface::testCollision(particle, deltaT, collision)) {
            // Time when ray intersects plane is found by:
            // t = (d - n * ray.position) / (n * ray.direction)
            const float time = (surfaceD - (normal * particle.position)) / (normal * particle.velocity);
            if (time <= deltaT) {
                Vec3f q = particle.position + time * particle.velocity;
                if ((((b - a) ^ (q - a)) * normal >= 0 &&
                     ((c - b) ^ (q - b)) * normal >= 0 &&
                     ((a - c) ^ (q - c)) * normal >= 0) ||
                    (((c - a) ^ (q - a)) * normal >= 0 &&
                     ((d - c) ^ (q - c)) * normal >= 0 &&
                     ((a - d) ^ (q - d)) * normal >= 0)) {
                    collision.location = q;
                    collision.time = time;
                    return true;
                }
            }
        }
        
        return false;
    }
};



#endif
