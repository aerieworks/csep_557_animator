//
//  surface.h
//  Animator
//
//  Created by Annabelle Richard on 11/23/14.
//  Copyright (c) 2014 Annabelle Richard. All rights reserved.
//

#ifndef Animator_surface_h
#define Animator_surface_h

#include <math.h>
#include <iostream>
#include "mat.h"
#include "particle.h"
#include "vec.h"

using namespace std;

#define COLLISION_EPSILON 0.05

struct Collision {
    Vec3f location;
    Vec3f normal;
    float time;
};

class Collidable {
public:
    virtual ~Collidable() {}
    
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

class CollidableQuad : public Surface {
    Vec3f d;
    float surfaceD;
public:
    CollidableQuad(const Vec3f a, const Vec3f b, const Vec3f c, const Vec3f d) : Surface(a, b, c), d(d) {
        surfaceD = normal * a;
    }
    
    virtual bool testCollision(const Particle& particle, const float deltaT, Collision& collision) const {
        if (Surface::testCollision(particle, deltaT, collision)) {
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

class CollidableCylinder : public Collidable {
    Mat4f toWorldTxMatrix;
    Mat4f fromWorldTxMatrix;
    float height;
    float radius;
    
public:
    CollidableCylinder(const Mat4f& modelViewMatrix, const float height, const float radius) : height(height), radius(radius) {
        toWorldTxMatrix = modelViewMatrix;
        fromWorldTxMatrix = modelViewMatrix.inverse();
    }
    
    virtual bool testCollision(const Particle& particle, const float deltaT, Collision& collision) const {
        Vec4f velocityTx = fromWorldTxMatrix * Vec4f(particle.velocity[0], particle.velocity[1], particle.velocity[2], 0);
        Vec4f startPosTx = fromWorldTxMatrix * Vec4f(particle.position[0], particle.position[1], particle.position[2], 1);
        Vec4f endPosTx = startPosTx + deltaT * velocityTx;
        if (endPosTx[2] <= height + COLLISION_EPSILON &&
            endPosTx[2] >= -COLLISION_EPSILON &&
            sqrt(endPosTx[0] * endPosTx[0] + endPosTx[1] * endPosTx[1]) <= radius + COLLISION_EPSILON) {
            const float theta = atan(endPosTx[1] / endPosTx[0]);
            Vec4f normal = Vec4f(endPosTx[0], endPosTx[1], 0, 0);
            normal.normalize();
            const Vec4f normalWorldTx = toWorldTxMatrix * normal;

            collision.location = particle.position + deltaT * particle.velocity;
            collision.time = deltaT;
            collision.normal = Vec3f(normalWorldTx[0], normalWorldTx[1], normalWorldTx[2]);
            return true;
        }
        
        return false;
    }
};

#endif
