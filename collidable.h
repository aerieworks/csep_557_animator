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

#define COLLISION_EPSILON 0.075
#define HEIGHT_EPSILON 0.025

struct Collision {
    Vec3f location;
    Vec3f normal;
    float time;
};

static int testCount = 0;
class Collidable {
public:
    virtual ~Collidable() {}
    
    virtual bool testCollision(const Particle& particle, const float deltaT, Collision& collision) const = 0;
};

class Surface : public Collidable {
protected:
    Vec3f point, normal;
    
    virtual bool isInBounds(const Particle& particle, const float deltaT, Collision& collision) const = 0;
    
public:
    Surface(const Vec3f point, const Vec3f normal) : point(point), normal(normal) {
        this->normal.normalize();
    }
    
    virtual bool testCollision(const Particle& particle, const float deltaT, Collision& collision) const {
        if ((particle.position - point) * normal <= COLLISION_EPSILON) {
            if (particle.velocity * normal <= 0) {
                collision.normal = normal;
                collision.time = deltaT;
                if (isInBounds(particle, deltaT, collision)) {
                    return true;
                }
            }
        }
        
        return false;
    }
};

class CollidableQuad : public Surface {
    const Vec3f a, b, c, d;
    const float surfaceD;
    
protected:
    virtual bool isInBounds(const Particle& particle, const float deltaT, Collision& collision) const {
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
        
        return false;
    }
    
public:
    CollidableQuad(const Vec3f a, const Vec3f b, const Vec3f c, const Vec3f d) :
        Surface(a, (b - a) ^ (c - a)),
        a(a),
        b(b),
        c(c),
        d(d),
        surfaceD(normal * a) {
    }
};

class CollidableCircle : public Surface {
    const float radius;
    
protected:
    virtual bool isInBounds(const Particle& particle, const float deltaT, Collision& collision) const {
        collision.location = particle.position + deltaT * particle.velocity;
        return (collision.location - point).length() <= radius + COLLISION_EPSILON;
    }
    
public:
    CollidableCircle(const Vec3f origin, const Vec3f normal, const float radius) : Surface(origin, normal), radius(radius) {}
};

class CollidableCylinder : public Collidable {
    Mat4f toWorldTxMatrix;
    Mat4f fromWorldTxMatrix;
    float height;
    float radius;
    CollidableCircle top;
    CollidableCircle bottom;
    
public:
    CollidableCylinder(const Mat4f& modelViewMatrix, const float height, const float radius) :
        height(height),
        radius(radius),
        bottom(Vec3f(modelViewMatrix * Vec4f(0, 0, 0, 1)), Vec3f(modelViewMatrix * Vec4f(0, 0, -1, 0)), radius),
        top(Vec3f(modelViewMatrix * Vec4f(0, 0, height, 1)), Vec3f(modelViewMatrix * Vec4f(0, 0, 1, 0)), radius) {
            
        toWorldTxMatrix = modelViewMatrix;
        fromWorldTxMatrix = modelViewMatrix.inverse();
    }
    
    virtual bool testCollision(const Particle& particle, const float deltaT, Collision& collision) const {
        testCount += 1;
        Vec4f velocityTx = fromWorldTxMatrix * Vec4f(particle.velocity[0], particle.velocity[1], particle.velocity[2], 0);
        Vec4f startPosTx = fromWorldTxMatrix * Vec4f(particle.position[0], particle.position[1], particle.position[2], 1);

        if (top.testCollision(particle, deltaT, collision) || bottom.testCollision(particle, deltaT, collision)) {
            return true;
        }
        
        Vec4f endPosTx = startPosTx + deltaT * velocityTx;
        if (endPosTx[2] <= height + HEIGHT_EPSILON &&
            endPosTx[2] >= -HEIGHT_EPSILON &&
            sqrt(endPosTx[0] * endPosTx[0] + endPosTx[1] * endPosTx[1]) <= radius + COLLISION_EPSILON) {
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
