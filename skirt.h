//
//  skirt.h
//  Animator
//
//  Created by Annabelle Richard on 11/24/14.
//  Copyright (c) 2014 Annabelle Richard. All rights reserved.
//

#ifndef Animator_skirt_h
#define Animator_skirt_h

#include <vector>
#include "modelerapp.h"
#include "particleSystem.h"

static const int PARTICLES_PER_RING = 4;
static const float RING_WIDTH = 0.5;

class Skirt : public ParticleCollection {
    int rows, cols;
    ConstantForce gravity;
    std::vector<Spring*> springForces;
    
    void sanitizeVector(Vec3f& v) const {
        for (int i = 0; i < 3; i++) {
            if (fabs(v[i]) < 10e-5) {
                v[i] = 0;
            }
        }
    }
    
    void addSpringForce(const int p1Index, const int p2Index) {
        cerr << "Adding spring: " << p1Index << ", " << p2Index << endl;
        Spring* springForce = new Spring(50, 75.0, (particles[p1Index]->position - particles[p2Index]->position).length(), *particles[p1Index], *particles[p2Index]);
        springForces.push_back(springForce);
        this->addForce(*springForce);
    }
public:
    Skirt(const Mat4f modelViewMatrix, const float waistRadius, const float length)
        : ParticleCollection(),
          gravity(Vec3f(0, -9.81, 0)),
          cols(8),
          rows(5) {

        // For each ring...
        for (int y = 0; y < rows; y++) {
            // For each radial position...
            for (int x = 0; x < cols; x++) {
                const int pIndex = y * cols + x;
                
                // Create a particle.
                Vec3f position(modelViewMatrix * Vec4f(0.5 * x, 0.5 * y, 0, 1));
                sanitizeVector(position);
                
                Particle* p = new Particle(0, 0.1, position, 0.1, Vec3f((double)(cols - x)/(double)cols, 0, (double)y/(double)rows));
                this->addParticle(p);
                
                if (x == 0 && (y == 0 || y == rows - 1)) {
                    p->fixed(true);
                }
                
                if (x > 0) {
                    addSpringForce(pIndex - 1, pIndex);
                }
                
                if (y > 0) {
                    addSpringForce(pIndex - cols, pIndex);
                    if (x > 0) {
                        addSpringForce(pIndex - cols - 1, pIndex);
                    }
                    if (x < cols - 1) {
                        addSpringForce(pIndex - cols + 1, pIndex);
                    }
                }
            }
        }
        
        this->addForce(gravity);
    }
    
    ~Skirt() {
        ParticleCollection::~ParticleCollection();
        
        // Clean up spring forces.
        for (auto spring_iter = springForces.begin(); spring_iter != springForces.end(); ++spring_iter) {
            delete (*spring_iter);
        }
    }
    
    void addSurfaces() {
        for (int x = 0; x < cols - 1; x++) {
            for (int y = 0; y < rows - 1; y++) {
                Particle* p1 = particles[y * cols + x];
                Particle* p2 = particles[y * cols + x + 1];
                Particle* p3 = particles[(y + 1) * cols + x];
                Particle* p4 = particles[(y + 1) * cols + x + 1];
                CollidableQuad* q = new CollidableQuad(p1->position, p2->position, p3->position, p4->position);
                q->exclude(p1);
                q->exclude(p2);
                q->exclude(p3);
                q->exclude(p4);
                ModelerApplication::Instance()->AddCollidable(q);
            }
        }
    }
    
    virtual void drawParticles(const float time) {
        setAmbientColor(0.8, 0, 0.8);   
        glBegin(GL_TRIANGLES);
            for (int x = 0; x < cols - 1; x++) {
                for (int y = 0; y < rows - 1; y++) {
                    Particle* p1 = particles[y * cols + x];
                    Particle* p2 = particles[y * cols + x + 1];
                    Particle* p3 = particles[(y + 1) * cols + x];
                    Particle* p4 = particles[(y + 1) * cols + x + 1];
                    glVertex3f(p1->position[0], p1->position[1], p1->position[2]);
                    glVertex3f(p4->position[0], p4->position[1], p4->position[2]);
                    glVertex3f(p3->position[0], p3->position[1], p3->position[2]);

                    glVertex3f(p1->position[0], p1->position[1], p1->position[2]);
                    glVertex3f(p2->position[0], p2->position[1], p2->position[2]);
                    glVertex3f(p4->position[0], p4->position[1], p4->position[2]);
                }
            }
        glEnd();
    }
};

#endif
