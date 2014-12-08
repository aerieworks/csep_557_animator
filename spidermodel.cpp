// The sample box model.  You should build a file
// very similar to this for when you make your model in
// order to plug in to the animator project.

#pragma warning (disable : 4305)
#pragma warning (disable : 4244)
#pragma warning(disable : 4786)
#pragma warning (disable : 4312)

#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include "mat.h"
#include "particleSystem.h"
#include "vec.h"
#include <FL/gl.h>
#include <math.h>
#include <iostream>

#define PI 3.14159265359

using namespace std;

// This is a list of the controls for the RobotArm
// We'll use these constants to access the values 
// of the controls from the user interface.
enum BoxModelControls
{ 
    TIME, XPOS, YPOS, ZPOS, DIRECTION,
    ABDOMEN_LENGTH, ABDOMEN_WIDTH, ABDOMEN_HEIGHT, ABDOMEN_OFFSET,
    HEAD_LENGTH, HEAD_WIDTH, HEAD_HEIGHT,
    LEG_RADIUS, TOE_RADIUS, LEG_UPPER_LENGTH, LEG_MIDDLE_LENGTH, LEG_LOWER_LENGTH, FOOT_LENGTH,
    _1_HIP_POS, _2_HIP_POS, _3_HIP_POS, _4_HIP_POS,
    L1_FOOT_X, L1_FOOT_Y, L1_FOOT_Z,
    L2_FOOT_X, L2_FOOT_Y, L2_FOOT_Z,
    L3_FOOT_X, L3_FOOT_Y, L3_FOOT_Z,
    L4_FOOT_X, L4_FOOT_Y, L4_FOOT_Z,
    R1_FOOT_X, R1_FOOT_Y, R1_FOOT_Z,
    R2_FOOT_X, R2_FOOT_Y, R2_FOOT_Z,
    R3_FOOT_X, R3_FOOT_Y, R3_FOOT_Z,
    R4_FOOT_X, R4_FOOT_Y, R4_FOOT_Z,
    MAX_DELTA,
    EMIT_SNOW,
    NUMCONTROLS,
};

enum LegSide
{
    LEFT = -1,
    RIGHT = 1
};

// We'll be getting the instance of the application a lot;
// might as well have it as a macro.

#define VAL(x) (ModelerApplication::Instance()->GetControlValue(x))

// Create a sinusoidal transition from v0 to v0+vd
#define sinize(v0, vd, t0, t1, t) ((v0) + (vd) * ((cos(((double)(t) / (t1)) * PI + PI) + 1) / 2))

#define DEG(x) (180/PI * (x))
#define RAD(x) ((x) * PI/180)

Mat4d getModelViewMatrix()
{
    Mat4d matrix;
    glGetDoublev(GL_MODELVIEW_MATRIX, matrix.n);
    // OpenGL stores matrices in column-major order, but the Mat class uses row-major order.
    // Transpose the matrix to swap the rows and columns into their correct arrangement.
    return matrix.transpose();
}

class Bone
{
public:
    double angle;
    const Vec3d axis;

    Bone(const BoxModelControls length, const Vec3d axis) : length(length), axis(axis), angle(0) {}

    double getLength()
    {
        if (length == NUMCONTROLS)
        {
            return 0;
        }
        return VAL(length);
    }
private:
    const BoxModelControls length;
};

class Leg
{
public:
    Leg(LegSide side, BoxModelControls position, BoxModelControls footX, BoxModelControls footY, BoxModelControls footZ)
        : side(side), position(position), footX(footX), footY(footY), footZ(footZ)
    {
        jointCount = 4;
        bones = new Bone*[jointCount];
        int index = 0;
        bones[index++] = new Bone(NUMCONTROLS, Vec3d(0, side, 0));
        bones[index++] = new Bone(LEG_UPPER_LENGTH, Vec3d(1, 0, 0));
        bones[index++] = new Bone(LEG_MIDDLE_LENGTH, Vec3d(1, 0, 0));
        bones[index++] = new Bone(LEG_LOWER_LENGTH, Vec3d(1, 0, 0));
    }
    
    ~Leg()
    {
        for (int i = 0; i < jointCount; i++)
        {
            delete bones[i];
        }
        delete bones;
    }
    
    Mat4d getModelViewTransformations(const Mat4d cameraTransforms)
    {
        // Multiply by the inverse of the camera transformations to get just the ModelView transformations.
        return cameraTransforms.inverse() * getModelViewMatrix();
    }
    
    void draw(const Mat4d& cameraTransforms)
    {
        const double legPositionRadians = RAD(VAL(position));
        Vec4d curHipLocation(side * VAL(HEAD_WIDTH) * sin(legPositionRadians), 0, VAL(HEAD_LENGTH) * cos(legPositionRadians), 1);
        
        // Calculate the hip's world coordinates, so we can figure out if it has moved.
        Mat4d mvTransforms = getModelViewTransformations(cameraTransforms);
        Vec4d curHipLocationWorld = mvTransforms * curHipLocation;
        
        // Get the foot's world coordinates (VALs are in world space already), so we can figure out if it has moved.
        Vec4d curFootLocation(VAL(footX), VAL(footY), VAL(footZ), 1);
        
        glPushMatrix();
        glTranslated(curHipLocation[0], curHipLocation[1], curHipLocation[2]);
        drawSphere(VAL(LEG_RADIUS));
        
        // If neither the hip nor the foot have moved in world space, then we don't need to recalculate the limb position.
        // We can skip the expensive IK operations.
        if (curHipLocationWorld != prevHipLocation || curFootLocation != prevFootLocation)
        {
            printf("Recalculating foot position.\n");
            prevHipLocation = curHipLocationWorld;
            prevFootLocation = curFootLocation;
            
            // Convert foot world coordinates into current space (hip-relative coordinates).
            mvTransforms = getModelViewTransformations(cameraTransforms).inverse();
            Vec4d target = mvTransforms * curFootLocation;
            findAngles(Vec3d(target[0], target[1], target[2]));
        }
        
        drawWithAngles();
        glPopMatrix();
    }
    
    Vec3f getFootPosition()
    {
        return Vec3f(VAL(footX), VAL(footY), VAL(footZ));
    }
private:
    const double JACOBIAN_EPSILON = 0.01;
    const double JACOBIAN_THRESHOLD = 0.1;
    const int JACOBIAN_ITERATION_LIMIT = 800;
    const double JACOBIAN_MAX_DELTA = 0.75;
    
    LegSide side;
    BoxModelControls position;
    Bone** bones;
    int jointCount;
    Vec4d prevHipLocation;
    Vec4d prevFootLocation;
    const BoxModelControls footX, footY, footZ;

    int iteration;
    
    void drawWithAngles()
    {
        glPushMatrix();
        int index = 0;
        // Upper
        glRotated(DEG(bones[index++]->angle), 0, -side, 0);
        glRotated(DEG(bones[index++]->angle), -1, 0, 0);
        drawCylinder(VAL(LEG_UPPER_LENGTH), VAL(LEG_RADIUS), VAL(LEG_RADIUS));
        glTranslated(0, 0, VAL(LEG_UPPER_LENGTH));
        drawSphere(VAL(LEG_RADIUS));
        // Middle
        glRotated(DEG(bones[index++]->angle), -1, 0, 0);
        drawCylinder(VAL(LEG_MIDDLE_LENGTH), VAL(LEG_RADIUS), VAL(LEG_RADIUS));
        glTranslated(0, 0, VAL(LEG_MIDDLE_LENGTH));
        drawSphere(VAL(LEG_RADIUS));
        // Lower
        glRotated(DEG(bones[index++]->angle), -1, 0, 0);
        drawCylinder(VAL(LEG_LOWER_LENGTH), VAL(LEG_RADIUS), VAL(LEG_RADIUS));
        glTranslated(0, 0, VAL(LEG_LOWER_LENGTH));
        drawSphere(VAL(LEG_RADIUS));
        glPopMatrix();
    }
    
    double* findAngles(const Vec3d target)
    {
        bool done = false;
        for (int i = 0; i < jointCount; i++)
        {
            // Reset the bones to their initial condition.
            bones[i]->angle = 0;
        }
        double* deltas = new double[jointCount];
        iteration = 0;
        Vec4d effector(buildTransformationMatrix(bones, jointCount) * Vec4d(0, 0, 0, 1));
        while (!done && iteration < JACOBIAN_ITERATION_LIMIT)
        {
            // Initialize Jacobian.
            double** J = new double*[3];
            // Fill the Jacobian.
            for (int r = 0; r < 3; r++)
            {
                J[r] = new double[jointCount];
            }
            buildJacobian(J, jointCount, bones);
            // Apply the adjustment value.
            double* deltaTheta = solveJacobian(J, target - effector, jointCount);
            // Adjust bone positions.
            for (int i = 0; i < jointCount; i++)
            {
                bones[i]->angle += deltaTheta[i];
                deltas[i] = bones[i]->angle;
            }
            effector = buildTransformationMatrix(bones, jointCount) * Vec4d(0, 0, 0, 1);
            double curDifference = getMagnitude(target - effector);
            if (curDifference <= JACOBIAN_THRESHOLD)
            {
                // Close enough, call it good.
                done = true;
            }

            iteration += 1;
            
            for (int r = 0; r < 3; r++)
            {
                  delete J[r];
            }
            delete J;
            delete deltaTheta;
        }
 
        return deltas;
    }

    void buildJacobian(double ** J, const int jointCount, Bone* bones[])
    {
        // Programmatically formulating the actual forward kinematics (and their derivative) is hard.
        // Computing matrix transformations is easy.
        // So we approximate a Jacobian matrix of the forward kinematics by computing the transformation
        // matrix for the limb given current joint angles, and given small changes to each joint (one by one).
        Vec4d origin(0, 0, 0, 1);
        Vec4d start = buildTransformationMatrix(bones, jointCount) * origin;
        
        for (int i = 0; i < jointCount; i++)
        {
            Vec4d p = buildTransformationMatrix(bones, jointCount, i) * origin;
            for (int j = 0; j < 3; j++)
            {
                J[j][i] = p[j] - start[j];
            }
        }
    }
    
    Mat4d buildTransformationMatrix(Bone* bones[], const int count, const int deltaIndex = -1)
    {
        Mat4d m;
        for (int i = 0; i < count; i++)
        {
            double angle = bones[i]->angle + (deltaIndex == i ? 1 : 0);
            m = m * m.createRotation(angle, -bones[i]->axis[0], -bones[i]->axis[1], -bones[i]->axis[2])
                * m.createTranslation(0, 0, bones[i]->getLength());
        }
        return m;
    }
    
    void printMatrix(double** mat, const int rows, const int cols)
    {
        for (int r = 0; r < rows; r++)
        {
            printVector(mat[r], cols);
        }
    }
    
    void printVector(double* vec, const int size)
    {
        for (int i = 0; i < size; i++)
        {
            cerr << vec[i] << " ";
        }
        cerr << endl;
    }
    
    double** transpose(double** mat, const int srcRows, const int srcCols)
    {
        double** t = new double*[srcCols];
        for (int destR = 0; destR < srcCols; destR++)
        {
            t[destR] = new double[srcRows];
            for (int destC = 0; destC < srcRows; destC++)
            {
                t[destR][destC] = mat[destC][destR];
            }
        }
        return t;
    }
    
    double* solveJacobian(double** J, const Vec3d& deltaX, const int jointCount)
    {
        Vec3d clampedDeltaX;
        for (int i = 0; i < 3; i++)
        {
            double w = deltaX[i];
            clampedDeltaX[i] = (fabs(w) <= JACOBIAN_MAX_DELTA) ? w : (JACOBIAN_MAX_DELTA * w/fabs(w));
        }
        double** JT = transpose(J, 3, jointCount);
        double* deltaTheta = multiply(JT, clampedDeltaX, jointCount);
        for (int i = 0; i < jointCount; i++)
        {
            deltaTheta[i] *= JACOBIAN_EPSILON;
            if (fabs(deltaTheta[i]) < 1e-5)
            {
                deltaTheta[i] = 0;
            }
        }
        
        for (int i = 0; i < 3; i++)
        {
            delete JT[i];
        }
        delete JT;
        
        return deltaTheta;
    }

    double* multiply(double** mat, const Vec3d& vec, const int rows)
    {
        double* result = new double[rows];
        for (int r = 0; r < rows; r++)
        {
            result[r] = 0;
            for (int c = 0; c < 3; c++)
            {
                result[r] += mat[r][c] * vec[c];
            }
        }
        return result;
    }
    
    double getMagnitude(Vec3d vec)
    {
        return fabs(vec[0]) + fabs(vec[1]) + fabs(vec[2]);
    }
};

// To make a SpiderModel, we inherit off of ModelerView
class SpiderModel : public ModelerView
{
public:
    // Constructor for the model.  In your model, 
    // make sure you call the ModelerView constructor,
    // as done below.
    SpiderModel(int x, int y, int w, int h, char *label)
        : ModelerView(x,y,w,h,label), frame(0),
            left2Snow(0.01, 0.05, Vec3f(1.0, 1.0, 1.0)),
            left3Snow(0.01, 0.05, Vec3f(1.0, 1.0, 1.0)),
            left4Snow(0.01, 0.05, Vec3f(1.0, 1.0, 1.0)),
            right1Snow(0.01, 0.05, Vec3f(1.0, 1.0, 1.0)),
            right2Snow(0.01, 0.05, Vec3f(1.0, 1.0, 1.0)),
            right3Snow(0.01, 0.05, Vec3f(1.0, 1.0, 1.0)),
            right4Snow(0.01, 0.05, Vec3f(1.0, 1.0, 1.0)),
            gravity(Vec3f(0, -9.81, 0)),
            wind(Vec3f(-3.0, 0, 0)),
            airResistance(0.25),
            left1Leg(LEFT, _1_HIP_POS, L1_FOOT_X, L1_FOOT_Y, L1_FOOT_Z),
            left2Leg(LEFT, _2_HIP_POS, L2_FOOT_X, L2_FOOT_Y, L2_FOOT_Z),
            left3Leg(LEFT, _3_HIP_POS, L3_FOOT_X, L3_FOOT_Y, L3_FOOT_Z),
            left4Leg(LEFT, _4_HIP_POS, L4_FOOT_X, L4_FOOT_Y, L4_FOOT_Z),
            right1Leg(RIGHT, _1_HIP_POS, R1_FOOT_X, R1_FOOT_Y, R1_FOOT_Z),
            right2Leg(RIGHT, _2_HIP_POS, R2_FOOT_X, R2_FOOT_Y, R2_FOOT_Z),
            right3Leg(RIGHT, _3_HIP_POS, R3_FOOT_X, R3_FOOT_Y, R3_FOOT_Z),
            right4Leg(RIGHT, _4_HIP_POS, R4_FOOT_X, R4_FOOT_Y, R4_FOOT_Z)
    {
        Vec3f snowVelocity(3.0, 3.0, 0);
        Vec3f snowVelocityJitter(2.0, 1.0, 2.0);
        float maxParticleAge = 10.0;
        left2Snow.setInitialVelocity(snowVelocity);
        left2Snow.setVelocityJitter(snowVelocityJitter);
        left2Snow.setMaxParticleAge(maxParticleAge);
        left2Snow.addForce(gravity);
        left2Snow.addForce(airResistance);
        left3Snow.setInitialVelocity(snowVelocity);
        left3Snow.setVelocityJitter(snowVelocityJitter);
        left3Snow.setMaxParticleAge(maxParticleAge);
        left3Snow.addForce(gravity);
        left3Snow.addForce(airResistance);
        left4Snow.setInitialVelocity(snowVelocity);
        left4Snow.setVelocityJitter(snowVelocityJitter);
        left4Snow.setMaxParticleAge(maxParticleAge);
        left4Snow.addForce(gravity);
        left4Snow.addForce(airResistance);
        right1Snow.setInitialVelocity(snowVelocity);
        right1Snow.setVelocityJitter(snowVelocityJitter);
        right1Snow.setMaxParticleAge(maxParticleAge);
        right1Snow.addForce(gravity);
        right1Snow.addForce(airResistance);
        right2Snow.setInitialVelocity(snowVelocity);
        right2Snow.setVelocityJitter(snowVelocityJitter);
        right2Snow.setMaxParticleAge(maxParticleAge);
        right2Snow.addForce(gravity);
        right2Snow.addForce(airResistance);
        right3Snow.setInitialVelocity(snowVelocity);
        right3Snow.setVelocityJitter(snowVelocityJitter);
        right3Snow.setMaxParticleAge(maxParticleAge);
        right3Snow.addForce(gravity);
        right3Snow.addForce(airResistance);
        right4Snow.setInitialVelocity(snowVelocity);
        right4Snow.setVelocityJitter(snowVelocityJitter);
        right4Snow.setMaxParticleAge(maxParticleAge);
        right4Snow.addForce(gravity);
        right4Snow.addForce(airResistance);
        ParticleSystem* ps = ModelerApplication::Instance()->GetParticleSystem();
        ps->addParticleCollection(&left2Snow);
        ps->addParticleCollection(&left3Snow);
        ps->addParticleCollection(&left4Snow);
        ps->addParticleCollection(&right1Snow);
        ps->addParticleCollection(&right2Snow);
        ps->addParticleCollection(&right3Snow);
        ps->addParticleCollection(&right4Snow);
    }
    
    virtual void draw();
private:
    ConstantForce gravity;
    ConstantForce wind;
    ViscousDrag airResistance;
    ParticleEmitter left2Snow;
    ParticleEmitter left3Snow;
    ParticleEmitter left4Snow;
    ParticleEmitter right1Snow;
    ParticleEmitter right2Snow;
    ParticleEmitter right3Snow;
    ParticleEmitter right4Snow;
    int frame;
    Leg left1Leg;
    Leg left2Leg;
    Leg left3Leg;
    Leg left4Leg;
    Leg right1Leg;
    Leg right2Leg;
    Leg right3Leg;
    Leg right4Leg;
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createSpiderModel(int x, int y, int w, int h, char *label)
{ 
    return new SpiderModel(x,y,w,h,label);
}

// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out RobotArm
void SpiderModel::draw()
{
    // This call takes care of a lot of the nasty projection
    // matrix stuff.  Unless you want to fudge directly with the 
	// projection matrix, don't bother with this ...
    ModelerView::draw();
    
    // Save camera transforms.
    const Mat4d cameraTransforms = getModelViewMatrix();

    // draw the floor
	setAmbientColor(1.0f,1.0f,1.0f);
	setDiffuseColor(1.0f,1.0f,1.0f);
	glPushMatrix();
	glTranslated(-10,0,-10);
	drawBox(20,0.01f,20);
    ModelerApplication::Instance()->AddCollidable(new CollidableQuad(Vec3f(10, 0, 10), Vec3f(10, 0, -10), Vec3f(-10, 0, -10), Vec3f(-10, 0, 10)));
	glPopMatrix();

    setDiffuseColor(0.52f,0.29f,0);
    glTranslated(VAL(XPOS), VAL(YPOS), VAL(ZPOS));
    glRotated(VAL(DIRECTION), 0, 1, 0);

    // Abdomen
    glPushMatrix();
    glRotated(180, 0, 1, 0);
    glTranslated(0, 0, VAL(ABDOMEN_OFFSET));
    glScaled(VAL(ABDOMEN_WIDTH), VAL(ABDOMEN_HEIGHT), VAL(ABDOMEN_LENGTH));
    drawSphere(1);
    glPopMatrix();
    
    // Cephalothorax (head)
    glTranslated(0, 0, -VAL(HEAD_LENGTH)/4);
    glTranslated(0, 0, VAL(HEAD_LENGTH)/4);
    glPushMatrix();
    glScaled(VAL(HEAD_WIDTH), VAL(HEAD_HEIGHT), VAL(HEAD_LENGTH));
    drawSphere(1);
    glPopMatrix();
    
    int emissionRate = VAL(EMIT_SNOW) == 1 ? 50 : 0;
    left2Snow.setEmissionRate(emissionRate);
    left3Snow.setEmissionRate(emissionRate);
    left4Snow.setEmissionRate(emissionRate);
    right1Snow.setEmissionRate(emissionRate);
    right2Snow.setEmissionRate(emissionRate);
    right3Snow.setEmissionRate(emissionRate);
    right4Snow.setEmissionRate(emissionRate);
    left1Leg.draw(cameraTransforms);
    left2Leg.draw(cameraTransforms);
    left2Snow.setPosition(left2Leg.getFootPosition());
    left3Leg.draw(cameraTransforms);
    left3Snow.setPosition(left3Leg.getFootPosition());
    left4Leg.draw(cameraTransforms);
    left4Snow.setPosition(left4Leg.getFootPosition());
    right1Leg.draw(cameraTransforms);
    right1Snow.setPosition(right1Leg.getFootPosition());
    right2Leg.draw(cameraTransforms);
    right2Snow.setPosition(right2Leg.getFootPosition());
    right3Leg.draw(cameraTransforms);
    right3Snow.setPosition(right3Leg.getFootPosition());
    right4Leg.draw(cameraTransforms);
    right4Snow.setPosition(right4Leg.getFootPosition());

    frame = (frame + 1) % 60;
}

int main()
{
	// Initialize the controls
	// Constructor is ModelerControl(name, minimumvalue, maximumvalue, 
	//								 stepsize, defaultvalue)
    // You will want to modify this to accommodate your model.
    ModelerControl controls[NUMCONTROLS];
    controls[TIME]   = ModelerControl("Time", 0, 1, 0.01f, 0);
	controls[XPOS]   = ModelerControl("X Position", -5, 5, 0.1f, 0);
	controls[YPOS]   = ModelerControl("Y Position",  0, 10, 0.1f, 4);
	controls[ZPOS]   = ModelerControl("Z Position", -5, 5, 0.1f, 0);
    controls[DIRECTION] = ModelerControl("Direction", 0, 360, 0.1f, 0);
    controls[ABDOMEN_LENGTH] = ModelerControl("Abdomen Length", 0, 5, 0.1f, 2.03);
    controls[ABDOMEN_WIDTH] = ModelerControl("Abdomen Width", 0, 5, 0.1f, 1.44);
    controls[ABDOMEN_HEIGHT] = ModelerControl("Abdomen Height", 0, 5, 0.1f, 1.23);
    controls[ABDOMEN_OFFSET] = ModelerControl("Abdomen Offset", 0, 5, 0.1f, 2.01);
    controls[HEAD_LENGTH] = ModelerControl("Head Length", 0, 5, 0.1f, 1.53);
    controls[HEAD_WIDTH] = ModelerControl("Head Width", 0, 5, 0.1f, 1.36);
    controls[HEAD_HEIGHT] = ModelerControl("Head Height", 0, 5, 0.1f, 1);
    controls[LEG_RADIUS] = ModelerControl("Leg Radius", 0, 1, 0.1f, 0.15);
    controls[TOE_RADIUS] = ModelerControl("Toe Radius", 0, 1, 0.1f, 0.1);
    controls[LEG_UPPER_LENGTH] = ModelerControl("Leg Length (Top)", 1, 5, 0.1f, 3.5);
    controls[LEG_MIDDLE_LENGTH] = ModelerControl("Leg Length (Middle)", 1, 5, 0.1f, 2.5);
    controls[LEG_LOWER_LENGTH] = ModelerControl("Leg Length (Bottom)", 1, 5, 0.1f, 2.75);
    controls[FOOT_LENGTH] = ModelerControl("Foot Length", 1, 5, 0.1f, 2);

    controls[_1_HIP_POS] = ModelerControl("Hip Position (Front)", 0, 1, 0.1f, 30.6);
    controls[_2_HIP_POS] = ModelerControl("Hip Position (Front Mid.)", 0, 1, 0.1f, 61.2);
    controls[_3_HIP_POS] = ModelerControl("Hip Position (Back Mid.)", 0, 1, 0.1f, 81);
    controls[_4_HIP_POS] = ModelerControl("Hip Position (Back)", 0, 1, 0.1f, 95.4);

    controls[L1_FOOT_X] = ModelerControl("L1: Foot X", -20, 20, 0.1f, -3);
    controls[L1_FOOT_Y] = ModelerControl("L1: Foot Y", -20, 20, 0.1f, 0);
    controls[L1_FOOT_Z] = ModelerControl("L1: Foot Z", -20, 20, 0.1f, 4);
    controls[L2_FOOT_X] = ModelerControl("L2: Foot X", -20, 20, 0.1f, -5);
    controls[L2_FOOT_Y] = ModelerControl("L2: Foot Y", -20, 20, 0.1f, 0);
    controls[L2_FOOT_Z] = ModelerControl("L2: Foot Z", -20, 20, 0.1f, 2);
    controls[L3_FOOT_X] = ModelerControl("L3: Foot X", -20, 20, 0.1f, -5);
    controls[L3_FOOT_Y] = ModelerControl("L3: Foot Y", -20, 20, 0.1f, 0);
    controls[L3_FOOT_Z] = ModelerControl("L3: Foot Z", -20, 20, 0.1f, -1);
    controls[L4_FOOT_X] = ModelerControl("L4: Foot X", -20, 20, 0.1f, -4);
    controls[L4_FOOT_Y] = ModelerControl("L4: Foot Y", -20, 20, 0.1f, 0);
    controls[L4_FOOT_Z] = ModelerControl("L4: Foot Z", -20, 20, 0.1f, -3);

    controls[R1_FOOT_X] = ModelerControl("R1: Foot X", -20, 20, 0.1f, 3);
    controls[R1_FOOT_Y] = ModelerControl("R1: Foot Y", -20, 20, 0.1f, 0);
    controls[R1_FOOT_Z] = ModelerControl("R1: Foot Z", -20, 20, 0.1f, 4);
    controls[R2_FOOT_X] = ModelerControl("R2: Foot X", -20, 20, 0.1f, 5);
    controls[R2_FOOT_Y] = ModelerControl("R2: Foot Y", -20, 20, 0.1f, 0);
    controls[R2_FOOT_Z] = ModelerControl("R2: Foot Z", -20, 20, 0.1f, 2);
    controls[R3_FOOT_X] = ModelerControl("R3: Foot X", -20, 20, 0.1f, 5);
    controls[R3_FOOT_Y] = ModelerControl("R3: Foot Y", -20, 20, 0.1f, 0);
    controls[R3_FOOT_Z] = ModelerControl("R3: Foot Z", -20, 20, 0.1f, -1);
    controls[R4_FOOT_X] = ModelerControl("R4: Foot X", -20, 20, 0.1f, 4);
    controls[R4_FOOT_Y] = ModelerControl("R4: Foot Y", -20, 20, 0.1f, 0);
    controls[R4_FOOT_Z] = ModelerControl("R4: Foot Z", -20, 20, 0.1f, -3);

    controls[MAX_DELTA] = ModelerControl("Max Delta", 0, 100, 0.1f, 100);
    controls[EMIT_SNOW] = ModelerControl("Emit Snow", 0, 1, 1, 0);
    
    // Initialize the modeler application with your model and the
    // appropriate array of controls.
    ParticleSystem ps;
    ModelerApplication::Instance()->SetParticleSystem(&ps);
    ModelerApplication::Instance()->Init(&createSpiderModel, controls, NUMCONTROLS);


	// make sure we give back the memory to older OSs that don't 
	// clear your memory pool after shutdown.
    int Result = ModelerApplication::Instance()->Run();
    ModelerApplication::Instance()->SetParticleSystem(NULL);
    delete ModelerApplication::Instance();
	return Result;
}

