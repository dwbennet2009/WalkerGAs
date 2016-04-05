#include <vector>
#include "SegmentHex.h"

void SegmentHex::initialize(double posX, double posY, b2PolygonShape inptShape, int inptCatBits, int inptMaskBits)
{

        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(posX,posY);

        shape = inptShape;

        catBits = inptCatBits;
        maskBits = inptMaskBits;

}

void SegmentHex::create(b2World* world)
{

        body = world->CreateBody(&bodyDef);

        b2FixtureDef tempFix;
        tempFix.shape = &shape;
        tempFix.density = 1.0;
        tempFix.friction = 0.3;
        tempFix.filter.categoryBits = catBits;
        tempFix.filter.maskBits = maskBits;

        body->CreateFixture(&tempFix);

//      body->SetTransform(body->GetPosition(), 60*DEGTORAD);
}

void SegmentHex::define(double vxIn[], double vyIn[])
{
        for(int i = 0; i < 6; i++) {
                vx[i] = vxIn[i];
                vy[i] = vyIn[i];
        }
}

double SegmentHex::getVx(int indx)
{
        return vx[indx];
}

double SegmentHex::getVy(int indx)
{
        return vy[indx];
}

double SegmentHex::getInitAngle()
{
        return initAngle;
}

b2Body* SegmentHex::getBody()
{
        return body;
}
void SegmentHex::addJoint(b2Body* body2, double setX1, double setY1, double setX2, double setY2, double initAngle, b2World* world)
{

        b2RevoluteJointDef myJointDef;
        myJointDef.collideConnected = false;
        myJointDef.enableLimit = true;
        double myJointAngle;

        myJointDef.enableMotor = true;
        myJointDef.maxMotorTorque = 10000;

        myJointDef.bodyA = body2;
        myJointDef.bodyB = body;
        myJointDef.localAnchorA.Set(setX2, setY2);
        myJointDef.localAnchorB.Set(setX1, setY1);
        myJointDef.referenceAngle = initAngle*DEGTORAD;
        myJointDef.upperAngle = (initAngle+100)*DEGTORAD;
        myJointDef.lowerAngle = (initAngle-100)*DEGTORAD;

        joints.push_back( (b2RevoluteJoint*)world->CreateJoint(&myJointDef) );
}

void SegmentHex::setJointSpeed(int jointIndex, double speed)
{
        joints[jointIndex]->SetMotorSpeed(speed);
}


