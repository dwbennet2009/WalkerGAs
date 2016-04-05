#ifndef WALKERORGANISM_H
#define WALKERORGANISM_H

#include <vector>
#include "SegmentHex.h"
#include "Gene.h"

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#define PI 3.14159



enum _objectCategory {
        EDGE =  0x0001,
        BODY =  0x0002,
        LEFT =  0x0004,
        RIGHT = 0x0008,
};

class WalkerOrganism
{

	

	public:
		
		void setBody(	double length, double width);
		void setLeg(	int legNum,
			 	double upperLength, double upperWidth,
				double lowerLength, double lowerWidth,
				double footLength, double footWidth);
		void setJoints(b2World* m_world);
		void addJoint(b2Body* body1, b2Body* body2, double setX1, double setY1, double setX2, double setY2,  double initAngle, double setAmp, double setPhase, double setFreq, b2World* worldIn);
		void createOrg(b2World* m_world);
		b2Vec2 getBodyPos();
		void setMotors(Gene* t1);
		void stepMotors(double t);
		~WalkerOrganism();
	
		double score;
		double height;	
		Gene* g1;
	private:
		SegmentHex body;

		SegmentHex frontLeftLeg[3];
		SegmentHex frontRightLeg[3];
		SegmentHex backLeftLeg[3];
		SegmentHex backRightLeg[3];

		double bodyWidth, bodyLength;
		double frontLeftLengths[3], frontLeftWidths[3];
		double frontRightLengths[3], frontRightWidths[3];
		double backLeftLengths[3], backLeftWidths[3];
		double backRightLengths[3], backRightWidths[3];
		std::vector<WalkerJoint*> joints;
		b2World* world;
};

void WalkerOrganism::setBody( double length, double width)
{
	SegmentHex segBody;

	segBody.initialize(5, 15, 0, BODY, LEFT | RIGHT | EDGE | BODY);
	segBody.setShape( length, width);

	bodyLength = length; bodyWidth = width;
		
	body = segBody;

}

void WalkerOrganism::setLeg( int legNum, double upperLength, double upperWidth, double lowerLength, double lowerWidth, double footLength, double footWidth)
{
	SegmentHex segUpper, segLower, segFoot;

	//0, 1, 2, 3  | FrLeft, FrRight, BaLeft, BaRight

	if(legNum==0) //front left
	{
		segUpper.initialize(10, 10, -90, LEFT, LEFT | EDGE | BODY);
		segLower.initialize(10, 5, -90, LEFT, LEFT | EDGE | BODY);
		segFoot.initialize(10, 2, 90, LEFT, LEFT | EDGE | BODY);
		frontLeftLengths[0] = upperLength; frontLeftWidths[0] = upperWidth;
		frontLeftLengths[1] = lowerLength; frontLeftWidths[1] = lowerWidth;
		frontLeftLengths[2] = footLength; frontLeftWidths[2] = footWidth;


	}
	else if(legNum==1) //front Right
	{
		segUpper.initialize(10, 10, -90, RIGHT, RIGHT | EDGE | BODY);
		segLower.initialize(10, 5, -90, RIGHT, RIGHT | EDGE | BODY);
		segFoot.initialize(10, 2, 90, RIGHT, RIGHT | EDGE | BODY);
		frontRightLengths[0] = upperLength; frontRightWidths[0] = upperWidth;
		frontRightLengths[1] = lowerLength; frontRightWidths[1] = lowerWidth;
		frontRightLengths[2] = footLength; frontRightWidths[2] = footWidth;
	}
	else if(legNum==2) //back left
	{
		segUpper.initialize(0, 10, -90, LEFT, LEFT | EDGE | BODY);
		segLower.initialize(0, 5, -90, LEFT, LEFT | EDGE | BODY);
		segFoot.initialize(0, 2, 90, LEFT, LEFT | EDGE | BODY);
		backLeftLengths[0] = upperLength; backLeftWidths[0] = upperWidth;
		backLeftLengths[1] = lowerLength; backLeftWidths[1] = lowerWidth;
		backLeftLengths[2] = footLength; backLeftWidths[2] = footWidth;
	}
	else if(legNum==3) //back right
	{
		segUpper.initialize(0, 10, -90, RIGHT, RIGHT | EDGE | BODY);
		segLower.initialize(0, 5, -90, RIGHT, RIGHT | EDGE | BODY);
		segFoot.initialize(0, 2, 90, RIGHT, RIGHT | EDGE | BODY);
		backRightLengths[0] = upperLength; backRightWidths[0] = upperWidth;
		backRightLengths[1] = lowerLength; backRightWidths[1] = lowerWidth;
		backRightLengths[2] = footLength; backRightWidths[2] = footWidth;
	}

	segUpper.setShape(upperLength, upperWidth);	
	segLower.setShape(lowerLength, lowerWidth);	
	segFoot.setShape(footLength, footWidth);	

	if( legNum==0 ) {
		frontLeftLeg[0] = segUpper;
		frontLeftLeg[1] = segLower;
		frontLeftLeg[2] = segFoot;
	}
	if( legNum==1 ) {
		frontRightLeg[0] = segUpper;
		frontRightLeg[1] = segLower;
		frontRightLeg[2] = segFoot;
	}
	if( legNum==2 ) {
		backLeftLeg[0] = segUpper;
		backLeftLeg[1] = segLower;
		backLeftLeg[2] = segFoot;
	}
	if( legNum==3 ) {
		backRightLeg[0] = segUpper;
		backRightLeg[1] = segLower;
		backRightLeg[2] = segFoot;
	}
}

void WalkerOrganism::addJoint(b2Body* body1, b2Body* body2, double setX1, double setY1, double setX2, double setY2,  double initAngle, double setAmp, double setPhase, double setFreq, b2World* worldIn)
{
	b2RevoluteJointDef myJointDef;
        myJointDef.collideConnected = false;
        myJointDef.enableLimit = true;
        double myJointAngle;

        myJointDef.enableMotor = true;
        myJointDef.maxMotorTorque = 10000;

        myJointDef.bodyA = body1;
        myJointDef.bodyB = body2;
        myJointDef.localAnchorA.Set(setX1, setY1);
        myJointDef.localAnchorB.Set(setX2, setY2);
        myJointDef.referenceAngle = initAngle*DEGTORAD;
        myJointDef.upperAngle = (30)*DEGTORAD;
        myJointDef.lowerAngle = (-50)*DEGTORAD;

        joints.push_back( (WalkerJoint*)world->CreateJoint(&myJointDef) );


}

void WalkerOrganism::setJoints(b2World* m_world)
{

        this->addJoint(body.getBody(), frontLeftLeg[0].getBody() , 0.9*bodyLength/ 2, -bodyWidth/ 2, frontLeftLengths[0]/ 2, 0, 90,10,PI/2, 5,  m_world);
        this->addJoint(body.getBody(), frontRightLeg[0].getBody(), 0.9*bodyLength/ 2, -bodyWidth/ 2, frontRightLengths[0]/ 2, 0, 90,10,0, 5,  m_world);
        this->addJoint(body.getBody(), backLeftLeg[0].getBody(), -0.9*bodyLength/ 2, -bodyWidth/ 2, backLeftLengths[0]/ 2, 0, 90,10, PI/2, 5,  m_world);
        this->addJoint(body.getBody(), backRightLeg[0].getBody(), -0.9*bodyLength/ 2, -bodyWidth/ 2, backRightLengths[0]/ 2, 0, 90,10,0, 5,  m_world);

        this->addJoint(frontLeftLeg[0].getBody(), frontLeftLeg[1].getBody(), -frontLeftLengths[0]/ 2, 0, frontLeftLengths[1]/ 2, 0, -30,0,0,0,  m_world);
        this->addJoint(frontRightLeg[0].getBody(), frontRightLeg[1].getBody(), -frontRightLengths[0]/ 2, 0, frontRightLengths[1]/ 2, 0, -30,0,0,0,  m_world);
        this->addJoint(backLeftLeg[0].getBody(), backLeftLeg[1].getBody(), -backLeftLengths[0]/ 2, 0, backLeftLengths[1]/ 2, 0, 30,0,0,0,  m_world);
        this->addJoint(backRightLeg[0].getBody(), backRightLeg[1].getBody(), -backRightLengths[0]/ 2, 0, backRightLengths[1]/ 2, 0, 30,0,0,0,  m_world);

        this->addJoint(frontLeftLeg[1].getBody(), frontLeftLeg[2].getBody(), -frontLeftLengths[1]/ 2, 0,-frontLeftLengths[2]/ 2, 0 ,-120,0,0,0,  m_world);
        this->addJoint(frontRightLeg[1].getBody(), frontRightLeg[2].getBody(), -frontRightLengths[1]/ 2, 0,-frontRightLengths[2]/ 2, 0 , -120,0,0,0,  m_world);
        this->addJoint(backLeftLeg[1].getBody(), backLeftLeg[2].getBody(), -backLeftLengths[1]/ 2, 0, -backLeftLengths[2]/ 2, 0 , -150,0,0,0,  m_world);
        this->addJoint(backRightLeg[1].getBody(), backRightLeg[2].getBody(), -backRightLengths[1]/ 2, 0, -backRightLengths[2]/ 2, 0 , -150,0,0,0,  m_world);

}


void WalkerOrganism::createOrg(b2World* m_world)
{
	world = m_world;
	body.create(world);
	for(int i = 0; i < 3; i++) frontLeftLeg[i].create(world);
	for(int i = 0; i < 3; i++) frontRightLeg[i].create(world);
	for(int i = 0; i < 3; i++) backLeftLeg[i].create(world);
	for(int i = 0; i < 3; i++) backRightLeg[i].create(world);


	//Transform needed to zero out Angles
	body.getBody()->SetTransform(body.getBody()->GetPosition(),0);
	for(int i = 0; i < 3; i++) frontLeftLeg[i].getBody()->SetTransform(frontLeftLeg[i].getBody()->GetPosition(),0);
	for(int i = 0; i < 3; i++) frontRightLeg[i].getBody()->SetTransform(frontRightLeg[i].getBody()->GetPosition(),0);
	for(int i = 0; i < 3; i++) backLeftLeg[i].getBody()->SetTransform(backLeftLeg[i].getBody()->GetPosition(),0);
	for(int i = 0; i < 3; i++) backRightLeg[i].getBody()->SetTransform(backRightLeg[i].getBody()->GetPosition(),0);



}

b2Vec2 WalkerOrganism::getBodyPos()
{
	return body.getBody()->GetPosition();
}


void WalkerOrganism::setMotors(Gene* t1)
{

	g1 = t1;

	int indxList[12] = {	0,1,2,
				3,4,5,
				0,1,2,
				3,4,5};

	int i = 0;

	for(std::vector<WalkerJoint*>::iterator it = joints.begin(); it < joints.end(); it++)
	{
		int j = indxList[i];

		(*it)->setMotorVars(t1->getAmp(j), t1->getPhase(j), t1->getFreq(j));
		i++;

	}
	
}

void WalkerOrganism::stepMotors(double t)
{
	for(std::vector<WalkerJoint*>::iterator it = joints.begin(); it < joints.end(); it++)
        {
                double amp = (*it)->ampMotor;
                double phase = (*it)->phaseMotor;
                double freq = (*it)->freqMotor;

                (*it)->SetMotorSpeed(amp*cos(freq*t + phase));
        }


}

WalkerOrganism::~WalkerOrganism(void)
{
	world->DestroyBody(body.getBody());
	for(int i = 0; i < 3; i++)
	{
		world->DestroyBody(frontLeftLeg[i].getBody());
		world->DestroyBody(frontRightLeg[i].getBody());
		world->DestroyBody(backLeftLeg[i].getBody());
		world->DestroyBody(backRightLeg[i].getBody());
	}
}
#endif
