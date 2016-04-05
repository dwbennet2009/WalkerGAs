#include <iostream>
#include <time.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
//#include "glui/GL/glui.h"

#ifndef WALKERGAS_H
#define WALKERGAS_H

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

double frame[25];
double angRLA[25], angRLC[25], angRLP[25];
double angRRA[25], angRRC[25], angRRP[25];
double angFLA[25], angFLC[25], angFLP[25];
double angFRA[25], angFRC[25], angFRP[25];


enum _objectCategory {
	EDGE = 	0x0001,
	BODY = 	0x0002,
	LEFT = 	0x0004,
	RIGHT = 0x0008,
};

class SegmentHex
{
	public:
		void getX();
		void getY();
		double getInitAngle();
		double getVx(int indx);
		double getVy(int indx);
		void define(double vx[], double vy[]);
		void initialize(double posX, double posY, b2PolygonShape inptShape, int inptCatBits, int inptMaskBits);
		void create(b2World* world);
		b2Body* getBody();
		void addJoint(b2Body* body2, double setX1, double setY1, double setX2, double setY2, double initAngle, b2World* world);
		void setJointSpeed(int jointIndex, double speed);
	private:
		int x;
		int y;	
		double initAngle;
		double vx[6];
		double vy[6];
		b2PolygonShape shape;		
		b2BodyDef bodyDef;
		b2Body* body;
		int catBits;
		int maskBits;
		std::vector<b2RevoluteJoint*> joints;
};

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

//	body->SetTransform(body->GetPosition(), 60*DEGTORAD);
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

class WalkerGAs : public Test
{

	public:

	SegmentHex segLoBL, segLoBR, segUpBL, segUpBR;
	SegmentHex segLoFL, segLoFR, segUpFL, segUpFR;
	SegmentHex segBody;
		
	struct timespec tv;
	double time_init, time_now;

	WalkerGAs() {

	std::string inptLine;
	std::ifstream inptFile("/home/dannyb/CompPhys/HorseAnalysis/horse.dat");

	int indx = 0;
	while(!inptFile.eof())
	{		
		std::getline( inptFile, inptLine);
		std::stringstream ss(inptLine);
		
		ss >> frame[indx] >> angRLA[indx] >> angRLC[indx] >> angRLP [indx]>> angRRA[indx] >> angRRC[indx] >> angRRP[indx]>> angFLA[indx] >> angFLC[indx] >> angFLP[indx]>> angFRA[indx] >> angFRC[indx] >> angFRP[indx] ;	
		indx++;
	}
	for(int i = 0; i < indx-1; i++)
	{
		std::cout<<frame[i]<<" | "<<angRRA[i]<<"\n";
	}

	m_world->SetGravity(b2Vec2(0, -0));
	
	double vx[6] = {-3,  3,  4,  3,  -3,  -4};
	double vy[6] = {-0.35,  -0.35,  0,  0.35,  0.35, 0};

	double vxb[6] = {-3.5, 3.5, 4.5, 3.5, -3.5, -4.5}; 
	double vyb[6] = {-1.5, -1.5, 0, 1.5, 1.5,  0};

	segLoFL.define(vx, vy);
	segLoFR.define(vx, vy);
	segUpFL.define(vx, vy);
	segUpFR.define(vx, vy);
	segLoBL.define(vx, vy);
	segLoBR.define(vx, vy);
	segUpBL.define(vx, vy);
	segUpBR.define(vx, vy);

	segBody.define(vxb, vyb);

	b2Vec2 legV[6], bodyV[6];
	for(int i = 0; i < 6; i++) {		
		legV[i].Set(segLoFL.getVx(i), segLoFL.getVy(i));
		bodyV[i].Set(segBody.getVx(i), segBody.getVy(i));
	}

	b2PolygonShape legShape, bodyShape;
	legShape.Set(legV,6);
	bodyShape.Set(bodyV,6);

	b2FixtureDef myFixDef;
	b2BodyDef myBodyDef;

	//void SegmentHex::initialize(double posX, double posY, b2PolygonShape shape, int catBits, int maskBits)


	//Create Segments

	//Lower Right Front Leg
	segLoFR.initialize(10, 5, legShape, RIGHT, RIGHT | EDGE | BODY);
	segLoFR.create(m_world);
	
	//Upper Right Front Leg
	segUpFR.initialize(10, 12, legShape, RIGHT, RIGHT | EDGE | BODY);
	segUpFR.create(m_world);

	//Lower Left Front Leg
	segLoFL.initialize(10, 5, legShape, LEFT, LEFT | EDGE | BODY);
	segLoFL.create(m_world);

	//Upper Left Front Leg
	segUpFL.initialize(10, 12, legShape, LEFT, LEFT | EDGE | BODY);
	segUpFL.create(m_world);

	//Right Back Front Leg
	segLoBR.initialize(0, 5, legShape, RIGHT, RIGHT | EDGE | BODY);
	segLoBR.create(m_world);
	
	//Upper Right Back Leg
	segUpBR.initialize(0, 12, legShape, RIGHT, RIGHT | EDGE | BODY);
	segUpBR.create(m_world);

	//Lower Left Back Leg
	segLoBL.initialize(0, 5, legShape, LEFT, LEFT | EDGE | BODY);
	segLoBL.create(m_world);

	//Upper Left Back Leg
	segUpBL.initialize(0, 12, legShape, LEFT, LEFT | EDGE | BODY);
	segUpBL.create(m_world);

	segBody.initialize(5, 30, bodyShape, BODY, LEFT | RIGHT | EDGE | BODY);
	segBody.create(m_world);

	//Create Joints	

	//Joints

	std::cout<<angFLA[0]<<" | "<<angFRA[0]<<"\n";


	std::cout<<segUpFL.getBody()->GetAngle()*RADTODEG<<" | "<<segUpFR.getBody()->GetAngle()*RADTODEG<<"\n";

	segUpFL.addJoint(segLoFL.getBody(), -3.5, 0, 3.5, 0 , 0, m_world);



	segUpFL.getBody()->SetTransform(segUpFL.getBody()->GetPosition(), (90-angFLA[0])*DEGTORAD);
	segUpFR.getBody()->SetTransform(segUpFR.getBody()->GetPosition(), (90-angFRA[0])*DEGTORAD);



	std::cout<<segUpFL.getBody()->GetAngle()*RADTODEG<<" | "<<segUpFR.getBody()->GetAngle()*RADTODEG<<"\n";


	segBody.addJoint(segUpFL.getBody(), 4, 0, 3.5, 0, 0, m_world);

	segUpFR.addJoint(segLoFR.getBody(), -3.5, 0, 3.5, 0, 0, m_world);
	segBody.addJoint(segUpFR.getBody(), 4, 0, 3.5, 0, 0, m_world);
	segUpBL.addJoint(segLoBL.getBody(), -3.5, 0, 3.5, 0, -(180-angRLC[0])/2, m_world);
	segBody.addJoint(segUpBL.getBody(), -4, 0, 3.5, 0, -(180-angRLA[0])/2, m_world);
	segUpBR.addJoint(segLoBR.getBody(), -3.5, 0, 3.5, 0, -(180-angRRC[0])/2, m_world);
	segBody.addJoint(segUpBR.getBody(), -4, 0, 3.5, 0, -(180-angRRA[0])/2, m_world);

	std::cout<<"Done\n";

	//World Edges

	myBodyDef.type = b2_staticBody;
	myFixDef.filter.categoryBits = EDGE; 			//Is a RIGHT
	myFixDef.filter.maskBits = LEFT | RIGHT | EDGE | BODY; 	//Collide with LEFT, RIGHT, EDGE, BODY

	myFixDef.friction = 0.6;

	b2EdgeShape edgeShape;
	myFixDef.shape = &edgeShape;
	myBodyDef.position.Set(0, 0);
	edgeShape.Set( b2Vec2(-40, 0), b2Vec2(40, 0) );
	b2Body* staticBody1 = m_world->CreateBody(&myBodyDef);
	staticBody1->CreateFixture(&myFixDef);
	
	myBodyDef.position.Set(0, 40);
	edgeShape.Set( b2Vec2(-40, 0), b2Vec2(40, 0) );
	b2Body* staticBody2 = m_world->CreateBody(&myBodyDef);
	staticBody2->CreateFixture(&myFixDef);

	myBodyDef.position.Set(-40, 20);
	edgeShape.Set( b2Vec2(0, 20), b2Vec2(0, -20) );
	b2Body* staticBody3 = m_world->CreateBody(&myBodyDef);
	staticBody3->CreateFixture(&myFixDef);

	myBodyDef.position.Set(40, 20);
	edgeShape.Set( b2Vec2(0, 20), b2Vec2(0, -20) );
	b2Body* staticBody4 = m_world->CreateBody(&myBodyDef);
	staticBody4->CreateFixture(&myFixDef);
	
	}

	void Step(Settings *settings)
	{
		Test::Step(settings);

		b2Vec2 position = segBody.getBody()->GetPosition();
		position.y = position.y - 15;
		Test::ShiftOrigin(position);
	
		double t = m_stepCount * (1.0f / settings->hz);
	
		int count = int(t/0.1)%20;	

		double vFLA = -(angFLA[count+1]-angFLA[count])/2/0.1;
	
	//segUpFL.getBody()->SetTransform(segUpFL.getBody()->GetPosition(), (180-angFLA[0])*DEGTORAD);
	//segUpFR.getBody()->SetTransform(segUpFR.getBody()->GetPosition(), (180-angFRA[0])*DEGTORAD);
	//segUpBL.getBody()->SetTransform(segUpBL.getBody()->GetPosition(), (180-angRLA[0])*DEGTORAD);
	//segUpBR.getBody()->SetTransform(segUpBR.getBody()->GetPosition(), (180-angRRA[0])*DEGTORAD);


	}

	static Test* Create()
	{
		return new WalkerGAs;
	}


};

#endif
