#include <iostream>
#include <time.h>
//#include "glui/GL/glui.h"

#ifndef FOOTEST2_H
#define FOOTEST2_H

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

class SegmentHex2 : public Test 
{
	public:
		void initialize();
		void create(b2World* world);

	private:
		b2FixtureDef fixDef;		
		b2BodyDef bodyDef;
		b2Body* body;
};


void SegmentHex2::initialize()
{


	b2PolygonShape legShape;
	legShape.SetAsBox(1,1);

	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(0,20);

	fixDef.shape = &legShape;	
	fixDef.density = 1.0;
	fixDef.friction = 0.3;
	
}

void SegmentHex2::create(b2World* world)
{

	body = world->CreateBody(&bodyDef);
	std::cerr<<"DEBUG 1\n";	
	body->CreateFixture(&fixDef);     ///Program fails on this line
	//b2FixtureDef tempFix = fixDef;	
	//b2PolygonShape legShape;
	//legShape.SetAsBox(1,1);
	//tempFix.shape = &legShape;
	//tempFix.density = 1.0;
	//body->CreateFixture(&tempFix);
	std::cerr<<"DEBUG 2\n";	
}

class FooTest2 : public Test
{

	public:
		
	FooTest2() {

	SegmentHex2 segLoFR;

	///////// Create a polygon, a FixtureDef, and BodyDef
	b2PolygonShape legShape;
	legShape.SetAsBox(1,1);
	b2FixtureDef myFixDef;
	b2BodyDef myBodyDef;

	myBodyDef.type = b2_dynamicBody;
	myBodyDef.position.Set(0,20);

	myFixDef.shape = &legShape;	
	myFixDef.density = 1.0;
	myFixDef.friction = 0.3;


	///////// Use that FixtureDef and BodyDef to create testBody1 and apply its Fixture

	b2Body* testBody1;
	testBody1 = m_world->CreateBody(&myBodyDef);
	testBody1->CreateFixture(&myFixDef);

	////// This works fine.
	

	///// Using the segment class, create a polygon, a FixtureDef, and a BodyDef within the initialize method
	segLoFR.initialize();
	///// Use the FixtureDef and BodyDef to create a body within the Create method.
	segLoFR.create(m_world);
	///// This does NOT work, at CreateFixture, gives the error
	///// pure virtual method called
	///// terminate called without an active exception
	///// Abort (core dumped)


	}

	void Step(Settings *settings)
	{
	}

	static Test* Create()
	{
		return new FooTest2;
	}

};

#endif
