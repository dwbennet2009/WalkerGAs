#ifndef SEGMENTHEX_H
#define SEGMENTHEX_H
#include <vector>


#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

class WalkerJoint : public b2RevoluteJoint
{
	public:
	void setMotorVars(double ampInpt, double phaseInpt, double freqInpt);
	double ampMotor;
	double phaseMotor;
	double freqMotor;
};

void WalkerJoint::setMotorVars(double ampInpt, double phaseInpt, double freqInpt)
{

	ampMotor = ampInpt;
	phaseMotor = phaseInpt;
	freqMotor = freqInpt;
}

class SegmentHex
{   
        public:
                double getInitAngle();
                void initialize(double posX, double posY, double angle, int inptCatBits, int inptMaskBits);
                void create(b2World* world);
                b2Body* getBody();
                void addJoint(b2Body* body2, double setX1, double setY1, double setX2, double setY2,  double initAngle, double setAmp, double setPhase, double setFreq, b2World* world);
                b2RevoluteJoint* getJoint(int jointIndex);
                void setJointSpeed(int jointIndex, double speed);
		void setShape(double length, double width);
		void stepMotors(double t);
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
                std::vector<WalkerJoint*> joints;

};

void SegmentHex::initialize(double posX, double posY, double angle, int inptCatBits, int inptMaskBits)
{

        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(posX,posY);

        bodyDef.angle= angle*RADTODEG;

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
	tempFix.filter.groupIndex = -1;

        body->CreateFixture(&tempFix);

//      body->SetTransform(body->GetPosition(), 60*DEGTORAD);
}

double SegmentHex::getInitAngle()
{
        return initAngle;
}

b2Body* SegmentHex::getBody()
{
        return body;
}

void SegmentHex::setJointSpeed(int jointIndex, double speed)
{
        joints[jointIndex]->SetMotorSpeed(speed);
}

b2RevoluteJoint* SegmentHex::getJoint(int jointIndex)
{
        return joints[jointIndex];
}

void SegmentHex::setShape(double length, double width)
{
        double vx[6] = {-length/2, length/2, length/2, -length/2};  
        double vy[6] = {-width/2, -width/2, width/2, width/2};
	
	b2Vec2 shapeVec[6];
	for(int i = 0; i < 4; i++)
	{
		shapeVec[i].Set(vx[i], vy[i]);
	}	

	shape.Set(shapeVec,4);

}


void SegmentHex::stepMotors(double t)
{
	for(std::vector<WalkerJoint*>::iterator it = joints.begin(); it < joints.end(); it++)
	{
		double freq = (*it)->freqMotor;
		double phase = (*it)->phaseMotor;
		double amp = (*it)->ampMotor;				

		(*it)->SetMotorSpeed(amp*cos(freq*t + phase));
	}
}

#endif
