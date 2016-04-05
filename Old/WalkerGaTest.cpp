#include "WalkerGaTest.h"

WalkerGaTest::WalkerGaTest()
{

   std::string inptLine;
   std::ifstream inptFile("/home/dannyb/CompPhys/HorseAnalysis/horse.dat");

   int indx = 0;
   while (!inptFile.eof())
   {
      std::getline(inptFile, inptLine);
      std::stringstream ss(inptLine);

      ss >> frame[indx] >> angRLA[indx] >> angRLC[indx] >> angRLP[indx] >> angRRA[indx] >> angRRC[indx] >> angRRP[indx]
         >> angFLA[indx] >> angFLC[indx] >> angFLP[indx] >> angFRA[indx] >> angFRC[indx] >> angFRP[indx];
      indx++;
   }
   for (int i = 0; i < indx - 1; i++)
   {
      std::cout << frame[i] << " | " << angRRA[i] << "\n";
   }

   m_world->SetGravity(b2Vec2(0, -0));

   double vx[6] = { -3, 3, 4, 3, -3, -4 };
   double vy[6] = { -0.35, -0.35, 0, 0.35, 0.35, 0 };

   double vxb[6] = { -3.5, 3.5, 4.5, 3.5, -3.5, -4.5 };
   double vyb[6] = { -1.5, -1.5, 0, 1.5, 1.5, 0 };

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
   for (int i = 0; i < 6; i++)
   {
      legV[i].Set(segLoFL.getVx(i), segLoFL.getVy(i));
      bodyV[i].Set(segBody.getVx(i), segBody.getVy(i));
   }

   b2PolygonShape legShape, bodyShape;
   legShape.Set(legV, 6);
   bodyShape.Set(bodyV, 6);

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

   std::cout << angFLA[0] << " | " << angFRA[0] << "\n";

   std::cout << segUpFL.getBody()->GetAngle() * RADTODEG << " | " << segUpFR.getBody()->GetAngle() * RADTODEG << "\n";

   segUpFL.addJoint(segLoFL.getBody(), -3.5, 0, 3.5, 0, 0, m_world);

   segUpFL.getBody()->SetTransform(segUpFL.getBody()->GetPosition(), (90 - angFLA[0]) * DEGTORAD);
   segUpFR.getBody()->SetTransform(segUpFR.getBody()->GetPosition(), (90 - angFRA[0]) * DEGTORAD);

   std::cout << segUpFL.getBody()->GetAngle() * RADTODEG << " | " << segUpFR.getBody()->GetAngle() * RADTODEG << "\n";

   segBody.addJoint(segUpFL.getBody(), 4, 0, 3.5, 0, 0, m_world);

   segUpFR.addJoint(segLoFR.getBody(), -3.5, 0, 3.5, 0, 0, m_world);
   segBody.addJoint(segUpFR.getBody(), 4, 0, 3.5, 0, 0, m_world);
   segUpBL.addJoint(segLoBL.getBody(), -3.5, 0, 3.5, 0, -(180 - angRLC[0]) / 2, m_world);
   segBody.addJoint(segUpBL.getBody(), -4, 0, 3.5, 0, -(180 - angRLA[0]) / 2, m_world);
   segUpBR.addJoint(segLoBR.getBody(), -3.5, 0, 3.5, 0, -(180 - angRRC[0]) / 2, m_world);
   segBody.addJoint(segUpBR.getBody(), -4, 0, 3.5, 0, -(180 - angRRA[0]) / 2, m_world);

   std::cout << "Done\n";

   //World Edges

   myBodyDef.type = b2_staticBody;
   myFixDef.filter.categoryBits = EDGE;  //Is a RIGHT
   myFixDef.filter.maskBits = LEFT | RIGHT | EDGE | BODY;  //Collide with LEFT, RIGHT, EDGE, BODY

   myFixDef.friction = 0.6;

   b2EdgeShape edgeShape;
   myFixDef.shape = &edgeShape;
   myBodyDef.position.Set(0, 0);
   edgeShape.Set(b2Vec2(-40, 0), b2Vec2(40, 0));
   b2Body* staticBody1 = m_world->CreateBody(&myBodyDef);
   staticBody1->CreateFixture(&myFixDef);

   myBodyDef.position.Set(0, 40);
   edgeShape.Set(b2Vec2(-40, 0), b2Vec2(40, 0));
   b2Body* staticBody2 = m_world->CreateBody(&myBodyDef);
   staticBody2->CreateFixture(&myFixDef);

   myBodyDef.position.Set(-40, 20);
   edgeShape.Set(b2Vec2(0, 20), b2Vec2(0, -20));
   b2Body* staticBody3 = m_world->CreateBody(&myBodyDef);
   staticBody3->CreateFixture(&myFixDef);

   myBodyDef.position.Set(40, 20);
   edgeShape.Set(b2Vec2(0, 20), b2Vec2(0, -20));
   b2Body* staticBody4 = m_world->CreateBody(&myBodyDef);
   staticBody4->CreateFixture(&myFixDef);

}

void WalkerGaTest::Step(Settings *settings)
{
   Test::Step(settings);

   b2Vec2 position = segBody.getBody()->GetPosition();
   position.y = position.y - 15;
   Test::ShiftOrigin(position);

   double t = m_stepCount * (1.0f / settings->hz);

   int count = int(t / 0.1) % 20;

   double vFLA = -(angFLA[count + 1] - angFLA[count]) / 2 / 0.1;

   //segUpFL.getBody()->SetTransform(segUpFL.getBody()->GetPosition(), (180-angFLA[0])*DEGTORAD);
   //segUpFR.getBody()->SetTransform(segUpFR.getBody()->GetPosition(), (180-angFRA[0])*DEGTORAD);
   //segUpBL.getBody()->SetTransform(segUpBL.getBody()->GetPosition(), (180-angRLA[0])*DEGTORAD);
   //segUpBR.getBody()->SetTransform(segUpBR.getBody()->GetPosition(), (180-angRRA[0])*DEGTORAD);

}

static Test* WalkerGaTest::Create()
{
   return new WalkerGaTest;
}
