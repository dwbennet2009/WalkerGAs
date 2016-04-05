#ifndef WALKERGAS_H
#define WALKERGAS_H
#include <iostream>
#include <time.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "SegmentHex.h"
#include "WalkerOrganism.h"
#include "Generation.h"
//#include "Gene.h"

//#include "glui/GL/glui.h"



#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#define PI 3.14159

std::string fileName = "/home/dannyb/CompPhys/Box2D_v2.3.0/Box2D/Testbed/Tests/gaFile.txt";
double frame[25];
double angRLA[25], angRLC[25], angRLP[25];
double angRRA[25], angRRC[25], angRRP[25];
double angFLA[25], angFLC[25], angFLP[25];
double angFRA[25], angFRC[25], angFRP[25];

class WalkerGAs : public Test
{
	
	public:

	struct timespec tv;
	double time_init, time_now;

	std::vector<WalkerOrganism*> walkerList;
	std::vector<Gene*> geneList;

	std::vector<double> finalScores;	
	std::vector<Gene*> finalGenes;
	
	b2Body* staticBody1;
	b2Body* staticBody2;

	WalkerGAs() {

	std::string inptLine;
	std::ifstream inptFile("/home/dannyb/CompPhys/HorseAnalysis/horse.dat");

	Generation genNow(fileName);
	genNow.geneMate();
	genNow.geneMutate();

	for(int i = 0; i < 50; i++)
	{
        	Gene* t1 = new Gene();

		t1->setGeneString(genNow.getGeneString(i));		
		geneList.push_back(t1);
	}

	int indx = 0;
	while(!inptFile.eof())
	{		
		std::getline( inptFile, inptLine);
		std::stringstream ss(inptLine);
		
		ss >> frame[indx] >> angRLA[indx] >> angRLC[indx] >> angRLP [indx]>> angRRA[indx] >> angRRC[indx] >> angRRP[indx]>> angFLA[indx] >> angFLC[indx] >> angFLP[indx]>> angFRA[indx] >> angFRC[indx] >> angFRP[indx] ;	
		indx++;
	}

	m_world->SetGravity(b2Vec2(0, -25));
	
	double scale = 6;
	
	double RUlength = 31;  double RUwidth = 11;
	double RLlength = 26;  double RLwidth = 5;
	double FUlength = 29;  double FUwidth = 9;
	double FLlength = 20;  double FLwidth = 4;
	double Bwidth = 30;    double Blength = 90;
	double FFtlength = 15; double FFtwidth = 5 ;
	double RFtlength = 15; double RFtwidth = 5;	
	
	for(int i = 0; i < geneList.size(); i++)
	{	
		WalkerOrganism* orgOne = new WalkerOrganism();
		walkerList.push_back(orgOne);
	}	

	for(std::vector<WalkerOrganism*>::iterator iter = walkerList.begin(); iter < walkerList.end(); iter++)
        {

		(*iter)->setBody( Blength / scale,  Bwidth / scale );
		(*iter)->setLeg( 0, FUlength / scale, FUwidth / scale, 
				  FLlength / scale, FLwidth / scale,
				  FFtlength / scale, FFtwidth / scale );
		(*iter)->setLeg( 1, FUlength / scale, FUwidth / scale, 
				  FLlength / scale, FLwidth / scale,
				  FFtlength / scale, FFtwidth / scale );
		(*iter)->setLeg( 2, RUlength / scale, RUwidth / scale, 
				  RLlength / scale, RLwidth / scale,
				  RFtlength / scale, RFtwidth / scale );
		(*iter)->setLeg( 3, RUlength / scale, RUwidth / scale, 
				  RLlength / scale, RLwidth / scale,
				  RFtlength / scale, RFtwidth / scale );
		(*iter)->createOrg(m_world);
		(*iter)->setJoints(m_world);
	
	}

	
	for(int i = 0; i < walkerList.size(); i++)
	{
		walkerList[i]->setMotors(geneList[i]);
	}

	//orgOne.setMotors(t1);

	
	b2FixtureDef myFixDef;
	b2BodyDef myBodyDef;

	//void SegmentHex::initialize(double posX, double posY, b2PolygonShape shape, int catBits, int maskBits)



	//World Edges

	myBodyDef.type = b2_staticBody;
	myFixDef.filter.categoryBits = EDGE; 			//Is a RIGHT
	myFixDef.filter.maskBits = LEFT | RIGHT | EDGE | BODY; 	//Collide with LEFT, RIGHT, EDGE, BODY

	myFixDef.friction = 0.8;

	b2EdgeShape edgeShape;
	myFixDef.shape = &edgeShape;
	myBodyDef.position.Set(0, 0);
	edgeShape.Set( b2Vec2(-40, 0), b2Vec2(400, 0) );
	staticBody1 = m_world->CreateBody(&myBodyDef);
	staticBody1->CreateFixture(&myFixDef);

	myFixDef.shape = &edgeShape;
	myBodyDef.position.Set(-40, 0);
	edgeShape.Set( b2Vec2(0, 40), b2Vec2(0, -40) );
	staticBody2 = m_world->CreateBody(&myBodyDef);
	staticBody2->CreateFixture(&myFixDef);
	

	}

	void Step(Settings *settings)
	{
		
		Test::Step(settings);

		b2Vec2 position = walkerList[0]->getBodyPos();
		b2Vec2 edgePosition = staticBody2->GetPosition();
		b2Vec2 edgePositiony = staticBody1->GetPosition();

		


	
		double time = (m_stepCount * 1. / 60.);

		for(int i = 0; i < walkerList.size(); i++)
		{
                	walkerList[i]->stepMotors(time);
        		walkerList[i]->height = walkerList[i]->getBodyPos().y - edgePositiony.y;
			walkerList[i]->score = walkerList[i]->getBodyPos().x - edgePosition.x;

		}
		
		position.y = position.y - 15;
		Test::ShiftOrigin(position);
	
		if(m_stepCount>=100)
		{	
			int imax = walkerList.size();
			for(int i = 0; i < imax; i++)
			{
				if(walkerList[i]->height<= 2.6||m_stepCount>=1500)
				{
					finalScores.push_back(walkerList[i]->score);
					finalGenes.push_back(walkerList[i]->g1);

					delete walkerList[i];
					walkerList.erase(walkerList.begin()+i);
						
					i = i - 1;
					imax = imax - 1;
				}
			}		
		}

		if(walkerList.size()==0)
		{
			int i, j, flag = 1;
			int temp;
			Gene* tempGene;
			int numLength = finalGenes.size();
			for(i = 1; i < numLength && flag; i++)
			{
				flag = 0;
				for( j = 0; j < (numLength - 1); j++)
				{
					if(finalScores[j+1] > finalScores[j])
					{
						temp = finalScores[j];
						finalScores[j] = finalScores[j+1];
						finalScores[j+1] = temp;

						tempGene = finalGenes[j];
						finalGenes[j] = finalGenes[j+1];
						finalGenes[j+1] = tempGene;

						flag = 1;
					}
				}
			}
				

			for(i = 0; i < finalGenes.size(); i++)
			{
				std::cout<<finalGenes[i]->getGeneString()<<" | "<<finalScores[i]<<"\n";
			}
			std::cout<<"\n";	
				
			std::ofstream gaFile;
			gaFile.open(fileName.c_str());
			for(i = 0; i < finalGenes.size(); i++)
			{
				gaFile<<finalGenes[i]->getGeneString()<<" "<<finalScores[i]<<"\n";
			}
			gaFile.close();
	
		}
	
	}

	static Test* Create()
	{
		return new WalkerGAs;
	}


};

#endif
