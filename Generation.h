#ifndef GENERATION_H
#define GENERATION_H

#include <iostream>
#include <string>
#include <sstream>

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#define PI 3.14159

int championCount = 5;
double mutateChance = 0.30;
double mutateStrength = 1;

class Generation
{
	public:
		Generation(std::string fileName);
		void geneMate();
		void geneMutate();			
		std::string getGeneString(int indx);
	private: 
		int geneLength;
		std::vector<double> scoreList;
		std::vector<std::string> geneList;
		std::vector<std::string> geneListNew;
};

Generation::Generation(std::string fileName)
{	
	std::ifstream fileIn(fileName.c_str());

	std::string lineIn;
	if(fileIn.is_open())
	{
		while(getline(fileIn,lineIn))
		{
			std::string geneTemp, scoreTemp;
			double dScoreTemp;
			std::istringstream iss(lineIn);
			iss>>geneTemp;
			iss>>scoreTemp;
			dScoreTemp=std::atof(scoreTemp.c_str());
			
			scoreList.push_back(dScoreTemp);
			geneList.push_back(geneTemp);
		}
	}
	geneLength = geneList.size();
		
}

void Generation::geneMate()
{
	std::cout<<"Creating new gene list with top "<<championCount<<" champions from previous generation.\n\n";
	for(int i = 0; i < geneLength; i++)
	{
		geneListNew.push_back(geneList[i%championCount]);	
	}
}

void Generation::geneMutate()
{
	std::cout<<"Mutating new gene list with a chance of "<<mutateChance<<" and a strength of "<<mutateStrength<<".\n\n";
	for(int i = 0; i < geneListNew.size(); i++)
	{
		for(int j = 0; j < geneListNew[i].length(); j++)
		{
			int rnd = rand() % 100;	
			if(rnd < (mutateChance * 100))
			{
				int oldInt, newInt;
				std::string sNewInt;
				
				oldInt = std::atoi(geneListNew[i].substr(j,1).c_str());
				newInt = (rand()%2 ==1) ? oldInt + 1 : oldInt - 1;
				if(newInt == 10) newInt = 0;
				else if(newInt == -1) newInt = 9;
				sNewInt = static_cast<std::ostringstream*>(&(std::ostringstream()<<newInt))->str();
				//std::cout<<rnd<<" -> Mutate: From "<<oldInt<<" to "<<newInt<<"\n";
				
				//std::cout<<geneListNew[i]<<"\n";

				geneListNew[i].replace(j,1,sNewInt);
				//std::cout<<geneListNew[i]<<"\n\n";

			}
		}
	}
	for(int i = 0; i < geneListNew.size(); i++)
	{
		std::cout<<geneListNew[i]<<"\n";
	}
}

std::string Generation::getGeneString(int indx)
{
	return geneListNew[indx];
}
#endif

