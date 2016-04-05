#ifndef GENE_H
#define GENE_H

#include <iostream>
#include <string>
#include <sstream>


#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#define PI 3.14159

class Gene
{
	public:
	void display();
	Gene();
	std::string toString(int n);
	int toInt(std::string str);
	double getAmp(int indx);
	double getPhase(int indx);
	double getFreq(int indx);
	std::string getGeneString();
	void setGeneString(std::string geneInpt);
	private: 
	std::string strVal;

};

Gene::Gene(void)
{	
	strVal = "";

	int rndX = 0;
	for(int i = 0; i < 18; i++)
	{
		rndX = rand() % 100;
		if(rndX==0)
			strVal = strVal + "00";	
		else if(rndX>0&&rndX<10)
			strVal = strVal + "0" + this->toString(rndX);
		else
			strVal = strVal + this->toString(rndX);
	}

}

std::string Gene::toString(int n)
{
	std::string cvtStr;
	std::ostringstream convert;
	convert<<n;
	cvtStr = convert.str();
	
	return cvtStr;
}
int Gene::toInt(std::string str)
{
	int cvtInt;
	std::istringstream convert(str);
	convert>>cvtInt;
	return cvtInt;
}


void Gene::display()
{
	std::cout<<strVal<<"\n";
}

double Gene::getAmp(int indx)
{
	double cvtAmp;
	std::string strAmp;
	strAmp = strVal.substr(0+indx*6,2);
	cvtAmp = 1 + (double)this->toInt(strAmp)/100. * 7;
	return cvtAmp;			
}
double Gene::getPhase(int indx)
{
	double cvtPhase;
	std::string strAmp;
	strAmp = strVal.substr(2+indx*6,2);
	cvtPhase = (double)this->toInt(strAmp)/100. * 2 * PI;
	return cvtPhase;			
}
double Gene::getFreq(int indx)
{
	double cvtFreq;
	std::string strAmp;
	strAmp = strVal.substr(4+indx*6,2);
	cvtFreq = 1 + (double)this->toInt(strAmp)/100. * 4;	

	return cvtFreq;			
}

std::string Gene::getGeneString()
{
	return strVal;
}

void Gene::setGeneString(std::string geneInpt)
{
	strVal = geneInpt;
}


#endif

