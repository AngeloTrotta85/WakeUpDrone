//============================================================================
// Name        : WakeUpDrone.cpp
// Author      : Angelo Trotta
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <stdlib.h>
#include <stdio.h>
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <algorithm>    // std::find
#include <vector>       // std::vector
#include <list>       // std::list
#include <cstdlib>
#include <ctime>

#include <boost/range/irange.hpp>
#include <boost/math/special_functions/factorials.hpp>
#include "MyCoord.h"

using namespace std;
using namespace boost;

class InputParser{
public:
	InputParser (int &argc, char **argv){
		for (int i=1; i < argc; ++i)
			this->tokens.push_back(std::string(argv[i]));
	}
	const std::string& getCmdOption(const std::string &option) const{
		std::vector<std::string>::const_iterator itr;
		itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
		if (itr != this->tokens.end() && ++itr != this->tokens.end()){
			return *itr;
		}
		static const std::string empty_string("");
		return empty_string;
	}
	bool cmdOptionExists(const std::string &option) const{
		return std::find(this->tokens.begin(), this->tokens.end(), option)
		!= this->tokens.end();
	}
private:
	std::vector <std::string> tokens;
};


double fRand(double fMin, double fMax)
{
    double f = (double)rand() / (double)RAND_MAX;
    return fMin + f * (fMax - fMin);
}

void generateRandomSensors(std::list<MyCoord> &pl, int ss, int ns) {
	for (int i : boost::irange(0, ns)) { // i goes from 0 to ns-1
		MyCoord newCoord = MyCoord(fRand(0, ss), fRand(0, ss));
		pl.push_back(newCoord);
		cout << "Sensor: " << i << " --> " << newCoord << endl;
		//cout << newCoord.x << ";" << newCoord.y << endl;
	}
}

void generateRandomUAVs(std::list<MyCoord> &pl, int ss, int nu) {
	for (int i : boost::irange(0, nu)) { // i goes from 0 to nu-1
		MyCoord newCoord = MyCoord(fRand(0, ss), fRand(0, ss));
		pl.push_back(newCoord);
		cout << "UAV: " << i << " --> " << newCoord << endl;
		//cout << newCoord.x << ";" << newCoord.y << endl;
	}
}

void importPoints(std::string inputFileName, std::list<MyCoord> &pl) {
	std::ifstream fileInput(inputFileName, std::ifstream::in);
	std::string str;
	double x, y;

	if(fileInput.is_open()) {
		while (std::getline(fileInput, str)) {
			sscanf(str.c_str(), "%lf;%lf", &x, &y);

			pl.push_back(MyCoord(x, y));

			//cout << "From file: " << str << ". Parsed: " << x << ";" << y << endl;
		}

		fileInput.close();
	}
}


void writeOnFilePoints(std::string fn, std::list<MyCoord> pointList) {
	std::ofstream fout(fn, std::ofstream::out);

	if (fout.is_open()) {
		for (auto& p : pointList) { // i goes from 0 to (ni-1) inclusive
			fout << p.x << ";" << p.y << endl;
		}
		fout.close();
	}

}

int main(int argc, char **argv) {
	std::list<MyCoord> sensorsList;
	std::list<MyCoord> uavsList;
	int scenarioSize = 100;
	int nSensors = 40;
	int nUAV = 3;

	cout << "Wake-up Drone BEGIN!!!" << endl;

	InputParser input(argc, argv);

	const std::string &inputSensorsFileName = input.getCmdOption("-is");
	const std::string &inputUAVsFileName = input.getCmdOption("-iu");
	const std::string &outputSensorsFileName = input.getCmdOption("-os");
	const std::string &outputUAVsFileName = input.getCmdOption("-ou");
	const std::string &inputNumSensors = input.getCmdOption("-ns");
	const std::string &inputNumUAV = input.getCmdOption("-nu");
	const std::string &scenarioMaxVal = input.getCmdOption("-scenario");
	const std::string &seedUser = input.getCmdOption("-seed");

	if (!seedUser.empty()) {
		int seedR = atoi(seedUser.c_str());
		std::srand(seedR);
	}
	else {
		std::srand(std::time(0)); // use current time as seed for random generator
	}
	if (!scenarioMaxVal.empty()) {
		scenarioSize = atoi(scenarioMaxVal.c_str());
	}
	if (!inputNumSensors.empty()) {
		nSensors = atoi(inputNumSensors.c_str());
	}
	if (!inputNumUAV.empty()) {
		nUAV = atoi(inputNumUAV.c_str());
	}

	if (inputSensorsFileName.empty()) {
		generateRandomSensors(sensorsList, scenarioSize, nSensors);
		if (!outputSensorsFileName.empty()) {
			writeOnFilePoints(outputSensorsFileName, sensorsList);
		}
	}
	else {
		importPoints(inputSensorsFileName, sensorsList);
	}

	if (inputUAVsFileName.empty()) {
		generateRandomUAVs(uavsList, scenarioSize, nUAV);
		if (!outputUAVsFileName.empty()) {
			writeOnFilePoints(outputUAVsFileName, uavsList);
		}
	}
	else {
		importPoints(inputUAVsFileName, uavsList);
	}



	cout << "Wake-up Drone FINISH!!!" << endl;
	return EXIT_SUCCESS;
}
