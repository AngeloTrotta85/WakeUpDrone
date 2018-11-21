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

const char* COLOR_LIST_10[] = {
		"green",
		"blue",
		"red",
		"gold",
		"magenta",
		"brown",
		"black",
		"darkorange",
		"salmon",
		"greenyellow"
};

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


class CoordCluster{
public:
	CoordCluster (MyCoord *uav, int cID){
		clusterID = cID;
		clusterHead = new MyCoord(uav->x, uav->y);
		clusterUAV = uav;
	}

	void clear(void) {
		pointsList_bkp.clear();
		for (auto& pp : pointsList) {
			pointsList_bkp.push_back(pp);
		}
		pointsList.clear();
	}

	bool checkChange(void) {
		bool ris = false;
		for (auto& pp : pointsList) {
			bool found = false;
			for (auto& pp_b : pointsList_bkp) {
				if (pp_b == pp) {
					found = true;
					break;
				}
			}
			if (found == false) {
				ris = true;
				break;
			}
		}
		return (ris && (pointsList.size() == pointsList_bkp.size()));
	}

	void printCluter(void) {
		cout << "UAV " << *clusterUAV << " - ";
		for (auto it1 = pointsList.begin(); it1 != pointsList.end(); it1++){
			bool ch = (*it1 == clusterHead);
			if (ch) {
				cout << "*";
			}
			cout << *(*it1);
			if (ch) {
				cout << "*";
			}
			cout << " ";
		}
	}

public:
	//std::list<MyCoord> pointsList;
	std::list<MyCoord *> pointsList;
	std::list<MyCoord *> pointsList_bkp;
	MyCoord *clusterHead;
	MyCoord *clusterUAV;
	int clusterID;
};


double fRand(double fMin, double fMax)
{
    double f = (double)rand() / (double)RAND_MAX;
    return fMin + f * (fMax - fMin);
}

void generateRandomSensors(std::list<MyCoord *> &pl, int ss, int ns) {
	for (int i : boost::irange(0, ns)) { // i goes from 0 to ns-1
		MyCoord *newCoord = new MyCoord(fRand(0, ss), fRand(0, ss));
		pl.push_back(newCoord);
		cout << "Sensor: " << i << " --> " << newCoord << endl;
		//cout << newCoord.x << ";" << newCoord.y << endl;
	}
}

void generateRandomUAVs(std::list<MyCoord *> &pl, int ss, int nu) {
	for (int i : boost::irange(0, nu)) { // i goes from 0 to nu-1
		MyCoord *newCoord = new MyCoord(fRand(0, ss), fRand(0, ss));
		pl.push_back(newCoord);
		cout << "UAV: " << i << " --> " << newCoord << endl;
		//cout << newCoord.x << ";" << newCoord.y << endl;
	}
}

void importPoints(std::string inputFileName, std::list<MyCoord *> &pl) {
	std::ifstream fileInput(inputFileName, std::ifstream::in);
	std::string str;
	double x, y;

	if(fileInput.is_open()) {
		while (std::getline(fileInput, str)) {
			sscanf(str.c_str(), "%lf;%lf", &x, &y);
			MyCoord *np = new MyCoord(x, y);
			pl.push_back(np);

			//cout << "From file: " << str << ". Parsed: " << x << ";" << y << endl;
		}

		fileInput.close();
	}
}


void writeOnFilePoints(std::string fn, std::list<MyCoord *> pointList) {
	std::ofstream fout(fn, std::ofstream::out);

	if (fout.is_open()) {
		for (auto& p : pointList) { // i goes from 0 to (ni-1) inclusive
			fout << p->x << ";" << p->y << endl;
		}
		fout.close();
	}

}

void generateDOTfile(std::string outFileName, std::vector<CoordCluster *> &clustVec, double pSize){
	std::ofstream fout(outFileName, std::ofstream::out);
	int count = 1;
	int count_uav = 1;

	if (fout.is_open()) {

		/*double maxCorrelation = 0;
		int maxIdx = 0;
		for (int i = 0; i < (int) clustVec.size(); i++) {
		//for (auto& cv : clustVec) {
			double actMaxCorrelation = clustVec[i]->getMaxCorrelation();

			if (actMaxCorrelation > maxCorrelation) {
				maxCorrelation = actMaxCorrelation;
				maxIdx = i;
			}

		}*/
		//return maxCorrelation;

		/*for (int i : boost::irange(0,ni)) { // i goes from 0 to (ni-1) inclusive
			if (i >= 0) {
				fout << (std::rand() % 10000) << ";" << (std::rand() % 10000) << endl;
			}
		}*/

		fout << "graph G{" << endl;

		for (int i = 0; i < (int) clustVec.size(); i++) {
			std::string color = std::string(COLOR_LIST_10[i%10]);

			for (auto& p : clustVec[i]->pointsList) {
				fout << "S" << count << " [shape=\"point\" color=\"" << color << "\" pos=\""
						<< p->x << "," << p->y << "!\" width=" << pSize << ", height=" << pSize << "]" << endl;

				/*if (i == maxIdx) {
					fout << "S" << count << "_rad [shape=\"circle\" color=\"" << "black" << "\" style=\"dotted\" label=\"\" pos=\""
							<< p->x << "," << p->y << "!\" width=" << (2.0/maxCorrelation) << ", height=" << (2.0/maxCorrelation) << "]" << endl;
				}*/

				count++;
			}

			fout << "U" << count_uav << " [shape=\"point\" color=\"" << color << "\" pos=\""
					<< clustVec[i]->clusterUAV->x << "," << clustVec[i]->clusterUAV->y << "!\" width=" << pSize*3 << ", height=" << pSize*3 << "]" << endl;

			fout << "C" << count_uav << " [shape=\"point\" color=\"" << color << "\" pos=\""
					<< clustVec[i]->clusterHead->x << "," << clustVec[i]->clusterHead->y << "!\" width=" << pSize*2 << ", height=" << pSize*2 << "]" << endl;

			++count_uav;
		}

		fout << "}" << endl;

		fout.close();
	}
}

void kmeans_clustering(std::vector<CoordCluster *> &cv, std::list<MyCoord *> &sl) {
	bool changed = true;
	int nIter = 100;

	while (changed && (nIter > 0)) {
		// clear the clusters
		for (auto& cc : cv) {
			cc->clear();
		}

		// assign the sensors to the clusters
		for (auto& ss : sl) {
			CoordCluster *closestCL = nullptr;
			double minDist = std::numeric_limits<double>::max();

			for (auto& cc : cv) {
				double ddd = ss->distance(*cc->clusterHead);
				if (ddd < minDist) {
					minDist = ddd;
					closestCL = cc;
				}
			}

			if (closestCL != nullptr) {
				closestCL->pointsList.push_back(ss);
			}
		}

		// update the cluster heads
		for (auto& cc : cv) {
			MyCoord newClusterCoord = MyCoord(0, 0);

			for (auto& ss : cc->pointsList) {
				newClusterCoord += *ss;
			}

			if (cc->pointsList.size() > 0) {
				newClusterCoord /= (double) cc->pointsList.size();
			}

			cc->clusterHead->x = newClusterCoord.x;
			cc->clusterHead->y = newClusterCoord.y;
		}

		// check if updated
		changed = false;
		for (auto& cc : cv) {
			if (cc->checkChange()) {
				changed = true;
				break;
			}
		}
		nIter--;
	}
}

int main(int argc, char **argv) {
	std::list<MyCoord *> sensorsList;
	std::list<MyCoord *> uavsList;
	std::vector<CoordCluster *> clustersVec;
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
	const std::string &dotFileOutput = input.getCmdOption("-dot");

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

	clustersVec.resize(uavsList.size(), nullptr);

	int idd = 0;
	for (auto& uav : uavsList) {
		clustersVec[idd] = new CoordCluster(uav, idd);
		++idd;
	}

	kmeans_clustering(clustersVec, sensorsList);

	if (!dotFileOutput.empty()) {
		generateDOTfile(dotFileOutput, clustersVec, ((double) scenarioSize)/50.0);
	}

	cout << "Wake-up Drone FINISH!!!" << endl;
	return EXIT_SUCCESS;
}
