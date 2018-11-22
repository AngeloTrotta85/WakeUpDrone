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
#include <random>
#include <chrono>

#include <boost/range/irange.hpp>
#include <boost/math/special_functions/factorials.hpp>
#include "MyCoord.h"

#define K_D 50
#define K_T 30
#define K_E 20
#define K_L 20

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

std::default_random_engine *generator_rand;

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

class Sensor;
class UAV;

class Readings {
public:
	Readings(Sensor *s, UAV *u, int timestamp, double val) {
		sensor = s;
		uav = u;
		value = val;
		read_time = timestamp;
	}

public:
	Sensor *sensor;
	UAV *uav;
	int  read_time;
	double value;
};

class Sensor {
public:
	Sensor(MyCoord sensCoord, double re) {
		coord = sensCoord;
		residual_energy = re;
	}

public:
	MyCoord coord;
	double residual_energy;
	std::list<Readings *> mySensorReadings;
};

class UAV {
public:
	UAV(MyCoord recCoord) {
		recharge_coord = recCoord;
	}

public:
	MyCoord recharge_coord;
	std::list<Readings *> mySensorReadings;
};

class CoordCluster{
public:
	CoordCluster (UAV *uav, int cID){
		clusterID = cID;
		clusterHead = new MyCoord(uav->recharge_coord.x, uav->recharge_coord.y);
		clusterUAV = uav;
	}

	void clear(void) {
		pointsList_bkp.clear();
		for (auto& pp : pointsList) {
			pointsList_bkp.push_back(pp);
		}
		pointsList.clear();
	}

	bool checkNotChange(void) {
		/*cout << "Checking ";
		for (auto& pp : pointsList) cout << pp << " ";
		cout << " VS ";
		for (auto& pp : pointsList_bkp) cout << pp << " ";
		cout << endl;*/
		bool ris = true;
		for (auto& pp : pointsList) {
			bool found = false;
			for (auto& pp_b : pointsList_bkp) {
				if (pp_b == pp) {
					found = true;
					break;
				}
			}
			if (found == false) {
				ris = false;
				break;
			}
		}
		return (ris && (pointsList.size() == pointsList_bkp.size()));
	}

public:
	//std::list<MyCoord> pointsList;
	std::list<Sensor *> pointsList;
	std::list<Sensor *> pointsList_bkp;
	MyCoord *clusterHead;
	UAV *clusterUAV;
	int clusterID;
};

double algebraic_sum(double a, double b) {
	return (a + b - (a * b));
}

void generateRandomSensors(std::list<Sensor *> &pl, int ss, int ns) {

	for (int i : boost::irange(0, ns)) { // i goes from 0 to ns-1
		std::uniform_real_distribution<double> uniform_distribution(0.0, ss);
		std::normal_distribution<double> n_distribution(10000.0,1000.0);

		Sensor *newS = new Sensor(MyCoord(uniform_distribution(*generator_rand), uniform_distribution(*generator_rand)), n_distribution(*generator_rand));
		pl.push_back(newS);
		cout << "Sensor: " << i << " --> " << newS->coord << endl;
	}
}

void generateRandomUAVs(std::list<UAV *> &pl, int ss, int nu) {
	for (int i : boost::irange(0, nu)) { // i goes from 0 to nu-1
		std::uniform_real_distribution<double> uniform_distribution(0.0, ss);
		UAV *newU = new UAV(MyCoord(uniform_distribution(*generator_rand), uniform_distribution(*generator_rand)));
		pl.push_back(newU);
		cout << "UAV: " << i << " --> " << newU->recharge_coord << endl;
	}
}

void importSensorsFromFile(std::string inputFileName, std::list<Sensor *> &pl) {
	std::ifstream fileInput(inputFileName, std::ifstream::in);
	std::string str;
	double x, y, e;
	if(fileInput.is_open()) {
		while (std::getline(fileInput, str)) {
			sscanf(str.c_str(), "%lf;%lf;%lf", &x, &y, &e);
			Sensor *np = new Sensor(MyCoord(x, y), e);
			pl.push_back(np);
			//cout << "From file: " << str << ". Parsed sensor: " << x << ";" << y << endl;
		}
		fileInput.close();
	}
}

void importUAVsFromFile(std::string inputFileName, std::list<UAV *> &pl) {
	std::ifstream fileInput(inputFileName, std::ifstream::in);
	std::string str;
	double x, y;
	if(fileInput.is_open()) {
		while (std::getline(fileInput, str)) {
			sscanf(str.c_str(), "%lf;%lf", &x, &y);
			UAV *np = new UAV(MyCoord(x, y));
			pl.push_back(np);
			//cout << "From file: " << str << ". Parsed sensor: " << x << ";" << y << endl;
		}
		fileInput.close();
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


void writeOnFileSensors(std::string fn, std::list<Sensor *> pointList) {
	std::ofstream fout(fn, std::ofstream::out);
	if (fout.is_open()) {
		for (auto& p : pointList) { // i goes from 0 to (ni-1) inclusive
			fout << p->coord.x << ";" << p->coord.y << ";" << p->residual_energy << endl;
		}
		fout.close();
	}
}
void writeOnFileUAVs(std::string fn, std::list<UAV *> pointList) {
	std::ofstream fout(fn, std::ofstream::out);
	if (fout.is_open()) {
		for (auto& p : pointList) { // i goes from 0 to (ni-1) inclusive
			fout << p->recharge_coord.x << ";" << p->recharge_coord.y << endl;
		}
		fout.close();
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

void generateDOTfile(std::string outFileName, std::vector<CoordCluster *> &clustVec, double sSize, double pSize){
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

		fout 	<< "{node [style=invis] P00 P01 P10 P11}"
				<< "P00 [pos = \"0,0!\"]"
				<< "P01 [pos = \"0," << sSize << "!\"]"
				<< "P10 [pos = \"" << sSize << ",0!\"]"
				<< "P11 [pos = \"" << sSize << "," << sSize << "!\"]"
				<< endl << endl;

		for (int i = 0; i < (int) clustVec.size(); i++) {
			std::string color = std::string(COLOR_LIST_10[i%10]);

			for (auto& p : clustVec[i]->pointsList) {
				fout << "S" << count << " [shape=\"point\" color=\"" << color << "\" pos=\""
						<< p->coord.x << "," << p->coord.y << "!\" width=" << pSize << ", height=" << pSize << "]" << endl;

				/*if (i == maxIdx) {
					fout << "S" << count << "_rad [shape=\"circle\" color=\"" << "black" << "\" style=\"dotted\" label=\"\" pos=\""
							<< p->x << "," << p->y << "!\" width=" << (2.0/maxCorrelation) << ", height=" << (2.0/maxCorrelation) << "]" << endl;
				}*/

				count++;
			}

			fout << "U" << count_uav << " [shape=\"point\" color=\"" << color << "\" pos=\""
					<< clustVec[i]->clusterUAV->recharge_coord.x << "," << clustVec[i]->clusterUAV->recharge_coord.y << "!\" width=" << pSize*3 << ", height=" << pSize*3 << "]" << endl;

			fout << "C" << count_uav << " [shape=\"point\" color=\"" << color << "\" pos=\""
					<< clustVec[i]->clusterHead->x << "," << clustVec[i]->clusterHead->y << "!\" width=" << pSize*2 << ", height=" << pSize*2 << "]" << endl;

			++count_uav;
		}

		fout << "}" << endl;

		fout.close();
	}
}

double calculate_loss_energy(Sensor *se, int tk, std::list<Sensor *> &sl) {
	double ris = 0;
	for (auto& ss : sl) {
		if (ss != se) {
			double exp_exponent = (se->residual_energy - ss->residual_energy) * K_E;
			double actLoss = 1.0 / (1.0 + exp(exp_exponent));
			ris = algebraic_sum(ris, actLoss);
		}
	}
	return ris;
}

double calculate_loss_last(Sensor *se, int tk, std::list<Sensor *> &sl) {
	double ris = 0;
	for (auto& ss : sl) {
		if (ss != se) {
			int last_tk = 0;
			if (ss->mySensorReadings.size()> 0) last_tk = ss->mySensorReadings.front()->read_time;
			double exp_exponent = (tk - last_tk) * K_L;
			double actLoss = 1.0 - (1.0 / exp(exp_exponent));
			ris = algebraic_sum(ris, actLoss);
		}
	}
	return ris;
}

double calculate_loss_distance(Sensor *s1, Sensor *s2) {
	double exp_exponent = s1->coord.distance(s2->coord) * K_D;
	return (1.0 / exp(exp_exponent));
}

double calculate_loss_time(int t1, int t2) {
	double exp_exponent = abs(t1 - t2) * K_T;
	return (1.0 / exp(exp_exponent));
}

double calculate_loss_correlation(Sensor *se, int tk, std::list<Sensor *> &sl) {
	double ris = 0;
	for (auto& ss : sl) {
		if (ss != se) {
			for (auto& r : ss->mySensorReadings) {
				double actLoss = calculate_loss_distance(se, ss) + calculate_loss_time(tk, r->read_time);
				ris = algebraic_sum(ris, actLoss);
			}
		}
	}
	return ris;
}

double calculate_loss_full(Sensor *se, int tk, std::list<Sensor *> &sl, double alpha, double beta, double gamma) {
	double ris = algebraic_sum(alpha * calculate_loss_energy(se, tk, sl), beta * calculate_loss_last(se, tk, sl));
	return algebraic_sum(ris, calculate_loss_correlation(se, tk, sl));
}

void kmeans_clustering(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl) {
	bool changed = true;
	int nIter = 200;

	while (changed && (nIter > 0)) {
	//while (nIter > 0) {
		// clear the clusters
		for (auto& cc : cv) {
			cc->clear();
		}

		// assign the sensors to the clusters
		for (auto& ss : sl) {
			CoordCluster *closestCL = nullptr;
			double minDist = std::numeric_limits<double>::max();

			for (auto& cc : cv) {
				double ddd = ss->coord.distance(*cc->clusterHead);
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
				newClusterCoord += ss->coord;
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
			if (!cc->checkNotChange()) {
				changed = true;
				break;
			}
		}
		nIter--;
	}
	cout << "k-means ended in " << 200 - nIter << " iterations" << endl;
}


void kmeans_clustering_withReadings(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now) {
	bool changed = true;
	int nIter = 200;

	while (changed && (nIter > 0)) {
	//while (nIter > 0) {
		// clear the clusters
		for (auto& cc : cv) {
			cc->clear();
		}

		// assign the sensors to the clusters
		for (auto& ss : sl) {
			CoordCluster *closestCL = nullptr;
			double minDist = std::numeric_limits<double>::max();

			for (auto& cc : cv) {
				double multiplier = 0.0;
				if (cc->pointsList.size() > 0) {
					//multiplier = (1.0 - calculate_loss_full(ss, time_now, cc->pointsList,  0.25, 0.25, 0.5));
					multiplier = calculate_loss_full(ss, time_now, cc->pointsList,  0.25, 0.25, 0.5);
				}
				double ddd = ss->coord.distance(*cc->clusterHead) * multiplier;

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
				newClusterCoord += ss->coord;
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
			if (!cc->checkNotChange()) {
				changed = true;
				break;
			}
		}
		nIter--;
	}
	cout << "k-means ended in " << 200 - nIter << " iterations" << endl;
}

void generate_readings(std::list<Sensor *> &sl, std::list<UAV *> &ul, int maxTime) {
	std::uniform_real_distribution<double> uniform_distribution(0.0, 1.0);
	std::uniform_int_distribution<int> uniform_distribution_uav(0.0, ul.size()-1);
	for (auto& ss : sl) {
		for (int i = 0 ; i < maxTime; i++) {
			double r_reading = uniform_distribution(*generator_rand);

			if (r_reading < 0.025) {
				int r_uav = uniform_distribution_uav(*generator_rand);

				//UAV *uOK = *ul.begin();
				auto uOK = ul.begin();
				while (r_uav > 0) {
					--r_uav;
					uOK++;
				}
				UAV *u_reading = *uOK;
				Readings *read_new = new Readings(ss, u_reading, i, uniform_distribution(*generator_rand));
				ss->mySensorReadings.push_front(read_new);
				u_reading->mySensorReadings.push_front(read_new);
			}

		}
	}
}

int main(int argc, char **argv) {
	std::list<Sensor *> sensorsList;
	std::list<UAV *> uavsList;
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
		generator_rand = new std::default_random_engine(seedR);
	}
	else {
		unsigned seedR = std::chrono::system_clock::now().time_since_epoch().count();
		generator_rand = new std::default_random_engine(seedR);
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
			writeOnFileSensors(outputSensorsFileName, sensorsList);
		}
	}
	else {
		importSensorsFromFile(inputSensorsFileName, sensorsList);
	}

	if (inputUAVsFileName.empty()) {
		generateRandomUAVs(uavsList, scenarioSize, nUAV);
		if (!outputUAVsFileName.empty()) {
			writeOnFileUAVs(outputUAVsFileName, uavsList);
		}
	}
	else {
		importUAVsFromFile(inputUAVsFileName, uavsList);
	}

	clustersVec.resize(uavsList.size(), nullptr);

	int idd = 0;
	for (auto& uav : uavsList) {
		clustersVec[idd] = new CoordCluster(uav, idd);
		++idd;
	}

	//kmeans_clustering(clustersVec, sensorsList);

	generate_readings(sensorsList, uavsList, 100);
	kmeans_clustering_withReadings(clustersVec, sensorsList, 100);

	if (!dotFileOutput.empty()) {
		generateDOTfile(dotFileOutput, clustersVec, scenarioSize, ((double) scenarioSize)/50.0);
	}

	cout << "Wake-up Drone FINISH!!!" << endl;
	return EXIT_SUCCESS;
}
