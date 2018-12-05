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

#define K_D 0.08
#define K_T 0.02
#define K_E 0.0004
#define K_L 0.6

#define ALPHA 0.15
#define BETA 0.15
#define GAMMA 0.7

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
		id = idSensGen++;
	}
	Sensor(MyCoord sensCoord, double re, int id_new) {
		coord = sensCoord;
		residual_energy = re;
		id = id_new;
	}

public:
	MyCoord coord;
	double residual_energy;
	std::list<Readings *> mySensorReadings;
	int id;
	static int idSensGen;
};
int Sensor::idSensGen = 0;

class UAV {
public:
	UAV(MyCoord recCoord) {
		recharge_coord = recCoord;
		id = idUAVGen++;
	}
	UAV(MyCoord recCoord, int id_new) {
		recharge_coord = recCoord;
		id = id_new;
	}

public:
	MyCoord recharge_coord;
	std::list<Readings *> mySensorReadings;
	int id;
	static int idUAVGen;
};
int UAV::idUAVGen = 0;

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
		cout << "Sensor: " << i << " --> " << newS->coord << " - Energy: " << newS->residual_energy << endl;
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

void importReadingsFromFile(std::string inputFileName, std::list<Sensor *> &sl, std::list<UAV *> &ul, int t) {
	std::ifstream fileInput(inputFileName, std::ifstream::in);
	std::string str;
	int t_in, s, u;
	double val;

	if(fileInput.is_open()) {
		while (std::getline(fileInput, str)) {
			Sensor *s_r = nullptr;
			UAV *u_r = nullptr;

			sscanf(str.c_str(), "%d;%lf;%d;%d", &t_in, &val, &s, &u);

			for (auto& s_ok : sl) {
				if (s_ok->id == s) {
					s_r = s_ok;
					break;
				}
			}
			for (auto& u_ok : ul) {
				if (u_ok->id == u) {
					u_r = u_ok;
					break;
				}
			}

			if ((s_r != nullptr) && (u_r != nullptr)) {
				Readings *nr = new Readings(s_r, u_r, t_in, val);
				s_r->mySensorReadings.push_back(nr);
				u_r->mySensorReadings.push_back(nr);
			}
			//cout << "From file: " << str << ". Parsed sensor: " << x << ";" << y << endl;
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

bool compare_readings (const Readings *first, const Readings *second) {
	return ( first->read_time < second->read_time );
}
void writeOnFileReadings(std::string fn, std::list<Sensor *> &sl, int t) {
	std::ofstream fout(fn, std::ofstream::out);
	if (fout.is_open()) {
		std::list <Readings *> rl;
		for (auto& s : sl) {
			for (auto& r : s->mySensorReadings) {
				rl.push_back(r);
			}
		}
		rl.sort(compare_readings);
		for (auto& r : rl) {
			fout << r->read_time << ";" << r->value << ";" << r->sensor->id << ";" << r->uav->id << endl;
		}
		fout.close();
	}
}

double calculate_loss_energy(Sensor *se, int tk, std::list<Sensor *> &sl) {
	double ris = 0;
	for (auto& ss : sl) {
		if (ss != se) {
			//double old_ris = ris;
			double actLoss = 0;
			if (se->residual_energy < ss->residual_energy) {
				//double exp_exponent = (se->residual_energy - ss->residual_energy) * K_E;
				//double actLoss = 1.0 / (1.0 + exp(exp_exponent));

				double exp_exponent = (ss->residual_energy - se->residual_energy) * K_E;
				actLoss = 1.0 - (1.0 / exp(exp_exponent));

				//ris = algebraic_sum(ris, actLoss);
				if (actLoss > ris) {
					ris = actLoss;
				}
			}
			//cout << "Sensor Energy - " << se->residual_energy << " - " << ss->residual_energy << " = " << (se->residual_energy - ss->residual_energy);
			//cout << " - old:" << old_ris << " +^ actLoss:" << actLoss << " = ris:" << ris << endl;
		}
	}
	//cout << endl;
	return ris;
}

double calculate_loss_last(Sensor *se, int tk, std::list<Sensor *> &sl) {
	double ris = 0;
	int my_tk = 0;
	int older_R = std::numeric_limits<int>::max();
	int newer_R = 0;

	if (se->mySensorReadings.size()> 0) {
		my_tk = se->mySensorReadings.front()->read_time;
	}

	for (auto& ss : sl) {
		int last_tk = 0;
		if (ss->mySensorReadings.size()> 0) {
			last_tk = ss->mySensorReadings.front()->read_time;
		}

		if (last_tk < older_R)
			older_R = last_tk;

		if (last_tk > newer_R)
			newer_R = last_tk;
	}

	//check my time ('se' can not be in 'sl')
	if (my_tk < older_R)
		older_R = my_tk;

	if (my_tk > newer_R)
		newer_R = my_tk;

	if (newer_R > older_R) {
		ris = (((double) my_tk) - ((double) older_R)) / (((double) newer_R) - ((double) older_R));
		ris = pow(ris, K_L);
	}
	/*for (auto& ss : sl) {
		if (ss != se) {
			double old_ris = ris;

			int last_tk = 0;
			if (ss->mySensorReadings.size()> 0) last_tk = ss->mySensorReadings.front()->read_time;
			double exp_exponent = (tk - last_tk) * K_L;
			double actLoss = 1.0 - (1.0 / exp(exp_exponent));
			ris = algebraic_sum(ris, actLoss);

			cout << "Sensor Last - " << tk << " - " << last_tk << " = " << (tk - last_tk);
			cout << " - old:" << old_ris << " +^ actLoss:" << actLoss << " = ris:" << ris << endl;
		}
	}*/
	//cout << "Sensor Last - " << older_R << " - " << newer_R << " --> " << my_tk << ". RIS:" << ris << endl;
	//cout << endl;
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
		//if (ss != se) {
		for (auto& r : ss->mySensorReadings) {
			//double old_ris = ris;

			double loss_dist = calculate_loss_distance(se, ss);
			double loss_time = calculate_loss_time(tk, r->read_time);
			//double actLoss = algebraic_sum(loss_dist, loss_time);
			double actLoss = loss_dist * loss_time;
			//ris = algebraic_sum(ris, actLoss);
			if (actLoss > ris) {
				ris = actLoss;
			}

			//cout << "Sensor Correlation - DIST: " << loss_dist << " TIME:" << loss_time;
			//cout << " - old:" << old_ris << " +^ actLoss:" << actLoss << " = ris:" << ris << endl;
		}
		//}
	}
	//cout << endl;
	return ris;
}

double calculate_loss_full(Sensor *se, int tk, std::list<Sensor *> &sl, double alpha, double beta, double gamma) {
	double loss_energy = calculate_loss_energy(se, tk, sl);
	double loss_last = calculate_loss_last(se, tk, sl);
	double loss_corr = calculate_loss_correlation(se, tk, sl);

	double ris = algebraic_sum( algebraic_sum(alpha * loss_energy, beta * loss_last), gamma * loss_corr);

	//cout << "calculate_loss_full: " << ris
	//		<< "; loss_energy: " << loss_energy
	//		<< "; loss_last: " << loss_last
	//		<< "; loss_corr: " << loss_corr
	//		<< endl << flush;

	return ris;
}

void generateDOTfile(std::string outFileName, std::vector<CoordCluster *> &clustVec, std::list<Sensor *> &sensList,
		double sSize, double pSize, int timeNow){
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
				double sLossFull = calculate_loss_full(p, timeNow, sensList, ALPHA, BETA, GAMMA);
				//double sLossLast = calculate_loss_last(p, timeNow, sensList);
				//double sLossEnergy = calculate_loss_energy(p, timeNow, sensList);
				//double sLossCorr = calculate_loss_correlation(p, timeNow, sensList);


				//cout << "Sensor " << count
				//		<< " has loss full: " << sLossFull
				//		<< " has loss last: " << sLossLast
				//		<< " has loss energy: " << sLossEnergy
				//		<< " has loss correlation: " << sLossCorr
				//		<< endl;


				double sSize = pSize * (2 - sLossFull);
				fout << "S" << count << " [shape=\"point\" color=\"" << color
						<< "\" pos=\"" << p->coord.x << "," << p->coord.y << "!\" width="
						<< sSize << ", height=" << sSize << "]" << endl;

				/*if (i == maxIdx) {
					fout << "S" << count << "_rad [shape=\"circle\" color=\"" << "black" << "\" style=\"dotted\" label=\"\" pos=\""
							<< p->x << "," << p->y << "!\" width=" << (2.0/maxCorrelation) << ", height=" << (2.0/maxCorrelation) << "]" << endl;
				}*/

				count++;
			}

			fout << "U" << count_uav << " [shape=\"star\" color=\"" << color << "\" pos=\""
					<< clustVec[i]->clusterUAV->recharge_coord.x << "," << clustVec[i]->clusterUAV->recharge_coord.y << "!\" width=" << pSize*3 << ", height=" << pSize*3 << "]" << endl;

			fout << "C" << count_uav << " [shape=\"diamond\" color=\"" << color << "\" pos=\""
					<< clustVec[i]->clusterHead->x << "," << clustVec[i]->clusterHead->y << "!\" width=" << pSize*2 << ", height=" << pSize*2 << "]" << endl;

			++count_uav;
		}

		fout << "}" << endl;

		fout.close();
	}
}

void printLossStats(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now) {
	for (auto& c : cv) {
		double sumLosses = 0;
		double avgLoss = 0;
		double minLoss = std::numeric_limits<double>::max();
		double maxLoss = -1;

		for (auto& s : c->pointsList) {
			//double actLoss_only = calculate_loss_full(s, time_now, c->pointsList,  ALPHA, BETA, GAMMA);
			double actLoss_only = calculate_loss_full(s, time_now, sl,  ALPHA, BETA, GAMMA);
			double actLoss = s->coord.distance(c->clusterUAV->recharge_coord) * actLoss_only;

			sumLosses += actLoss;

			if (actLoss > maxLoss) {
				maxLoss = actLoss;
			}
			if (actLoss < minLoss) {
				minLoss = actLoss;
			}
		}
		if (c->pointsList.size() > 0) {
			avgLoss = sumLosses / ((double) c->pointsList.size());
		}

		cout << "Cluster " << c->clusterID << "[" << COLOR_LIST_10[(c->clusterID)%10] << "] has avLoss: " << avgLoss <<
				"[" << minLoss << "; " << maxLoss << "]"
				" sumLosses: " << sumLosses << endl;
	}
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
					//multiplier = (1.0 - calculate_loss_full(ss, time_now, cc->pointsList,  ALPHA, BETA, GAMMA));
					multiplier = calculate_loss_full(ss, time_now, cc->pointsList,  ALPHA, BETA, GAMMA);
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

void randomizeClusters(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl) {
	unsigned int n4cluster = floor(double(sl.size()) / double(cv.size()));
	unsigned int remainingP = sl.size() - (cv.size() * n4cluster);

	std::vector<Sensor *> sl_bkp;
	for (auto& s : sl) {
		sl_bkp.push_back(s);
	}
	std::random_shuffle(sl_bkp.begin(), sl_bkp.end());

	auto it_s = sl_bkp.begin();

	for (auto& c : cv) {

		if (it_s == sl_bkp.end()) break;

		for (unsigned int j = 0; j < n4cluster; ++j) {
			c->pointsList.push_back(*it_s);
			it_s++;
		}

		if (it_s == sl_bkp.end()) break;

		if (remainingP > 0) {
			c->pointsList.push_back(*it_s);
			--remainingP;
			it_s++;
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
}

void equalizeLoss(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now) {
	for (auto& c : cv) {
		double sumLosses = 0;
		double avgLoss = 0;
		double minLoss = std::numeric_limits<double>::max();
		double maxLoss = -1;

		for (auto& s : c->pointsList) {
			//double actLoss_only = calculate_loss_full(s, time_now, c->pointsList,  ALPHA, BETA, GAMMA);
			double actLoss_only = calculate_loss_full(s, time_now, sl,  ALPHA, BETA, GAMMA);
			double actLoss = s->coord.distance(c->clusterUAV->recharge_coord) * actLoss_only;

			sumLosses += actLoss;

			if (actLoss > maxLoss) {
				maxLoss = actLoss;
			}
			if (actLoss < minLoss) {
				minLoss = actLoss;
			}
		}
		if (c->pointsList.size() > 0) {
			avgLoss = sumLosses / ((double) c->pointsList.size());
		}

		cout << "Cluster " << c->clusterID << " has avLoss: " << avgLoss << "[" << minLoss << "; " << maxLoss << "]" << endl;
	}
}

void equalizerLoss_clustering(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now) {
	randomizeClusters(cv, sl);

	equalizeLoss(cv, sl, time_now);
}

void randomMinimumLoss_clustering(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now) {
	unsigned int n4cluster = floor(double(sl.size()) / double(cv.size()));
	unsigned int remainingP = sl.size() - (cv.size() * n4cluster);

	std::vector<Sensor *> sl_bkp;
	for (auto& s : sl) {
		sl_bkp.push_back(s);
	}
	std::random_shuffle(sl_bkp.begin(), sl_bkp.end());

	for (auto& s : sl_bkp) {
		double actLoss_only = calculate_loss_full(s, time_now, sl,  ALPHA, BETA, GAMMA);
		CoordCluster *closesrC = nullptr;
		double minLossC = std::numeric_limits<double>::max();

		for (auto& c : cv) {
			if ((c->pointsList.size() < n4cluster) || ((c->pointsList.size() == n4cluster) && (remainingP > 0))) {
				double actLoss = s->coord.distance(c->clusterUAV->recharge_coord) * actLoss_only;

				if (actLoss < minLossC) {
					actLoss = minLossC;
					closesrC = c;
				}
			}
		}
		if (closesrC != nullptr) {
			if (closesrC->pointsList.size() == n4cluster) {
				--remainingP;
			}
			closesrC->pointsList.push_back(s);
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
}

void roundRobinMinimumLossLocal_clustering(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now) {

	std::list<Sensor *> sl_bkp;
	for (auto& s : sl) {
		sl_bkp.push_back(s);
	}
	std::vector<CoordCluster *> cv_bkp;
	for (auto& c : cv) {
		cv_bkp.push_back(c);
	}
	std::random_shuffle(cv_bkp.begin(), cv_bkp.end());

	auto it_c = cv_bkp.begin();
	while (!sl_bkp.empty()) {
		std::list<Sensor *>::iterator closestSensor; // = sl_bkp.begin();
		double minLossC = std::numeric_limits<double>::max();


		for (auto it_s = sl_bkp.begin(); it_s != sl_bkp.end(); it_s++) {
			double actLoss_only = calculate_loss_full(*it_s, time_now, sl,  ALPHA, BETA, GAMMA);
			double actLossLocal_only = calculate_loss_full(*it_s, time_now, (*it_c)->pointsList,  ALPHA, BETA, GAMMA);
			double actLoss_sum = algebraic_sum(actLoss_only, actLossLocal_only);
			double distance = (*it_s)->coord.distance((*it_c)->clusterUAV->recharge_coord);
			double actLoss = distance * actLoss_sum;

			//cout << "Losses ->"
			//		<< " actLoss_only: " << actLoss_only
			//		<< " actLossLocal_only: " << actLossLocal_only << "(pl_size: " << (*it_c)->pointsList.size() << ")"
			//		<< " actLoss_sum: " << actLoss_sum
			//		<< " actLoss: " << actLoss << endl << flush;

			if (actLoss < minLossC) {
				minLossC = actLoss;
				closestSensor = it_s;
			}
		}

		(*it_c)->pointsList.push_back(*closestSensor);
		sl_bkp.erase(closestSensor);

		it_c++;
		if (it_c == cv_bkp.end()){
			std::random_shuffle(cv_bkp.begin(), cv_bkp.end());
			it_c = cv_bkp.begin();
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

}

void roundRobinMinimumLoss_clustering(std::vector<CoordCluster *> &cv, std::list<Sensor *> &sl, int time_now) {

	std::list<Sensor *> sl_bkp;
	for (auto& s : sl) {
		sl_bkp.push_back(s);
	}

	auto it_c = cv.begin();
	while (!sl_bkp.empty()) {
		std::list<Sensor *>::iterator closestSensor;
		double minLossC = std::numeric_limits<double>::max();

		for (auto it_s = sl_bkp.begin(); it_s != sl_bkp.end(); it_s++) {
			double actLoss_only = calculate_loss_full(*it_s, time_now, sl,  ALPHA, BETA, GAMMA);
			double actLoss = (*it_s)->coord.distance((*it_c)->clusterUAV->recharge_coord) * actLoss_only;

			if (actLoss < minLossC) {
				minLossC = actLoss;
				closestSensor = it_s;
			}
		}

		(*it_c)->pointsList.push_back(*closestSensor);
		sl_bkp.erase(closestSensor);

		it_c++;
		if (it_c == cv.end()){
			it_c = cv.begin();
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

void printLogsSensors (std::list<Sensor *> &sl, int timeNow) {
	int count = 0;
	for (auto& p : sl) {
		double sLossFull = calculate_loss_full(p, timeNow, sl, ALPHA, BETA, GAMMA);
		double sLossLast = calculate_loss_last(p, timeNow, sl);
		double sLossEnergy = calculate_loss_energy(p, timeNow, sl);
		double sLossCorr = calculate_loss_correlation(p, timeNow, sl);

		cout  << "Sensor " << count
				<< " --> " << p->coord << ", Energy: " << p->residual_energy
				<< " LF: " << sLossFull
				<< " LL: " << sLossLast
				<< " LE: " << sLossEnergy
				<< " LC: " << sLossCorr
				<< " - ";

		cout << "(";
		for (auto& r : p->mySensorReadings) {
			cout << r->read_time << " ";
		}
		cout << ")" << endl;

		count++;
	}
}

int main(int argc, char **argv) {
	std::list<Sensor *> sensorsList;
	std::list<UAV *> uavsList;
	std::vector<CoordCluster *> clustersVec;
	int scenarioSize = 100;
	int nSensors = 40;
	int nUAV = 3;
	int time_N = 100;

	cout << "Wake-up Drone BEGIN!!!" << endl;

	InputParser input(argc, argv);

	const std::string &inputSensorsFileName = input.getCmdOption("-is");
	const std::string &inputUAVsFileName = input.getCmdOption("-iu");
	const std::string &outputSensorsFileName = input.getCmdOption("-os");
	const std::string &outputUAVsFileName = input.getCmdOption("-ou");
	const std::string &inputNumSensors = input.getCmdOption("-ns");
	const std::string &inputNumUAV = input.getCmdOption("-nu");
	const std::string &inputReadingsFileName = input.getCmdOption("-ir");
	const std::string &outputReadingsFileName = input.getCmdOption("-or");
	const std::string &scenarioMaxVal = input.getCmdOption("-scenario");
	const std::string &seedUser = input.getCmdOption("-seed");
	const std::string &dotFileOutput = input.getCmdOption("-dot");
	const std::string &inputTimeSim = input.getCmdOption("-time");
	const std::string &algotype = input.getCmdOption("-algo");

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
	if (!inputTimeSim.empty()) {
		time_N = atoi(inputTimeSim.c_str());
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

	if (inputReadingsFileName.empty()) {
		generate_readings(sensorsList, uavsList, time_N);
		if (!outputReadingsFileName.empty()) {
			writeOnFileReadings(outputReadingsFileName, sensorsList, time_N);
		}
	}
	else {
		importReadingsFromFile(inputReadingsFileName, sensorsList, uavsList, time_N);
	}

	clustersVec.resize(uavsList.size(), nullptr);

	int idd = 0;
	for (auto& uav : uavsList) {
		clustersVec[idd] = new CoordCluster(uav, idd);
		++idd;
	}

	printLogsSensors(sensorsList, time_N);

	if (!algotype.empty()) {
		if (algotype.compare("kmS") == 0) {
			kmeans_clustering(clustersVec, sensorsList); //simple k-means
		}
		else if (algotype.compare("kmR") == 0) {
			kmeans_clustering_withReadings(clustersVec, sensorsList, time_N);	//k-means with loss in the distance calculus
		}
		else if (algotype.compare("eqLoss") == 0) {
			equalizerLoss_clustering(clustersVec, sensorsList, time_N);	// just random for now
		}
		else if (algotype.compare("eqRandLoss") == 0) {
			randomMinimumLoss_clustering(clustersVec, sensorsList, time_N);	// from each random sensor choose the closest cluster
		}
		else if (algotype.compare("rrMinLoss") == 0) {
			roundRobinMinimumLoss_clustering(clustersVec, sensorsList, time_N);	// round robin cluster, choose mis loss sensor
		}
		else if (algotype.compare("rrMinLossLocal") == 0) {
			roundRobinMinimumLossLocal_clustering(clustersVec, sensorsList, time_N);	// round robin cluster, choose mis loss sensor
		}
		else {
			cerr << "Unknown algotype: \"" << algotype << "\". Using default simple k-means" << endl;
			kmeans_clustering(clustersVec, sensorsList);
		}

		printLossStats(clustersVec, sensorsList, time_N);

	}
	else {
		//default simple k-means
		kmeans_clustering(clustersVec, sensorsList);
	}

	//kmeans_clustering(clustersVec, sensorsList);
	//kmeans_clustering_withReadings(clustersVec, sensorsList, time_N);

	if (!dotFileOutput.empty()) {
		generateDOTfile(dotFileOutput, clustersVec, sensorsList, scenarioSize, ((double) scenarioSize)/50.0, time_N);
	}

	cout << "Wake-up Drone FINISH!!!" << endl;
	return EXIT_SUCCESS;
}
