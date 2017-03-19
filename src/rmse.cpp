/**
 * main.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: MBRDNA
 */

#include <iostream>
#include "Eigen/Dense"
#include <vector>
#include "FusionEKF.h"
#include "measurement_package.h"
#include "tools.h"
#include <fstream>
#include <sstream>
#include <stdlib.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void readStateSeq(ifstream &file, vector<VectorXd> &state_seq){
	string line;

	VectorXd v;

	while (getline(file, line)) {
		istringstream iss(line);

		float x_gt;
		float y_gt;
		float vx_gt;
		float vy_gt;
		iss >> x_gt;
		iss >> y_gt;
		iss >> vx_gt;
		iss >> vy_gt;
		v = VectorXd(4);
		v << x_gt, y_gt, vx_gt, vy_gt;
		state_seq.push_back(v);
	}
}

int main(int argc, char* argv[]) {

	string usage_instructions = "Usage instructions: ./CalcRMSE path/to/estimations.txt path/to/ground_truth.txt";
	bool has_valid_args = false;

	// make sure the user has provided input and output files
	if (argc == 1)
	{
		cerr << usage_instructions << endl;
	}

	if (argc == 2)
	{
		cerr << "Please include an ground_truth file.\n" << usage_instructions << endl;
	}

	if (argc == 3)
	{
		has_valid_args = true;
	}

	if (argc > 3)
	{
		cerr << "Too many arguments.\n" << usage_instructions << endl;
	}

	if (!has_valid_args)
	{
		exit(EXIT_FAILURE);
	}

	// get estimations file
	string estimations_file_name_ = argv[1];
	ifstream estimations_file_(estimations_file_name_.c_str(), std::ifstream::in);

	// get ground truth file
	string ground_truth_file_name_ = argv[2];
	ifstream ground_truth_file_(ground_truth_file_name_.c_str(), std::ifstream::in);

	if (!estimations_file_.is_open())
	{
		cerr << "Cannot open estimations file: " << estimations_file_name_ << endl;
		exit(EXIT_FAILURE);
	}

	if (!ground_truth_file_.is_open())
	{
		cerr << "Cannot open ground truth file: " << ground_truth_file_name_ << endl;
		exit(EXIT_FAILURE);
	}

	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	readStateSeq(estimations_file_, estimations);
	readStateSeq(ground_truth_file_, ground_truth);

	// compute the accuracy (RMSE)
	Tools tools;
	VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);
	cout << "Accuracy - RMSE:\n" << rmse << endl;

	// close files
	if (estimations_file_.is_open())
	{
		estimations_file_.close();
	}

	if (ground_truth_file_.is_open())
	{
		ground_truth_file_.close();
	}

	return 0;
}
