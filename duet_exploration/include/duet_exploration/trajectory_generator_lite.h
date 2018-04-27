#ifndef _TRAJECTORY_GENERATOR_H_
#define _TRAJECTORY_GENERATOR_H_

#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include "duet_exploration/mosek.h"
#include "duet_exploration/bezier_base.h"
#include "duet_exploration/dataType.h"

using namespace std;
using namespace Eigen;

class TrajectoryGenerator {
private:

public:
        MatrixXd _Path;
        VectorXd _Radius;
        VectorXd _Time;
        VectorXd _Scale;

        double initScale, lstScale;

        TrajectoryGenerator();

        ~TrajectoryGenerator();

        /* Use Bezier curve for the trajectory */
        MatrixXd BezierPloyCoeffGeneration(
            const vector<Cube> &corridor,
            const MatrixXd &MQM,
            const VectorXd &C,
            const VectorXd &Cv,
            const VectorXd &Ca,
            const MatrixXd &pos,
            const MatrixXd &vel,
            const MatrixXd &acc,
            const double maxVel,
            const double maxAcc,
            const int traj_order,
            const int minimize_order,
            double & obj, 
            const double margin );  // define the order to which we minimize.   1 -- velocity, 2 -- acceleration, 3 -- jerk, 4 -- snap  
        
         MatrixXd BezierPloyCoeffGeneration2D(
            const vector<Cube2D> &corridor,
            const MatrixXd &MQM,
            const VectorXd &C,
            const VectorXd &Cv,
            const VectorXd &Ca,
            const MatrixXd &pos,
            const MatrixXd &vel,
            const MatrixXd &acc,
            const double maxVel,
            const double maxAcc,
            const int traj_order,
            const int minimize_order,
            double & obj, 
            const double margin );
};

#endif