#include <iostream>
#include <fstream>

// GTSAM
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/utilities.h>

using namespace gtsam;
using namespace std;

int main(int argc, char** argv) {

    NonlinearFactorGraph graph;

    noiseModel::Diagonal::shared_ptr priorNoise =
        noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));

    graph.add(PriorFactor<Pose2>(1, Pose2(0,0,0), priorNoise));

    // Add odometry factors
    noiseModel::Diagonal::shared_ptr model =
        noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

    graph.add(BetweenFactor<Pose2>(1, 2, Pose2(2, 0, 0 ), model));
    graph.add(BetweenFactor<Pose2>(2, 3, Pose2(2, 0, M_PI_2), model));
    graph.add(BetweenFactor<Pose2>(3, 4, Pose2(2, 0, M_PI_2), model));
    graph.add(BetweenFactor<Pose2>(4, 5, Pose2(2, 0, M_PI_2), model));

    // Add pose constraint
    graph.add(BetweenFactor<Pose2>(5, 2, Pose2(2, 0, M_PI_2), model));

    graph.print("\nFactor Graph:\n");

    Values initialEstimate;
    initialEstimate.insert(1, Pose2(0, 0, 0));
    initialEstimate.insert(2, Pose2(2, 0, 0));
    initialEstimate.insert(3, Pose2(2, 0, M_PI_2));
    initialEstimate.insert(4, Pose2(2, 0, M_PI_2));
    initialEstimate.insert(5, Pose2(2, 0, M_PI_2));
    initialEstimate.insert(6, Pose2(2, 0, M_PI_2));
    initialEstimate.print("\nInitial Estimate:\n");

    Matrix test = gtsam::utilities::extractPose2(initialEstimate);
    std::ofstream traj_file;
    traj_file.open("fg_test_traj.txt", std::ofstream::trunc);
    // traj_file << test;
    // traj_file << test.format(CSVFormat);
    
    for (int i=0; i<test.rows(); ++i) {
        for (int j=0; j<test.cols(); ++j) {
            traj_file << test(i,j);
            if (j != test.cols()-1) {
                traj_file << " ";
            }
        }
        traj_file << "\n";
    }
    traj_file.close();
    
    std::ofstream file;
    file.open("fg_test.graph", std::ofstream::trunc);
    graph.saveGraph(file);
    file.close();
}