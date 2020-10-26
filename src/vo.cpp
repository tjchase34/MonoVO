#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>

// GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>

using namespace std;
using namespace gtsam;

#define MAX_FEATURES 1000
#define MAX_GOOD 0.55f
#define KEYFRAME 5

int main(int argc, char** argv) {

    // Check arguments, get datapath
    if (argc != 3) {
        std::cout << "Invalid args specified." << std::endl;
        return -1;
    }

    std::string dataPath = argv[1];
    std::string outputPath = argv[2];

    // Get all frame paths for the KITTI dataset
    std::vector<cv::String> frame_paths;
    cv::glob(dataPath + "/*.png", frame_paths, false);

    int numFrames = frame_paths.size();
    // int numFrames = 1000;

    // Check if all the image paths were found successfully
    if (!numFrames) {
        std::cout << "Error grabbing frame paths." << std::endl;
        return -1;
    }

    std::cout << "Computing trajectories from " << numFrames << " frames..." << std::endl;
    clock_t begin = clock();

    // Create ORB
    cv::Ptr<cv::Feature2D> orb = cv::ORB::create(MAX_FEATURES);

    // Global rotation, translation trajectory (we build this up at every step)
    cv::Mat R_jec, t_jec;
    std::vector<cv::Mat> trajectory;

    // First run flag
    bool first = true;

    double scale = 1.00;

    // Make Graph
    NonlinearFactorGraph graph;

    // Add a prior on the first pose, setting it to the origin
    // The prior is needed to fix/align the whole trajectory at world frame
    // A prior factor consists of a mean value and a noise model (covariance matrix)
    // auto priorNoise = noiseModel::Diagonal::Sigmas(
    //     (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 0.1));

    // graph.addPrior(0, Pose3(), priorNoise);
    graph.addPrior(0, Pose2(0,0,0), priorNoise);

    // Specify uncertainty on first pose prior and also for between factor (simplicity reasons)
    // auto odomNoise = noiseModel::Diagonal::Sigmas((Vector(6)<<0.3,0.3,0.3,0.1,0.1,0.1).finished());
    auto odomNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

    Values initial;
    initial.insert(0, Pose2(0,0,0));

    // Loop through frames
    int node = 0;
    cv::Mat last_frame;
    for (int i=0; i<numFrames-1; i+=KEYFRAME) {

        if (i == 0) {
          last_frame = cv::imread(frame_paths[i]);
          cv::cvtColor(last_frame, last_frame, cv::COLOR_BGR2GRAY);
        }

        if (i%100 == 0 && i != 0) {
            clock_t now = clock();
            double elapsed_secs = double(now - begin) / CLOCKS_PER_SEC;
            std::cout << "Processed " << i << " frames... (" << elapsed_secs << "s)" << std::endl;
        }

        // Read frame i
        cv::Mat curr_frame;
        curr_frame = cv::imread(frame_paths[i]);

        // Convert image to grayscale
        cv::cvtColor(curr_frame, curr_frame, cv::COLOR_BGR2GRAY);

        // Keypoint/descriptor stores 
        std::vector<cv::KeyPoint> kp1, kp2;
        cv::Mat desc1, desc2;

        // Detect features in i and i+1
        orb->detectAndCompute(last_frame, cv::Mat(), kp1, desc1);
        orb->detectAndCompute(curr_frame, cv::Mat(), kp2, desc2);

        // Match them (brute force)
        cv::BFMatcher bf = cv::BFMatcher(cv::NORM_HAMMING, true);
        std::vector<cv::DMatch> matches;
        bf.match(desc1, desc2,  matches);

        // MATCH VIEWING
        // std::vector<cv::DMatch> some_matches = std::vector<cv::DMatch>(matches.begin(), matches.begin()+20);
        // cv::Mat frame_matches;
        // cv::drawMatches(frame1, kp1, frame2, kp2, some_matches, frame_matches, cv::Scalar::all(-1), cv::Scalar::all(-1));
        // cv::imshow("Matches",frame_matches);
        // cv::waitKey(0);

        // sort matches
        std::sort(matches.begin(), matches.end(), [](cv::DMatch a, cv::DMatch b) {return a.distance > b.distance;});

        // Grab pts1/pts2
        std::vector<cv::Point2f> pts1, pts2;
        for (int i=0; i<matches.size() * MAX_GOOD; i++) {
            pts1.push_back(kp1[matches[i].queryIdx].pt);
            pts2.push_back(kp2[matches[i].trainIdx].pt);
        }

        //TODO: add a fucntion to load these values directly from KITTI's calib files
        // WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
        double focal = 718.8560;
        cv::Point2d pp(607.1928, 185.2157);

        // Grab pose (R, t) from essential
        cv::Mat E, R, t, mask;
        E = cv::findEssentialMat(pts2, pts1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
        cv::recoverPose(E, pts2, pts1, R, t, focal, pp, mask);

        // Add odometry factors
        // Create odometry (Between) factors between consecutive poses
        // Rot3 rot = Rot3(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
        //                 R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
        //                 R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));

        // Point3 p = Point3(t.at<double>(0), t.at<double>(1), t.at<double>(2));
        Point2 p = Point2(t.at<double>(0), t.at<double>(1));

        // Get angle of rotation in radians
        double theta = atan2(R.at<double>(2,1), R.at<double>(1,1));
        // Convert radians to degrees
        // theta = theta*(180.0/3.141592653589793238463);

        graph.add(BetweenFactor<Pose2>(node, node+1, Pose2(theta,p), odomNoise));
        // initial.insert(node+1, Pose3(rot, p));
        initial.insert(node+1, Pose2(theta, p));
        ++node;
        last_frame = curr_frame;
    }

    graph.print("\nFactor Graph:\n");

    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("Final Result:\n");

    std::ofstream file;
    file.open(outputPath, std::ofstream::trunc);
    graph.saveGraph(file);
    file.close();

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    
    std::cout << "Finished! Total time taken: " << elapsed_secs << "s" << std::endl;

    return 0;
}