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
#include <gtsam/nonlinear/utilities.h>


using namespace std;
using namespace gtsam;


#define MAX_FEATURES 2500


double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)	{
    string line;
    int i = 0;
    ifstream myfile ("ground_truth_00.txt");
    double x =0, y=0, z = 0;
    double x_prev, y_prev, z_prev;
    if (myfile.is_open())
    {
        while (( getline (myfile,line) ) && (i<=frame_id))
        {
        z_prev = z;
        x_prev = x;
        y_prev = y;
        std::istringstream in(line);
        for (int j=0; j<12; j++)  {
            in >> z ;
            if (j==7) y=z;
            if (j==3)  x=z;
        }

        i++;
        }
        myfile.close();
    }

    else {
        cout << "Unable to open file";
        return 0;
    }

    return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

}

vector<cv::Mat> loadGroundTruth(string filename) {
    vector<cv::Mat> gt;
    ifstream file(filename);
    std::string line;

    while (getline(file, line)) {
        cv::Mat temp = cv::Mat::zeros(3, 4, CV_64FC1);
        // cout << line << endl;
        sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
            &temp.at<double>(0,0), &temp.at<double>(0,1), &temp.at<double>(0,2), &temp.at<double>(0,3),
            &temp.at<double>(1,0), &temp.at<double>(1,1), &temp.at<double>(1,2), &temp.at<double>(1,3),
            &temp.at<double>(2,0), &temp.at<double>(2,1), &temp.at<double>(2,2), &temp.at<double>(2,3));

        gt.push_back(temp);
    }

    return gt;
}


int main(int argc, char** argv) {

    // Check arguments, get datapath
    if (argc != 4) {
        std::cout << "Invalid args specified." << std::endl;
        std::cout << "Usage is: ./build/MonoVO [INPUT IMAGE FOLDER] [NUM FRAMES] [KEYFRAME NUM]" << std::endl;
        return -1;
    }

    std::string dataPath = argv[1];
    int numFrames = atoi(argv[2]);
    int keyFrames = atoi(argv[3]);

    // Get all frame paths for the KITTI dataset
    std::vector<cv::String> frame_paths;
    cv::glob(dataPath + "/*.png", frame_paths, false);

    // Check if all the image paths were found successfully
    if (!numFrames) {
        std::cout << "Error grabbing frame paths." << std::endl;
        return -1;
    }

    vector<cv::Mat> groundTruth = loadGroundTruth("ground_truth_00.txt");

    std::cout << "Computing trajectories from " << numFrames << " frames..." << std::endl;
    clock_t begin = clock();

    // Create ORB
    cv::Ptr<cv::ORB> detector = cv::ORB::create(MAX_FEATURES, 1.4f, 10, 35, 0, 2, cv::ORB::HARRIS_SCORE, 35, 20);

    cv::Mat curr_frame;
    cv::Mat last_frame;

    // TODO: Read these from KITTI calibration files
    double focal = 718.8560;
    cv::Point2d pp(607.1928, 185.2157);

    // Running trajectory
	cv::Mat traj_t, traj_R;
    // Trajectory output
	cv::Mat traj = cv::Mat::zeros(750, 750, CV_8UC3);

    // Factor graph
    NonlinearFactorGraph graph;

    // Add a prior on the first pose, setting it to the origin
    // The prior is needed to fix/align the whole trajectory at world frame
    // A prior factor consists of a mean value and a noise model (covariance matrix)
    auto priorNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    // auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 0.1));

    graph.addPrior(0, Pose3(), priorNoise);

    // Specify uncertainty on first pose prior and also for between factor (simplicity reasons)
    auto odomNoise = noiseModel::Diagonal::Sigmas((Vector(6)<<0.3,0.3,0.3,0.1,0.1,0.1).finished());
    // auto odomNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

    Values initial;
    initial.insert(0, Pose3());

    // Loop through frames
    int node = 0;
    for (int i=0; i<numFrames-1; i+=keyFrames) {

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
        curr_frame = cv::imread(frame_paths[i]);

        // Convert image to grayscale
        cv::cvtColor(curr_frame, curr_frame, cv::COLOR_BGR2GRAY);

        // Keypoint/descriptor stores 
        std::vector<cv::KeyPoint> kp1, kp2;
        cv::Mat desc1, desc2;

        // Detect features in i and i+1
        detector->detectAndCompute(last_frame, cv::Mat(), kp1, desc1);
        detector->detectAndCompute(curr_frame, cv::Mat(), kp2, desc2);

        // Match them (brute force)
        cv::BFMatcher bf = cv::BFMatcher(cv::NORM_HAMMING2, true);
        std::vector<cv::DMatch> matches;
        bf.match(desc1, desc2,  matches);
        sort(matches.begin(), matches.end());

        // sort matches
        // std::sort(matches.begin(), matches.end(), [](cv::DMatch a, cv::DMatch b) {return a.distance > b.distance;});

        // MATCH VIEWING
        // std::vector<cv::DMatch> some_matches = std::vector<cv::DMatch>(matches.begin(), matches.begin()+100);
        // cv::Mat frame_matches;
        // cv::drawMatches(last_frame, kp1, curr_frame, kp2, some_matches, frame_matches, cv::Scalar::all(-1), cv::Scalar::all(-1));
        // cv::imshow("Matches",frame_matches);
        // cv::waitKey(0);

        // Grab pts1/pts2
        std::vector<cv::Point2f> pts1, pts2;
        for (int i=0; i<matches.size(); i++) {
            pts1.push_back(kp1[matches[i].queryIdx].pt);
            pts2.push_back(kp2[matches[i].trainIdx].pt);
        }

        // Grab pose (R, t) from essential
        cv::Mat E, R, t, mask;
        E = cv::findEssentialMat(pts2, pts1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
        int inliers = cv::recoverPose(E, pts2, pts1, R, t, focal, pp, mask);

        if (traj_t.empty() && traj_R.empty()) {
            traj_t = t.clone();
            traj_R = R.clone();
        }

        double scale = getAbsoluteScale(i, 0, t.at<double>(2));

        // Bad translation estimate (we're moving backwards or something), skip
        // if ((scale<0.1) && (-t.at<double>(2) > t.at<double>(0)) && (-t.at<double>(2) > t.at<double>(1))) {
        //     continue;
        // }

        if ((scale<0.1) && (-t.at<double>(2) > t.at<double>(0)) || (-t.at<double>(2) > t.at<double>(1))) {
            continue;
        }

        traj_t = traj_t + scale*(traj_R*t);
        traj_R = R*traj_R;

        // Add odometry factors
        // Create odometry (Between) factors between consecutive poses
        Rot3 rot = Rot3(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));

        Point3 p = Point3(t.at<double>(0), t.at<double>(1), -t.at<double>(2));
        // Point2 p = Point2(t.at<double>(0), -t.at<double>(2));

        // Get angle of rotation in radians
        // double theta = atan2(R.at<double>(2,1), R.at<double>(1,1));
        // Convert radians to degrees
        // theta = theta*(180.0/3.141592653589793238463);

        graph.add(BetweenFactor<Pose3>(node, node+1, Pose3(rot, p), odomNoise));
        initial.insert(node+1, Pose3(rot, p));
        ++node;
        last_frame = curr_frame;

        // VIEWER
        cv::Mat gt = groundTruth[i];
        int x_truth = int(gt.at<double>(0,3));
        int z_truth = -int(gt.at<double>(2,3));

        int x = int(traj_t.at<double>(0));
        int z = -int(traj_t.at<double>(2));

        // Offsets to move trajectory towards the middle of viewer
        int x_offset = 375;
        int z_offset = 600;

        // Our calculated traj (red)
        cv::circle(traj, cv::Point(x+x_offset,z+z_offset), 1, CV_RGB(255,0,0), 2);
        // Ground truth (green)
        cv::circle(traj, cv::Point(x_truth+x_offset,z_truth+z_offset), 1, CV_RGB(0,255,0), 2);
        cv::imshow("Road facing camera", curr_frame);
        cv::imshow("Trajectory", traj );
        cv::waitKey(1);
    }

    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

    // graph.print("\nFactor Graph:\n");
    // result.print("Final Result:\n");

    // Write graph to file
    // std::ofstream file;
    // file.open(outputPath, std::ofstream::trunc);
    // graph.saveGraph(file);
    // file.close();

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Finished! Total time taken: " << elapsed_secs << "s" << std::endl;

    // Recover initial poses and write to file
    Matrix poses = gtsam::utilities::extractPose3(initial);
    std::ofstream poses_file;
    std::string outFile = "results/" + to_string(numFrames) + "_" + to_string(keyFrames) + "_poses_initial.txt";
    poses_file.open(outFile, std::ofstream::trunc);
    
    for (int i=0; i<poses.rows(); ++i) {
        for (int j=0; j<poses.cols(); ++j) {
            poses_file << poses(i,j);
            if (j != poses.cols()-1) {
                poses_file << " ";
            }
        }
        poses_file << "\n";
    }
    poses_file.close();

    // Recover optimized poses and write to file
    poses = gtsam::utilities::extractPose3(result);
    outFile = "results/" + to_string(numFrames) + "_" + to_string(keyFrames) + "_poses_optimized.txt";
    poses_file.open(outFile, std::ofstream::trunc);
    
    for (int i=0; i<poses.rows(); ++i) {
        for (int j=0; j<poses.cols(); ++j) {
            poses_file << poses(i,j);
            if (j != poses.cols()-1) {
                poses_file << " ";
            }
        }
        poses_file << "\n";
    }
    poses_file.close();

    // Write trajectory image
    std::string traj_filename = "results/" + to_string(numFrames) + "_" + to_string(keyFrames) + "_traj.png";
    cv::imwrite(traj_filename, traj);

    return 0;
}