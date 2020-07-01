#include "opencv2/opencv.hpp"
#include <iostream>


#define MAX_FEATURES = 1000;
#define MAX_GOOD = 0.15f;


int main(int argc, char** argv) {

    // Check arguments, get datapath
    if (argc != 2) {
        std::cout << "Invalid args specified." << std::endl;
        return -1;
    }

    std::string dataPath = argv[1];

    // Get all frame paths for the KITTI dataset
    std::vector<cv::String> frame_paths;
    glob(dataPath + "/*.png", frame_paths, false);

    int numFrames = frame_paths.size();

    // Check if all the image paths were found successfully
    if (!numFrames) {
        std::cout << "Error grabbing frame paths." << std::endl;
        return -1;
    }

    // Create ORB
    cv::Ptr<cv::feature2d> orb = cv::ORB::create(MAX_FEATURES);

    // Global rotation, translation trajectory (we build this up at every step)
    cv::Mat R_jec, t_jec;
    std::vector<std::pair<cv::Mat, cv::Mat>> trajectory;

    // First run flag
    bool first = true;

    // Loop through frames
    for (int i=0; i<numFrames-1; i++) {

        // Read frame i and i+1
        cv::Mat frame1, frame2;
        frame1 = cv::imread(frame_paths[i]);
        frame2 = cv::imread(frame_paths[i+1]);

        // Convert images to grayscale
        cv::cvtColor(frame1, frame1, cv::CV_BGR2GRAY);
        cv::cvtColor(frame2, frame2, cv::CV_BGR2GRAY);

        // Keypoint/descriptor stores 
        std::vector<cv::keypoint> kp1, kp2;
        cv::Mat desc1, desc2;

        // Detect features in i and i+1
        orb->detectAndCompute(frame_1, Mat(), kp1, desc1);
        orb->detectAndCompute(frame_2, Mat(), kp2, desc2);

        // Match them (brute force)
        cv::BFMatcher bf = cv::BFMatcher(cv::NORM_HAMMING, true);
        std::vector<cv::DMatch> matches;
        bf.match(desc1, desc2,  matches);

        // sort matches
        std::sort(matches.begin(), matches.end(), [](cv::DMatch a, cv::DMatch b) {return a.distance > b.distance});

        // Grab pts1/pts2
        vector<cv::Point2f> pts1, pts2;
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

        // Compute global trajectory
        if (fist) {
            R_jec = R.clone();
            t_jec = t.clone();
            first = false;
        } else {
            R_jec = R*R_jec;
            t_jec = t_jec + (R_jec*t);
        }

        // Append to the global trajectory for plotting afterward
        trajectory.push_back(std::make_pair(R_jec, t_jec));

    }

    // Write out global trajectory

    return 0;
}