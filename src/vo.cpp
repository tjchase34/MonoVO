#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>

// GTSAM
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>


#define MAX_FEATURES 1000
#define MAX_GOOD 0.15f


double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)	{
  
  std::string line;
  int i = 0;
  std::ifstream myfile ("../ground_truth_00.txt");
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
      //cout << line << '\n';
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
    std::cout << "Unable to open file";
    return 0;
  }

  return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

}

void featureTracking(cv::Mat img_1, cv::Mat img_2, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2, std::vector<uchar>& status)	{ 

//this function automatically gets rid of points for which tracking fails

  std::vector<float> err;					
  cv::Size winSize = cv::Size(21,21);																								
  cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

  cv::calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

  //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  int indexCorrection = 0;
  for( int i=0; i<status.size(); i++)
     {  cv::Point2f pt = points2.at(i- indexCorrection);
     	if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
     		  if((pt.x<0)||(pt.y<0))	{
     		  	status.at(i) = 0;
     		  }
     		  points1.erase (points1.begin() + i - indexCorrection);
     		  points2.erase (points2.begin() + i - indexCorrection);
     		  indexCorrection++;
     	}

     }
}


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
    glob(dataPath + "/*.png", frame_paths, false);

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

    // Loop through frames
    for (int i=0; i<numFrames-1; i++) {

        if (i%100 == 0 && i != 0) {
            clock_t now = clock();
            double elapsed_secs = double(now - begin) / CLOCKS_PER_SEC;
            std::cout << "Processed " << i << " frames... (" << elapsed_secs << "s)" << std::endl;
        }

        // Read frame i and i+1
        cv::Mat frame1, frame2;
        frame1 = cv::imread(frame_paths[i]);
        frame2 = cv::imread(frame_paths[i+1]);

        // Convert images to grayscale
        cv::cvtColor(frame1, frame1, CV_BGR2GRAY);
        cv::cvtColor(frame2, frame2, CV_BGR2GRAY);

        // Keypoint/descriptor stores 
        std::vector<cv::KeyPoint> kp1, kp2;
        cv::Mat desc1, desc2;

        // Detect features in i and i+1
        orb->detectAndCompute(frame1, cv::Mat(), kp1, desc1);
        orb->detectAndCompute(frame2, cv::Mat(), kp2, desc2);

        // Match them (brute force)
        cv::BFMatcher bf = cv::BFMatcher(cv::NORM_HAMMING, true);
        std::vector<cv::DMatch> matches;
        bf.match(desc1, desc2,  matches);

        // sort matches
        std::sort(matches.begin(), matches.end(), [](cv::DMatch a, cv::DMatch b) {return a.distance > b.distance;});

        // Grab pts1/pts2
        std::vector<cv::Point2f> pts1, pts2;
        for (int i=0; i<matches.size() * MAX_GOOD; i++) {
            pts1.push_back(kp1[matches[i].queryIdx].pt);
            pts2.push_back(kp2[matches[i].trainIdx].pt);
        }

        // Optical flow
        // cv::Mat status;
        // featureTracking(frame1, frame2, kp1, kp2, status);

        //TODO: add a fucntion to load these values directly from KITTI's calib files
        // WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
        double focal = 718.8560;
        cv::Point2d pp(607.1928, 185.2157);

        // Grab pose (R, t) from essential
        cv::Mat E, R, t, mask;
        E = cv::findEssentialMat(pts2, pts1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
        cv::recoverPose(E, pts2, pts1, R, t, focal, pp, mask);

        // scale = getAbsoluteScale(i, 0, t.at<double>(2));

        // Compute global trajectory
        // if (first) {
        //     R_jec = R.clone();
        //     t_jec = t.clone();
        //     first = false;
        // } else {
        //     if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
        //       R_jec = R*R_jec;
        //       t_jec = t_jec + scale*(R_jec*t);
        //     }
        // }

        // Append to the global trajectory for plotting afterward
        cv::Mat concat;
        cv::hconcat(R_jec, t_jec, concat);
        trajectory.push_back(concat);


        //TODO: Add pose3(R, t) to factor graph






        // std::cout << concat << std::endl; 

        // cvNamedWindow("1", CV_WINDOW_AUTOSIZE);
        // cvNamedWindow("2", CV_WINDOW_AUTOSIZE);
        // imshow("1", frame1);
        // imshow("2", frame2);

        // cv::waitKey(0);

    }

    std::ofstream file;
    file.open(outputPath, std::ofstream::trunc);

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    
    std::cout << "Finished! Total time taken: " << elapsed_secs << "s" << std::endl;
    std::cout << "Writing trajectories (" << trajectory.size() << ")..." << std::endl; 

    // Write out trajectories
    for (int i=0; i<trajectory.size(); i++) {

        cv::Mat curTraj = trajectory[i];

        // Loop over (j,k)
        for (int j=0; j<3; j++) {
            for (int k=0; k<4; k++) {
                // file << std::fixed << std::setprecision(6) << curTraj.at<double>(j,k);
                file << curTraj.at<double>(j,k);
                if (j != 2 || k != 3) {
                    file << " ";
                }
            }
        }

        file << std::endl;
    }

    file.close();

    return 0;
}