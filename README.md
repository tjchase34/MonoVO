# MonoVO
Monocular Visual Odometry (VO) system written in C++.

## To Build:

1. `git clone https://github.com/tjchase34/MonoVO.git`
2. `cd MonoVO/`
3. `mkdir build`
4. `cd build/`
5. `cmake ..`
6. `make`

## To Run:

From the root project directory:
* `./build/MonoVO [INPUT IMAGE FOLDER] [OUTPUT POSE SERIES FILENAME] [NUM FRAMES] [KEYFRAME NUM]`

Example run with KITTI images (sequence 00) at `/mnt/d/Shared/data/images/`, processing the first 500 frames, and selecting every 10th frame as a keyframe:
* `./build/MonoVO /mnt/d/Shared/data/images/ pose_results.txt 500 10`
