# TestCalibration
This code is used to calibrate the camera using OpenCv algorithme. cv::calibrateCamera() provides us with the (3  3) intrinsic camera matrix as well as the (1  5) distortion coefcients. After that we use cv::initUndistortRectifyMap(...), which uses the camera matrix and distortion coefcients from the camera calibration.
