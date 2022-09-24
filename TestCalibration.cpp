// TestCalibration.cpp : This file contains the 'main' function. Program execution begins and ends there.
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv) {
    //this is my test code
   
    // The standard code
    (void)argc;
    (void)argv;

    std::vector<cv::String> fileNames;
    // The folder directory for the camera's image
    cv::glob("..FolderPath..\\Image*.png", fileNames, false);

    // The size of the pattern of the checkerboard
    cv::Size patternSize(25 - 1, 18 - 1);

    // vector for the checkerboard corners in each image.
    std::vector<std::vector<cv::Point2f>> q(fileNames.size());

    // Generate checkerboard (world) coordinates Q. 
    // The world coordinate for our checkerboard Q
    std::vector<std::vector<cv::Point3f>> Q;
    // The board has 25 x 18
    int checkerBoard[2] = { 25,18 };
    // fields with a size of 15x15mm
    int fieldSize = 15;

    // Defining the world coordinates for 3D points (x,y,z)
    std::vector<cv::Point3f> objP;

    for (int i = 1; i< checkerBoard[1]; i++)
    {
        for (int j = 1; j < checkerBoard[0]; j++)
        {
            objP.push_back(cv::Point3f(j * fieldSize, i * fieldSize, 0));
        }
    }

    std::vector<cv::Point2f> imgPoint;
    // Detect feature points
    std::size_t i = 0;
    for (auto const& f : fileNames) {
        std::cout << std::string(f) << std::endl;

        cv::Mat img = cv::imread(fileNames[i]);
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        // Read in the image an call cv::findChessboardCorners().
        //Find the corners of the checker board markers cv::findChessboardCorners(). 
        bool patternFound= cv::findChessboardCorners(gray, patternSize, q[i], 
            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
            + cv::CALIB_CB_FAST_CHECK);
        
        // Use cv::cornerSubPix() to refine and improve the accuracy of the detected corners
        if (patternFound)
        {
            
           // here we use alot of defualt values from the third parameter.. 
          cv::cornerSubPix(gray, q[i], cv::Size(11, 11), cv::Size(-1, -1), 
              cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            Q.push_back(objP);
        }

        // Display
        cv::drawChessboardCorners(img, patternSize, q[i], patternFound);
        cv::imshow("chessboard detection", img);
        cv::waitKey(0);

        i++;
    }


    cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
    cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

    // rotation matrix and translation matrix. 
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
        cv::CALIB_FIX_PRINCIPAL_POINT + cv::CALIB_ZERO_TANGENT_DIST;

    cv::Size frameSize(1440, 1080);

    std::cout << "Calibrating..." << std::endl;
    // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
    // and output parameters as declared above...

    float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);
    std::cout << "Reprojection error = " << error << "\nK =\n"
        << K << "\nk=\n"
        << k << std::endl;

    // Precompute lens correction interpolation
    cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);

    // Show lens corrected images
    for (auto const& f : fileNames) {
        std::cout << std::string(f) << std::endl;

        cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);

        cv::Mat imgUndistorted;
        // 5. Remap the image using the precomputed interpolation maps.
        cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
        // Display
        cv::imshow("undistorted image", imgUndistorted);
        cv::waitKey(0);
    }

    return 0;
}
