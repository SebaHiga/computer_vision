#include "opencv2/opencv.hpp"
#include "geometry.hpp"
#include <iostream>

using namespace cv;
using namespace std;

void searchByColor(cv::InputArray hsv, cv::OutputArray out, Scalar lower, Scalar upper);

int main(int, char**){
    VideoCapture cam(0); // open the default camera

    if(!cam.isOpened())  // check if we succeeded
        return -1;
    Mat frame;

    namedWindow("frame",1);
    while(1)
    {
        Mat frame, hsv, filtered;
        cam >> frame; // get a new frame from camera

        Scalar lower_filter(99, 136, 49), upper_filter(153, 255, 255);

        cvtColor(frame, hsv, COLOR_BGR2HSV);

        searchByColor(frame, filtered, lower_filter, upper_filter);
        // inRange(hsv, lower_filter, upper_filter, filtered);

        //flipping image
        flip(filtered, filtered, 1);

        imshow("frame", filtered);
        if(waitKey(1) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

void searchByColor(cv::InputArray hsv, cv::OutputArray out, Scalar lower, Scalar upper){
    Mat filtered, kernel, coord;

    cv::inRange(hsv.getMat(), lower, upper, filtered);
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
    cv::morphologyEx(filtered, filtered, cv::MORPH_CLOSE, kernel);

    cv::Moments m = moments(filtered, true);
    cv::Point p(m.m10/m.m00, m.m01/m.m00);

    out.assign(filtered);
}