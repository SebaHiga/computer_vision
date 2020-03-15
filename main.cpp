#include "opencv2/opencv.hpp"
#include "geometry.hpp"
#include <iostream>

using namespace cv;
using namespace std;

geom::Point searchByColor(cv::InputArray hsv, cv::OutputArray out, Scalar lower, Scalar upper);

int main(int, char**){
    VideoCapture cam(0); // open the default camera

    if(!cam.isOpened())  // check if we succeeded
        return -1;
    
    Mat frame;
    geom::Point dot;

    namedWindow("frame",1);
    while(1)
    {
        Mat frame, hsv, filtered;
        cam >> frame; // get a new frame from camera

        Scalar lower_filter(109, 104, 54), upper_filter(128, 255, 255);

        cvtColor(frame, hsv, COLOR_BGR2HSV);

        dot = searchByColor(hsv, filtered, lower_filter, upper_filter);

        circle(frame, cv::Point(dot.top, dot.left), 5, Scalar(255,0,0), -1);

        imshow("frame", frame);
        if(waitKey(1) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

geom::Point searchByColor(cv::InputArray hsv, cv::OutputArray out, Scalar lower, Scalar upper){
    Mat filtered, kernel, coord, crop;

    cv::inRange(hsv.getMat(), lower, upper, filtered);
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1), cv::Point(0, 0));
    cv::morphologyEx(filtered, filtered, cv::MORPH_CLOSE, kernel);

    cv::Moments m = moments(filtered, true);
    cv::Point p(m.m10/m.m00, m.m01/m.m00);

    cv::Rect roi(p.x - 100, p.y - 100, 200, 200);

    crop = filtered(roi);

    m = moments(crop, true);
    cv::Point dot(m.m10/m.m00, m.m01/m.m00);

    out.assign(filtered);
    return geom::Point(p.x + dot.x - 100, p.y + dot.y - 100);
}