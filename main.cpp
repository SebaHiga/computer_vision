#include "opencv2/opencv.hpp"
#include <iostream>
#include "tracker.hpp"
#include "geometry.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <string>

using namespace cv;
using namespace std;

geom::Point searchByColor(cv::InputArray hsv, cv::OutputArray out, Scalar lower, Scalar upper);

int main(int, char**){
    VideoCapture cam(0); // open the default camera

    if(!cam.isOpened())  // check if we succeeded
        return -1;
    
    Mat frame;
    geom::Point p;
    Tracker track;

    namedWindow("frame",1);
    while(1)
    {
        Mat frame, hsv, filtered;
        cam >> frame; // get a new frame from camera

        Scalar lower_filter(109, 104, 54), upper_filter(128, 255, 255);

        cvtColor(frame, hsv, COLOR_BGR2HSV);

        p = searchByColor(hsv, filtered, lower_filter, upper_filter);

        vector<geom::Point> points;

        if(p.originDistance() > 100){
            points.push_back(geom::Point(p));
        }

        track.update(points);

        for (int i = 0; i < track.count; i++){
            stringstream ss;
            ss << track.objects[i].id;
            string str;
            str = ss.str();

            putText(frame, str,
                    cv::Point(
                    track.objects[i].position.top, track.objects[i].position.left),
                    2, 1, Scalar(255, 0, 0), 2);

            cv::Point speedLine(track.objects[i].getSpeedPoint().cv_getPoint());

            line(frame, track.objects[i].position.cv_getPoint(),
                     speedLine, Scalar(0, 0, 255), 4);
        }

        imshow("frame", frame);
        if(waitKey(1) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

geom::Point searchByColor(cv::InputArray hsv, cv::OutputArray out, Scalar lower, Scalar upper){
    Mat filtered, kernel, coord, crop;
    int const margin = 50;

    cv::inRange(hsv.getMat(), lower, upper, filtered);
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1), cv::Point(0, 0));
    cv::morphologyEx(filtered, filtered, cv::MORPH_CLOSE, kernel);

    cv::Moments m = moments(filtered, true);
    cv::Point p(m.m10/m.m00, m.m01/m.m00);


    out.assign(filtered);
    // return geom::Point(p.x, p.y);

    cv::Rect roi(p.x - (margin/2), p.y - (margin/2), margin, margin);
    crop = filtered(roi);

    cv::Mat nonzero;
    findNonZero(crop, nonzero);

    if(nonzero.total() < 150){
        return geom::Point(0, 0);
    }

    m = moments(crop, true);
    cv::Point dot(m.m10/m.m00, m.m01/m.m00);

    return geom::Point(p.x + dot.x - (margin/2), p.y + dot.y - (margin/2));
}