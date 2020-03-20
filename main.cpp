#include "opencv2/opencv.hpp"
#include <iostream>
#include "tracker.hpp"
#include "geometry.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>

using namespace cv;
using json = nlohmann::json;

vector<geom::Point> parseJSON(string str);

int main(int, char**){
    VideoCapture cam("../videos/prueba.mp4"); // open the default camera

    if(!cam.isOpened())  // check if we succeeded
        return -1;
    
    Mat frame;
    geom::Point p;
    Tracker track;

    namedWindow("frame", cv::WINDOW_AUTOSIZE);

    std::ifstream file("../records/prueba.txt");

    std::string line;

    while(getline(file, line))
    {
        Mat frame;
        cam >> frame; // get a new frame from camera

        vector<geom::Point> points;

        // add function to parse json and add to points vector
        points = parseJSON(line);

        track.update(&points);

        for (int i = 0; i < track.objects.size(); i++){
            if(track.objects[i].valid){
                stringstream ss;
                ss << track.objects[i].id;
                string str;
                str = ss.str();

                cv::Point speedLine(track.objects[i].getSpeedPoint().cv_getPoint());

                // cv::line(frame, track.objects[i].position.cv_getPoint(),
                //         speedLine, Scalar(0, 0, 255), 4);

                cv::circle(frame, track.objects[i].position.cv_getPoint(),
                        MAX_RANGE, Scalar(0, 255, 255), 2, 4);

                putText(frame, str,
                        cv::Point(
                        track.objects[i].position.top, track.objects[i].position.left),
                        2, 1, Scalar(255, 255, 0), 2);

                // for(int k = 0; k < track.objects[i].trackline.size(); k++){
                //     cv::circle(frame, track.objects[i].trackline[k].cv_getPoint(),
                //         1, Scalar(255, 0, 255), 2, 4);
                // }
                for(int k = 1; k < track.objects[i].trackline.size() - 1; k++){
                    cv::line(frame, track.objects[i].trackline[k-1].cv_getPoint(),
                        track.objects[i].trackline[k].cv_getPoint(), Scalar(255, 0, 255), 2, 4);
                }
            }
        }

        imshow("frame", frame);
        if(waitKey(1) == 27) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

vector<geom::Point> parseJSON(string str){
    json j = json::parse(str);
    vector<geom::Point> points;

    for (json::iterator it = j.begin(); it != j.end(); ++it) {
        if(it.value()["class_id"] == 0){
            int top, left, width, height;

            top = it.value()["box"]["top"];
            left = it.value()["box"]["left"];
            height = it.value()["box"]["width"];
            width = it.value()["box"]["height"];

            int center_top, center_left;

            center_top = (top + height/2);
            center_left = (left + width/2);

            points.push_back(geom::Point(center_left, center_top));
        }
    }

    return points;
}