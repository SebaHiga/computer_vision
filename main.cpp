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

vector<Object> parseJSON(string str);
geom::Point searchByColor(cv::InputArray hsv, cv::OutputArray out, Scalar lower, Scalar upper);
void getPointsFromObject(int class_id, vector<geom::Point> *p, vector<Object> *obj);

int main(int argc, char** argv){
    if(argc < 1){
        printf("Please add a name file to interact.\n");
        return 0;
    }

    string path_video = "../videos/" + string(argv[1]) + ".mp4";
    string path_records = "../records/" + string(argv[1]) + ".txt";

    VideoCapture cam(path_video); // open the default camera

    if(!cam.isOpened())  // check if we succeeded
        return -1;
    
    Mat frame;
    geom::Point p;
    Tracker track;

    namedWindow("frame", cv::WINDOW_AUTOSIZE);

    std::ifstream file(path_records);

    std::string line;

    while(getline(file, line))
    {
        Mat frame;
        cam >> frame; // get a new frame from camera

        vector<geom::Point> points;
        vector<Object> obj;

        // add function to parse json and add to points vector
        obj = parseJSON(line);

        // getPointsFromObject(0, &points, &obj);

        // track.update(&points);
        track.update(&obj);


        for (int i = 0; i < track.objects.size(); i++){
            if(track.objects[i].valid){
                stringstream ss;
                ss << track.objects[i].id;
                string str;
                str = ss.str();

                // cv::Point speedLine(track.objects[i].getSpeedPoint().cv_getPoint());

                // cv::line(frame, track.objects[i].position.cv_getPoint(),
                //         speedLine, Scalar(0, 0, 255), 4);

                // cv::circle(frame, track.objects[i].position.cv_getPoint(),
                //         MAX_RANGE, Scalar(0, 0, 0), 2, 8);

                putText(frame, str,
                        cv::Point(
                        track.objects[i].position.top, track.objects[i].position.left),
                        1, 1, Scalar(255, 255, 0), 2);

                for(int k = 1; k < track.objects[i].trackline.size() - 1; k++){
                    cv::line(frame, track.objects[i].trackline[k-1].cv_getPoint(),
                        track.objects[i].trackline[k].cv_getPoint(), track.objects[i].getColor(), 3, 4);
                }
            }
        }

        imshow("frame", frame);
        if(waitKey(1) == 27) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

vector<Object> parseJSON(string str){
    json j = json::parse(str);
    vector<Object> objs;

    for (json::iterator it = j.begin(); it != j.end(); ++it) {
        if(it.value()["class_id"] == 0){
            int top, left, width, height, class_id, object_id;

            top = it.value()["box"]["top"];
            left = it.value()["box"]["left"];
            height = it.value()["box"]["width"];
            width = it.value()["box"]["height"];

            class_id = it.value()["class_id"];
            object_id = it.value()["object_id"];

            int center_top, center_left;

            center_top = (top + height/2);
            center_left = (left + width/2);

            objs.push_back( Object(class_id, object_id,
                            geom::Point(center_left, center_top)) );
        }
    }

    return objs;
}


geom::Point searchByColor(cv::InputArray hsv, cv::OutputArray out, Scalar lower, Scalar upper){
    cv::Mat filtered, kernel, coord, segmented;

    int margin = 500;
    int const max_segment = 5;

    cv::inRange(hsv.getMat(), lower, upper, filtered);
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1), cv::Point(0, 0));
    cv::morphologyEx(filtered, filtered, cv::MORPH_CLOSE, kernel);

    cv::Moments m = moments(filtered, true);
    cv::Point p(m.m10/m.m00, m.m01/m.m00);


    out.assign(filtered);

    cv::Mat nonzero;
    findNonZero(filtered, nonzero);

    // if there's not much information quit
    if(nonzero.total() < 200){
        return geom::Point(0, 0);
    }

    int width = margin, height = margin;

    for (int i = 1; i < max_segment; i++){
        // if out of boundaries, correct roi margins
        margin *= 0.7;
        width = margin;
        height = margin;

        if( filtered.cols < (p.x - margin/2 + margin ) ){
            width = margin - 2 * (p.x + margin/2 - filtered.cols);
        }
        else if(( p.x - margin/2 ) < 0){
            width = margin - 2 * (margin/2 - p.x);
        }

        if( filtered.rows < (p.y - margin/2 + margin ) ){
            height = margin - 2 * (p.y + margin/2 - filtered.rows);
        }
        else if(( p.y - margin/2 ) < 0){
            height = margin - 2 * (margin/2 - p.y);
        }

        // Segment image and reanalyze 
        // Creating mask
        cv::Mat mask(filtered.rows, filtered.cols, CV_8UC1, Scalar(255, 255, 255));
        cv::Rect roi(p.x - (width/2), p.y - (height/2), width, height);
        rectangle(mask, roi, Scalar(0, 0, 0), -1);

        // Segment on image
        subtract(filtered, mask, segmented);

        findNonZero(segmented, nonzero);

        if(nonzero.total() < 10){
            return geom::Point(0, 0);
        }

        m = moments(segmented, true);
        p.x = m.m10/m.m00;
        p.y = m.m01/m.m00;

        rectangle(filtered, roi, Scalar(255, 255, 0));
    }

    return geom::Point(p.x, p.y);
}

void getPointsFromObject(int class_id, vector<geom::Point> *p, vector<Object> *obj){
    for(int i = 0; i < obj->size(); i++){
        if((*obj)[i].class_id == class_id){
            p->push_back((*obj)[i].position);
        }
    }
}