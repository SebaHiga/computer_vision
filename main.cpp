#include "opencv2/opencv.hpp"

using namespace cv;

int main(int, char**){
    VideoCapture cam(0); // open the default camera

    if(!cam.isOpened())  // check if we succeeded
        return -1;
    Mat frame;

    namedWindow("frame",1);
    while(1)
    {
        Mat frame;
        cam >> frame; // get a new frame from camera
        // cvtColor(frame, frame, COLOR_BGR2GRAY);
        // GaussianBlur(frame, frame, Size(7,7), 1.5, 1.5);
        // Canny(frame, frame, 0, 30, 3);
        imshow("frame", frame);
        if(waitKey(1) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}