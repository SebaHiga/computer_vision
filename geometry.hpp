#ifndef __GEOMETRY_HPP__
#define __GEOMETRY_HPP__

#include <math.h>

namespace geom {

    class Vector{
    public:
        float angle;
        float module;

        Vector() : angle(0), module(0){};
        ~Vector(){};

        Vector update(Vector v){
            module = (module + v.module)/2;
            angle = (angle + v.angle)/2;  
        }

        void operator=(Vector v){
            module = v.module;
            angle = v.angle; 
        }

        void operator=(float num){
            module = num;
            angle = num; 
        }

        Vector operator/=(int div){
            module /= div;
            angle /= div; 
        }

        Vector operator+=(Vector v){
            module += v.module;
            angle += v.angle; 
        }
    };

    class Point {
    public:
        int top, left;
        bool updated;

        Point() : top(0), left(0), updated(true){

        }

        Point(int _top, int _left) : top(_top), left(_left), updated(true){

        }

        float getDistance (Point p){
            return hypot(p.top-top, p.left-left);
        }

        Vector getVelocity (Point p){
            // if new vector then return 0 vel
            if(!top || !left){
                Vector v;
                return v;
            }

            Vector speed;

            speed.module = hypot(p.top - top, p.left - left);
            speed.angle = atan2f32(p.top - top, p.left - left);

            return speed;
        }

        float originDistance (void){
            return hypot(top, left);
        }

        void update (Point point){
            top = point.top;
            left = point.left;
        }

        void update (Vector speed){
            top = top + speed.module * sin(speed.angle);
            left = left + speed.module * cos(speed.angle);
        }

        void operator=(Point p){
            top = p.top;
            left = p.left;
        }

        // opencv format

        cv::Point cv_getPoint(){
            return cv::Point(top, left);
        }
    };



}

#endif