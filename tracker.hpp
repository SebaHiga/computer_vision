#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

#include "geometry.hpp"
#include <iostream>
#include <vector>

using namespace std;
using namespace geom;

#define MAX_DISS 100

//max range

class Object {
public:
    int id;
    int maxDiss;
    bool updated;

    geom::Point position;
    geom::Vector speed;

    Object();

    Object(int id, geom::Point pos) : id(id), position(pos), updated(true), maxDiss(MAX_DISS){

    }

    ~Object(){};
    bool dissapeared(){
        if(maxDiss){
            maxDiss--;
            return false;
        }
        else{
            return true;
        }
    }
};

class Tracker {
private:
    static const int maxRange = 200;

public:
    int count;
    vector<Object> objects;
    Tracker(void) : count(0){};
    ~Tracker(){};

    // returns the total difference of tracked objects after updating
    int update(vector<geom::Point> points);
};


int Tracker::update(vector<geom::Point> points){
    vector<Object> old_objects = objects;

    if(objects.size() == 0){
        objects.push_back(Object(count, points[0]));
        count = 0;
    }
    
    for (int i = 0; i < objects.size(); i++){
        objects[i].updated = false;
    }

    for (int i = 0; i < points.size(); i++){
        int index;
        float minDist, dist;

        minDist = points[i].getDistance(objects[0].position);
        index = 0;

        for(int j = 1; j < objects.size(); j++){
            dist = points[i].getDistance(objects[j].position);

            if (dist < minDist){
                minDist = dist;
                index = j;
            }       
        }

        if(minDist < maxRange){
            objects[index].position.update(points[i]);
            objects[index].updated = true;
        }
        else{
            objects.push_back(Object(count, points[i]));
            count++;
        }
    }

    for (int i = 0; i < objects.size(); i++){
        if(!objects[i].updated){
            if(objects[i].dissapeared()){
                objects.erase(objects.begin() + i);
                i = 0;
                count--;
            }
        }
    }


    return 1;
}

#endif
