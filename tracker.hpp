#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

#include "geometry.hpp"
#include <iostream>
#include <vector>

using namespace std;
using namespace geom;

#define MAX_DISS 30

//max range

class Object {
public:
    int id;
    int maxDiss;
    bool updated;

    geom::Point position;
    geom::Point position_old;

    geom::Vector speed;

    Object();

    Object(int id, geom::Point pos) : id(id), position(pos), updated(true), maxDiss(MAX_DISS){
        speed.module = 0;
        speed.angle = 0;
    }

    ~Object(){};
    bool dissapeared(){
        if(maxDiss){
            maxDiss--;
            position_old = position;

            position.update(speed);

            if(speed.module){
                speed.module -= 1;
            }

            return false;
        }
        else{
            return true;
        }
    }

    void update(geom::Point p){
        geom::Vector vel = position.getVelocity(p);
        
        vel.module = (vel.module + speed.module)/2;
        vel.angle = (vel.angle + speed.angle)/2;        

        speed = vel;

        position_old = position;
        position = p;

        updated = true;
    }

    geom::Point getSpeedPoint(){
        return geom::Point(position.top + speed.module * sin(speed.angle) * 2,
                            position.left + speed.module * cos(speed.angle) * 2);
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
    void addObject(geom::Point p);
    int getNewID();
};


int Tracker::update(vector<geom::Point> points){
    vector<Object> old_objects = objects;

    for (int i = 0; i < objects.size(); i++){
        objects[i].updated = false;
    }

    for (int i = 0; i < points.size(); i++){
        int index;
        float minDist = 999999, dist;

        // find minimun distance for each updated object
        for(int j = 0; j < objects.size(); j++){
            if(objects[i].updated){
                continue;
            }

            dist = points[i].getDistance(objects[j].position);

            if (dist < minDist){
                minDist = dist;
                index = j;
            }       
        }

        // refresh positions or add objects
        if(minDist < maxRange){
            objects[index].update(points[i]);
        }
        else{
            addObject(points[i]); 
        }
    }

    // compute lost objects
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

void Tracker::addObject(geom::Point p){
    if(objects.size() == 0){
        objects.push_back(Object(getNewID(), p));
        count = 1;  
    }
    else{
        objects.push_back(Object(getNewID(), p));
        count++;  
    }
}

int Tracker::getNewID(){
    vector<int> vct;
    int id;
    bool used;

    for (int i = 0; i < objects.size(); i++){
        vct.push_back(objects[i].id);
    }

    for (id = 0; id < 100; id++){
        used = false;

        for(int i = 0; i < vct.size(); i++){
            if(id == vct[i]){
                used = true;
            }
        }

        if(!used){
            break;
        }
    }

    return id;
}

#endif
