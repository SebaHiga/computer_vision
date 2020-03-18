#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

#include "geometry.hpp"
#include <iostream>
#include <vector>

using namespace std;
using namespace geom;

#define MAX_DISS 30
#define MIN_APP 5

//max range

class Object {
public:
    int id;
    int maxDiss;
    bool updated;

    int minAppearance;
    bool valid;

    geom::Point position;
    geom::Point position_old;

    geom::Vector speed;

    Object();

    Object(int id, geom::Point pos) :   id(id), position(pos), updated(true),
                                        maxDiss(MAX_DISS), minAppearance(MIN_APP), valid(false){
        speed.module = 0;
        speed.angle = 0;
    }

    ~Object(){};
    bool dissapeared(){
        if(valid){
            if(maxDiss){
                maxDiss--;
                position_old = position;

                position.update(speed);

                if(speed.module){
                    speed.module *= 1;
                }

                return false;
            }
            else{
                return true;
            }
        }
        return true;
    }

    void update(geom::Point p){
        geom::Vector vel = position.getVelocity(p);
        
        vel.module = vel.module;
        vel.angle = (vel.angle + speed.angle)/2;        

        speed = vel;

        position_old = position;
        position = p;

        updated = true;

        maxDiss = MAX_DISS;

        if(minAppearance){
            minAppearance--;
            if(!minAppearance){
                valid = true;
            }
        }
    }

    geom::Point getSpeedPoint(){
        return geom::Point(position.top + speed.module * sin(speed.angle) * 2,
                            position.left + speed.module * cos(speed.angle) * 2);
    }
};

class Tracker {
private:
    static const int maxRange = 250;

public:
    vector<Object> objects;
    Tracker(void){};
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
        points[i].updated = false;
    }

    // valid priority analysis
    for (int i = 0; i < points.size(); i++){
        if(points[i].updated){
            continue;
        }

        int index = 0;
        float minDist = 999999, dist;

        // find minimun distance for each updated object
        for(int j = 0; j < objects.size(); j++){
            if(!objects[j].updated && objects[j].valid){
                dist = points[i].getDistance(objects[j].position);

                if (dist < minDist){
                    minDist = dist;
                    index = j;
                }
            }
        }

        // refresh positions or add objects
        if(minDist < maxRange){
            objects[index].update(points[i]);
            points[i].updated = true;
        }
    }

    // invalid object analysis
    for (int i = 0; i < points.size(); i++){
        if(points[i].updated){
            continue;
        }

        int index = 0;
        float minDist = 999999, dist;

        // find minimun distance for each updated object
        for(int j = 0; j < objects.size(); j++){
            if(!objects[j].updated && !objects[j].valid){
                dist = points[i].getDistance(objects[j].position);

                if (dist < minDist){
                    minDist = dist;
                    index = j;
                }
            }
        }

        // refresh positions or add objects
        if(minDist < maxRange){
            objects[index].update(points[i]);
        }
        else{
            addObject(points[i]);
        }
        points[i].updated = true;
    }

    // compute lost objects
    for (int i = 0; i < objects.size(); i++){
        if(!objects[i].updated){
            if(objects[i].dissapeared()){
                objects.erase(objects.begin() + i);
                i = 0;
            }
        }
    }

    return 1;
}

void Tracker::addObject(geom::Point p){
    objects.push_back(Object(getNewID(), p));
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
