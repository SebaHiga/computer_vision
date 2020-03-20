#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

#include "geometry.hpp"
#include <iostream>
#include <vector>
#include <stdlib.h>

using namespace std;
using namespace geom;

#define MAX_DISS 20
#define MIN_APP 4
#define MAX_DOTS 30
#define MAX_RANGE 55

//max range

class Object {
public:
    int id;
    int maxDiss;
    bool updated;

    int b, g, r;

    int minAppearance;
    bool valid;

    geom::Point position;
    geom::Point position_old;

    std::vector<geom::Vector> speed;

    std::vector<geom::Point> trackline;

    Object();

    Object(int id, geom::Point pos) :   id(id), position(pos), updated(true),
                                        maxDiss(MAX_DISS), minAppearance(MIN_APP), valid(false){
        speed.push_back(geom::Vector());
        trackline.push_back(pos);
        b = rand() % 256;
        g = rand() % 256;
        r = rand() % 256;

    }

    ~Object(){};
    bool dissapeared(){
        if(valid){
            if(maxDiss){
                maxDiss--;
                position_old = position;

                speed[speed.size()].module *= 0.5;
                position.update(speed[speed.size()]);
                trackline.push_back(position);
                
                if(trackline.size() > MAX_DOTS){
                    trackline.erase(trackline.begin());
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
        geom::Vector vel = position.getVelocity(p), accumulator;
        
        for(int i = 0; i < speed.size(); i++){
            accumulator += speed[i];
        }
        accumulator += vel;

        if(speed.size()){
            accumulator /= speed.size() + 1;
        }

        speed.push_back(accumulator);

        if(speed.size() > 3){
            speed.erase(speed.begin());
        }

        position_old = position;
        position = p;

        updated = true;

        maxDiss = MAX_DISS;

        trackline.push_back(p);
        if(trackline.size() > MAX_DOTS){
            trackline.erase(trackline.begin());
        }

        if(minAppearance){
            minAppearance--;
            if(!minAppearance){
                valid = true;
            }
        }
    }

    geom::Point getSpeedPoint(){
        return geom::Point(position.top + speed[speed.size()].module * sin(speed[speed.size()].angle) * 10,
                            position.left + speed[speed.size()].module * cos(speed[speed.size()].angle) * 10);
    }

    cv::Scalar getColor(){
        return cv::Scalar(b, g, r);
    }
};

class Tracker {
private:
    static const int maxRange = MAX_RANGE;

public:
    vector<Object> objects;
    Tracker(void){};
    ~Tracker(){};

    // returns the total difference of tracked objects after updating
    int update(vector<geom::Point> *points);
    void addObject(geom::Point p);
    int getNewID();
};


int Tracker::update(vector<geom::Point> *points){
    for (int i = 0; i < objects.size(); i++){
        objects[i].updated = false;
    }

    for (int i = 0; i < points->size(); i++){
        (*points)[i].updated = false;
    }

    // valid priority analysis
    for (int i = 0; i < points->size(); i++){
        if((*points)[i].updated){
            continue;
        }

        int index = 0;
        float minDist = 999999, dist;

        // find minimun distance for each updated object
        for(int j = 0; j < objects.size(); j++){
            if(!objects[j].updated && objects[j].valid){
                dist = (*points)[i].getDistance(objects[j].position);

                if (dist < minDist){
                    minDist = dist;
                    index = j;
                }
            }
        }

        // refresh positions or add objects
        if(minDist < maxRange){
            objects[index].update((*points)[i]);
            (*points)[i].updated = true;
        }
    }

    // invalid object analysis
    for (int i = 0; i < points->size(); i++){
        if((*points)[i].updated){
            continue;
        }

        int index = 0;
        float minDist = 999999, dist;

        // find minimun distance for each updated object
        for(int j = 0; j < objects.size(); j++){
            if(!objects[j].updated && !objects[j].valid){
                dist = (*points)[i].getDistance(objects[j].position);

                if (dist < minDist){
                    minDist = dist;
                    index = j;
                }
            }
        }

        // refresh positions or add objects
        if(minDist < maxRange){
            objects[index].update((*points)[i]);
        }
        else{
            addObject((*points)[i]);
        }
        (*points)[i].updated = true;
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
