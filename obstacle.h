//
// Created by mrds on 17.01.18.
//

#ifndef PATHPLANNING_OBSTACLE_H
#define PATHPLANNING_OBSTACLE_H


#include "coordinate.h"

class obstacle {
public:
    obstacle();
    obstacle(double x, double y, double xl, double yl);
    coordinate getCoord();
    double getX();
    double getY();
    double getXl();
    double getYl();
private:
    coordinate c;
    double xl;
    double yl;
};


#endif //PATHPLANNING_OBSTACLE_H
