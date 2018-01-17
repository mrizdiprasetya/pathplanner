//
// Created by user on 10/24/17.
//

#ifndef PATHPLANNING_NODE_H
#define PATHPLANNING_NODE_H


#include "coordinate.h"

class node {
public:
    node();
    node(coordinate c, double cost, int parent);
    coordinate getCoord();
    double getCost();
    int getParent();
    void setCoord(double x, double y);
    void setCoord(coordinate c);
    void setParent(int parent);
    void setCost(double cost);

private:
    coordinate c;
    double cost;
    int parent;

};


#endif //PATHPLANNING_NODE_H
