//
// Created by user on 10/24/17.
//

#ifndef PATHPLANNING_COORDINATE_H
#define PATHPLANNING_COORDINATE_H


class coordinate {
public:
    coordinate();
    coordinate(double x, double y);
    void setCoord(double x, double y);
    double getX();
    double getY();

private:
    double x;
    double y;

};


#endif //PATHPLANNING_COORDINATE_H
