//
// Created by user on 10/24/17.
//

#include "coordinate.h"

coordinate::coordinate() {

}

coordinate::coordinate(double x, double y) {
    coordinate::x = x;
    coordinate::y = y;
}

double coordinate::getX() {
    return coordinate::x;
}

double coordinate::getY() {
    return coordinate::y;
}

void coordinate::setCoord(double x, double y) {
    coordinate::x = x;
    coordinate::y = y;
}