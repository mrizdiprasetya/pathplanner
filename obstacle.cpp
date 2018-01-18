//
// Created by mrds on 17.01.18.
//

#include "obstacle.h"

obstacle::obstacle() {

}

obstacle::obstacle(double x, double y, double xl, double yl) {
    obstacle::c.setCoord(x,y);
}

coordinate obstacle::getCoord() {
    return obstacle::c;
}

double obstacle::getXl() {
    return obstacle::xl;
}

double obstacle::getYl() {
    return obstacle::yl;
}

double obstacle::getX() {
    return obstacle::c.getX();
}

double obstacle::getY() {
    return obstacle::c.getY();
}