//
// Created by user on 10/24/17.
//

#include "node.h"

node::node() {

}

node::node(coordinate c, double cost, int parent) {
    node::c = c;
    node::cost = cost;
    node::parent = parent;
}

void node::setCoord(coordinate c) {
    node::c = c;
}

void node::setCoord(double x, double y) {
    node::c.setCoord(x,y);
}

coordinate node::getCoord() {
    return node::c;
}

double node::getCost() {
    return node::cost;
}

int node::getParent() {
    return node::parent;
}

void node::setCost(double cost) {
    node::cost = cost;
}

void node::setParent(int parent) {
    node::parent = parent;
}