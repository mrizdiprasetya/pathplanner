//
// Created by user on 10/24/17.
//

#ifndef PATHPLANNING_PATHPLANNER_H
#define PATHPLANNING_PATHPLANNER_H


#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include "node.h"

class pathplanner {
public:
    pathplanner();
    void start();
    void setStart(node n);
    void setGoal(node n);
    int getNodeSize();

private:
    std::vector<node> nodes;
    std::queue<coordinate> qList;
    std::thread threads[8];
    std::mutex m1, m2, m3;
    node q_start;
    node q_goal;
    coordinate generatePoint();
    coordinate steer(coordinate c1, coordinate c2, double val);
    void setEps(double eps);
    void setR(double r);
    void setChance(double chance);
    void initT();
    void reinitT();
    void reloop();
    void checkGoal();
    void deleteT();
    void generatePointT(int pid);
    void generateTree(int pid);
    void generatePath();
    void optimzer();
    double calcDist(coordinate c1, coordinate c2);
    double chance;
    double eps;
    double r;
    double xMax;
    double yMax;
    bool goalReached;
};


#endif //PATHPLANNING_PATHPLANNER_H
