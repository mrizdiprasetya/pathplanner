//
// Created by user on 10/24/17.
//

#include <cmath>
#include "pathplanner.h"

pathplanner::pathplanner() {
    pathplanner::eps = 1;
    pathplanner::r = 1;
    pathplanner::chance = 0.5;
    pathplanner::xMax = 500;
    pathplanner::yMax = 500;
    pathplanner::goalReached = false;
    srand(time(NULL));
}

void pathplanner::start() {
    pathplanner::nodes.push_back(pathplanner::q_start);
    pathplanner::initT();
    while(!goalReached)
    {
        pathplanner::reloop();
    }
    pathplanner::deleteT();
    pathplanner::generatePath();
}

coordinate pathplanner::generatePoint() {
    coordinate point;
    if( (((double)rand())/((double)RAND_MAX)) < pathplanner::chance)
    {
        point = pathplanner::q_goal.getCoord();
    }
    else
    {
        point.setCoord(((double)rand())/((double)RAND_MAX)*pathplanner::xMax, ((double)rand())/((double)RAND_MAX)*pathplanner::yMax);
    }
    return point;
}

void pathplanner::initT() {
    for(int i = 0; i<6; i++)
    {
        if(i < 3)
        {
            threads[i] = std::thread(&pathplanner::generatePointT,this,i);
        }
        else
        {
            threads[i] = std::thread(&pathplanner::generateTree,this,i);
        }
    }
    threads[6] = std::thread(&pathplanner::reinitT,this);
    threads[7] = std::thread(&pathplanner::checkGoal,this);
}

void pathplanner::reloop() {
    if(!threads[6].joinable())
    {
        threads[6] = std::thread(&pathplanner::reinitT,this);
    }
    if(!threads[7].joinable())
    {
        threads[7] = std::thread(&pathplanner::checkGoal,this);
    }
}

void pathplanner::checkGoal() {
    m1.lock();
    coordinate point, point2;
    point = q_goal.getCoord();
    for(int j = 0; j<pathplanner::nodes.size(); j++)
    {
        point2 = pathplanner::nodes[j].getCoord();
        if (point2.getX() == point.getX()) {
            if (point2.getY() == point.getY()) {
                pathplanner::goalReached = true;
            }
        }
    }
    m1.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    threads[7].detach();
}

void pathplanner::deleteT() {
    for(int i = 0; i<8; i++)
    {
        if(threads[i].joinable())
        {
            threads[i].join();
        }
    }
}

void pathplanner::reinitT() {
    for(int i = 0; i<6; i++)
    {
        if(i < 3)
        {
            if(!threads[i].joinable())
            {
                threads[i] = std::thread(&pathplanner::generatePointT,this,i);
            }
        }
        else
        {
            if(!threads[i].joinable())
            {
                threads[i] = std::thread(&pathplanner::generateTree,this,i);
            }
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    threads[6].detach();
}

void pathplanner::generatePointT(int pid) {
    m2.lock();
    coordinate point;
    point = pathplanner::generatePoint();
    pathplanner::qList.push(point);
    m2.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    threads[pid].detach();
}

void pathplanner::generateTree(int pid) {
    coordinate point, point2;
    std::vector<node> q_nearest;
    node q_new;
    int ok = 0;
    double tmp;
    int index;
    double ndist = 10000;
    m3.lock();
    if(!pathplanner::qList.empty())
    {
        point = qList.front();
        qList.pop();
        ok = 1;
    }
    else
    {
        printf("EMPTY\n");
    }
    m3.unlock();

    if(ok) {
        for (int j = 0; j < pathplanner::nodes.size(); j++) {
            tmp = pathplanner::calcDist(nodes[j].getCoord(), point);
            if (tmp < ndist) {
                ndist = tmp;
                index = j;
            }
        }

        q_new.setCoord(pathplanner::steer(point, pathplanner::nodes[index].getCoord(), ndist));
        q_new.setCost(pathplanner::calcDist(q_new.getCoord(), pathplanner::nodes[index].getCoord()) +
                      pathplanner::nodes[index].getCost());

        for (int j = 0; j < pathplanner::nodes.size(); j++) {
            if ((pathplanner::calcDist(pathplanner::nodes[j].getCoord(), q_new.getCoord())) <= pathplanner::r) {
                q_nearest.push_back(pathplanner::nodes[j]);
            }
        }

        node q_min;
        double C_min;

        q_min = pathplanner::nodes[index];
        C_min = q_new.getCost();

        for (int j = 0; j < q_nearest.size(); j++) {
            if ((q_nearest[j].getCost() + pathplanner::calcDist(q_nearest[j].getCoord(), q_new.getCoord())) < C_min) {
                q_min = q_nearest[j];
                C_min = q_nearest[j].getCost() + pathplanner::calcDist(q_nearest[j].getCoord(), q_new.getCoord());
            }
        }

        m3.lock();
        point2 = q_min.getCoord();
        for (int j = 0; j < pathplanner::nodes.size(); j++) {
            point = pathplanner::nodes[j].getCoord();
            if (point.getX() == point2.getX()) {
                if (point.getY() == point2.getY()) {
                    q_new.setParent(j);
                }
            }
        }
        pathplanner::nodes.push_back(q_new);
        m3.unlock();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    threads[pid].detach();
}

void pathplanner::optimzer() {

}

void pathplanner::generatePath() {
    node q_end;
    coordinate point;
    point = pathplanner::q_goal.getCoord();
    double tmp;
    double ndist = 10000;
    int index;
    for (int j = 0; j < pathplanner::nodes.size(); j++) {
        tmp = pathplanner::calcDist(nodes[j].getCoord(), point);
        if (tmp < ndist) {
            ndist = tmp;
            index = j;
        }
    }
    pathplanner::q_goal.setParent(index);
    q_end = pathplanner::q_goal;
    while(q_end.getParent() != 0)
    {
        index = q_end.getParent();
        point = pathplanner::nodes[index].getCoord();
        printf("PATH => X: %f, Y: %f\n", point.getX(), point.getY());
        q_end = pathplanner::nodes[index];
    }
}

void pathplanner::setEps(double eps) {
    pathplanner::eps = eps;
}

void pathplanner::setChance(double chance) {
    pathplanner::chance = chance;
}

void pathplanner::setR(double r) {
    pathplanner::r = r;
}

void pathplanner::setGoal(node n) {
    pathplanner::q_goal = n;
}

void pathplanner::setStart(node n) {
    pathplanner::q_start = n;
}

double pathplanner::calcDist(coordinate c1, coordinate c2) {
    return sqrt(pow((c1.getX() - c2.getX()),2) + pow((c1.getY() - c2.getY()),2));
}

coordinate pathplanner::steer(coordinate c1, coordinate c2, double val) {
    coordinate qnew;
    if(val >= pathplanner::eps)
    {
        qnew.setCoord((c2.getX() + ((c1.getX() - c2.getX())*pathplanner::eps)/pathplanner::calcDist(c1,c2)), (c2.getY() + ((c1.getY() - c2.getY())*pathplanner::eps)/pathplanner::calcDist(c1,c2)));
    }
    else
    {
        qnew.setCoord(c1.getX(), c1.getY());
    }
    return qnew;
}

int pathplanner::getNodeSize() {
    return pathplanner::nodes.size();
}

bool pathplanner::noCollision(node n2, node n1) {
    int nc = 1;
    int ints1, ints2, ints3, ints4;
    coordinate A, B;
    coordinate C1, C2, C3, C4;
    coordinate D1, D2, D3, D4;
    double obs[4];
    for(int j = 0; j<pathplanner::ooo.size(); j++)
    {
        obs[0] = o[j].getX() - 2;
        obs[1] = o[j].getY() - 2;
        obs[2] = o[j].getX() + o[j].getXl() + 2;
        obs[3] = o[j].getY() + o[j].getYl() + 2;

        C1.setCoord(obs[0],obs[1]);
        C2.setCoord(obs[0],obs[1]);
        C3.setCoord(obs[2],obs[3]);
        C4.setCoord(obs[2],obs[3]);

        D1.setCoord(obs[0],obs[3]);
        D2.setCoord(obs[2],obs[1]);
        D3.setCoord(obs[2],obs[1]);
        D4.setCoord(obs[0],obs[3]);

        ints1 = pathplanner::ccw(A,C1,D1) != pathplanner::ccw(B,C1,D1) && pathplanner::ccw(A,B,C1) != pathplanner::ccw(A,B,D1);
        ints2 = pathplanner::ccw(A,C2,D2) != pathplanner::ccw(B,C2,D2) && pathplanner::ccw(A,B,C2) != pathplanner::ccw(A,B,D2);
        ints3 = pathplanner::ccw(A,C3,D3) != pathplanner::ccw(B,C3,D3) && pathplanner::ccw(A,B,C3) != pathplanner::ccw(A,B,D3);
        ints4 = pathplanner::ccw(A,C4,D4) != pathplanner::ccw(B,C4,D4) && pathplanner::ccw(A,B,C4) != pathplanner::ccw(A,B,D4);

        if(ints1 == 0 && ints2 == 0 && ints3 == 0 && ints4 == 0)
        {
            nc = nc && 1;
        }
        else
        {
            nc = nc && 0;
        }
    }
    return nc;
}

bool pathplanner::ccw(coordinate A, coordinate B, coordinate C) {
    return (C.getY()-A.getY()) * (B.getX()-A.getX()) > (B.getY()-A.getY()) * (C.getX()-A.getX());
}

void pathplanner::setObstacle() {

}