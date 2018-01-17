#include <thread>
#include "pathplanner.h"

int main() {
    pathplanner planner;

    //Set Current POS
    coordinate c1(1,1);

    //Set Goal POS
    coordinate c2(499,499);

    //Create Node
    node start(c1, 0, 0);
    node goal(c2, 0, 0);

    //Save Node to Planner
    planner.setStart(start);
    planner.setGoal(goal);

    //Start Path Planner
    planner.start();

    printf("Finish with %d nodes\n", planner.getNodeSize());
    return 0;
}