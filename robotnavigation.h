#ifndef ROBOTNAVIGATION_H_
#define ROBOTNAVIGATION_H_

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include "node.h"
#include "configuration.h"

class RobotNavigation {
 public:
    RobotNavigation();
    ~RobotNavigation();
    // main function to search the path from start to goal
    void SearchPath();

 private:
    // initialization
    void Initialization();
    // return the path if found
    void ReturnPath(Posture* cur);
    // calculate the heuristic cost from current node to goal
    double GetHvalue(Posture* current);
    // return the next posture of vehicle
    Posture* NextPosture(Posture* current, int move);
    // return the string step based on int number
    std::string GetMovement(int m);
    // start
    Posture start_;
    // goal
    Posture goal_;
    // possible movement of vehicle
    std::vector<int> move_;
    // 2d map of obstacles
    std::vector<std::vector<int> > obstacles_;
    // flag indicating if path is found
    bool find_path_;
};

#endif  // ROBOTNAVIGATION_H_
