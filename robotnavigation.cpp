#include <algorithm>
#include <queue>
#include <unordered_set>
#include <vector>
#include "robotnavigation.h"

RobotNavigation::RobotNavigation() {
    Initialization();
}

RobotNavigation::~RobotNavigation() {
    obstacles_.clear();
}

void RobotNavigation::Initialization() {
    // set up movement
    move_ = {Movement::FL, Movement::FR, Movement::FS,
                    Movement::BL, Movement::BR, Movement::BS};
    // set up start and goal point
    start_ = Posture(2, 2, 0);
    goal_ = Posture(6, 12, 2);
    // set up obstacles with vector
    obstacles_ = std::vector<std::vector<int> > (N, std::vector<int>(M, 0));
    for (int i = 0; i < 5; i++) {
        obstacles_[i][6] = 1;
        obstacles_[i][7] = 1;
        obstacles_[i][8] = 1;
    }
    find_path_ = false;
}

void RobotNavigation::SearchPath() {
    auto comp = [](Posture* p1, Posture* p2) { return (p1->H + p1->G) >
                                                (p2->H + p2->G); };
    std::priority_queue<Posture*, std::vector<Posture*>, decltype(comp)> q(comp);
    q.push(&start_);
    std::vector<std::vector<std::vector<int> > > visited(N, std::vector<std::vector<int>>(M, std::vector<int>(Z, 0)));
    visited[start_.x][start_.y][start_.direction] = 1;
    while (!q.empty()) {
        Posture* cur = q.top();
        q.pop();
        if ((*cur) == goal_) {
            // print path here
            ReturnPath(cur);
            find_path_ = true;
            break;
        }
        for (auto m:move_) {
            // check if next step is obstacle
            Posture* next = NextPosture(cur, m);
            if (next->x >=0 && next->x < N && next->y >=0 && next->y < M &&
                    obstacles_[next->x][next->y] == 0 &&
                    visited[next->x][next->y][next->direction] == 0) {
                next->path.append(cur->path);
                next->path.append(GetMovement(m));
                next->H = GetHvalue(next);
                next->G = cur->G + 1;
                q.push(next);
                visited[next->x][next->y][next->direction] = 1;
            }
        }
    }
    if (!find_path_) {
        std::cout << "Path not achievable" << std::endl;
    }
}

void RobotNavigation::ReturnPath(Posture* cur) {
    std::cout << cur->G << ",";
    for (int i = 0; i < cur->path.size(); i = i + 2) {
        if ( i != cur->path.size() - 2) {
            std::cout << (cur->path)[i] << (cur->path)[i+1] << ",";
        } else {
            std::cout << (cur->path)[i] << (cur->path)[i+1] << std::endl;
        }
    }
}

double RobotNavigation::GetHvalue(Posture* current) {
    return std::sqrt((goal_.x - current->x)*(goal_.x - current->x)
            + (goal_.y - current->y)*(goal_.y - current->y));
}

std::string RobotNavigation::GetMovement(int m) {
    switch (m) {
        case Movement::FL: {
            return "FL";
        }
        case Movement::FR: {
            return "FR";
        }
        case Movement::FS: {
            return "FS";
        }
        case Movement::BL: {
            return "BL";
        }
        case Movement::BR: {
            return "BR";
        }
        case Movement::BS: {
            return "BS";
        }
        default: {
            std::cout << "error" << std::endl;
        }
    }
}

Posture* RobotNavigation::NextPosture(Posture* cur, int move) {
    int x = 0, y = 0, orientation = 0;
    switch(move) {
        case Movement::FL: { //FL
            if (cur->direction == 0) {
                orientation = 1; x = cur->x -1; y = cur->y - 1;
            } else if (cur->direction == 1) {
                orientation = 2; x = cur->x + 1; y = cur->y - 1;
            } else if (cur->direction == 2) {
                orientation = 3; x = cur->x + 1; y = cur->y + 1;
            } else {
                orientation = 0; x = cur->x - 1; y = cur->y + 1;
            }
            break;
        }
        case Movement::FR: { //FR
            if (cur->direction == 0) {
                orientation = 3; x = cur->x -1; y = cur->y + 1;
            } else if (cur->direction == 1) {
                orientation = 0; x = cur->x - 1; y = cur->y - 1;
            } else if (cur->direction == 2) {
                orientation = 1; x = cur->x + 1; y = cur->y - 1;
            } else {
                orientation = 2; x = cur->x + 1; y = cur->y + 1;
            }
            break;
        }
        case Movement::FS: { // FS
            if (cur->direction == 0) {
                orientation = 0; x = cur->x - 1; y = cur->y;
            } else if (cur->direction == 1) {
                orientation = 1; x = cur->x; y = cur->y - 1;
            } else if (cur->direction == 2) {
                orientation = 2; x = cur->x + 1; y = cur->y;
            } else {
                orientation = 3; x = cur->x; y = cur->y + 1;
            }
            break;
        }
        case Movement::BL: { // BL
            if (cur->direction == 0) {
                orientation = 3; x = cur->x + 1; y = cur->y - 1;
            } else if (cur->direction == 1) {
                orientation = 0; x = cur->x + 1; y = cur->y + 1;
            } else if (cur->direction == 2) {
                orientation = 1; x = cur->x - 1; y = cur->y + 1;
            } else {
                orientation = 2; x = cur->x - 1; y = cur->y - 1;
            }
            break;
        }
        case Movement::BR: { // BR
            if (cur->direction == 0) {
                orientation = 1; x = cur->x + 1; y = cur->y + 1;
            } else if (cur->direction == 1) {
                orientation = 2; x = cur->x - 1; y = cur->y + 1;
            } else if (cur->direction == 2) {
                orientation = 3; x = cur->x - 1; y = cur->y - 1;
            } else {
                orientation = 0; x = cur->x + 1; y = cur->y - 1;
            }
            break;
        }
        case Movement::BS: { // BS
            if (cur->direction == 0) {
                orientation = 0; x = cur->x + 1; y = cur->y;
            } else if (cur->direction == 1) {
                orientation = 1; x = cur->x; y = cur->y + 1;
            } else if (cur->direction == 2) {
                orientation = 2; x = cur->x - 1; y = cur->y;
            } else {
                orientation = 3; x = cur->x; y = cur->y - 1;
            }
            break;
        }
    }
    Posture* nextposture = new Posture(x, y, orientation);
    return nextposture;
}


