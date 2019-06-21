#ifndef NODE_H_
#define NODE_H_

#include <iostream>
#include <string>

struct Posture {
 public:
    Posture() = default;
    Posture(int a, int b, int c):
        x(a), y(b), direction(c), G(0.0), H(0.0) {}
    int x;
    int y;
    double G;
    double H;
    int direction;
    std::string path;
    bool operator == (const Posture& pos) {
        return pos.x == x && pos.y == y && pos.direction == direction;}
};

#endif  // NODE_H_
