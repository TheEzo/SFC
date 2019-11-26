#include "robot.h"
#include <iostream>


int main(int argc, char **argv) {


    double max_velocity = 0.7, w_0 = 0.6, c0 = 0.0;
    double weight[2] = {0.4, 0.9};
    double global_best[2] = {0.0, 0.0};
    int robot_count = 8;
    int map_size = 150; // center in [0;0] size to each side of map -- 300*300 is total size
    double cent_x = 0, cent_y = 0;

    Robot *robot[robot_count];
    for(int i = 0; i < robot_count; i++){
        robot[i] = new Robot(w_0, max_velocity, map_size, weight);
    }

    while(1){
        for(int i = 0; i < robot_count; i++){
            robot[i]->count(global_best);
        }
        break;
    }
    return 0;
}