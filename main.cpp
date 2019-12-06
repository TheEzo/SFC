#include "robot.h"
#include <iostream>
#include <limits>
#include <random>
#include <ctime>

int main(int argc, char **argv) {
    srand(time(NULL));
    std::default_random_engine generator(rand());
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    double max_velocity = 0.7, w_0 = 0.6, c0 = 0.0;
    double weight[2] = {0.4, 0.9};
    int robot_count = 8;
    int map_size = 150; // center in [0;0] size to each side of map -- 300*300 is total size
    double source[2] = {5, 5}; // acoustic source

    Robot *robot[robot_count];
    double tmp_velocity[2], tmp_position[2], tmp_r[2];
    for(int i = 0; i < robot_count; i++){
        for(int j = 0; j < 2; j++){
            // random position in map
            tmp_position[j] = uniform(generator) * map_size * 2 - map_size;
            // initial velocity is also random, but less than max_velocity
            tmp_velocity[j] = uniform(generator) * 2 * max_velocity - max_velocity;
            // random r1, r2 <0;1>
            tmp_r[j] = uniform(generator);
        }
        robot[i] = new Robot(w_0, max_velocity, map_size, weight, source, tmp_position, tmp_velocity, tmp_r);
    }

    double best_fitness = -std::numeric_limits<double>::infinity();
    double current_fitness, *tmp_coordinates;
    double global_best[2];
    int index;
    for(int x = 0; x< 400; x++){
        for(int i = 0; i < robot_count; i++){
            current_fitness = robot[i]->get_fitness();
            if(current_fitness > best_fitness){
                best_fitness = current_fitness;
                tmp_coordinates = robot[i]->get_coordinates();
                global_best[0] = tmp_coordinates[0];
                global_best[1] = tmp_coordinates[1];
                index = i;
            }
        }
        for(int i = 0; i < robot_count; i++){
            current_fitness = robot[i]->update(global_best, source);
            if(current_fitness > best_fitness){
                best_fitness = current_fitness;
                tmp_coordinates = robot[i]->get_coordinates();
                global_best[0] = tmp_coordinates[0];
                global_best[1] = tmp_coordinates[1];
                index = i;
            }
        }
        std::cout << x<< "  " << index << ": <" << global_best[0] << "; " << global_best[1] << ">" << std::endl;
    }
    for(int i = 0; i < robot_count; i++){
        tmp_coordinates = robot[i]->get_coordinates();
        global_best[0] = tmp_coordinates[0];
        global_best[1] = tmp_coordinates[1];
        std::cout << i << ": <" << global_best[0] << "; " << global_best[1] << ">" << std::endl;
    }
    return 0;
}