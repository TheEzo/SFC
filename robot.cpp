#include "robot.h"

#include <random>
#include <algorithm>
#include <iostream>

Robot::Robot(double w_0, double max_velocity, int map_size, double *weight_limit) {
    std::default_random_engine generator;
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    // initial weight
    this->w_0 = w_0;
    this->max_velocity = max_velocity;

    // map size
    x_min = y_min = -map_size;
    x_max = y_max = map_size;

    // random r1, r2 <0;1>
    r1 = uniform(generator);
    r2 = uniform(generator);

    for(int i = 0; i < 2; i++){
        this->weight_limit[i] = weight_limit[i];
        // initial weight i w_0
        weight[i] = w_0;
        // random position in map
        position[i] = uniform(generator) * map_size * 2 - map_size;
        // robot best position is current position
        best_pos[i] = position[i];
        // initial velocity is also random, but less than max_velocity
        velocity[i] = uniform(generator) * 2 * max_velocity - max_velocity;
    }

    // set delta to 0, so initial weight is used as default
    delta_f = 0, delta_f_prev = 0;
}

Robot::~Robot() = default;

void Robot::count(const double *global_best) {
    // For each dimension
    double new_position[2], new_velocity[2], new_weight[2], c;

    for(int i = 0; i < 2; i++){
        new_weight[i] = get_weight(weight[i], i, global_best[i]);
        // acceleration coefficient c = c1 = c2 = current step weight(w) + 1
        c = new_weight[i]  + 1;
        new_velocity[i] = new_weight[i] * velocity[i] +  c * r1 * (best_pos[i] - position[i]) +
                                                         c * r2 * (global_best[i] - position[i]);
        if(new_velocity[i] > max_velocity)
            std::cerr << "Velocity too big" << std::endl;
    }

    for(int i = 0; i < 2; i++){
        weight[i] = new_weight[i];
        position[i] = new_position[i];
        velocity[i] = new_velocity[i];
    }
}

double Robot::get_weight(double w, int i, double global_best) {
    double c_n = 1.0;
    double l = pow(2 * y_max, 2);

    if(delta_f_prev > 0 && delta_f > 0){
        double func = (pow((position[i] - global_best), 3)/(c_n * l));
        w = min(weight_limit[1], w + w_0 * exp(-func));
    }
    else if(delta_f_prev < 0 && delta_f < 0){
        double func = (pow((position[i] - global_best), 2)/(c_n * l));
        w = max(weight_limit[0], w - w_0 * exp(-func));
    }
    else{
        // nothing to do
    }

    return w;
}

void Robot::func(double x, double y) {

}

