#include "robot.h"

#include <random>
#include <algorithm>
#include <iostream>
#include <limits>

Robot::Robot(double w_0, double max_velocity, int map_size, const double *weight_limit, const double *source,
             const double *position, const double *velocity, const double *r) {

    // initial weight
    this->w_0 = w_0;
    this->max_velocity = max_velocity;

    // map size
    x_min = y_min = -map_size;
    x_max = y_max = map_size;

    r1 = r[0];
    r2 = r[1];

    robot_fitness = -std::numeric_limits<double>::infinity();
    robot_fitness_prev = -std::numeric_limits<double>::infinity();

    for(int i = 0; i < 2; i++){
        this->weight_limit[i] = weight_limit[i];
        // initial weight is w_0
        weight[i] = w_0;
        this->position[i] = position[i];
        this->velocity[i] = velocity[i];
    }
    func(source);

    // set delta to 0, so initial weight is used as default
    delta_f = 0, delta_f_prev = 0;
}

Robot::~Robot() = default;

double Robot::update(const double *global_best, const double *source) {
    if(robot_fitness == 0.0)
        return 0.0; // already on target

    // For each dimension
    double new_position[2], c;

    for(int i = 0; i < 2; i++) {
        // acceleration coefficient c = c1 = c2 = current step weight(w) + 1
        c = weight[i] + 1;
        velocity[i] = weight[i] * velocity[i] + c * r1 * (best_pos[i] - position[i]) +
                                                c * r2 * (global_best[i] - position[i]);
        if (velocity[i] > max_velocity)
            velocity[i] = max_velocity;

        new_position[i] = position[i] + velocity[i];
        if (new_position[i] > 150)
            position[i] = 150.0;
        else if (new_position[i] < -150)
            position[i] = -150;
        else
            position[i] = new_position[i];

    }
    func(source);
//    cout << robot_fitness << " --- <" << position[0] << "; " << position[1] << "> " << endl;
    if(robot_fitness > robot_fitness_prev){
        best_pos[0] = position[0];
        best_pos[1] = position[1];
    }
    for(int i = 0; i < 2; i++) {
        // todo update d(k)???
        update_deltas();
        weight[i] = get_weight(weight[i], i, global_best[i]);
    }
    return robot_fitness;
}

double Robot::get_weight(double w, int i, double global_best) {
    double c_n = 1.0;
    double l = pow(2 * y_max, 2);

    if(delta_f_prev > 0 && delta_f > 0){
        double function = (pow((position[i] - global_best), 3)/(c_n * l));
        w = min(weight_limit[1], w + w_0 * exp(-function));
    }
    else if(delta_f_prev < 0 && delta_f < 0){
        double function = (pow((position[i] - global_best), 2)/(c_n * l));
        w = max(weight_limit[0], w - w_0 * exp(-function));
    }
    else{} // nothing to do
    return w;
}

void Robot::func(const double *source) {
    // function is (x-xs)^2 + (y-ys)^2 where xs and ys is source location
    robot_fitness = -(pow(position[0] - source[0], 2) + pow(position[1] - source[1], 2));
}

double *Robot::get_coordinates(){
    return position;
}

double Robot::get_fitness() {
    return robot_fitness;
}

void Robot::update_deltas() {
    delta_f_prev = delta_f;
    if(robot_fitness > robot_fitness_prev)
        delta_f = 1;
    else
        delta_f = -1;
    robot_fitness_prev = robot_fitness;
}