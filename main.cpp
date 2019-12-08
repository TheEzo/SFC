#include "robot.h"
#include <iostream>
#include <limits>
#include <random>
#include <ctime>
#include <vector>


int print_coordinates(Robot **robot, int count, double divergence){
    double *tmp_coordinates;
    double fitness;
    int total = 0;
    for(int i = 0; i < count; i++){
        tmp_coordinates = robot[i]->get_coordinates();
        std::cout << "Robot[" << i << "]: <" << tmp_coordinates[0] << "; " <<
        tmp_coordinates[1] << ">  fitness: " << robot[i]->get_fitness() << std::endl;
        if(abs(robot[i]->get_fitness()) < divergence)
            total++;
    }
    return total;
}


int main(int argc, char **argv) {
    // simulation args
    double max_velocity = 0.7, w_0 = 0.6, c0 = 0.0;
    double weight[2] = {0.4, 0.9};
    int robot_count = 8;
    int map_size = 150; // center in [0;0] size to each side of map -- 300*300 is total size
    double source[2] = {5, 5}; // acoustic source
    // end of args

    srand(time(NULL));
    std::default_random_engine generator(rand());
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
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
    int printed = 0;
    int index;
    vector<int> done_robots;
    int x = 0;
    bool best_changed;
    double divergence = 0.4;
    if(argc == 2)
        divergence = atof(argv[1]);
    vector<int>::iterator it;

    while(x++ < 1000){
        best_changed = false;
        for(int i = 0; i < robot_count; i++){
            current_fitness = robot[i]->get_fitness();
            if(current_fitness > best_fitness){
                best_fitness = current_fitness;
                tmp_coordinates = robot[i]->get_coordinates();
                global_best[0] = tmp_coordinates[0];
                global_best[1] = tmp_coordinates[1];
                index = i;
                best_changed = true;
            }
        }
        for(int i = 0; i < robot_count; i++){
            current_fitness = robot[i]->update(global_best, source);
            if(abs(current_fitness) < divergence){
                for(it = done_robots.begin(); it < done_robots.end(); it++){
                    if(*it == i)
                        break;
                }
                if(it == done_robots.end())
                    done_robots.push_back(i);
            }
            if(current_fitness > best_fitness){
                best_changed = true;
                best_fitness = current_fitness;
                tmp_coordinates = robot[i]->get_coordinates();
                global_best[0] = tmp_coordinates[0];
                global_best[1] = tmp_coordinates[1];
                index = i;
            }
        }
        if(best_changed)
            cout << "New best position >> step: "<< x << " robot[" << index << "]: <" << global_best[0] << "; " << global_best[1] << ">" << std::endl;
        if(done_robots.size() - printed > 0){
            cout << done_robots.size() - printed <<
            " robot(s) have achieved target with allowed divergence " << divergence <<
            " in step " << x << endl;
            printed = done_robots.size();
        }
        if(printed == robot_count) {
            cout << endl << "Target achieved with allowed divergence " << divergence <<
                 " in " << x << " steps" << endl;
            /*print_coordinates(robot, robot_count);
            cout << endl << "Setting divergence close to zero to test, how many robots will achieve accurate target position" << endl;
            achieved = true;
            divergence = 0.0;*/
            break;
        }
    }
    //cout << "Robot statuses with divergence " << divergence <<
    //" in " << x << " steps" << endl;
    cout << endl;
    int total = print_coordinates(robot, robot_count, divergence);
    cout << endl << total << " robot(s) have achieved target with requested divergence" << endl;
    return 0;
}
