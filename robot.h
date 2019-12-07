#ifndef SFC_ROBOT_H
#define SFC_ROBOT_H

using namespace std;

class Robot {
public:
    Robot(double, double, int, const double *, const double *, const double *, const double *, const double *);
    ~Robot();
    double update(const double *, const double *source);
    /**
     * Getter for setting global best coordinates
     * @return
     */
    double *get_coordinates();
    double get_fitness();

private:
    /**
     * Compute weight for k+1 step
     * @param w - current weight
     * @param i - index of coordinates (whether it is x or y)
     * @param global_best - global best position
     * @return weight for coordinates
     */
    void set_weight(int i, double global_best);

    /**
    * Compute robot fitness function
    * @param source - source of acoustic signal
    * @return fitness function
    */
    void func(const double *source);
    void update_deltas();

    // Map size
    double x_min, x_max, y_min, y_max;
    // initial weight; max_velocity
    double w_0, max_velocity;

    double r1, r2;

    double weight_limit[2];
    double weight[2];
    double velocity[2];
    double position[2];
    double best_pos[2];
    int delta_f, delta_f_prev;
    double robot_fitness;
    double robot_fitness_prev;
};


#endif //SFC_ROBOT_H
