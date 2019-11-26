#ifndef SFC_ROBOT_H
#define SFC_ROBOT_H

using namespace std;

class Robot {
public:
    Robot(double, double, int, double *);
    ~Robot();
    void count(const double *);

private:
    double get_weight(double, int, double);
    void func(double, double);

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
};


#endif //SFC_ROBOT_H
