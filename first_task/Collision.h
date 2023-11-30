//
// Created by daniil on 28.11.23.
//

#ifndef TASK_RELAX_COLLISION_H
#define TASK_RELAX_COLLISION_H

#include "../Vector_3d.h"
#include "../second_task/IntegralSpace.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using Vec_speed = Vector_3d<double>;
using Vec_nodes = Vector_3d<int>;

template<std::size_t N_vx = 10, std::size_t N_vy = 10, std::size_t N_vz = 10>
class Collision : IntegralSpace<N_vx, N_vy, N_vz> {
public:
    Vec_nodes node_1;
    Vec_nodes node_2;
    double s;
    double e;
    bool isGood;

private:
    //семинар 2
    int get_center_node(double v, double dv);

    //семинар 2
    Vec_nodes get_vec_center_node(const Vec_speed& v);

    //семинар 1
    Vec_nodes calc_vec_allowed_node(double x, double y, double z);

    //семинар 2
    double get_relative_energy(const Vec_speed& v, const Vec_speed& v_centre);

    //семинар 2
    int find_id_min_node(double v, double dv);

    //семинар 2
    Vec_nodes find_min_node(Vec_speed v);

    //семинар 2
    //конце второго семинара, находим минимальный по расстоянию узел
    Vec_nodes find_second_node(Vec_speed v1, Vec_speed v_mean, const Vec_nodes& sum_ind,
                               double energy_0, bool is_energy0_bigger);

    //семинар 2
    double find_r(double energy_0, Vec_nodes lambda_mu, Vec_nodes lambda_mu_new, Vec_speed v_mean);

public:
    Collision() {}

    //семинар 1
    Collision(std::vector<double>& node_vector, double v_max, double d_max = 1.0);

    //семинар 1-2
    std::tuple<Vec_nodes, Vec_nodes, Vec_nodes, Vec_nodes, double> get_new_v();

    bool operator == (const Collision& collision) const;

//    std::ostream& operator << (std::ostream& out, const Collision& collision);
};

#endif //TASK_RELAX_COLLISION_H
