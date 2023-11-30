//
// Created by daniil on 28.11.23.
//

#ifndef TASK_RELAX_INTEGRALSPACE_H
#define TASK_RELAX_INTEGRALSPACE_H

#include "../Vector_3d.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using Vec_v = Vector_3d<double>;
using Vec_nodes = Vector_3d<int>;

template<std::size_t N_vx, std::size_t N_vy, std::size_t N_vz>
struct IntegralSpace {
public:
    double d_max = 1.0;  // в площадях d^2, где d - диаметр молекулы
    double v_cut = 5.0;
    Vec_v vec_dv = calc_vec_dv();
public:

    //семинар 1
    double calc_d_v(int n) const {
        return 2 * v_cut / n;
    }

    //семинар 1
    Vec_v calc_vec_dv() const {
        return Vec_v(calc_d_v(N_vx), calc_d_v(N_vy), calc_d_v(N_vz));
    }

    //семинар 2
    double v_iter(int iteration, double d_v) const {
        return -v_cut + (iteration + 0.5) * d_v;
    }

    //семинар 2
    Vec_v iter_vec_v(const Vec_nodes& node) const {
        Vec_v v;
        v.x = v_iter(node.x, vec_dv.x);
        v.y = v_iter(node.y, vec_dv.y);
        v.z = v_iter(node.z, vec_dv.z);
        return v;
    }

    //семинар 1
    double get_v_cut() const {
        return this->v_cut;
    }

    //семинар 1
    void set_v_cut(double v_max) {
        this->v_cut = v_max;
        this->vec_dv = calc_d_v();
    }

    IntegralSpace() {}
    IntegralSpace(double v_max) : v_cut(v_max) {}
    IntegralSpace(double v_max, double d_max) : v_cut(v_max), d_max(d_max) {}
};


#endif //TASK_RELAX_INTEGRALSPACE_H
