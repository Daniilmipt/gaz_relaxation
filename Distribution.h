//
// Created by daniil on 28.11.23.
//

#ifndef TASK_RELAX_DISTRIBUTION_H
#define TASK_RELAX_DISTRIBUTION_H

#include "Vector_3d.h"
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
const static double NORM_COEF = 1 / (2 * M_PI) / sqrt(2 * M_PI);

struct DistributionFunc {
    double temp = 1.0;
    DistributionFunc(double temperature) : temp(temperature) {}

    virtual double operator() (const Vec_speed&) = 0;
};

struct Maxwell : DistributionFunc {
    double operator() (const Vec_speed& v) override {
        return NORM_COEF * std::exp(v.norm_squared() / (-2.0 * temp))
                            / sqrt(std::pow(temp, 3));
    }
};
struct TwoGauss : DistributionFunc {
    Vec_speed u;
    TwoGauss(double temperature = 1.0,
             Vec_speed shift = {2.0, 0.0, 0.0}) :
                                                             DistributionFunc(temperature),
                                                             u(std::move(shift)) {}

    double operator() (const Vec_speed& v) override {
        return NORM_COEF * 0.5 *
                        (
                                std::exp((v - u).norm_squared() / (-2.0 * temp))
                                + std::exp((v + u).norm_squared() / (-2.0 * temp))
                        ) / sqrt(std::pow(temp, 3));
    }
};

#endif //TASK_RELAX_DISTRIBUTION_H
