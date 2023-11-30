//
// Created by daniil on 30.11.23.
//

#include "Collision.h"


//семинар 1
template<std::size_t N_vx, std::size_t N_vy, std::size_t N_vz>
Collision<N_vx, N_vy, N_vz>::Collision(std::vector<double>& node_vector,
                                       double v_max,
                                       double d_max)
                                       : IntegralSpace<N_vx, N_vy, N_vz>(v_max, d_max) {

    node_1 = calc_vec_allowed_node(node_vector[0], node_vector[1], node_vector[2]);
    node_2 = calc_vec_allowed_node(node_vector[3], node_vector[4], node_vector[5]);

    if (this->calc_v(node_1).norm() < this->v_cut &&
        this->calc_v(node_2).norm() < this->v_cut) {
        isGood = true;
    } else {
        isGood = false;
    }
    s = node_vector[6] * this->d_max * this->d_max;
    e = node_vector[7] * 2 * M_PI;
}


//семинар 2
template<std::size_t N_vx, std::size_t N_vy, std::size_t N_vz>
int Collision<N_vx, N_vy, N_vz>::get_center_node(double v, double dv) {
    //N = 2* v_cut / dv
    int nd_center = std::lround(v / dv + 0.5 * 2 * this->v_cut / dv - 0.5);
    return nd_center;
}


//семинар 2
template<std::size_t N_vx, std::size_t N_vy, std::size_t N_vz>
Vec_nodes Collision<N_vx, N_vy, N_vz>::get_vec_center_node(const Vec_speed& v) {
    Vec_nodes node_center;
    node_center.x = get_center_node(v.x, this->vec_dv.x);
    node_center.y = get_center_node(v.y, this->vec_dv.y);
    node_center.z = get_center_node(v.z, this->vec_dv.z);
    return node_center;
}


//семинар 1
template<std::size_t N_vx, std::size_t N_vy, std::size_t N_vz>
Vec_nodes Collision<N_vx, N_vy, N_vz>::calc_vec_allowed_node(double x, double y, double z) {
    Vec_speed v_grid = Vec_speed(x, y, z);
    Vec_speed v = v_grid * 2 * this->get_v_cut() - this->get_v_cut();
    Vec_nodes ind = get_vec_center_node(v);
    return ind;
}


//семинар 2
template<std::size_t N_vx, std::size_t N_vy, std::size_t N_vz>
double Collision<N_vx, N_vy, N_vz>::get_relative_energy(const Vec_speed& v, const Vec_speed& v_centre) {
    return (v - v_centre).norm_squared();
}


//семинар 2
template<std::size_t N_vx, std::size_t N_vy, std::size_t N_vz>
int Collision<N_vx, N_vy, N_vz>::find_id_min_node(double v, double dv) {
    //N = 2* v_cut / dv
    int node_id = std::floor((v + this->get_v_cut()) / dv - 0.5);  // TODO: устранить повтор в find_near_i
    return node_id;
}


//семинар 2
template<std::size_t N_vx, std::size_t N_vy, std::size_t N_vz>
Vec_nodes Collision<N_vx, N_vy, N_vz>::find_min_node(const Vec_speed v) {
    Vec_nodes min_node;
    min_node.x = find_id_min_node(v.x, this->vec_dv.x);
    min_node.y = find_id_min_node(v.y, this->vec_dv.y);
    min_node.z = find_id_min_node(v.z, this->vec_dv.z);
    return min_node;
}


//семинар 2
template<std::size_t N_vx, std::size_t N_vy, std::size_t N_vz>
//конце второго семинара, находим минимальный по расстоянию узел
Vec_nodes Collision<N_vx, N_vy, N_vz>::find_second_node(Vec_speed v1, Vec_speed v_mean, const Vec_nodes& sum_ind,
                           double energy_0, bool is_energy0_bigger) {
    Vec_nodes center = find_min_node(v1);  // cenetre index
    Vec_nodes best_ind;
    double min = 1'000'000'000;

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                double vec_cut = this->get_v_cut();

                Vec_nodes cur_ind = center + Vec_nodes(i, j, k);
                Vec_speed cur_vec = this->calc_v(cur_ind);

                // после разлета узел
                // суммарный вектор после разлеты должен быть тем же,
                // поэтому так считаем paired_ind
                Vec_nodes paired_ind = sum_ind - cur_ind;
                Vec_speed paired_vec = this->calc_v(paired_ind);

                if ((cur_vec.norm_squared() < vec_cut*vec_cut) &&
                    (paired_vec.norm_squared() < vec_cut*vec_cut)) {
                    double cur_energy = get_relative_energy(cur_vec, v_mean);

                    if (((energy_0 > cur_energy) == is_energy0_bigger) &&
                        (energy_0 != cur_energy)) {
                        double distance = (v1 - cur_vec).norm_squared();
                        if (distance < min) { // хотим найти min
                            min = distance;
                            best_ind = cur_ind;
                        }
                    }
                }
            }
        }
    }
    if (min == 1'000'000'000) {
        this->isGood = false;
    }
    return best_ind;
}


//семинар 2
template<std::size_t N_vx, std::size_t N_vy, std::size_t N_vz>
double Collision<N_vx, N_vy, N_vz>::find_r(double energy_0,
                                           Vec_nodes lambda_mu,
                                           Vec_nodes lambda_mu_new,
                                           Vec_speed v_mean) {

    double energy_1 = get_relative_energy(this->calc_v(lambda_mu), v_mean);
    double energy_2 = get_relative_energy(this->calc_v(lambda_mu_new), v_mean);
    return (energy_0 - energy_1) / (energy_2 - energy_1);
}


//семинар 1-2
template<std::size_t N_vx, std::size_t N_vy, std::size_t N_vz>
std::tuple<Vec_nodes, Vec_nodes, Vec_nodes, Vec_nodes, double> Collision<N_vx, N_vy, N_vz>::get_new_v() {
    Vec_nodes lambda, lambda_new, mu, mu_new;
    double r;
    if (isGood) {
        // Находим скорости полсе столкновения
        double theta = 2.0 * std::acos(std::sqrt(s));
        Vec_speed v1 = this->calc_v(node_1);
        Vec_speed v2 = this->calc_v(node_2);
        Vec_speed v_relative = v2 - v1;

        double v_relative_norm = v_relative.norm();
        double v_relative_xy = v_relative.x_mult_y();

        Vec_speed new_v_relative;
        if (v_relative_xy == 0) {
            new_v_relative.x = v_relative_norm * std::sin(e) * std::sin(theta);
            new_v_relative.y = v_relative_norm * std::cos(e) * std::sin(theta);
            new_v_relative.z = v_relative_norm * std::cos(theta);
        } else {
            new_v_relative.x = v_relative.x * std::cos(theta)
                               - (v_relative.x*v_relative.z/v_relative_xy) * std::cos(e) * std::sin(theta)
                               + v_relative_norm * v_relative.y / v_relative_xy * std::sin(e) * std::sin(theta);

            new_v_relative.y = v_relative.y * std::cos(theta)
                               - (v_relative.y * v_relative.z / v_relative_xy) * std::cos(e) * std::sin(theta)
                               - v_relative_norm * v_relative.x / v_relative_xy * std::sin(e) * std::sin(theta);

            new_v_relative.z = v_relative.z * std::cos(theta)
                               + v_relative_xy * std::cos(e) * std::sin(theta);
        }
        //новые разлетные скорости
        Vec_speed u1 = (v1 + v2) / 2 - new_v_relative / 2;
        Vec_speed u2 = (v1 + v2) / 2 + new_v_relative / 2;


        // Находим аппроксимипующие скорости (индексы) и r
        // семинар 2
        Vec_nodes node_center_1 = get_vec_center_node(u1);
        Vec_nodes node_center_2 = get_vec_center_node(u2);
        Vec_speed u_center_1 = this->calc_v(node_center_1);
        Vec_speed u_center_2 = this->calc_v(node_center_2);

        if ((u_center_1.norm() >= this->v_cut)
            || (u_center_2.norm() >= this->v_cut)) {
                isGood = false;
        } else {
                Vec_speed v_mean = (v1 + v2) / 2;  // скорость центра масс
                double energy_0 = get_relative_energy(v1, v_mean);
                double energy_center = get_relative_energy(u_center_1, v_mean);
                if (energy_0 == energy_center) {
                    lambda = node_center_1;
                    lambda_new = node_center_1;
                    mu = node_center_2;
                    mu_new = node_center_2;
                    r = 1;
                } else {
                    if (energy_0 < energy_center) {
                        lambda_new = node_center_1;
                        mu_new = node_center_2;
                        lambda = this->find_second_node(
                                u1,
                                v_mean,
                                node_center_1 + node_center_2,
                                energy_0,
                                true
                        );
                        mu = lambda_new + mu_new - lambda;
                    } else {
                        lambda = node_center_1;
                        mu = node_center_2;
                        lambda_new = this->find_second_node(
                                u1,
                                v_mean,
                                node_center_1 + node_center_2,
                                energy_0,
                                false);
                        mu_new = lambda + mu - lambda_new;
                    }
                    if (isGood) {
                        r = find_r(energy_0, lambda, lambda_new, v_mean);
                    }
                }
        }
        // TODO: cтереть после проверки
        // if (isGood) {
        //     double energy_0 = v1.squared() + v2.squared();
        //     double energy_1 = calc_v(lambda).squared() + calc_v(mu).squared();
        //     double energy_2 = calc_v(lambda_new).squared() + calc_v(mu_new).squared();
        //     double r_add = (energy_0 - energy_1) / (energy_2 - energy_1);
        //     Vec_speed v_lambda = calc_v(lambda);
        //     Vec_speed v_mu = calc_v(mu);
        //     Vec_speed v_lambda_new = calc_v(lambda_new);
        //     Vec_speed v_mu_new = calc_v(mu_new);
        //     std::cout << energy_0 << " " << energy_1 << " " << energy_2 << '\n';
        //     std::cout << v_lambda + v_mu - v1 - v2 << v_lambda_new + v_mu_new - v1 - v2<< '\n';
        //     std::cout << r << " " << r_add << "\n\n";
        // }
    }
    return std::make_tuple(lambda, mu, lambda_new, mu_new, r);
}


template<std::size_t N_vx, std::size_t N_vy, std::size_t N_vz>
bool Collision<N_vx, N_vy, N_vz>::operator == (const Collision& collision) const {
    if (this->node_2 == collision.node_2)
        return true;
    return false;
}


//template<std::size_t N_vx, std::size_t N_vy, std::size_t N_vz>
//std::ostream& operator << (std::ostream& out, const Collision<N_vx, N_vy, N_vz>& collision) {
//    out << collision->calc_v(collision.node_1) << "\t"
//        << collision->calc_v(collision.node_2) << "\t"
//        << collision.s << "\t"
//        << collision.e;
//    return out;
//}
