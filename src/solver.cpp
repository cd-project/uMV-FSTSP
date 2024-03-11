﻿//
// Created by cuong on 18/01/2024.
//
#include "../include/solver.h"

#include <chrono>
#include <iostream>
#include <vector>
#include <unordered_set>

Sortie::Sortie(int t, int la, int re, std::vector<int> &middle) {
    target = t;
    l = la;
    r = re;
    phi = middle;
}

Sortie::Sortie(int h) {
    target = h;
}

//Result::Result(double solver_cost, double recalculated_cost, std::vector<Sortie>& st) {
//    cost = c;
//    sortie = st;
//}
Result::Result(double c, std::vector<Sortie> &st) {
    cost = c;
    sortie = st;
}

Result::Result(double c, double re_c, double t, std::vector<Sortie> &st) {
    cost = c;
    recalculated_cost = re_c;
    sortie = st;
    time_spent = t;
}
inline bool exist(const std::vector<int>& vec, int element) {
    // Use std::find to search for the element in the vector
    return std::find(vec.begin(), vec.end(), element) != vec.end();
}
std::vector<std::pair<std::vector<int>, std::vector<int> > > generateSetsAndComplements(
        const std::vector<int> &elements) {
    auto n = elements.size();
    std::vector<std::pair<std::vector<int>, std::vector<int> > > result;

    // Total number of subsets is 2^n
    int totalSubsets = 1 << n;

    for (int i = 0; i < totalSubsets; ++i) {
        std::vector<int> setS, setT;

        for (int j = 0; j < n; ++j) {
            // Check if jth bit is set in the binary representation of i
            if (i & (1 << j)) {
                setS.push_back(elements[j]);
            } else {
                setT.push_back(elements[j]);
            }
        }
        if (setS.size() != 0 && setT.size() != 0) {
            result.push_back(std::make_pair(setS, setT));
        }
    }
    //    result.push_back(std::make_pair(elements, std::vector<int>{}));

    return result;
}

void setPrint(std::vector<int> &set) {
    for (int i: set) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
}


// Paper: https://drive.google.com/file/d/1CpYCse--JWmrnBY566Obe8IMpcClTpnI/view?usp=sharing

//Result Solver::uMVFSTSPSolver(int n_thread, int e) {
//    try {
//        auto tau = instance->tau;
//        auto d = instance->tau_prime;
//        auto dtl = e;
//        auto sl = 1, sr = 1;
//        auto n = instance->num_node;
//        auto s = 0, t = n;
//        auto c_prime = instance->c_prime;
//        std::vector<int> c_prime_0;
//        c_prime_0.push_back(0);
//        for (int i : c_prime) {
//            c_prime_0.push_back(i);
//        }
//        c_prime_0.push_back(n);
//        std::cout << "Printing number of nodes: " << n << std::endl;
//        std::vector<int> C;
//        std::vector<int> V;
//        for (int i = 0; i < n + 1; i++) {
//            if (i == 0 || i == n) {
//                V.push_back(i);
//            }
//            else {
//                V.push_back(i);
//                C.push_back(i);
//            }
//        }
//        std::vector<int> c_s;
//        std::vector<int> c_t;
//        for (int i = 0; i < n + 1; i++) {
//            if (i == 0) {
//                c_s.push_back(i);
//            }
//            else if (i == n) {
//                c_t.push_back(i);
//            }
//            else {
//                c_s.push_back(i);
//                c_t.push_back(i);
//            }
//        }
//
//        std::cout << std::endl;
//        GRBEnv env;
//        GRBModel model(env);
//        // y: (i, j) in A, truck route
//        auto** y = new GRBVar * [n + 1];
//        for (int i : c_s) {
//            y[i] = reinterpret_cast<GRBVar*>(new GRBVar * [n + 1]);
//            for (int j : V) {
//                y[i][j] = model.addVar(0, 1, 0.0, GRB_INTEGER, "y_" + std::to_string(i) + "_" + std::to_string(j));
//                if (i == j) {
//                    model.addConstr(y[i][j] == 0);
//                }
//            }
//        }
//        model.addConstr(y[s][t] == 0);
//
//
//        auto** x = new GRBVar * [n + 1];
//        for (int i : c_s) {
//            x[i] = reinterpret_cast<GRBVar*>(new GRBVar * [n + 1]);
//            for (int j : V) {
//                x[i][j] = model.addVar(0, 1, 0.0, GRB_BINARY, "x_" + std::to_string(i) + "_" + std::to_string(j));
//                if (i == j) {
//                    model.addConstr(x[i][j] == 0);
//                }
//
//            }
//        }
//
//        model.addConstr(x[s][t] == 0);
//
//        // gamma_h_ij
//        auto*** gamma = new GRBVar * *[n + 1];
//        for (int h : C) {
//            gamma[h] = reinterpret_cast<GRBVar**>(new GRBVar * *[n + 1]);
//            for (int i : c_s) {
//                gamma[h][i] = reinterpret_cast<GRBVar*>(new GRBVar * [n + 1]);
//                for (int j : V) {
//                    gamma[h][i][j] = model.addVar(0, 1, 0.0, GRB_BINARY, "gamma_" + std::to_string(h) + "_" + std::to_string(i) + "_" + std::to_string(j));
//                    for (int heavy : instance->heavy) {
//                        if (h == heavy) {
//                            model.addConstr(gamma[h][i][j] == 0);
//                        }
//                    }
//                    if (i == j) {
//                        model.addConstr(gamma[h][i][j] == 0);
//                    }
//                }
//            }
//            model.addConstr(gamma[h][s][t] == 0);
//        }
//
//        std::vector<GRBVar> theta(n + 1);
//        for (int h : V) {
//            theta[h] = model.addVar(0, 1, 0.0, GRB_BINARY, "theta_" + std::to_string(h));
//            for (auto heavy : instance->heavy) {
//                if (h == heavy) {
//                    model.addConstr(theta[h] == 0);
//                }
//            }
//            if (h == s || h == t) {
//                model.addConstr(theta[h] == 0);
//            }
//        }
//        auto** omega = new GRBVar * [n + 1];
//        for (int h : C) {
//            omega[h] = reinterpret_cast<GRBVar*>(new GRBVar * [n + 1]);
//            for (int i : V) {
//                omega[h][i] = model.addVar(0, 1, 0.0, GRB_BINARY, "omega_" + std::to_string(h) + "_" + std::to_string(i));
//
//                for (int heavy : instance->heavy) {
//                    if (h == heavy) {
//                        model.addConstr(omega[h][i] == 0);
//                    }
//                }
//                if (h == i || i == t) {
//                    model.addConstr(omega[h][i] == 0);
//                }
//            }
//        }
//        auto** delta = new GRBVar * [n + 1];
//        for (int h : C) {
//            delta[h] = reinterpret_cast<GRBVar*>(new GRBVar * [n + 1]);
//            for (int j : V) {
//                delta[h][j] = model.addVar(0, 1, 0.0, GRB_BINARY, "delta_" + std::to_string(h) + "_" + std::to_string(j));
//                for (int heavy : instance->heavy) {
//                    if (h == heavy) {
//                        model.addConstr(delta[h][j] == 0);
//                    }
//                }
//                if (h == j || j == s) {
//                    model.addConstr(delta[h][j] == 0);
//                }
//            }
//        }
//
//        std::vector<GRBVar> sigma(n + 1);
//        for (int h : c_t) {
//            sigma[h] = model.addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "sigma_" + std::to_string(h));
//
//            for (int heavy : instance->heavy) {
//                if (h == heavy) {
//                    model.addConstr(sigma[h] == 0);
//                }
//            }
//        }
//        auto** phi = new GRBVar * [n + 1];
//        for (int h : C) {
//            phi[h] = reinterpret_cast<GRBVar*>(new GRBVar * [n + 1]);
//            for (int i : c_s) {
//                phi[h][i] = model.addVar(0, 1, 0.0, GRB_BINARY, "phi_" + std::to_string(h) + "_" + std::to_string(i));
//                for (int heavy : instance->heavy) {
//                    if (heavy == h) {
//                        model.addConstr(phi[h][i] == 0);
//                    }
//                }
//                if (h == i) {
//                    model.addConstr(phi[h][i] == 0);
//                }
//            }
//        }
//        GRBLinExpr objective;
//        for (int i : c_s) {
//            for (int j : c_t) {
//                objective += tau[i][j] * y[i][j];
//            }
//        }
//
//        for (int h : C) {
//            objective += (sl + sr) * theta[h];
//            objective -= sl * omega[h][s];
//        }
//        for (int h : c_t) {
//            objective += sigma[h];
//        }
//        GRBLinExpr sum_theta;
//        // Constraint 1
//        GRBLinExpr lhs_1, rhs_1;
//        for (int j : c_t) {
//            lhs_1 += y[s][j];
//        }
//        for (int i : c_s) {
//            rhs_1 += y[i][t];
//        }
//        model.addConstr(lhs_1, GRB_GREATER_EQUAL, 1, "C1_LHS");
//        model.addConstr(rhs_1, GRB_EQUAL, 1, "C1_RHS");
//
//        // Constraint 2
//        for (int i : C) {
//            GRBLinExpr lhs_2, rhs_2;
//            for (int j : c_t) {
//                lhs_2 += y[i][j];
//            }
//            for (int j : c_s) {
//                rhs_2 += y[j][i];
//
//            }
//            model.addConstr(lhs_2, GRB_EQUAL, rhs_2, "C2_EQUAL_i=" + std::to_string(i));
//            //        model.addConstr(lhs_2, GRB_LESS_EQUAL, 1, "C2_LHS_LEQ_1_i="+ std::to_string(i));
//            //        model.addConstr(rhs_2, GRB_LESS_EQUAL, 1, "C2_RHS_LEQ_1_i="+ std::to_string(i));
//        }
//        auto setAndCompss = generateSetsAndComplements(C);
//        for (auto& pair : setAndCompss) {
//            auto S = pair.first;
//            auto S_comp = pair.second;
//            S_comp.push_back(s);
//            S_comp.push_back(t);
//            GRBLinExpr lhs, rhs;
//            std::string cname = "C3";
//            for (int i : S) {
//                cname += "_" + std::to_string(i);
//                for (int j : S_comp) {
//                    if (j != s && i != t) {
//                        lhs += y[i][j];
//                    }
//                }
//            }
//            for (int h : S) {
//                rhs += theta[h];
//            }
//            cname += "_(";
//            for (int j : S_comp) {
//                cname += "_" + std::to_string(j);
//            }
//            cname += ")";
//            model.addConstr(S.size() * lhs >= S.size() - rhs, cname);
//        }
//        // Constraint 3
//        auto setAndComps = generateSetsAndComplements(C);
//        for (auto& set : setAndComps) {
//            auto S = set.first;
//            if (S.size() < 2) {
//                continue;
//            }
//            if (S.size() == 2 && S[0] == s && S[1] == t) {
//                continue;
//            }
//            std::string cname = "C3";
//            GRBLinExpr sum3;
//            for (auto h : S) {
//
//                if (h == s || h == t) {
//                    continue;
//                }
//                else {
//                    sum3 += 1 - theta[h];
//                }
//
//            }
//            for (int h2 : C) {
//                GRBLinExpr sum_gamma;
//                for (auto i : S) {
//                    cname += "_" + std::to_string(i);
//                    if (i != t) {
//                        for (auto j : S) {
//                            sum_gamma += gamma[h2][i][j];
//                        }
//                    }
//                    //                    model.addConstr(sum_gamma <= sum3);
//                }
//            }
//        }
//        //         Constraint 4
//        for (int h : C) {
//            GRBLinExpr lhs_4;
//            std::string cname = "C4_h=" + std::to_string(h);
//            for (int j : c_t) {
//                lhs_4 += gamma[h][s][j];
//            }
//            model.addConstr(lhs_4, GRB_EQUAL, omega[h][s], cname);
//        }
//
//        // Constraint 5
//        for (int h : C) {
//            GRBLinExpr lhs_5;
//            std::string cname = "C5_h=" + std::to_string(h);
//
//            for (int i : c_s) {
//                lhs_5 += gamma[h][i][t];
//            }
//            model.addConstr(lhs_5, GRB_EQUAL, delta[h][t], cname);
//        }
//        // Constraint 6
//        for (int i : c_s) {
//            for (int h : C) {
//                std::string cname = "C6_i=" + std::to_string(i) + "_h=" + std::to_string(h);
//                GRBLinExpr sum1, sum2;
//                for (int j : V) {
//                    sum1 += gamma[h][i][j];
//                }
//
//                for (int j : c_s) {
//                    sum2 += gamma[h][j][i];
//                }
//                model.addConstr(sum1 - sum2, GRB_EQUAL, omega[h][i] - delta[h][i], cname);
//            }
//        }
//        // Constraint 7
////    for (int j:c_t) {
////        std::string cname = "C7_s_j=" + std::to_string(j);
////        model.addConstr(y[s][j] + x[s][j], GRB_LESS_EQUAL, 1, cname);
////    }
//        // Constraint 8
//        for (int i : c_s) {
//            std::string cname = "C8_i=" + std::to_string(i) + "_t";
//            model.addConstr(y[i][t] + x[i][t], GRB_LESS_EQUAL, 1, cname);
//        }
//        // Constraint 9
//        for (int i : C) {
//            for (int j : C) {
//                std::string cname = "C9_i=" + std::to_string(i) + "_j=" + std::to_string(j);
//                model.addConstr(x[i][j] + x[j][i], GRB_LESS_EQUAL, 1, cname);
//            }
//        }
//
//        // Constraint 10
//        for (int h : C) {
//            GRBLinExpr sum;
//            std::string cname = "C10_h=" + std::to_string(h);
//            for (int j : c_t) {
//                sum += y[h][j];
//            }
//            model.addConstr(sum + theta[h], GRB_GREATER_EQUAL, 1, cname);
//        }
//        // Constraint 11
//        for (int i : c_s) {
//            for (int j : V) {
//                std::string cname = "C11_i=" + std::to_string(i) + "_j=" + std::to_string(j);
//                GRBLinExpr sum;
//                for (int h : c_prime) {
//                    sum += gamma[h][i][j];
//                }
//                model.addConstr(sum, GRB_LESS_EQUAL, y[i][j], cname);
//            }
//        }
//
//        // Constraint 12
//        for (int h : C) {
//            GRBLinExpr sum1, sum2;
//
//            for (int i : V) {
//                if (i != h && i != t) {
//                    sum1 += omega[h][i];
//                }
//            }
//            for (int j : V) {
//                if (j != s && j != h) {
//                    sum2 += delta[h][j];
//                }
//            }
//            model.addConstr(sum1, GRB_EQUAL, theta[h], "C12_RHS_EQUAL_h=" + std::to_string(h));
//            model.addConstr(sum2, GRB_EQUAL, theta[h], "C12_LHS_EQUAL_h=" + std::to_string(h));
//            model.addConstr(sum1 == sum2, "C12_EQUAL_h=" + std::to_string(h));
//        }
//        // Constraint 13
//        for (int i : c_s) {
//            for (int j : V) {
//                std::string cname = "C13_i=" + std::to_string(i) + "_j=" + std::to_string(j);
//                model.addConstr(x[i][j], GRB_LESS_EQUAL, theta[i] + theta[j], cname);
//            }
//        }
//        //        // Constraint 14
//        for (int i : c_s) {
//            for (int j : V) {
//                GRBLinExpr sum;
//                if (j != s && i != t && j != t) {
//                    sum += omega[j][i];
//                }
//                if (i != t && i != s) {
//                    sum += delta[i][j];
//                }
//
//                std::string cname = "C14_i=" + std::to_string(i) + "_j=" + std::to_string(j);
//                model.addConstr(x[i][j] <= sum, cname);
//            }
//        }
//        // Constraint 15
//        for (int i : c_s) {
//            GRBLinExpr sum1, sum2;
//            std::string cname = "C15_i=" + std::to_string(i);
//            for (int j : V) {
//                sum1 += x[i][j];
//            }
//            for (int h : c_prime) {
//                sum2 += omega[h][i];
//            }
//            sum2 += theta[i];
//            model.addConstr(sum1, GRB_EQUAL, sum2, "C15_LHS_RHS_EQUAL_i=" + std::to_string(i));
//            //        model.addConstr(sum1, GRB_LESS_EQUAL, 1, "C15_LHS_LEQ_1_i="+ std::to_string(i));
//            //        model.addConstr(sum2, GRB_LESS_EQUAL, 1, "C15_RHS_LEQ_1_i="+ std::to_string(i));
//        }
//        // Constraint 16
//        for (int j : c_t) {
//            GRBLinExpr sum1, sum2;
//            for (int i : c_s) {
//                sum1 += x[i][j];
//            }
//
//            for (int h : c_prime) {
//                sum2 += delta[h][j];
//            }
//            sum2 += theta[j];
//            model.addConstr(sum1, GRB_EQUAL, sum2, "C16_LHS_RHS_EQUAL_j=" + std::to_string(j));
//            //        model.addConstr(sum1, GRB_LESS_EQUAL, 1, "C16_LHS_LEQ_1_i="+ std::to_string(j));
//            //        model.addConstr(sum2, GRB_LESS_EQUAL, 1, "C16_RHS_LEQ_1_i="+ std::to_string(j));
//        }
//        // Constraint 17
//        for (int h : c_prime) {
//            GRBLinExpr sum;
//            std::string cname = "C17_h=" + std::to_string(h);
//            for (int i : c_s) {
//                for (int j : V) {
//                    sum += tau[i][j] * gamma[h][i][j];;
//                }
//            }
//            model.addConstr(sum, GRB_LESS_EQUAL, (dtl - sr) * theta[h], cname);
//        }
//        // Constraint 18
//        for (int h : c_prime) {
//            GRBLinExpr sum1;
//            GRBLinExpr sum2;
//            GRBLinExpr sum3;
//            std::string c18_name = "C18_h=" + std::to_string(h);
//            std::string c19_name = "C19_h=" + std::to_string(h);
//
//            for (int i : c_s) {
//                sum1 += d[i][h] * omega[h][i];
//            }
//            for (int j : V) {
//                sum2 += d[h][j] * delta[h][j];
//            }
//
//            for (int i : c_s) {
//                for (int j : V) {
//                    sum3 += tau[i][j] * gamma[h][i][j];
//                }
//            }
//            model.addConstr(sum1 + sum2, GRB_LESS_EQUAL, (dtl - sr) * theta[h], c18_name);
//            model.addConstr(sum1 + sum2 - sum3, GRB_LESS_EQUAL, sigma[h], c19_name);
//        }
//        // sum phi constraint
//        for (int h : C) {
//            for (int i : c_s) {
//                for (int j : c_s) {
//                    model.addConstr(gamma[h][i][j] + gamma[h][j][i] <= 1);
//                }
//            }
//        }
//        for (int h : C) {
//            GRBLinExpr lhs, rhs;
//            for (int i : c_s) {
//                for (int j : V) {
//                    lhs += gamma[h][i][j];
//                }
//            }
//            for (int i : V) {
//                if (i != t) {
//                    rhs += omega[h][i];
//                }
//                if (i != s && i != t) {
//                    rhs += phi[h][i];
//                }
//                rhs += delta[h][i];
//            }
//            model.addConstr(lhs <= rhs - 1);
//        }
//        auto s_c_V = generateSetsAndComplements(V);
//        for (auto& pair : s_c_V) {
//            auto S = pair.first;
//            if (S.size() < 2) {
//                continue;
//            }
//            else {
//                for (int h : C) {
//                    GRBLinExpr s1, s2;
//                    for (int i : S) {
//                        if (i != t) {
//                            s2 += omega[h][i];
//                        }
//                        if (i != s) {
//                            s2 += delta[h][i];
//                        }
//                        if (i != s && i != t) {
//                            s2 += phi[h][i];
//                        }
//                        for (int j : S) {
//                            if (i != j && i != t && j != s) {
//                                s1 += gamma[h][i][j];
//                            }
//                        }
//                    }
//                    //                model.addConstr(s1 <= s2); // render the model infeasible. what can be the fix?
//                }
//            }
//        }
//        // phi constraint
//        for (int h : C) {
//            for (int i : c_s) {
//                GRBLinExpr sum;
//                for (int j : c_t) {
//                    sum += gamma[h][i][j];
//                }
//                model.addConstr(sum == phi[h][i] + omega[h][i]);
//                model.addConstr(phi[h][i] + omega[h][i] <= 1);
//                model.addConstr(sum <= 1);
//            }
//        }
//        for (int h : C) {
//            GRBLinExpr sum_gamma, sum_sortie;
//            for (int i : c_s) {
//                for (int j : V) {
//                    sum_gamma += gamma[h][i][j];
//                }
//            }
//            for (int i : V) {
//                if (i != t) {
//                    sum_sortie += omega[h][i];
//                }
//                if (i != s) {
//                    sum_sortie += delta[h][i];
//                }
//                if (i != s && i != t) {
//                    sum_sortie += phi[h][i];
//                }
//            }
//            //        model.addConstr(sum_gamma <= sum_sortie - 1);
//        }
//        model.setObjective(objective, GRB_MINIMIZE);
//        //    model.set(GRB_IntParam_Threads, n_thread);
//        model.update();
//        model.write("model.lp");
//        model.optimize();
//        std::cout << "Truck arcs: " << std::endl;
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (y[i][j].get(GRB_DoubleAttr_X) >= 1) {
//                    std::cout << i << " " << j << " " << y[i][j].get(GRB_DoubleAttr_X) << std::endl;
//                }
//            }
//        }
//        int theta_cnt = 0;
//        std::cout << "Theta:" << std::endl;
//        for (int h : C) {
//            if (theta[h].get(GRB_DoubleAttr_X) == 1) {
//                theta_cnt++;
//                std::cout << "Theta_" << h << " = " << theta[h].get(GRB_DoubleAttr_X) << std::endl;
//            }
//        }
//        std::cout << "Drone arcs:" << std::endl;
//        for (int i : c_s) {
//            for (int j : c_t) {
//                if (i != j && x[i][j].get(GRB_DoubleAttr_X) == 1) {
//                    std::cout << i << " " << j << std::endl;
//                }
//            }
//        }
//        std::cout << "Gamma:" << std::endl;
//        for (int h : C) {
//            for (int i : c_s) {
//                for (int j : c_t) {
//                    if (gamma[h][i][j].get(GRB_DoubleAttr_X) == 1) {
//                        std::cout << "gamma_" << h << "_" << i << "_" << j << "= 1" << std::endl;
//                    }
//                }
//            }
//        }
//        std::cout << "Sigma:" << std::endl;
//        for (int h : c_t) {
//            std::cout << "Sigma_" << h << " = " << sigma[h].get(GRB_DoubleAttr_X) << std::endl;
//        }
//        for (int h : C) {
//            if (theta[h].get(GRB_DoubleAttr_X) == 1) {
//                std::cout << "middle of " << h << " sortie are: ";
//                for (int i : c_s) {
//                    if (phi[h][i].get(GRB_DoubleAttr_X) == 1) {
//                        std::cout << i << " ";
//                    }
//                }
//                std::cout << std::endl;
//            }
//        }
//        std::cout << "Objective: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
//        std::cout << "Number of sorties: " << theta_cnt << std::endl;
//        std::vector<Sortie> st;
//        for (int h : C) {
//            if (theta[h].get(GRB_DoubleAttr_X) == 1) {
//                int start, end;
//                for (int i : c_s) {
//                    if (omega[h][i].get(GRB_DoubleAttr_X) == 1) {
//                        start = i;
//                    }
//                }
//                for (int j : V) {
//                    if (delta[h][j].get(GRB_DoubleAttr_X) == 1) {
//                        end = j;
//                    }
//                }
//                std::vector<int> middle;
//                for (int i : c_s) {
//                    if (phi[h][i].get(GRB_DoubleAttr_X) == 1) {
//                        middle.push_back(i);
//                    }
//                }
//                auto trip = Sortie(h, start, end, middle);
//                st.push_back(trip);
//            }
//        }
//        for (auto& sortie : st) {
//            std::cout << "---------------------------" << std::endl;
//            std::cout << "Sortie of " << sortie.target << " {" << std::endl;
//            std::cout << "l/r: (" << sortie.l << ", " << sortie.r << ")" << std::endl;
//            std::cout << "Truck routes: ";
//            for (int i : c_s) {
//                for (int j : V) {
//                    if (gamma[sortie.target][i][j].get(GRB_DoubleAttr_X) == 1) {
//                        std::cout << i << "->" << j << ", ";
//                    }
//                }
//            }
//            std::cout << std::endl;
//        }
//        return Result{ model.get(GRB_DoubleAttr_ObjVal), st };
//    }
//    catch (GRBException& e) {
//        std::cout << e.getMessage() << std::endl;
//    }
//
//}
//
//Result Solver::mvdSolver(int n_thread, int e) {
//    auto tau = instance->tau;
//    auto tau_prime = instance->tau_prime;
//    auto dtl = e;
//    auto sl = 1, sr = 1;
//    auto n = instance->num_node;
//
//    auto c_prime = instance->c_prime;
//    std::vector<int> c_prime_0;
//    c_prime_0.push_back(0);
//    for (int i : c_prime) {
//        c_prime_0.push_back(i);
//    }
//    c_prime_0.push_back(n);
//    std::cout << "Printing number of nodes: " << n << std::endl;
//    std::vector<int> C;
//    std::vector<int> V;
//    for (int i = 0; i < n + 1; i++) {
//        if (i == 0 || i == n) {
//            V.push_back(i);
//        }
//        else {
//            V.push_back(i);
//            C.push_back(i);
//        }
//    }
//
//    // C_s : set C(customer) union s (source)
//    // C_t : set C(customer) union t (terminal)
//    // create.
//    std::vector<int> c_s;
//    std::vector<int> c_t;
//    for (int i = 0; i < n + 1; i++) {
//        if (i == 0) {
//            c_s.push_back(i);
//        }
//        else if (i == n) {
//            c_t.push_back(i);
//        }
//        else {
//            c_s.push_back(i);
//            c_t.push_back(i);
//        }
//    }
//
//    std::cout << std::endl;
//    GRBEnv env;
//    GRBModel model(env);
//    auto K = C, k_prime = V;
//    auto O = 0;
//    auto D = n;
//    auto node_max_stage = k_prime.size();
//    auto arc_max_stage = node_max_stage - 1;
//
//    //
//    // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
//    //đường đi của vehicle là i; k \in 1..n;
//    //
//    auto** X = new GRBVar * [n + 1];
//    for (int i = 0; i <= D; i++) {
//        X[i] = reinterpret_cast<GRBVar*> (new GRBVar * [n + 1]);
//        for (int k = 1; k <= node_max_stage; k++) {
//            X[i][k] = model.addVar(0, 1, 0.0, GRB_BINARY, "X_" + std::to_string(i) + "_" + std::to_string(k));
//        }
//    }
//
//
//    // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
//    // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
//    auto*** x = new GRBVar * *[n + 1];
//    for (int k = 1; k <= arc_max_stage; k++) { // vi o node_max_stage thi khong di duoc nua.
//        x[k] = reinterpret_cast<GRBVar**>(new GRBVar * *[n + 1]);
//        for (int i = 0; i < D; i++) {
//            x[k][i] = reinterpret_cast<GRBVar*>(new GRBVar * [n + 1]);
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    x[k][i][j] = model.addVar(0, 1, 0.0, GRB_BINARY, "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j));
//                }
//            }
//        }
//    }
//
//
//    // phi^h equals to 1 if customer h is served by the drone
//    std::vector<GRBVar> phi(n);
//    for (int h : C) {
//        phi[h] = model.addVar(0, 1, 0.0, GRB_BINARY, "phi_" + std::to_string(h));
//    }
//
//    auto***** Z = new GRBVar * ***[n + 1];
//    for (int k = 1; k <= node_max_stage - 1; k++) {
//        Z[k] = reinterpret_cast<GRBVar****>(new GRBVar * ***[n + 1]);
//        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
//            if (k < k_p) {
//                Z[k][k_p] = reinterpret_cast<GRBVar***>(new GRBVar * **[n + 1]);
//                for (int i = 0; i < D; i++) {
//                    Z[k][k_p][i] = reinterpret_cast<GRBVar**>(new GRBVar * *[n + 1]);
//                    for (int j = 1; j <= D; j++) {
//                        if (i != j) {
//                            Z[k][k_p][i][j] = reinterpret_cast<GRBVar*>(new GRBVar * [n + 1]);
//                            for (int h : C) {
//                                if (h != i && h != j) {
//                                    Z[k][k_p][i][j][h] = model.addVar(0, 1, 0.0, GRB_BINARY,
//                                        "Z_" + std::to_string(k) + "_" + std::to_string(k_p) +
//                                        "_" + std::to_string(i) + "_" + std::to_string(j) + "_" +
//                                        std::to_string(h));
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }
//    }
//
//    // aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
//    auto** z = new GRBVar * [n + 1];
//    for (int k = 1; k <= node_max_stage - 1; k++) {
//        z[k] = reinterpret_cast<GRBVar*>(new GRBVar * [n + 1]);
//        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
//            if (k < k_p) {
//                z[k][k_p] = model.addVar(0, 1, 0.0, GRB_BINARY, "z_" + std::to_string(k) + "_" + std::to_string(k_p));
//            }
//        }
//    }
//
//    // arrival\departure variables a and d.
//    std::vector<GRBVar> a(n + 1);
//    std::vector<GRBVar> d(n + 1);
//    for (int k = 1; k <= node_max_stage; k++) {
//        a[k] = model.addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "a_" + std::to_string(k));
//        d[k] = model.addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "d_" + std::to_string(k));
//        model.addConstr(d[k] >= a[k], "C13_" + std::to_string(k));
//    }
//    model.addConstr(a[O] == 0);
//    model.addConstr(d[0] == 0);
//
//    ////////// Constraint C1
//    for (int k = 1; k <= node_max_stage - 1; k++) {
//        for (int i = 0; i < D; i++) {
//            if (i != D) { // i == D => khong co i -> j.
//                GRBLinExpr sum;
//                for (int j = 1; j <= D; j++) {
//                    if (i != j) {
//                        sum += x[k][i][j];
//                    }
//                }
//
//
//                // neu node i la node_stage k, i khac D thi kieu gi cung co canh i, j.
//                model.addConstr(X[i][k] == sum, "C1_" + std::to_string(k) + "_" + std::to_string(i));
//            }
//        }
//    }
//
//    //////////// C2
//    GRBLinExpr C2;
//    for (int i = 1; i <= D; i++) {
//        C2 += x[1][O][i];
//    }
//    model.addConstr(C2 == 1, "C2");
//
//    ///////////// C3: arc_stage
//    GRBLinExpr C3;
//    for (int k = 1; k <= arc_max_stage; k++) {
//        for (int i = 0; i < D; i++) {
//            C3 += x[k][i][D];
//        }
//    }
//    model.addConstr(C3 == 1, "C3");
//
//    ///////////// C4: arc_stage
//    for (int k = 1; k <= arc_max_stage - 1; k++) {
//        for (int i = 1; i < D; i++) {
//            GRBLinExpr lhs, rhs;
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    lhs += x[k + 1][i][j]; // từ i ra j là cạnh k+1.
//                }
//            }
//            for (int j = 0; j < D; j++) {
//                if (i != j) {
//                    rhs += x[k][j][i]; // từ j vao i la canh k.
//                }
//            }
//            // enter i tai stage k thi leave i tai stage k+1.
//            model.addConstr(lhs == rhs, "C4_" + std::to_string(k) + "_" + std::to_string(i));
//        }
//    }
//
//    ////////////// C6: node_stage
//    for (int k = 1; k < node_max_stage; k++) {
//        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
//            if (k < k_p) { // cho chac luon.
//                GRBLinExpr rhs;
//                for (int i = 0; i < D; i++) { // start can't include D.
//                    for (int j = 1; j <= D; j++) { // end can include D.
//                        if (i != j) {
//                            for (int h : C) {
//                                if (h != i && h != j) {
//                                    rhs += Z[k][k_p][i][j][h];
//                                }
//                            }
//                        }
//                    }
//                }
//                // z_(k,k') bool: co sortie di tu launch = k, end = k'. bang tong Z(k, k')_(i, j, h).
//                model.addConstr(z[k][k_p] == rhs, "C6_" + std::to_string(k) + "_to_" + std::to_string(k_p));
//            }
//        }
//    }
//
//
//    ////////// C7: node_stage
//    for (int k = 1; k <= node_max_stage - 1; k++) {
//        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
//            if (k < k_p) {
//                for (int l = k; l <= node_max_stage; l++) {
//                    for (int l_p = l + 1; l_p <= node_max_stage; l_p++) {
//                        if (k <= l && l < k_p && l < l_p) {
//                            // tranh drone bay cac doan giao nhau.
//                            model.addConstr(z[k][k_p] + z[l][l_p] <= 1, "C7_" + std::to_string(k) + "_" + std::to_string(k_p) + "_" + std::to_string(l) + "_" + std::to_string(l_p));
//                        }
//                    }
//                }
//            }
//        }
//    }
//
//    /////////// C8: node_stage
//    for (int i = 0; i < D; i++) {
//        for (int k = 1; k <= node_max_stage - 1; k++) {
//            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
//                if (k < k_p) {
//                    for (int j = 1; j <= D; j++) {
//                        if (i != j) {
//                            for (int h : C) {
//                                if (i != h && h != j) {
//                                    model.addConstr(Z[k][k_p][i][j][h] <= X[i][k], "C8_launch_NEED_REVIEW_COMBINE" + std::to_string(i) + "_" + std::to_string(k));
//                                    model.addConstr(Z[k][k_p][i][j][h] <= X[j][k_p], "C8_rendezvous_NEED_REVIEW_COMBINE" + std::to_string(j) + "_" + std::to_string(k_p));
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }
//    }
//
//    //////// C9: node_stage
//    for (int h : C) {
//        GRBLinExpr rhs;
//        for (int i = 0; i < D; i++) {
//            for (int j = 1; j <= D; j++) {
//                if (i != j && i != h && j != h) {
//                    for (int k = 1; k <= node_max_stage - 1; k++) {
//                        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
//                            if (k < k_p) {
//                                rhs += Z[k][k_p][i][j][h];
//                            }
//                        }
//                    }
//                }
//
//            }
//        }
//
//        // consistency constraint cho sortie phuc vu h.
//        model.addConstr(phi[h] == rhs, "C9_" + std::to_string(h));
//        model.addConstr(rhs <= 1);
//    }
//
//    //////////// C10: node_stage
//    for (int h : C) {
//        GRBLinExpr sum_k;
//        for (int k = 1; k <= node_max_stage; k++) {
//            sum_k += X[h][k];
//        }
//
//        // phuc vu h it nhat 1 lan.
//        model.addConstr(phi[h] + sum_k >= 1, "C10_" + std::to_string(h));
//    }
//
//    //////////////// C11: node_stage
//    for (int h : C) {
//        GRBLinExpr sum;
//        for (int k = 1; k <= node_max_stage - 1; k++) {
//            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
//                if (k < k_p) {
//                    for (int i = 0; i < D; i++) {
//                        for (int j = 1; j <= D; j++) {
//                            if (i != j && i != h && j != h) {
//                                sum += Z[k][k_p][i][j][h] * (tau_prime[i][h] + tau_prime[h][j]);
//                            }
//                        }
//                    }
//                }
//            }
//        }
//
//        // drone time constraint (i -> h, h -> j) <= dtl. trong giong tien xu ly.
//        model.addConstr(sum <= dtl, "C11_" + std::to_string(h));
//    }
//
//    /////////// C14: node_stage
//    for (int k = 1; k <= arc_max_stage; k++) {
//        GRBLinExpr sum;
//        for (int i = 0; i < D; i++) {
//            for (int j = 1; j <= D; j++) {
//                if (i != j) {
//                    sum += x[k][i][j] * tau[i][j];
//                }
//            }
//        }
//
//        // o to toi k+1 >= o to den k + thoi gian di chuyen tu k->k+1 (tau_(ij)).
//        model.addConstr(a[k + 1] >= d[k] + sum, "C14_" + std::to_string(k) + "_" + std::to_string(k + 1));
//    }
//
//    ////////// C15: node_stage
//    auto M = 100000;
//    for (int k = 1; k <= node_max_stage - 1; k++) {
//        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
//            if (k < k_p) {
//
//                // o to phai den k_p tu k trong khoang thoi gian <= dtl.
//                model.addConstr(a[k_p] - d[k] <= z[k][k_p] * dtl + (1 - z[k][k_p]) * M, "C15_" + std::to_string(k) + "_" + std::to_string(k_p));
//
//                GRBLinExpr rhs;
//                for (int i = 0; i < D; i++) {
//                    for (int j = 1; j <= D; j++) {
//                        if (i != j) {
//                            for (int h : C) {
//                                if (h != i && h != j) {
//                                    rhs += Z[k][k_p][i][j][h] * (tau_prime[i][h] + tau_prime[h][j]);
//                                }
//                            }
//                        }
//                    }
//                }
//
//                // vehicle phai doi drone truoc khi di chuyen.
//                model.addConstr(d[k_p] - d[k] >= rhs, "C16_" + std::to_string(k) + "_" + std::to_string(k_p));
//            }
//        }
//    }
//
//    GRBLinExpr objective;
//    objective += a[node_max_stage];
//    model.setObjective(objective, GRB_MINIMIZE);
//    model.update();
//    model.write("model.lp");
//    model.optimize();
//    //    return Result(0, std::vector());
//}

Result Solver::mvdSolverCPLEX(int n_thread, int e) {
    //    try {
    auto tau = instance->tau;
    auto tau_prime = instance->tau_prime;
    auto dtl = e;
    //dtl = 5;
    auto sl = 1, sr = 1;
    auto n = instance->num_node;

    auto c_prime = instance->c_prime;
    std::vector<int> c_prime_0;
    c_prime_0.push_back(0);
    for (int i: c_prime) {
        c_prime_0.push_back(i);
    }
    c_prime_0.push_back(n);

    std::cout << "Printing number of nodes: " << n << std::endl;
    std::vector<int> C;
    std::vector<int> V;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0 || i == n) {
            V.push_back(i);
        } else {
            V.push_back(i);
            C.push_back(i);
        }
    }

    // C_s : set C(customer) union s (source) ; source  = 0
    // C_t : set C(customer) union t (terminal); terminal = n
    // create.
    std::vector<int> c_s;
    std::vector<int> c_t;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0) {
            c_s.push_back(i);
        } else if (i == n) {
            c_t.push_back(i);
        } else {
            c_s.push_back(i);
            c_t.push_back(i);
        }
    }

    std::cout << std::endl;
    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);


    auto K = C, k_prime = V;
    auto O = 0;
    auto D = n;
    auto node_max_stage = n + 1;
    auto arc_max_stage = node_max_stage - 1;

    // Khai bao bien
    //
    // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
    //đường đi của vehicle là i; k \in 1..n;
    //
    IloArray<IloBoolVarArray> X(env, node_max_stage + 1);
    for (int k = 1; k <= node_max_stage; k++) {
        X[k] = IloBoolVarArray(env, D + 1);

        for (int i = 0; i <= D; i++) {
            {
                X[k][i] = IloBoolVar(env);
                auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
                //std::cout << v_name << std::endl;
                X[k][i].setName(v_name.c_str());

                if (k > 1 && i == 0) X[k][0].setBounds(0, 0);
            }
        }
    }


    model.add(X[1][0] == 1).setName("start depot is the first node");
    model.add(X[1][D] == 0).setName("ending depot cannot be the first node");

    for (int k = 1; k <= node_max_stage; k++) {
        IloExpr sum(env);
        for (int i = 0; i <= D; i++)
            sum += X[k][i];
        model.add(sum <= 1).setName(("C20_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
    }

    IloExpr arrival_depot(env);
    for (int k = 1; k <= node_max_stage; k++) {
        arrival_depot += X[k][D];
    }
    model.add(arrival_depot == 1).setName("C21_arrival_depot_once");

    // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
    // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
    IloArray<IloArray<IloBoolVarArray> > x(env, arc_max_stage + 1);
    for (int k = 1; k <= arc_max_stage; k++) {
        x[k] = IloArray<IloBoolVarArray>(env, D);
        for (int i = 0; i < D; i++) {
            x[k][i] = IloBoolVarArray(env, D + 1);

            for (int j = 1; j <= D; j++)
                if (i != j) {
                    x[k][i][j] = IloBoolVar(env);
                    auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
                    x[k][i][j].setName(v_name.c_str());
                }
        }
    }


    //// phi^h equals to 1 if customer h is served by the drone
    IloBoolVarArray phi(env, n);
    for (int h: C) {
        phi[h] = IloBoolVar(env);
        auto v_name = "phi_" + std::to_string(h);
        phi[h].setName(v_name.c_str());
    }
    for (auto heavy: instance->heavy) {
        model.add(phi[heavy] == 0);
    }

    IloArray<IloArray<IloArray<IloArray<IloBoolVarArray> > > > Z(env, node_max_stage);
    for (int k = 1; k <= node_max_stage - 1; k++) {
        Z[k] = IloArray<IloArray<IloArray<IloBoolVarArray> > >(env, node_max_stage + 1);
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            if (k < k_p) {
                Z[k][k_p] = IloArray<IloArray<IloBoolVarArray> >(env, D);
                for (int i = 0; i < D; i++) {
                    Z[k][k_p][i] = IloArray<IloBoolVarArray>(env, D + 1);
                    for (int j = 1; j <= D; j++) {
                        if (i != j) {
                            Z[k][k_p][i][j] = IloBoolVarArray(env, D);
                            for (int h: C) {
                                if (h != i && h != j)
                                    if (tau_prime[i][h] + tau_prime[h][j] <= dtl) {
                                        Z[k][k_p][i][j][h] = IloBoolVar(env);
                                        auto v_name = "Z_" + std::to_string(k) + "_" + std::to_string(k_p) +
                                                      "_" + std::to_string(i) + "_" + std::to_string(j) + "_" +
                                                      std::to_string(h);
                                        Z[k][k_p][i][j][h].setName(v_name.c_str());
                                        if (tau_prime[i][h] + tau_prime[h][j] > dtl) {
                                            model.add(Z[k][k_p][i][j][h] == 0);
                                        }
                                    }
                            }
                        }
                    }
                }
            }
        }
    }


    //// aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
    IloArray<IloBoolVarArray> z(env, node_max_stage);
    for (int k = 1; k <= node_max_stage - 1; k++) {
        z[k] = IloBoolVarArray(env, node_max_stage + 1);
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            {
                z[k][k_p] = IloBoolVar(env);
                auto v_name = "z_" + std::to_string(k) + "_" + std::to_string(k_p);
                z[k][k_p].setName(v_name.c_str());
            }
        }
    }


    // arrival\departure variables a and d.
    IloNumVarArray a(env, node_max_stage + 1);
    IloNumVarArray d(env, node_max_stage + 1);
    for (int k = 1; k <= node_max_stage; k++) {
        a[k] = IloNumVar(env);
        auto v_name = "a_" + std::to_string(k);
        a[k].setName(v_name.c_str());
        d[k] = IloNumVar(env);
        v_name = "d_" + std::to_string(k);
        d[k].setName(v_name.c_str());
        model.add(d[k] >= a[k]).setName(("C13_" + std::to_string(k)).c_str());
    }

    model.add(a[1] == 0).setName("arrival to depot at time 0");
    model.add(d[1] == 0).setName("depart from depot at time 0");;

    ////////// Constraint C1
    for (int k = 1; k < node_max_stage; k++) {
        for (int i = 0; i < D; i++) {
            {
                // i == D => khong co i -> j.
                IloExpr sum(env);
                for (int j = 1; j <= D; j++) {
                    if (i != j) {
                        sum += x[k][i][j];
                    }
                }

                // neu node i la node_stage k, i khac D thi kieu gi cung co canh i, j.
                model.add(X[k][i] == sum).setName(("C1_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
            }
        }
    }

    for (int k = 2; k <= node_max_stage; k++) {
        for (int i = 1; i <= D; i++) {
            {
                IloExpr sum(env);
                for (int j = 0; j < D; j++) {
                    if (i != j) {
                        sum += x[k - 1][j][i];
                    }
                }
                // arcs entering i at stage k.
                model.add(X[k][i] == sum).setName(("C1p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
            }
        }
    }

    //////////// C2 - depart from the depot
    IloExpr C2(env);
    for (int i = 1; i <= D; i++) {
        C2 += x[1][O][i];
    }

    IloConstraint c2(C2 == 1);
    model.add(c2).setName("C2");
    // , "C2"

    ///////////// C3: arc_stage
    IloExpr C3(env);
    //for (int k = 1; k <= arc_max_stage; k++) {
    //    for (int i = 0; i < D; i++) {
    //        C3 += x[k][i][D];
    //    }
    //}
    for (int k = 2; k <= node_max_stage; k++) {
        {
            C3 += X[k][D];
        }
    }
    // , "C3"
    model.add(C3 == 1).setName("C3"); // arrival to depot

    ///////////// C4: arc_stage - connectivity constraint
    //for (int k = 2; k <= arc_max_stage; k++) {
    //    for (int i = 1; i < D; i++) {
    //        IloExpr lhs(env);
    //        IloExpr rhs(env);
    //        for (int j = 1; j <= D; j++) {
    //            if (i != j) {
    //                lhs += x[k][i][j]; // từ i ra j là cạnh k+1.
    //            }
    //        }
    //        for (int j = 0; j < D; j++) {
    //            if (i != j) {
    //                rhs += x[k-1][j][i]; // từ j vao i la canh k.
    //            }
    //        }
    //        // enter i tai stage k thi leave i tai stage k+1.
    //        // , "C4_" + std::to_string(k) + "_" + std::to_string(i)
    //        model.add(lhs == rhs).setName(("C4_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
    //    }
    //}

    //////////////// C6: node_stage
    for (int k = 1; k < node_max_stage; k++) {
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            {
                // cho chac luon.
                IloExpr rhs(env);
                for (int i = 0; i < D; i++) {
                    // start can't include D.
                    for (int j = 1; j <= D; j++) {
                        // end can include D.
                        if (i != j) {
                            for (int h: C) {
                                if (h != i && h != j)
                                    if (tau_prime[i][h] + tau_prime[h][j] <= dtl) {
                                        rhs += Z[k][k_p][i][j][h];
                                    }
                            }
                        }
                    }
                }
                // z_(k,k') bool: co sortie di tu launch = k, end = k'. bang tong Z(k, k')_(i, j, h).
                // , "C6_" + std::to_string(k) + "_to_" + std::to_string(k_p)
                model.add(z[k][k_p] == rhs).setName(("C6_" + std::to_string(k) + "_to_" + std::to_string(k_p)).c_str());
            }
        }
    }


    //////////// C7: node_stage
    for (int k = 1; k <= node_max_stage - 1; k++) {
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            for (int l = k; l < k_p; l++) {
                for (int l_p = l + 1; l_p <= node_max_stage; l_p++) {
                    if ((k != l) || (k_p != l_p)) {
                        // tranh drone bay cac doan giao nhau.
                        model.add(z[k][k_p] + z[l][l_p] <= 1).setName(
                                ("C7_" + std::to_string(k) + "_" + std::to_string(k_p) + "_" + std::to_string(l) + "_" +
                                 std::to_string(l_p)).c_str());
                    }
                }
            }
        }
    }


    /////////// C8: node_stage
    //for (int i = 0; i < D; i++) {
    //    for (int k = 1; k <= node_max_stage - 1; k++) {
    //        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
    //            if (k < k_p) {
    //                for (int j = 1; j <= D; j++) {
    //                    if (i != j) {
    //                        for (int h : C) {
    //                            if (i != h && h != j)
    //                                if (tau_prime[i][h] + tau_prime[h][j] <= dtl)
    //                            {
    //                                // , "C8_launch_" + std::to_string(i) + "_" + std::to_string(k)
    //                                model.add(Z[k][k_p][i][j][h] <= X[i][k]).setName(("C8_launch_" + std::to_string(i) + "_" + std::to_string(k)).c_str());
    //                                // , "C8_rendezvous_NEED_REVIEW_COMBINE" + std::to_string(j)+ "_" + std::to_string(k_p)
    //                                model.add(Z[k][k_p][i][j][h] <= X[j][k_p]).setName(("C8_rendezvous_" + std::to_string(j) + "_" + std::to_string(k_p)).c_str());
    //                            }
    //                        }
    //                    }
    //                }
    //            }
    //        }
    //    }
    //}


    for (int i = 0; i < D; i++) {
        for (int k = 1; k <= node_max_stage - 1; k++) {
            IloExpr lhs(env);

            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                {
                    for (int j = 1; j <= D; j++) {
                        if (i != j) {
                            for (int h: C) {
                                if (i != h && h != j)
                                    if (tau_prime[i][h] + tau_prime[h][j] <= dtl) {
                                        lhs += Z[k][k_p][i][j][h];
                                    }
                            }
                        }
                    }
                }
            }
            model.add(lhs <= X[k][i]).setName(("C8_launch_" + std::to_string(i) + "_" + std::to_string(k)).c_str());
        }
    }

    for (int j = 1; j <= D; j++) {
        for (int k_p = 2; k_p <= node_max_stage; k_p++) {
            IloExpr lhs(env);

            for (int k = 1; k < k_p; k++) {
                {
                    for (int i = 0; i < D; i++) {
                        if (i != j) {
                            for (int h: C) {
                                if (i != h && h != j)
                                    if (tau_prime[i][h] + tau_prime[h][j] <= dtl) {
                                        lhs += Z[k][k_p][i][j][h];
                                    }
                            }
                        }
                    }
                }
            }
            model.add(lhs <= X[k_p][j]).setName(
                    ("C8_rendezvous_" + std::to_string(j) + "_" + std::to_string(k_p)).c_str());
        }
    }

    ////////// C9: node_stage
    for (int h: C) {
        IloExpr rhs(env);
        for (int i = 0; i < D; i++) {
            for (int j = 1; j <= D; j++) {
                if (i != j && i != h && j != h)
                    if (tau_prime[i][h] + tau_prime[h][j] <= dtl) {
                        for (int k = 1; k <= node_max_stage - 1; k++) {
                            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                                {
                                    rhs += Z[k][k_p][i][j][h];
                                }
                            }
                        }
                    }
            }
        }

        // consistency constraint cho sortie phuc vu h.
        model.add(phi[h] == rhs).setName(("C9_" + std::to_string(h)).c_str());
    }

    //////////// C10: node_stage
    for (int h: C) {
        IloExpr sum_k(env);
        for (int k = 2; k < node_max_stage; k++) {
            sum_k += X[k][h];
        }
        // phuc vu h it nhat 1 lan.
        model.add(phi[h] + sum_k >= 1).setName(("C10_" + std::to_string(h)).c_str());
        //model.add(sum_k >= 1).setName(("C10_" + std::to_string(h)).c_str());
    }

    //////////////// C11: node_stage
    //for (int h : C) {
    //    IloExpr sum(env);
    //    for (int k = 1; k <= node_max_stage - 1; k++) {
    //        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
    //            if (k < k_p) {
    //                for (int i = 0; i < D; i++) {
    //                    for (int j = 1; j <= D; j++) {
    //                        if (i != j && i != h && j != h) {
    //                            if (tau_prime[i][h] + tau_prime[h][j] <= dtl) {
    //                                sum += Z[k][k_p][i][j][h] * (tau_prime[i][h] + tau_prime[h][j]);
    //                            }
    //                        }
    //                    }
    //                }
    //            }
    //        }
    //    }

    //    // drone time constraint (i -> h, h -> j) <= dtl. trong giong tien xu ly.
    //    // , "C11_" + std::to_string(h)
    //    model.add(sum <= dtl * phi[h]).setName(("C11_" + std::to_string(h)).c_str());
    //}

    /////////// C14: node_stage
    for (int k = 1; k <= arc_max_stage; k++) {
        IloExpr sum(env);
        for (int i = 0; i < D; i++) {
            for (int j = 1; j <= D; j++) {
                if (i != j) {
                    sum += x[k][i][j] * tau[i][j];
                }
            }
        }

        // o to toi k+1 >= o to den k + thoi gian di chuyen tu k->k+1 (tau_(ij)).
        model.add(a[k + 1] >= d[k] + sum).setName(("C14_" + std::to_string(k) + "_" + std::to_string(k + 1)).c_str());
    }

    ////////// C15: node_stage
    auto M = 1e5;
    for (int k = 1; k <= node_max_stage - 1; k++) {
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            if (k < k_p) {
                // o to phai den k_p tu k trong khoang thoi gian <= dtl.
                // , "C15_" + std::to_string(k) + "_" + std::to_string(k_p)
                model.add(a[k_p] - d[k] <= z[k][k_p] * dtl + (1 - z[k][k_p]) * M).setName(
                        ("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());

                IloExpr rhs(env);
                for (int i = 0; i < D; i++) {
                    for (int j = 1; j <= D; j++) {
                        if (i != j) {
                            for (int h: C) {
                                if (h != i && h != j) {
                                    //                                    if (tau_prime[i][h] + tau_prime[h][j] <= dtl) {
                                    rhs += Z[k][k_p][i][j][h] * (tau_prime[i][h] + tau_prime[h][j]);
                                    //                                    }
                                }
                            }
                        }
                    }
                }

                // vehicle phai doi drone truoc khi di chuyen.
                // , "C16_" + std::to_string(k) + "_" + std::to_string(k_p)
                model.add(d[k_p] - d[k] >= rhs).setName(
                        ("C16_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
            }
        }
    }

    model.add(IloMinimize(env, d[node_max_stage]));
    cplex.exportModel("cplex_model.lp");
    // Solve the model
    if (!cplex.solve()) {
        // Check if the problem is infeasible
        if (cplex.getStatus() == IloAlgorithm::Infeasible) {
            // Handle infeasibility
            std::cout << "The problem is infeasible." << std::endl;
            //                std::cout << "aaaaaaaaaaaaa" << std::endl;
            std::cout << "Infeasibility at: " << cplex.getInfeasibility(c2) << std::endl;
            // You can also retrieve the infeasible constraints using cplex.getInfeasibility() method
        } else {
            // Handle other solver errors
            std::cerr << "Solver error: " << cplex.getStatus() << std::endl;
        }
    } else {
        std::cout << "Feasible solution found!" << std::endl;
        std::cout << "Truck nodes:" << std::endl;
        for (int k = 1; k <= node_max_stage; k++) {
            for (int i = 0; i <= D; i++) {
                auto X_val = cplex.getValue(X[k][i]);
                //std::cout << "k = " << k << ", i = " << i << ":" << X_val << std::endl;

                if (abs(X_val - 1) < 1e-6) {
                    std::cout << "Stage " << k << " at customer " << i << std::endl;
                    break;
                }
            }
        }
        std::cout << "Truck arcs:" << std::endl;
        for (int k = 1; k <= arc_max_stage; k++) {
            for (int i = 0; i < D; i++)
                for (int j = 1; j <= D; j++)
                    if (i != j) {
                        auto X_val = cplex.getValue(x[k][i][j]);
                        //std::cout << "k = " << k << ", i = " << i << ":" << X_val << std::endl;

                        if (abs(X_val - 1) < 1e-6) {
                            std::cout << "Arc " << k << " connecting " << i << " and " << j
                                      << " with cost " << tau[i][j] << " " << std::endl;
                            break;
                        }
                    }
        }


        std::cout << "Drone served customers" << std::endl;
        //IloNumArray phi_val(env);
        //cplex.getValues(phi_val, phi);

        for (int h: C) {
            if (cplex.getValue(phi[h]) == 1) {
                std::cout << "Customer " << h << " served by drone." << std::endl;
                for (int k = 1; k < node_max_stage; k++)
                    for (int k_p = k + 1; k_p <= node_max_stage; k_p++)
                        if (abs(cplex.getValue(z[k][k_p]) - 1) < 1e-6) {
                            for (int i = 0; i < D; i++)
                                if (i != h)
                                    for (int j = 1; j <= D; j++)
                                        if (j != h && j != i) {
                                            auto Z_val = cplex.getValue(Z[k][k_p][i][j][h]);
                                            if (abs(Z_val - 1) < 1e-6) {
                                                std::cout << "Drone fly from " << i << " at stage " << k <<
                                                          " to serve " << h << " and then fly back to " << j
                                                          << " at stage " << k_p << std::endl;
                                            }
                                        }
                        }
            }
        }
        // Retrieve solution
        // Handle feasible solution
    }
    //    } catch (IloException &e) {
    //        std::cout << e.getMessage() << std::endl;
    //    }

    return Result();
}

#include<cassert>
#include <map>
#include <random>

Result Solver::mvdSolverCPLEXFewerVariables(int n_thread, int e) {
    //    try {
    auto tau = instance->tau;
    auto tau_prime = instance->tau_prime;
    auto dtl = e;
    //dtl = 5;
    auto sl = 1, sr = 1;
    auto n = instance->num_node;

    auto c_prime = instance->c_prime;
    std::vector<int> c_prime_0;
    c_prime_0.push_back(0);
    for (int i: c_prime) {
        c_prime_0.push_back(i);
    }
    c_prime_0.push_back(n);

    std::cout << "Printing number of nodes: " << n << std::endl;
    std::vector<int> C;
    std::vector<int> V;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0 || i == n) {
            V.push_back(i);
        } else {
            V.push_back(i);
            C.push_back(i);
        }
    }

    // C_s : set C(customer) union s (source) ; source  = 0
    // C_t : set C(customer) union t (terminal); terminal = n
    // create.
    std::vector<int> c_s;
    std::vector<int> c_t;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0) {
            c_s.push_back(i);
        } else if (i == n) {
            c_t.push_back(i);
        } else {
            c_s.push_back(i);
            c_t.push_back(i);
        }
    }

    std::cout << std::endl;
    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);

    auto K = C, k_prime = V;
    auto O = 0;
    auto D = n;
    auto node_max_stage = n + 1;
    auto arc_max_stage = node_max_stage - 1;

    // Khai bao bien
    //
    // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
    //đường đi của vehicle là i; k \in 1..n;
    //
    IloArray<IloBoolVarArray> X(env, node_max_stage + 1);
    for (int k = 1; k <= node_max_stage; k++) {
        X[k] = IloBoolVarArray(env, D + 1);

        for (int i = 0; i <= D; i++) {
            {
                X[k][i] = IloBoolVar(env);
                auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
                //std::cout << v_name << std::endl;
                X[k][i].setName(v_name.c_str());

                if (k > 1 && i == 0) X[k][0].setBounds(0, 0);
            }
        }
    }
    IloNumVarArray start_val(env);
    IloNumArray primalSol(env);


    model.add(X[1][0] == 1).setName("start depot is the first node");
    model.add(X[1][D] == 0).setName("ending depot cannot be the first node");

    for (int k = 1; k <= node_max_stage; k++) {
        IloExpr sum(env);
        for (int i = 0; i <= D; i++)
            sum += X[k][i];
        model.add(sum <= 1).setName(("C20_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
    }

    IloExpr arrival_depot(env);
    for (int k = 1; k <= node_max_stage; k++) {
        arrival_depot += X[k][D];
    }
    model.add(arrival_depot == 1).setName("C21_arrival_depot_once");

    // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
    // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
    IloArray<IloArray<IloBoolVarArray> > x(env, arc_max_stage + 1);
    for (int k = 1; k <= arc_max_stage; k++) {
        x[k] = IloArray<IloBoolVarArray>(env, D);
        for (int i = 0; i < D; i++) {
            x[k][i] = IloBoolVarArray(env, D + 1);

            for (int j = 1; j <= D; j++)
                if (i != j) {
                    x[k][i][j] = IloBoolVar(env);
                    auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
                    x[k][i][j].setName(v_name.c_str());
                }
        }
    }


    //// phi^h equals to 1 if customer h is served by the drone
    IloBoolVarArray phi(env, n);
    for (int h: C) {
        phi[h] = IloBoolVar(env);
        auto v_name = "phi_" + std::to_string(h);
        phi[h].setName(v_name.c_str());
    }

    for (int heavy: instance->heavy) {
        model.add(phi[heavy] == 0);
    }
    IloArray<IloArray<IloBoolVarArray> > Y(env, node_max_stage + 1), W(env, node_max_stage + 1);
    for (int k = 1; k <= node_max_stage; k++) {
        Y[k] = IloArray<IloBoolVarArray>(env, D + 1);
        W[k] = IloArray<IloBoolVarArray>(env, D + 1);

        for (int i = 0; i <= D; i++) {
            {
                Y[k][i] = IloBoolVarArray(env, C.size() + 1);
                W[k][i] = IloBoolVarArray(env, C.size() + 1);
                for (int h: C)
                    if (i != h) {
                        Y[k][i][h] = IloBoolVar(env);
                        Y[k][i][h].setName(("Y_" + std::to_string(k) + "_"
                                            + std::to_string(i) + "_" + std::to_string(h)).c_str());

                        W[k][i][h] = IloBoolVar(env);
                        W[k][i][h].setName(("W_" + std::to_string(k) + "_"
                                            + std::to_string(i) + "_" + std::to_string(h)).c_str());

                        if (i == 0 && k > 1) Y[k][i][h].setBounds(0, 0);
                        if (i == D && k == 1) W[k][i][h].setBounds(0, 0);
                        if (tau_prime[i][h] > dtl) Y[k][i][h].setBounds(0, 0);
                        if (tau_prime[h][i] > dtl) W[k][i][h].setBounds(0, 0);
                    }
            }
        }
    }


    //// C17 - $X^k_i \geq \sum_h X^k_{ih}$ (C17) - chỉ bay drone ở nơi mà xe ở đó
    for (int k = 1; k <= node_max_stage; k++) {
        for (int i = 0; i <= D; i++) {
            IloExpr expr(env);
            for (int h: C)
                if (h != i)
                    if (tau_prime[i][h] <= dtl) {
                        expr += Y[k][i][h];
                    }
            model.add(expr <= X[k][i]).setName(("C17_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
        }
    }
    // $X^k_i \geq \sum_h Y^k_{ih}$ (C17p) : chỉ bay drone tới nơi mà xe ở đó
    for (int k = 1; k <= node_max_stage; k++) {
        for (int i = 0; i <= D; i++) {
            IloExpr expr(env);
            for (int h: C)
                if (h != i)
                    if (tau_prime[h][i] <= dtl) {
                        expr += W[k][i][h];
                    }
            model.add(expr <= X[k][i]).setName(("C17p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
        }
    }

    // $\sum_{i} X^k_{ih}\tau^D_{ih} + \sum_{i} Y^{k'}_{ih}\tau^D_{hi} \leq D_d$
    //- drone duration constraint cho mỗi $h$ (C19)
    for (int h: C) {
        IloExpr expr(env);

        for (int k = 1; k <= node_max_stage; k++) {
            for (int i = 0; i <= D; i++) {
                if (i != h && i != D && tau_prime[i][h] <= dtl)
                    expr += Y[k][i][h] * tau_prime[i][h];

                if (i != h && i != 0 && tau_prime[h][i] <= dtl)
                    expr += W[k][i][h] * tau_prime[h][i];
            }
        }

        model.add(expr <= dtl * phi[h]).setName(("C19_" + std::to_string(h)).c_str());
    }
    //exit(0);

    //// aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
    IloArray<IloBoolVarArray> z(env, node_max_stage);
    for (int k = 1; k <= node_max_stage - 1; k++) {
        z[k] = IloBoolVarArray(env, node_max_stage + 1);
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            {
                z[k][k_p] = IloBoolVar(env);
                auto v_name = "z_" + std::to_string(k) + "_" + std::to_string(k_p);
                z[k][k_p].setName(v_name.c_str());
            }
        }
    }

    //// aux var Z_{k, k_p, h}: sortie launch from k and rendezvous at k_p.
    IloArray<IloArray<IloBoolVarArray> > Z(env, node_max_stage);
    for (int h: C) {
        Z[h] = IloArray<IloBoolVarArray>(env, node_max_stage + 1);
        for (int k = 1; k <= node_max_stage - 1; k++) {
            Z[h][k] = IloBoolVarArray(env, node_max_stage + 1);
            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                {
                    Z[h][k][k_p] = IloBoolVar(env);
                    auto v_name = "Z_" + std::to_string(h) + "_" + std::to_string(k) + "_" + std::to_string(k_p);
                    Z[h][k][k_p].setName(v_name.c_str());
                }
            }
        }
    }

    // $Z_{kk'} = \sum_{h}Z^h_{kk'}$: mỗi cặp (k,k') chỉ phục vụ tối đa một khách hàng (C22)
    for (int k = 1; k < node_max_stage; k++)
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            IloExpr expr(env);
            for (int h: C) {
                expr += Z[h][k][k_p];
            }
            model.add(expr == z[k][k_p]).setName(("C22_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
        }


    //C20:$\sum_{k'>k}Z_{kk'} = \sum_{i,h}Y^k_{ih}$ : với mỗi $k$,
    //ràng buộc liên kết drone đi ra từ stage $k$ và đoạn mà oto di chuyển không có drone. (C20)

    for (int h: C) {
        for (int k = 1; k <= node_max_stage - 1; k++) {
            IloExpr expr(env);
            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                {
                    expr += Z[h][k][k_p];
                }
            }

            for (int i = 0; i < D; i++) {
                if (i != h && tau_prime[i][h] <= dtl) {
                    expr -= Y[k][i][h];
                }
            }
            model.add(expr == 0).setName(("C20_" + std::to_string(k) + "_" + std::to_string(h)).c_str());
        }
    }

    for (int h: C) {
        for (int k_p = 2; k_p <= node_max_stage; k_p++) {
            IloExpr expr(env);
            for (int k = 1; k < k_p; k++) {
                {
                    expr += Z[h][k][k_p];
                }
            }

            for (int i = 1; i <= D; i++) {
                if (i != h && tau_prime[h][i] <= dtl) {
                    expr -= W[k_p][i][h];
                }
            }
            model.add(expr == 0).setName(("C20p_" + std::to_string(k_p)
                                          + "_" + std::to_string(h)).c_str());
        }
    }

    // arrival\departure variables a and d.
    IloNumVarArray a(env, node_max_stage + 1);
    IloNumVarArray d(env, node_max_stage + 1);
    for (int k = 1; k <= node_max_stage; k++) {
        a[k] = IloNumVar(env);
        auto v_name = "a_" + std::to_string(k);
        a[k].setName(v_name.c_str());
        d[k] = IloNumVar(env);
        v_name = "d_" + std::to_string(k);
        d[k].setName(v_name.c_str());
        model.add(d[k] >= a[k]).setName(("C13_" + std::to_string(k)).c_str());
    }

    model.add(a[1] == 0).setName("arrival to depot at time 0");
    model.add(d[1] == 0).setName("depart from depot at time 0");;

    ////////// Constraint C1
    for (int k = 1; k < node_max_stage; k++) {
        for (int i = 0; i < D; i++) {
            {
                // i == D => khong co i -> j.
                IloExpr sum(env);
                for (int j = 1; j <= D; j++) {
                    if (i != j) {
                        sum += x[k][i][j];
                    }
                }

                // neu node i la node_stage k, i khac D thi kieu gi cung co canh i, j.
                model.add(X[k][i] == sum).setName(("C1_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
            }
        }
    }

    for (int k = 2; k <= node_max_stage; k++) {
        for (int i = 1; i <= D; i++) {
            {
                IloExpr sum(env);
                for (int j = 0; j < D; j++) {
                    if (i != j) {
                        sum += x[k - 1][j][i];
                    }
                }
                // arcs entering i at stage k.
                model.add(X[k][i] == sum).setName(("C1p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
            }
        }
    }

    //////////// C2 - depart from the depot
    IloExpr C2(env);
    for (int i = 1; i <= D; i++) {
        C2 += x[1][O][i];
    }

    IloConstraint c2(C2 == 1);
    model.add(c2).setName("C2");
    // , "C2"

    ///////////// C3: arc_stage
    IloExpr C3(env);
    for (int k = 2; k <= node_max_stage; k++) {
        {
            C3 += X[k][D];
        }
    }
    // , "C3"
    model.add(C3 == 1).setName("C3"); // arrival to depot

    // $R_{k} = \sum_{k'}Z_{kk'}$: các đoạn bắt đầu từ k (C23)
    IloBoolVarArray R(env, node_max_stage + 1);
    for (int k = 1; k < node_max_stage; k++) {
        R[k].setName(("R_" + std::to_string(k)).c_str());
    }

    for (int k = 1; k < node_max_stage; k++) {
        IloExpr expr(env);
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            expr += z[k][k_p];
        }

        model.add(R[k] == expr).setName(("C23_" + std::to_string(k)).c_str());
    }


    //////// C7: node_stage
    //for (int k = 1; k <= node_max_stage - 1; k++) {
    //    for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
    //        for (int l = k; l < k_p; l++) {
    //            for (int l_p = l + 1; l_p <= node_max_stage; l_p++) {
    //                //if ((k != l) || (k_p != l_p))
    //                if ((k != l) || (k == l && k_p < l_p))
    //                {

    //                    // tranh drone bay cac doan giao nhau.
    //                    model.add(z[k][k_p] + z[l][l_p] <= 1).setName(("C7_" + std::to_string(k) + "_" + std::to_string(k_p) + "_" + std::to_string(l) + "_" + std::to_string(l_p)).c_str());
    //                }
    //            }
    //        }
    //    }
    //}

    // modified C7
    for (int k = 1; k <= node_max_stage - 1; k++) {
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            for (int l = k + 1; l < k_p; l++) {
                {
                    // tranh drone bay cac doan giao nhau.
                    if (k < l) {
                        model.add(z[k][k_p] + R[l] <= 1).setName(("C7m_" + std::to_string(k)
                                                                  + "_" + std::to_string(k_p) + "_" + std::to_string(l))
                                                                         .c_str());
                    } else {
                        //model.add(R[k] <= 1).setName(("C7k_" + std::to_string(k)
                        //    + "_" + std::to_string(k_p) + "_" + std::to_string(l)).c_str());
                    }
                }
            }
        }
    }


    for (int i = 0; i < D; i++) {
        for (int k = 1; k <= node_max_stage - 1; k++) {
            IloExpr lhs(env);

            for (int h: C) {
                if (i != h) {
                    lhs += Y[k][i][h];
                }
            }
            model.add(lhs <= X[k][i]).setName(("C8_launch_" + std::to_string(i) + "_" + std::to_string(k)).c_str());
        }
    }

    for (int j = 1; j <= D; j++) {
        for (int k_p = 2; k_p <= node_max_stage; k_p++) {
            IloExpr lhs(env);

            for (int h: C) {
                if (h != j) {
                    lhs += W[k_p][j][h];
                }
            }

            model.add(lhs <= X[k_p][j]).setName(
                    ("C8_rendezvous_" + std::to_string(j) + "_" + std::to_string(k_p)).c_str());
        }
    }


    // $\phi_h = \sum_{k,i}X^k_{ih} = \sum_{k,i}Y^k_{ih}$
    // - chỉ có duy nhất một điểm xuất phát và môt điểm đích cho
    //mỗi khách hàng $h$ được phục vụ bởi drone (C18)
    for (int h: C) {
        IloExpr rhs(env);
        for (int i = 0; i < D; i++) {
            if (i != h && tau_prime[i][h] <= dtl)
                for (int k = 1; k <= node_max_stage - 1; k++) {
                    rhs += Y[k][i][h];
                }
        }
        // consistency constraint cho sortie phuc vu h.
        model.add(phi[h] == rhs).setName(("C18_" + std::to_string(h)).c_str());
    }


    for (int h: C) {
        IloExpr rhs(env);
        for (int i = 1; i <= D; i++) {
            if (i != h && tau_prime[h][i] <= dtl)
                for (int k = 2; k <= node_max_stage; k++) {
                    rhs += W[k][i][h];
                }
        }
        // consistency constraint cho sortie phuc vu h.
        model.add(phi[h] == rhs).setName(("C18p_" + std::to_string(h)).c_str());
    }

    //////////// C10: node_stage
    for (int h: C) {
        IloExpr sum_k(env);
        for (int k = 2; k < node_max_stage; k++) {
            sum_k += X[k][h];
        }
        // phuc vu h it nhat 1 lan.
        model.add(phi[h] + sum_k >= 1).setName(("C10_" + std::to_string(h)).c_str());
    }


    /////////// C14: node_stage
    for (int k = 1; k <= arc_max_stage; k++) {
        IloExpr sum(env);
        for (int i = 0; i < D; i++) {
            for (int j = 1; j <= D; j++) {
                if (i != j) {
                    sum += x[k][i][j] * tau[i][j];
                }
            }
        }

        // o to toi k+1 >= o to den k + thoi gian di chuyen tu k->k+1 (tau_(ij)). (old model).
        // em nghĩ nên đổi thành bằng nhau, a_(k+1) chắc chắn bằng departure stage trước + di chuyển.
        // d[k+1] mới >= d[k]. trường hợp lớn hơn xảy ra khi truck phải đợi drone bay đến.
        model.add(a[k + 1] == d[k] + sum).setName(("C14_" + std::to_string(k) + "_" + std::to_string(k + 1)).c_str());
    }

    ////////// C15: node_stage
    auto M = 1e5;
    for (int k = 1; k <= node_max_stage - 1; k++) {
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            if (k < k_p) {
                // o to phai den k_p tu k trong khoang thoi gian <= dtl.
                // , "C15_" + std::to_string(k) + "_" + std::to_string(k_p)
                model.add(a[k_p] - d[k] <= z[k][k_p] * dtl + (1 - z[k][k_p]) * M).setName(
                        ("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());

                IloExpr rhs(env);
                for (int i = 0; i <= D; i++) {
                    for (int h: C) {
                        if (h != i && i != D) {
                            if (tau_prime[i][h] <= dtl) {
                                rhs += Y[k][i][h] * tau_prime[i][h];
                            }
                        }

                        if (h != i && i != 0) {
                            if (tau_prime[h][i] <= dtl) {
                                rhs += W[k_p][i][h] * tau_prime[h][i];
                            }
                        }
                    }
                    // vehicle phai doi drone truoc khi di chuyen.
                }
                model.add(d[k_p] - d[k] + (1 - z[k][k_p]) * 2 * dtl >= rhs).setName(
                        ("C21_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
            }
        }
    }
    for (int k = 1; k <= node_max_stage - 1; k++) {
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            IloExpr sum_Y(env);
            IloExpr sum_W(env);
            for (int i = 0; i <= D; i++) {
                for (int h: C) {
                    if (i != D) {
                        sum_Y += Y[k][i][h] * tau_prime[i][h];
                    }

                    sum_W += W[k_p][i][h] * tau_prime[h][i];
                }
            }
            //            model.add(d[k_p] >= d[k] + sum_Y + sum_W);
        }
    }
    //    IloExpr sum_sl_at_O(env);
    //    IloExpr sum_sl_sr(env);
    //    for (int k = 1; k <= node_max_stage-1; k++) {
    //        for (int h:C) {
    //            sum_sl_at_O += Y[k][O][h];
    //            sum_sl_sr += (sl + sr) * phi[h];
    //        }
    //    }

    // - sum_sl_at_O * sl + sum_sl_sr)
    for (int k = 1; k <= node_max_stage - 1; k++) {
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            IloExpr sum(env);
            for (int k_p_p = k; k_p_p <= k_p - 1; k_p_p++) {
                for (int i = 0; i < D; i++) {
                    for (int j = 1; j <= D; j++) {
                        if (i != j) {
                            sum += x[k_p_p][i][j] * tau[i][j];
                        }
                    }
                }
            }
            model.add(sum - M * (1 - z[k][k_p]) <= dtl);
        }
    }
    model.add(IloMinimize(env, d[node_max_stage]));
    cplex.exportModel("cplex_model_1.lp");
    start_val.add(X[2][9]);
    start_val.add(X[3][2]);
    start_val.add(X[4][4]);
    start_val.add(X[5][6]);
    start_val.add(X[6][7]);
    start_val.add(X[7][10]);
    start_val.add(X[8][11]);
    start_val.add(phi[1]);
    start_val.add(phi[3]);
    start_val.add(phi[5]);
    start_val.add(phi[8]);
    start_val.add(Y[0][1][1]);
    start_val.add(W[0][1][1]);
    primalSol.add(true);
    primalSol.add(true);
    primalSol.add(true);
    primalSol.add(true);
    primalSol.add(true);
    primalSol.add(true);
    primalSol.add(true);
    primalSol.add(true);
    primalSol.add(true);
    primalSol.add(true);
    primalSol.add(true);
    //    primalSol.add(true);
    //    primalSol.add(true);
    //    start_val.add(X[3][])
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << "Starting resolve......................." << std::endl;
    cplex.addMIPStart(start_val, primalSol);
    // Solve the model
    if (!cplex.solve()) {
        // Check if the problem is infeasible
        if (cplex.getStatus() == IloAlgorithm::Infeasible) {
            // Handle infeasibility
            std::cout << "The problem is infeasible." << std::endl;
            std::cout << "Infeasibility at: " << cplex.getInfeasibility(c2) << std::endl;
            // You can also retrieve the infeasible constraints using cplex.getInfeasibility() method
        } else {
            // Handle other solver errors
            std::cerr << "Solver error: " << cplex.getStatus() << std::endl;
        }
    } else {
        double obj = 0;
        std::cout << "Feasible solution found!" << std::endl;
        std::cout << "Truck nodes:" << std::endl;
        for (int k = 1; k <= node_max_stage; k++) {
            for (int i = 0; i <= D; i++) {
                auto X_val = cplex.getValue(X[k][i]);
                //                start_val.add(X[k][i]);
                //                primalSol.add(X_val);
                //std::cout << "k = " << k << ", i = " << i << ":" << X_val << std::endl;
                if (abs(X_val - 1) < 1e-6) {
                    auto d_k = cplex.getValue(d[k]);
                    std::cout << "Stage " << k << " at customer " << i << " with departure time is: " << d_k <<
                              std::endl;
                    break;
                }
            }
        }
        std::cout << "Truck arcs:" << std::endl;
        for (int k = 1; k <= arc_max_stage; k++) {
            for (int i = 0; i < D; i++)
                for (int j = 1; j <= D; j++)
                    if (i != j) {
                        auto X_val = cplex.getValue(x[k][i][j]);
                        //std::cout << "k = " << k << ", i = " << i << ":" << X_val << std::endl;
                        if (abs(X_val - 1) < 1e-6) {
                            std::cout << "Arc " << k << " connecting " << i << " and " << j
                                      << " with cost " << tau[i][j] << " " << std::endl;
                            break;
                        }
                    }
        }

        std::cout << "Drone served customers" << std::endl;
        //IloNumArray phi_val(env);
        //cplex.getValues(phi_val, phi);


        for (int k = 1; k < node_max_stage; k++)
            for (int kp = k + 1; kp <= node_max_stage; kp++) {
                //std::cout << k << " " << kp << std::endl;
                if (abs(cplex.getValue(z[k][kp]) - 1) < 1e-6) {
                    std::cout << "Drone flies from stage " << k << " to stage " << kp << std::endl;
                }
            }
        for (int h: C)
            for (int k = 1; k < node_max_stage; k++)
                for (int kp = k + 1; kp <= node_max_stage; kp++) {
                    //std::cout << k << " " << kp << std::endl;
                    if (abs(cplex.getValue(Z[h][k][kp]) - 1) < 1e-6) {
                        std::cout << "Drone flies from stage " << k << " to stage " << kp << " to serve customer " << h
                                  << std::endl;
                    }
                }

        for (int h: C) {
            //std::cout << "AAAAAAAAAAA Customer " << h << " served by drone." << std::endl;

            if (abs(cplex.getValue(phi[h]) - 1) < 1e-6) {
                std::cout << "Customer " << h << " served by drone." << std::endl;
                int sv_i = -1, sv_j = -1, sv_k = -1, sv_kp = -1;
                for (int k = 1; k <= node_max_stage; k++) {
                    for (int i = 0; i <= D; i++)
                        if (i != h) {
                            try {
                                //std::cout << "from stage " << k << " at customer " << i << " to serve" << h << std::endl;
                                auto Y_val = cplex.getValue(Y[k][i][h]);
                                if (abs(Y_val - 1) < 1e-6) {
                                    sv_i = i;
                                    sv_k = k;
                                    //std::cout << "XXX from stage " << k << " at customer " << i << " to serve" << h << std::endl;
                                }

                                //std::cout << "to stage " << k << " at customer" << i << " from " << h << std::endl;
                                auto W_val = cplex.getValue(W[k][i][h]);
                                if (abs(W_val - 1) < 1e-6) {
                                    sv_j = i;
                                    sv_kp = k;
                                    //std::cout << "ZZZ to stage " << k << " at customer" << i << " from " << h << std::endl;
                                }
                            } catch (...) {
                            }
                        }
                }
                ////assert(abs(cplex.getValue(z[sv_k][sv_kp]) - 1.0) < 1e-6);

                std::cout << "Drone fly from " << sv_i << " at stage " << sv_k <<
                          " to serve " << h << " and then fly back to " << sv_j
                          << " at stage " << sv_kp << ". " << std::endl;
                double travel_time = tau_prime[sv_i][h] + tau_prime[h][sv_j];
                auto drone_arrival_time = cplex.getValue(d[sv_k]) + travel_time;
                auto vehicle_departure_time = cplex.getValue(d[sv_kp]);
                auto truck_arrival_time = cplex.getValue(a[sv_kp]);
                std::cout << "Start arc cost: "
                          << tau_prime[sv_i][h] << ", end arc cost: " << tau_prime[h][sv_j] <<
                          ". Total drone travel time: " << travel_time << std::endl;

                //                    std::cout << "Drone/Vehicle time: " << drone_arrival_time << " " << vehicle_departure_time << std::endl;
                std::cout << "Drone arrival time: " << drone_arrival_time << std::endl;
                std::cout << "Truck arrival time: " << truck_arrival_time << std::endl;
                std::cout << "Truck departure time = max(d/a, t/a): " << vehicle_departure_time << std::endl;
                assert(drone_arrival_time <= vehicle_departure_time);
                assert(abs(cplex.getValue(Z[h][sv_k][sv_kp]) - 1.0) < 1e-6);

                assert(abs(cplex.getValue(z[sv_k][sv_kp]) - 1.0) < 1e-6);
            }
        }
        //            std::cout << cplex.getValue(Z[1][2][4]);
        std::cout << "Done!" << std::endl;
        std::cout << "Objective value: " << cplex.getObjValue() << std::endl;
        exit(0);
    }

    return Result();
}

Result Solver::mvdSolverWithLR(int n_thread, int e) {
    //    try {
    auto tau = instance->tau;
    auto tau_prime = instance->tau_prime;
    auto dtl = e;
    auto n = instance->num_node;
    std::vector<int> C;
    for (int i = 0; i < n + 1; i++) {
        if (i != 0 && i != n) {
            C.push_back(i);
        }
    }

    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    cplex.setParam(IloCplex::Param::Emphasis::MIP, CPX_MIPEMPHASIS_OPTIMALITY);
    cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);

    auto O = 0;
    auto D = n;
    auto node_max_stage = n + 1;
    auto arc_max_stage = node_max_stage - 1;


    // Khai bao bien
    //
    // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
    //đường đi của vehicle là i; k \in 1..n;
    //
    IloArray<IloBoolVarArray> X(env, node_max_stage + 1);
    for (int k = 1; k <= node_max_stage; k++) {
        X[k] = IloBoolVarArray(env, D + 1);
        for (int i = 0; i <= D; i++) {
            X[k][i] = IloBoolVar(env);
            auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
            //std::cout << v_name << std::endl;
            X[k][i].setName(v_name.c_str());
            if (k > 1 && i == 0) X[k][0].setBounds(0, 0);

        }
    }

    model.add(X[1][0] == 1).setName("start depot is the first node");
    model.add(X[1][D] == 0).setName("ending depot cannot be the first node");

    for (int k = 1; k <= node_max_stage; k++) {
        IloExpr sum(env);
        for (int i = 0; i <= D; i++)
            sum += X[k][i];
        model.add(sum <= 1).setName(("C20_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
    }

    IloExpr arrival_depot(env);
    for (int k = 1; k <= node_max_stage; k++) {
        arrival_depot += X[k][D];
    }
    model.add(arrival_depot == 1).setName("C21_arrival_depot_once");

    // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
    // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
    IloArray<IloArray<IloBoolVarArray> > x(env, arc_max_stage + 1);
    for (int k = 1; k <= arc_max_stage; k++) {
        x[k] = IloArray<IloBoolVarArray>(env, D);
        for (int i = 0; i < D; i++) {
            x[k][i] = IloBoolVarArray(env, D + 1);

            for (int j = 1; j <= D; j++)
                if (i != j) {
                    x[k][i][j] = IloBoolVar(env);
                    auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
                    x[k][i][j].setName(v_name.c_str());
                }
        }
    }


    //// phi^h equals to 1 if customer h is served by the drone
    IloBoolVarArray phi(env, n);
    for (int h: C) {
        phi[h] = IloBoolVar(env);
        auto v_name = "phi_" + std::to_string(h);
        phi[h].setName(v_name.c_str());
    }

    for (int heavy: instance->heavy) {
        if (heavy != D + 1) {
            model.add(phi[heavy] == 0);
        }
    }
    IloArray<IloArray<IloBoolVarArray> > Y(env, node_max_stage + 1), W(env, node_max_stage + 1);
    for (int k = 1; k <= node_max_stage; k++) {
        Y[k] = IloArray<IloBoolVarArray>(env, D + 1);
        W[k] = IloArray<IloBoolVarArray>(env, D + 1);

        for (int i = 0; i <= D; i++) {
            Y[k][i] = IloBoolVarArray(env, C.size() + 1);
            W[k][i] = IloBoolVarArray(env, C.size() + 1);
            for (int h: C)
                if (i != h) {
                    Y[k][i][h] = IloBoolVar(env);
                    Y[k][i][h].setName(("Y_" + std::to_string(k) + "_"
                                        + std::to_string(i) + "_" + std::to_string(h)).c_str());

                    W[k][i][h] = IloBoolVar(env);
                    W[k][i][h].setName(("W_" + std::to_string(k) + "_"
                                        + std::to_string(i) + "_" + std::to_string(h)).c_str());

                    if (i == 0 && k > 1) {
                        model.add(Y[k][i][h] == 0);
                    }
                    if (i == D && k == 1) {
                        model.add(W[k][i][h] == 0);
                    }
                    if (tau_prime[i][h] > dtl - sr) {
                        model.add(Y[k][i][h] == 0);
                    }
                    if (tau_prime[h][i] > dtl - sr) {
                        model.add(W[k][i][h] == 0);
                    }
                }
        }
    }


    //// C17 - $X^k_i \geq \sum_h X^k_{ih}$ (C17) - chỉ bay drone ở nơi mà xe ở đó
    for (int k = 1; k <= node_max_stage; k++) {
        for (int i = 0; i <= D; i++) {
            IloExpr expr(env);
            for (int h: C)
                if (h != i)
                    if (tau_prime[i][h] <= dtl - sr) {
                        expr += Y[k][i][h];
                    }
            model.add(expr <= X[k][i]).setName(("C17_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
        }
    }
    // $X^k_i \geq \sum_h Y^k_{ih}$ (C17p) : chỉ bay drone tới nơi mà xe ở đó
    for (int k = 1; k <= node_max_stage; k++) {
        for (int i = 0; i <= D; i++) {
            IloExpr expr(env);
            for (int h: C)
                if (h != i)
                    if (tau_prime[h][i] <= dtl - sr) {
                        expr += W[k][i][h];
                    }
            model.add(expr <= X[k][i]).setName(("C17p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
        }
    }

    // $\sum_{i} X^k_{ih}\tau^D_{ih} + \sum_{i} Y^{k'}_{ih}\tau^D_{hi} \leq D_d$
    //- drone duration constraint cho mỗi $h$ (C19)
    for (int h: C) {
        IloExpr expr(env);

        for (int k = 1; k <= node_max_stage; k++) {
            for (int i = 0; i <= D; i++) {
                if (i != h && i != D && tau_prime[i][h] <= dtl - sr)
                    expr += Y[k][i][h] * tau_prime[i][h];

                if (i != h && i != 0 && tau_prime[h][i] <= dtl - sr)
                    expr += W[k][i][h] * tau_prime[h][i];
            }
        }

        model.add(expr <= (dtl - sr) * phi[h]).setName(("C19_" + std::to_string(h)).c_str());
    }
    //exit(0);

    //// aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
    IloArray<IloBoolVarArray> z(env, node_max_stage);
    for (int k = 1; k <= node_max_stage - 1; k++) {
        z[k] = IloBoolVarArray(env, node_max_stage + 1);
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            {
                z[k][k_p] = IloBoolVar(env);
                auto v_name = "z_" + std::to_string(k) + "_" + std::to_string(k_p);
                z[k][k_p].setName(v_name.c_str());
            }
        }
    }

    //// aux var Z_{k, k_p, h}: sortie launch from k and rendezvous at k_p.
    IloArray<IloArray<IloBoolVarArray> > Z(env, node_max_stage);
    for (int h: C) {
        Z[h] = IloArray<IloBoolVarArray>(env, node_max_stage + 1);
        for (int k = 1; k <= node_max_stage - 1; k++) {
            Z[h][k] = IloBoolVarArray(env, node_max_stage + 1);
            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                {
                    Z[h][k][k_p] = IloBoolVar(env);
                    auto v_name = "Z_" + std::to_string(h) + "_" + std::to_string(k) + "_" + std::to_string(k_p);
                    Z[h][k][k_p].setName(v_name.c_str());
                }
            }
        }
    }

    // $Z_{kk'} = \sum_{h}Z^h_{kk'}$: mỗi cặp (k,k') chỉ phục vụ tối đa một khách hàng (C22)
    for (int k = 1; k < node_max_stage; k++)
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            IloExpr expr(env);
            for (int h: C) {
                expr += Z[h][k][k_p];
            }
            model.add(expr == z[k][k_p]).setName(("C22_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
        }


    //C20:$\sum_{k'>k}Z_{kk'} = \sum_{i,h}Y^k_{ih}$ : với mỗi $k$,
    //ràng buộc liên kết drone đi ra từ stage $k$ và đoạn mà oto di chuyển không có drone. (C20)

    for (int h: C) {
        for (int k = 1; k <= node_max_stage - 1; k++) {
            IloExpr expr(env);
            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                {
                    expr += Z[h][k][k_p];
                }
            }

            for (int i = 0; i < D; i++) {
                if (i != h && tau_prime[i][h] <= dtl - sr) {
                    expr -= Y[k][i][h];
                }
            }
            model.add(expr == 0).setName(("C20_" + std::to_string(k) + "_" + std::to_string(h)).c_str());
        }
    }

    for (int h: C) {
        for (int k_p = 2; k_p <= node_max_stage; k_p++) {
            IloExpr expr(env);
            for (int k = 1; k < k_p; k++) {
                {
                    expr += Z[h][k][k_p];
                }
            }

            for (int i = 1; i <= D; i++) {
                if (i != h && tau_prime[h][i] <= dtl - sr) {
                    expr -= W[k_p][i][h];
                }
            }
            model.add(expr == 0).setName(("C20p_" + std::to_string(k_p)
                                          + "_" + std::to_string(h)).c_str());
        }
    }

    // arrival\departure variables a and d.
    IloNumVarArray a(env, node_max_stage + 1);
    IloNumVarArray d(env, node_max_stage + 1);
    for (int k = 1; k <= node_max_stage; k++) {
        a[k] = IloNumVar(env);
        auto v_name = "a_" + std::to_string(k);
        a[k].setName(v_name.c_str());
        d[k] = IloNumVar(env);
        v_name = "d_" + std::to_string(k);
        d[k].setName(v_name.c_str());
        model.add(d[k] >= a[k]).setName(("C13_" + std::to_string(k)).c_str());
    }

    model.add(a[1] == 0).setName("arrival to depot at time 0");
    model.add(d[1] == 0).setName("depart from depot at time 0");;

    ////////// Constraint C1
    for (int k = 1; k < node_max_stage; k++) {
        for (int i = 0; i < D; i++) {
            {
                // i == D => khong co i -> j.
                IloExpr sum(env);
                for (int j = 1; j <= D; j++) {
                    if (i != j) {
                        sum += x[k][i][j];
                    }
                }

                // neu node i la node_stage k, i khac D thi kieu gi cung co canh i, j.
                model.add(X[k][i] == sum).setName(("C1_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
            }
        }
    }

    for (int k = 2; k <= node_max_stage; k++) {
        for (int i = 1; i <= D; i++) {
            {
                IloExpr sum(env);
                for (int j = 0; j < D; j++) {
                    if (i != j) {
                        sum += x[k - 1][j][i];
                    }
                }
                // arcs entering i at stage k.
                model.add(X[k][i] == sum).setName(("C1p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
            }
        }
    }

    //////////// C2 - depart from the depot
    IloExpr C2(env);
    for (int i = 1; i <= D; i++) {
        C2 += x[1][O][i];
    }

    IloConstraint c2(C2 == 1);
    model.add(c2).setName("C2");
    // , "C2"

    ///////////// C3: arc_stage
    IloExpr C3(env);
    for (int k = 2; k <= node_max_stage; k++) {
        {
            C3 += X[k][D];
        }
    }
    // , "C3"
    model.add(C3 == 1).setName("C3"); // arrival to depot

    // $R_{k} = \sum_{k'}Z_{kk'}$: các đoạn bắt đầu từ k (C23)
    IloBoolVarArray R(env, node_max_stage + 1);
    for (int k = 1; k < node_max_stage; k++) {
        R[k].setName(("R_" + std::to_string(k)).c_str());
    }

    for (int k = 1; k < node_max_stage; k++) {
        IloExpr expr(env);
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            expr += z[k][k_p];
        }

        model.add(R[k] == expr).setName(("C23_" + std::to_string(k)).c_str());
    }

    // modified C7
    for (int k = 1; k <= node_max_stage - 1; k++) {
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            for (int l = k + 1; l < k_p; l++) {
                {
                    // tranh drone bay cac doan giao nhau.
                    if (k < l) {
                        model.add(z[k][k_p] + R[l] <= 1).setName(("C7m_" + std::to_string(k)
                                                                  + "_" + std::to_string(k_p) + "_" + std::to_string(l))
                                                                         .c_str());
                    } else {
                        //                        model.add(R[k] <= 1).setName(("C7k_" + std::to_string(k);
                        //    + "_" + std::to_string(k_p) + "_" + std::to_string(l)).c_str());
                    }
                }
            }
        }
    }


    for (int i = 0; i < D; i++) {
        for (int k = 1; k <= node_max_stage - 1; k++) {
            IloExpr lhs(env);

            for (int h: C) {
                if (i != h) {
                    lhs += Y[k][i][h];
                }
            }
            model.add(lhs <= X[k][i]).setName(("C8_launch_" + std::to_string(i) + "_" + std::to_string(k)).c_str());
        }
    }

    for (int j = 1; j <= D; j++) {
        for (int k_p = 2; k_p <= node_max_stage; k_p++) {
            IloExpr lhs(env);

            for (int h: C) {
                if (h != j) {
                    lhs += W[k_p][j][h];
                }
            }

            model.add(lhs <= X[k_p][j]).setName(
                    ("C8_rendezvous_" + std::to_string(j) + "_" + std::to_string(k_p)).c_str());
        }
    }


    // $\phi_h = \sum_{k,i}X^k_{ih} = \sum_{k,i}Y^k_{ih}$
    // - chỉ có duy nhất một điểm xuất phát và môt điểm đích cho
    //mỗi khách hàng $h$ được phục vụ bởi drone (C18)
    for (int h: C) {
        IloExpr rhs(env);
        for (int i = 0; i < D; i++) {
            if (i != h && tau_prime[i][h] <= dtl - sr)
                for (int k = 1; k <= node_max_stage - 1; k++) {
                    rhs += Y[k][i][h];
                }
        }
        // consistency constraint cho sortie phuc vu h.
        model.add(phi[h] == rhs).setName(("C18_" + std::to_string(h)).c_str());
    }


    for (int h: C) {
        IloExpr rhs(env);
        for (int i = 1; i <= D; i++) {
            if (i != h && tau_prime[h][i] <= dtl - sr)
                for (int k = 2; k <= node_max_stage; k++) {
                    rhs += W[k][i][h];
                }
        }
        // consistency constraint cho sortie phuc vu h.
        model.add(phi[h] == rhs).setName(("C18p_" + std::to_string(h)).c_str());
    }

    //////////// C10: node_stage
    for (int h: C) {
        IloExpr sum_k(env);
        for (int k = 2; k < node_max_stage; k++) {
            sum_k += X[k][h];
        }
        // phuc vu h it nhat 1 lan.
        model.add(phi[h] + sum_k >= 1).setName(("C10_" + std::to_string(h)).c_str());
    }


    /////////// C14: node_stage
    for (int k = 1; k <= arc_max_stage; k++) {
        IloExpr sum(env);
        for (int i = 0; i < D; i++) {
            for (int j = 1; j <= D; j++) {
                if (i != j) {
                    sum += x[k][i][j] * tau[i][j];
                }
            }
        }

        // o to toi k+1 >= o to den k + thoi gian di chuyen tu k->k+1 (tau_(ij)). (old model).
        // em nghĩ nên đổi thành bằng nhau, a_(k+1) chắc chắn bằng departure stage trước + di chuyển.
        // d[k+1] mới >= d[k]. trường hợp lớn hơn xảy ra khi truck phải đợi drone bay đến.
        model.add(a[k + 1] == d[k] + sum).setName(("C14_" + std::to_string(k) + "_" + std::to_string(k + 1)).c_str());
    }

    ////////// C15: node_stage
    ///// big M calculation
    auto M = 0;
    for (int k = 1; k < node_max_stage; k++) {
        M += tau[k - 1][k];
    }

    for (int k = 2; k < node_max_stage / 2; k++) {
        model.add(X[k][D] == 0);
    }
    for (int k = 1; k <= node_max_stage - 1; k++) {
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            if (k < k_p) {
                // o to phai den k_p tu k trong khoang thoi gian <= dtl.
                // , "C15_" + std::to_string(k) + "_" + std::to_string(k_p)
                model.add(a[k_p] - d[k] <= z[k][k_p] * (dtl - sr) + (1 - z[k][k_p]) * M).setName(
                        ("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());

                IloExpr rhs(env);
                for (int i = 0; i <= D; i++) {
                    for (int h: C) {
                        if (h != i && i != D) {
                            if (tau_prime[i][h] <= dtl - sr) {
                                rhs += Y[k][i][h] * tau_prime[i][h];
                            }
                        }

                        if (h != i && i != 0) {
                            if (tau_prime[h][i] <= dtl - sr) {
                                rhs += W[k_p][i][h] * tau_prime[h][i];
                            }
                        }
                    }
                }
                // vehicle phai doi drone truoc khi di chuyen.
                //                model.add(d[k_p] - d[k] + (1 - z[k][k_p]) * 2 * (dtl-sr) >= rhs).setName(("C21_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
                //                model.add(d[k_p] - d[k] >= z[k][k_p] * sr - (1-z[k][k_p]) * 100000 + rhs);
            }
        }
    }

    for (int k = 1; k <= node_max_stage - 1; k++) {
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            if (k < k_p) {
                IloExpr sum_length_y_k(env);
                IloExpr sum_length_w_k_p(env);
                IloExpr sum_w_k_p(env);
                IloExpr sum_y_k_p(env);

                for (int i = 0; i < D; i++) {
                    for (int h: C) {
                        if (i != h) {
                            sum_length_y_k += Y[k][i][h] * tau_prime[i][h];
                        }
                    }
                }
                for (int i = 0; i < D; i++) {
                    for (int h: C) {
                        if (i != h) {
                            sum_y_k_p += Y[k_p][i][h] * sl;
                        }
                    }
                }
                for (int j = 1; j <= D; j++) {
                    for (int h: C) {
                        if (h != j) {
                            sum_length_w_k_p += W[k_p][j][h] * tau_prime[h][j];
                        }
                    }
                }

                for (int j = 1; j <= D; j++) {
                    for (int h: C) {
                        if (h != j) {
                            sum_w_k_p += W[k_p][j][h] * sr;
                        }
                    }
                }

                model.add(
                        d[k_p] >=
                        d[k] + sum_length_y_k + sum_length_w_k_p + sum_y_k_p + sum_w_k_p - (1 - z[k][k_p]) * M);
            }
        }
    }
    for (int k = 2; k <= node_max_stage; k++) {
        IloExpr sum(env);
        IloExpr sum_w_K(env);
        IloExpr sum_y_K(env);
        for (int i = 0; i < D; i++) {
            for (int j = 1; j <= D; j++) {
                if (i != j) {
                    sum += x[k - 1][i][j] * tau[i][j];
                }
            }
        }
        for (int i = 0; i < D; i++) {
            for (int h: C) {
                if (i != h && k != node_max_stage) {
                    sum_y_K += sl * Y[k][i][h];
                }
            }
        }

        for (int i = 1; i <= D; i++) {
            for (int h: C) {
                if (i != h) {
                    sum_w_K += sr * W[k][i][h];
                }
            }
        }
        auto constr_name = "CC_d_" + std::to_string(k) + "_constr";
        model.add(d[k] >= d[k - 1] + sum + sum_w_K + sum_y_K).setName(constr_name.c_str());
    }
    // for (int i = 1; i < D; i++) {
    //     for (int h: C) {
    //         if (i != h) {
    //             IloExpr sumY(env), sumK(env);
    //             for (int k = 1; k < node_max_stage; k++) {
    //                 for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
    //                     if (k_p > k) {
    //                         model.add(Y[k][i][h] + W[k_p][i][h] <= 1);
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

    // (k,h): k chỉ có thể là đến hoặc đi của khách hàng h.
    for (int k = 2; k < node_max_stage; k++) {
        for (int h : C) {
            IloExpr expr(env);
            for (int i : C)
                if (i != h && tau_prime[i][h] <= dtl)
                    expr += Y[k][i][h];
            for (int i : C)
                if (i != h && tau_prime[h][i] <= dtl)
                    expr += W[k][i][h];

            model.add(expr <= phi[h]).setName(("Cxyz_" + std::to_string(k) + "_" + std::to_string(h)).c_str());

        }
    }

    // LB constraint
    IloExpr lb_truck_tour(env);
    for (int k = 1; k < node_max_stage; k++) {
        for (int i = 0; i < D; i++)
            for (int j = 1; j <= D; j++)
                if (i != j)
                    lb_truck_tour += x[k][i][j] * tau[i][j];
    }
    model.add(lb_truck_tour <= d[node_max_stage]).setName("Lower_bound_obj");

    IloExpr lb_drone_tour(env);
    for (int k = 1; k < node_max_stage; k++) {
        for (int i = 0; i< D; i++) {
            for (int h:C) {
                if (i != h) {
                    lb_drone_tour += Y[k][i][h] * tau_prime[i][h];
                }
            }
        }
    }
    for (int k = 2; k <= node_max_stage; k++) {
        for (int j = 1; j <= D; j++) {
            for (int h:C) {
                if (j != h) {
                    lb_drone_tour += W[k][j][h] * tau_prime[h][j];
                }
            }
        }
    }
    model.add(lb_drone_tour <= d[node_max_stage]);

    for (int k = 2; k < node_max_stage/2; k++) {
        model.add(X[k][D] == 0);
    }

    model.add(IloMinimize(env, d[node_max_stage]));
    cplex.exportModel("cplex_model_1.lp");
    std::vector<Sortie> st;
    double obj = 0;
    // Solve the model

    auto startTime = std::chrono::high_resolution_clock::now();
    cplex.solve();
    auto endTime = std::chrono::high_resolution_clock::now();
    // Calculate the elapsed time
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // Check if the problem is infeasible

    if (cplex.getStatus() == IloAlgorithm::Infeasible) {
        // Handle infeasibility
        std::cout << "The problem is infeasible." << std::endl;
        std::cout << "Infeasibility at: " << cplex.getInfeasibility(c2) << std::endl;
        // You can also retrieve the infeasible constraints using cplex.getInfeasibility() method
    }

    std::cout << "Feasible solution found!" << std::endl;
    std::cout << "Truck nodes:" << std::endl;
    for (int k = 1; k <= node_max_stage; k++) {
        for (int i = 0; i <= D; i++) {
            auto X_val = cplex.getValue(X[k][i]);
            //std::cout << "k = " << k << ", i = " << i << ":" << X_val << std::endl;
            if (abs(X_val - 1) < 2e-5) {
                auto d_k = cplex.getValue(d[k]);
                auto a_k = cplex.getValue(a[k]);
                std::cout << "Stage " << k << " at customer " << i << " with arrival time is: " << a_k << std::endl;
                std::cout << "Stage " << k << " at customer " << i << " with departure time is: " << d_k << std::endl;
                break;
            }
        }
    }
    std::cout << "Truck arcs:" << std::endl;
    std::map<int, std::pair<int, int> > map_stage_truck_arc;
    for (int k = 1; k <= arc_max_stage; k++) {
        for (int i = 0; i < D; i++)
            for (int j = 1; j <= D; j++)
                if (i != j) {
                    auto X_val = cplex.getValue(x[k][i][j]);
                    //std::cout << "k = " << k << ", i = " << i << ":" << X_val << std::endl;
                    if (abs(X_val - 1) < 2e-5) {
                        std::cout << "Arc " << k << " connecting " << i << " and " << j
                                  << " with cost " << tau[i][j] << " " << std::endl;
                        obj += tau[i][j];
                        map_stage_truck_arc[k] = std::make_pair(i, j);
                        break;
                    }
                }
    }
    std::cout << "Drone served customers" << std::endl;
    //IloNumArray phi_val(env);
    //cplex.getValues(phi_val, phi);


    for (int k = 1; k < node_max_stage; k++)
        for (int kp = k + 1; kp <= node_max_stage; kp++) {
            //std::cout << k << " " << kp << std::endl;
            if (abs(cplex.getValue(z[k][kp]) - 1) < 1e-5) {
                std::cout << "Drone flies from stage " << k << " to stage " << kp << std::endl;
            }
        }
    for (int h: C)
        for (int k = 1; k < node_max_stage; k++)
            for (int kp = k + 1; kp <= node_max_stage; kp++) {
                //std::cout << k << " " << kp << std::endl;
                if (abs(cplex.getValue(Z[h][k][kp]) - 1) < 1e-5) {
                    std::cout << "Drone flies from stage " << k << " to stage " << kp << " to serve customer " << h <<
                              std::endl;
                }
            }

    for (int h: C) {
        if (abs(cplex.getValue(phi[h]) - 1) < 1e-5) {
            std::cout << cplex.getValue(phi[h]) << std::endl;
            std::cout << "Customer " << h << " served by drone." << std::endl;
            auto sortie = Sortie(h);
            st.push_back(sortie);
            int sv_i = -1, sv_j = -1, sv_k = -1, sv_kp = -1;
            for (int k = 1; k <= node_max_stage; k++) {
                for (int i = 0; i <= D; i++)
                    if (i != h) {
                        try {
                            //std::cout << "from stage " << k << " at customer " << i << " to serve" << h << std::endl;
                            auto Y_val = cplex.getValue(Y[k][i][h]);
                            if (abs(Y_val - 1) < 1e-5) {
                                std::cout << "Y_val: " << Y_val << std::endl;
                                sv_i = i;
                                sv_k = k;
                                //std::cout << "XXX from stage " << k << " at customer " << i << " to serve" << h << std::endl;
                            }

                            //std::cout << "to stage " << k << " at customer" << i << " from " << h << std::endl;
                            auto W_val = cplex.getValue(W[k][i][h]);
                            if (abs(W_val - 1) < 1e-5) {
                                std::cout << "W_val: " << W_val << std::endl;
                                sv_j = i;
                                sv_kp = k;
                                //std::cout << "ZZZ to stage " << k << " at customer" << i << " from " << h << std::endl;
                            }
                        } catch (...) {
                        }
                    }
            }
            ////assert(abs(cplex.getValue(z[sv_k][sv_kp]) - 1.0) < 1e-6);

            std::cout << "Drone fly from " << sv_i << " at stage " << sv_k <<
                      " to serve " << h << " and then fly back to " << sv_j
                      << " at stage " << sv_kp << ". " << std::endl;
            obj += (sl + sr);
            if (sv_i == O) {
                obj -= sl;
            }
            double drone_travel_time = tau_prime[sv_i][h] + tau_prime[h][sv_j];
            double truck_travel_time = 0;
            for (int k_start = sv_k; k_start <= sv_kp - 1; k_start++) {
                truck_travel_time += tau[map_stage_truck_arc[k_start].first][map_stage_truck_arc[k_start].second];
            }
            std::cout << "Truck travel time from stage " << sv_k << " to " << sv_kp << " is: " << truck_travel_time <<
                      std::endl;
            if (drone_travel_time > truck_travel_time) {
                obj += drone_travel_time - truck_travel_time;
            }
            auto drone_arrival_time = cplex.getValue(d[sv_k]) + drone_travel_time;
            auto vehicle_departure_time = cplex.getValue(d[sv_kp]);
            auto truck_arrival_time = cplex.getValue(a[sv_kp]);
            std::cout << "Start arc cost: "
                      << tau_prime[sv_i][h] << ", end arc cost: " << tau_prime[h][sv_j] << ". Total drone travel time: "
                      << drone_travel_time << std::endl;

            std::cout << "Drone arrival time: " << drone_arrival_time << std::endl;
            std::cout << "Truck arrival time: " << truck_arrival_time << std::endl;

            std::cout << "Truck departure time = max(d/a, t/a) plus (sl/sr): " << vehicle_departure_time << std::endl;
            assert(drone_arrival_time <= vehicle_departure_time);
            assert(abs(cplex.getValue(Z[h][sv_k][sv_kp]) - 1.0) < 1e-5);

            assert(abs(cplex.getValue(z[sv_k][sv_kp]) - 1.0) < 1e-5);
        }
    }

    std::cout << "Done!" << std::endl;
    std::cout << "-------------------------Re-calculated objective-----------------------" << std::endl;
    std::cout << obj << std::endl;
    std::cout << "------------------------------------------------------------------------" << std::endl;


    double c = cplex.getObjValue();
    std::cout << "Solve time: " << duration.count() / 1000.0 << std::endl;
    cplex.end();
    model.end();
    env.end();
    std::cout << "OBJECTIVE VALUE: " << c << ", NUMBER OF SORTIES: " << st.size() << "." << std::endl;
    return Result{c, obj, duration.count() / 1000.0, st};
}

Result Solver::HeuristicFixCallback(int n_thread, int e) {
    try {
        auto tau = instance->tau;
        auto tau_prime = instance->tau_prime;
        auto dtl = e;
        //dtl = 5;
        auto sl = 1, sr = 1;
        auto n = instance->num_node;

        auto c_prime = instance->c_prime;
        std::vector<int> c_prime_0;
        c_prime_0.push_back(0);
        for (int i: c_prime) {
            c_prime_0.push_back(i);
        }
        c_prime_0.push_back(n);

        std::cout << "Printing number of nodes: " << n << std::endl;
        std::vector<int> C;
        std::vector<int> V;
        for (int i = 0; i < n + 1; i++) {
            if (i == 0 || i == n) {
                V.push_back(i);
            } else {
                V.push_back(i);
                C.push_back(i);
            }
        }

        // C_s : set C(customer) union s (source) ; source  = 0
        // C_t : set C(customer) union t (terminal); terminal = n
        // create.
        std::vector<int> c_s;
        std::vector<int> c_t;
        for (int i = 0; i < n + 1; i++) {
            if (i == 0) {
                c_s.push_back(i);
            } else if (i == n) {
                c_t.push_back(i);
            } else {
                c_s.push_back(i);
                c_t.push_back(i);
            }
        }

        std::cout << std::endl;
        IloEnv env;
        IloModel model(env);
        IloCplex cplex(model);

        auto K = C, k_prime = V;
        auto O = 0;
        auto D = n;
        auto node_max_stage = n + 1;
        auto arc_max_stage = node_max_stage - 1;

        // Khai bao bien
        //
        // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
        //đường đi của vehicle là i; k \in 1..n;
        //
        IloArray<IloBoolVarArray> X(env, node_max_stage + 1);
        for (int k = 1; k <= node_max_stage; k++) {
            X[k] = IloBoolVarArray(env, D + 1);

            for (int i = 0; i <= D; i++) {
                {
                    X[k][i] = IloBoolVar(env);
                    auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
                    //std::cout << v_name << std::endl;
                    X[k][i].setName(v_name.c_str());

                    if (k > 1 && i == 0) X[k][0].setBounds(0, 0);
                }
            }
        }

        model.add(X[1][0] == 1).setName("start depot is the first node");
        model.add(X[1][D] == 0).setName("ending depot cannot be the first node");

        for (int k = 1; k <= node_max_stage; k++) {
            IloExpr sum(env);
            for (int i = 0; i <= D; i++)
                sum += X[k][i];
            model.add(sum <= 1).setName(("C20_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
        }

        IloExpr arrival_depot(env);
        for (int k = 1; k <= node_max_stage; k++) {
            arrival_depot += X[k][D];
        }
        model.add(arrival_depot == 1).setName("C21_arrival_depot_once");

        // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
        // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
        IloArray<IloArray<IloBoolVarArray> > x(env, arc_max_stage + 1);
        for (int k = 1; k <= arc_max_stage; k++) {
            x[k] = IloArray<IloBoolVarArray>(env, D);
            for (int i = 0; i < D; i++) {
                x[k][i] = IloBoolVarArray(env, D + 1);

                for (int j = 1; j <= D; j++)
                    if (i != j) {
                        x[k][i][j] = IloBoolVar(env);
                        auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
                        x[k][i][j].setName(v_name.c_str());
                    }
            }
        }


        //// phi^h equals to 1 if customer h is served by the drone
        IloBoolVarArray phi(env, n);
        for (int h: C) {
            phi[h] = IloBoolVar(env);
            auto v_name = "phi_" + std::to_string(h);
            phi[h].setName(v_name.c_str());
        }

        for (int heavy: instance->heavy) {
            model.add(phi[heavy] == 0);
        }
        IloArray<IloArray<IloBoolVarArray> > Y(env, node_max_stage + 1), W(env, node_max_stage + 1);
        for (int k = 1; k <= node_max_stage; k++) {
            Y[k] = IloArray<IloBoolVarArray>(env, D + 1);
            W[k] = IloArray<IloBoolVarArray>(env, D + 1);

            for (int i = 0; i <= D; i++) {
                {
                    Y[k][i] = IloBoolVarArray(env, C.size() + 1);
                    W[k][i] = IloBoolVarArray(env, C.size() + 1);
                    for (int h: C)
                        if (i != h) {
                            Y[k][i][h] = IloBoolVar(env);
                            Y[k][i][h].setName(("Y_" + std::to_string(k) + "_"
                                                + std::to_string(i) + "_" + std::to_string(h)).c_str());

                            W[k][i][h] = IloBoolVar(env);
                            W[k][i][h].setName(("W_" + std::to_string(k) + "_"
                                                + std::to_string(i) + "_" + std::to_string(h)).c_str());

                            if (i == 0 && k > 1) Y[k][i][h].setBounds(0, 0);
                            if (i == D && k == 1) W[k][i][h].setBounds(0, 0);
                            if (tau_prime[i][h] > dtl - sr) Y[k][i][h].setBounds(0, 0);
                            if (tau_prime[h][i] > dtl - sr) W[k][i][h].setBounds(0, 0);
                        }
                }
            }
        }


        //// C17 - $X^k_i \geq \sum_h X^k_{ih}$ (C17) - chỉ bay drone ở nơi mà xe ở đó
        for (int k = 1; k <= node_max_stage; k++) {
            for (int i = 0; i <= D; i++) {
                IloExpr expr(env);
                for (int h: C)
                    if (h != i)
                        if (tau_prime[i][h] <= dtl - sr) {
                            expr += Y[k][i][h];
                        }
                model.add(expr <= X[k][i]).setName(("C17_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
            }
        }
        // $X^k_i \geq \sum_h Y^k_{ih}$ (C17p) : chỉ bay drone tới nơi mà xe ở đó
        for (int k = 1; k <= node_max_stage; k++) {
            for (int i = 0; i <= D; i++) {
                IloExpr expr(env);
                for (int h: C)
                    if (h != i)
                        if (tau_prime[h][i] <= dtl - sr) {
                            expr += W[k][i][h];
                        }
                model.add(expr <= X[k][i]).setName(("C17p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
            }
        }

        // $\sum_{i} X^k_{ih}\tau^D_{ih} + \sum_{i} Y^{k'}_{ih}\tau^D_{hi} \leq D_d$
        //- drone duration constraint cho mỗi $h$ (C19)
        for (int h: C) {
            IloExpr expr(env);

            for (int k = 1; k <= node_max_stage; k++) {
                for (int i = 0; i <= D; i++) {
                    if (i != h && i != D && tau_prime[i][h] <= dtl - sr)
                        expr += Y[k][i][h] * tau_prime[i][h];

                    if (i != h && i != 0 && tau_prime[h][i] <= dtl - sr)
                        expr += W[k][i][h] * tau_prime[h][i];
                }
            }

            model.add(expr <= (dtl - sr) * phi[h]).setName(("C19_" + std::to_string(h)).c_str());
        }
        //exit(0);

        //// aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
        IloArray<IloBoolVarArray> z(env, node_max_stage);
        for (int k = 1; k <= node_max_stage - 1; k++) {
            z[k] = IloBoolVarArray(env, node_max_stage + 1);
            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                {
                    z[k][k_p] = IloBoolVar(env);
                    auto v_name = "z_" + std::to_string(k) + "_" + std::to_string(k_p);
                    z[k][k_p].setName(v_name.c_str());
                }
            }
        }

        //// aux var Z_{k, k_p, h}: sortie launch from k and rendezvous at k_p.
        IloArray<IloArray<IloBoolVarArray> > Z(env, node_max_stage);
        for (int h: C) {
            Z[h] = IloArray<IloBoolVarArray>(env, node_max_stage + 1);
            for (int k = 1; k <= node_max_stage - 1; k++) {
                Z[h][k] = IloBoolVarArray(env, node_max_stage + 1);
                for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                    {
                        Z[h][k][k_p] = IloBoolVar(env);
                        auto v_name = "Z_" + std::to_string(h) + "_" + std::to_string(k) + "_" + std::to_string(k_p);
                        Z[h][k][k_p].setName(v_name.c_str());
                    }
                }
            }
        }

        // $Z_{kk'} = \sum_{h}Z^h_{kk'}$: mỗi cặp (k,k') chỉ phục vụ tối đa một khách hàng (C22)
        for (int k = 1; k < node_max_stage; k++)
            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                IloExpr expr(env);
                for (int h: C) {
                    expr += Z[h][k][k_p];
                }
                model.add(expr == z[k][k_p]).setName(("C22_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
            }


        //C20:$\sum_{k'>k}Z_{kk'} = \sum_{i,h}Y^k_{ih}$ : với mỗi $k$,
        //ràng buộc liên kết drone đi ra từ stage $k$ và đoạn mà oto di chuyển không có drone. (C20)

        for (int h: C) {
            for (int k = 1; k <= node_max_stage - 1; k++) {
                IloExpr expr(env);
                for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                    {
                        expr += Z[h][k][k_p];
                    }
                }

                for (int i = 0; i < D; i++) {
                    if (i != h && tau_prime[i][h] <= dtl - sr) {
                        expr -= Y[k][i][h];
                    }
                }
                model.add(expr == 0).setName(("C20_" + std::to_string(k) + "_" + std::to_string(h)).c_str());
            }
        }

        for (int h: C) {
            for (int k_p = 2; k_p <= node_max_stage; k_p++) {
                IloExpr expr(env);
                for (int k = 1; k < k_p; k++) {
                    {
                        expr += Z[h][k][k_p];
                    }
                }

                for (int i = 1; i <= D; i++) {
                    if (i != h && tau_prime[h][i] <= dtl - sr) {
                        expr -= W[k_p][i][h];
                    }
                }
                model.add(expr == 0).setName(("C20p_" + std::to_string(k_p)
                                              + "_" + std::to_string(h)).c_str());
            }
        }

        // arrival\departure variables a and d.
        IloNumVarArray a(env, node_max_stage + 1);
        IloNumVarArray d(env, node_max_stage + 1);
        for (int k = 1; k <= node_max_stage; k++) {
            a[k] = IloNumVar(env);
            auto v_name = "a_" + std::to_string(k);
            a[k].setName(v_name.c_str());
            d[k] = IloNumVar(env);
            v_name = "d_" + std::to_string(k);
            d[k].setName(v_name.c_str());
            model.add(d[k] >= a[k]).setName(("C13_" + std::to_string(k)).c_str());
        }

        model.add(a[1] == 0).setName("arrival to depot at time 0");
        model.add(d[1] == 0).setName("depart from depot at time 0");;

        ////////// Constraint C1
        for (int k = 1; k < node_max_stage; k++) {
            for (int i = 0; i < D; i++) {
                {
                    // i == D => khong co i -> j.
                    IloExpr sum(env);
                    for (int j = 1; j <= D; j++) {
                        if (i != j) {
                            sum += x[k][i][j];
                        }
                    }

                    // neu node i la node_stage k, i khac D thi kieu gi cung co canh i, j.
                    model.add(X[k][i] == sum).setName(("C1_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
                }
            }
        }

        for (int k = 2; k <= node_max_stage; k++) {
            for (int i = 1; i <= D; i++) {
                {
                    IloExpr sum(env);
                    for (int j = 0; j < D; j++) {
                        if (i != j) {
                            sum += x[k - 1][j][i];
                        }
                    }
                    // arcs entering i at stage k.
                    model.add(X[k][i] == sum).setName(("C1p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
                }
            }
        }

        //////////// C2 - depart from the depot
        IloExpr C2(env);
        for (int i = 1; i <= D; i++) {
            C2 += x[1][O][i];
        }

        IloConstraint c2(C2 == 1);
        model.add(c2).setName("C2");
        // , "C2"

        ///////////// C3: arc_stage
        IloExpr C3(env);
        for (int k = 2; k <= node_max_stage; k++) {
            {
                C3 += X[k][D];
            }
        }
        // , "C3"
        model.add(C3 == 1).setName("C3"); // arrival to depot

        // $R_{k} = \sum_{k'}Z_{kk'}$: các đoạn bắt đầu từ k (C23)
        IloBoolVarArray R(env, node_max_stage + 1);
        for (int k = 1; k < node_max_stage; k++) {
            R[k].setName(("R_" + std::to_string(k)).c_str());
        }

        for (int k = 1; k < node_max_stage; k++) {
            IloExpr expr(env);
            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                expr += z[k][k_p];
            }

            model.add(R[k] == expr).setName(("C23_" + std::to_string(k)).c_str());
        }


        //////// C7: node_stage
        //for (int k = 1; k <= node_max_stage - 1; k++) {
        //    for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
        //        for (int l = k; l < k_p; l++) {
        //            for (int l_p = l + 1; l_p <= node_max_stage; l_p++) {
        //                //if ((k != l) || (k_p != l_p))
        //                if ((k != l) || (k == l && k_p < l_p))
        //                {

        //                    // tranh drone bay cac doan giao nhau.
        //                    model.add(z[k][k_p] + z[l][l_p] <= 1).setName(("C7_" + std::to_string(k) + "_" + std::to_string(k_p) + "_" + std::to_string(l) + "_" + std::to_string(l_p)).c_str());
        //                }
        //            }
        //        }
        //    }
        //}

        // modified C7
        for (int k = 1; k <= node_max_stage - 1; k++) {
            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                for (int l = k + 1; l < k_p; l++) {
                    {
                        // tranh drone bay cac doan giao nhau.
                        if (k < l) {
                            model.add(z[k][k_p] + R[l] <= 1).setName(("C7m_" + std::to_string(k)
                                                                      + "_" + std::to_string(k_p) + "_" +
                                                                      std::to_string(l)).c_str());
                        } else {
                            //model.add(R[k] <= 1).setName(("C7k_" + std::to_string(k)
                            //    + "_" + std::to_string(k_p) + "_" + std::to_string(l)).c_str());
                        }
                    }
                }
            }
        }


        for (int i = 0; i < D; i++) {
            for (int k = 1; k <= node_max_stage - 1; k++) {
                IloExpr lhs(env);

                for (int h: C) {
                    if (i != h) {
                        lhs += Y[k][i][h];
                    }
                }
                model.add(lhs <= X[k][i]).setName(("C8_launch_" + std::to_string(i) + "_" + std::to_string(k)).c_str());
            }
        }

        for (int j = 1; j <= D; j++) {
            for (int k_p = 2; k_p <= node_max_stage; k_p++) {
                IloExpr lhs(env);

                for (int h: C) {
                    if (h != j) {
                        lhs += W[k_p][j][h];
                    }
                }

                model.add(lhs <= X[k_p][j]).setName(
                        ("C8_rendezvous_" + std::to_string(j) + "_" + std::to_string(k_p)).c_str());
            }
        }


        // $\phi_h = \sum_{k,i}X^k_{ih} = \sum_{k,i}Y^k_{ih}$
        // - chỉ có duy nhất một điểm xuất phát và môt điểm đích cho
        //mỗi khách hàng $h$ được phục vụ bởi drone (C18)
        for (int h: C) {
            IloExpr rhs(env);
            for (int i = 0; i < D; i++) {
                if (i != h && tau_prime[i][h] <= dtl - sr)
                    for (int k = 1; k <= node_max_stage - 1; k++) {
                        rhs += Y[k][i][h];
                    }
            }
            // consistency constraint cho sortie phuc vu h.
            model.add(phi[h] == rhs).setName(("C18_" + std::to_string(h)).c_str());
        }


        for (int h: C) {
            IloExpr rhs(env);
            for (int i = 1; i <= D; i++) {
                if (i != h && tau_prime[h][i] <= dtl - sr)
                    for (int k = 2; k <= node_max_stage; k++) {
                        rhs += W[k][i][h];
                    }
            }
            // consistency constraint cho sortie phuc vu h.
            model.add(phi[h] == rhs).setName(("C18p_" + std::to_string(h)).c_str());
        }

        //////////// C10: node_stage
        for (int h: C) {
            IloExpr sum_k(env);
            for (int k = 2; k < node_max_stage; k++) {
                sum_k += X[k][h];
            }
            // phuc vu h it nhat 1 lan.
            model.add(phi[h] + sum_k >= 1).setName(("C10_" + std::to_string(h)).c_str());
        }


        /////////// C14: node_stage
        for (int k = 1; k <= arc_max_stage; k++) {
            IloExpr sum(env);
            for (int i = 0; i < D; i++) {
                for (int j = 1; j <= D; j++) {
                    if (i != j) {
                        sum += x[k][i][j] * tau[i][j];
                    }
                }
            }

            // o to toi k+1 >= o to den k + thoi gian di chuyen tu k->k+1 (tau_(ij)). (old model).
            // em nghĩ nên đổi thành bằng nhau, a_(k+1) chắc chắn bằng departure stage trước + di chuyển.
            // d[k+1] mới >= d[k]. trường hợp lớn hơn xảy ra khi truck phải đợi drone bay đến.
            model.add(a[k + 1] == d[k] + sum).setName(
                    ("C14_" + std::to_string(k) + "_" + std::to_string(k + 1)).c_str());
        }

        ////////// C15: node_stage
        auto M = 1e5;
        for (int k = 1; k <= node_max_stage - 1; k++) {
            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                if (k < k_p) {
                    // o to phai den k_p tu k trong khoang thoi gian <= dtl.
                    // , "C15_" + std::to_string(k) + "_" + std::to_string(k_p)
                    model.add(a[k_p] - d[k] <= z[k][k_p] * (dtl - sr) + (1 - z[k][k_p]) * M).setName(
                            ("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());

                    IloExpr rhs(env);
                    for (int i = 0; i <= D; i++) {
                        for (int h: C) {
                            if (h != i && i != D) {
                                if (tau_prime[i][h] <= dtl - sr) {
                                    rhs += Y[k][i][h] * tau_prime[i][h];
                                }
                            }

                            if (h != i && i != 0) {
                                if (tau_prime[h][i] <= dtl - sr) {
                                    rhs += W[k_p][i][h] * tau_prime[h][i];
                                }
                            }
                        }
                    }
                    // vehicle phai doi drone truoc khi di chuyen.
                    //                model.add(d[k_p] - d[k] + (1 - z[k][k_p]) * 2 * (dtl-sr) >= rhs).setName(("C21_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
                    model.add(d[k_p] - d[k] >= z[k][k_p] * sr - (1 - z[k][k_p]) * 100000 + rhs);
                }
            }
        }

        for (int k = 1; k <= node_max_stage - 1; k++) {
            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                if (k < k_p) {
                    IloExpr sum_length_y_k(env);
                    IloExpr sum_length_w_k_p(env);
                    IloExpr sum_w_k_p(env);
                    IloExpr sum_y_k_p(env);

                    for (int i = 0; i < D; i++) {
                        for (int h: C) {
                            if (i != h) {
                                sum_length_y_k += Y[k][i][h] * tau_prime[i][h];
                                sum_y_k_p += Y[k_p][i][h];
                            }
                        }
                    }
                    for (int j = 1; j <= D; j++) {
                        for (int h: C) {
                            if (h != j) {
                                sum_length_w_k_p += W[k_p][j][h] * tau_prime[h][j];
                                sum_w_k_p += W[k_p][j][h];
                            }
                        }
                    }
                    model.add(
                            d[k_p] >= d[k] + sum_length_y_k + sum_length_w_k_p + sl * sum_y_k_p + sr * sum_w_k_p - (
                                                                                                                           1 -
                                                                                                                           z[k][k_p]) *
                                                                                                                   M);
                }
            }
        }
        for (int k = 2; k <= node_max_stage; k++) {
            IloExpr sum(env);
            IloExpr sum_w_K(env);
            IloExpr sum_y_K(env);
            for (int i = 0; i < D; i++) {
                for (int j = 1; j <= D; j++) {
                    if (i != j) {
                        sum += x[k - 1][i][j] * tau[i][j];
                    }
                }
            }
            for (int i = 0; i < D; i++) {
                for (int h: C) {
                    if (i != h && k != node_max_stage) {
                        sum_y_K += sl * Y[k][i][h];
                    }
                }
            }

            for (int i = 1; i <= D; i++) {
                for (int h: C) {
                    if (i != h) {
                        sum_w_K += sr * W[k][i][h];
                    }
                }
            }
            auto constr_name = "CC_d_" + std::to_string(k) + "_constr";
            model.add(d[k] >= d[k - 1] + sum + sum_w_K + sum_y_K).setName(constr_name.c_str());
        }

        // - sum_sl_at_O * sl + sum_sl_sr)
        for (int k = 1; k <= node_max_stage - 1; k++) {
            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                IloExpr sum(env);
                for (int k_p_p = k; k_p_p <= k_p - 1; k_p_p++) {
                    for (int i = 0; i < D; i++) {
                        for (int j = 1; j <= D; j++) {
                            if (i != j) {
                                sum += x[k_p_p][i][j] * tau[i][j];
                            }
                        }
                    }
                }
                model.add(sum - M * (1 - z[k][k_p]) <= dtl - sr);
            }
        }
        model.add(IloMinimize(env, d[node_max_stage]));
        cplex.exportModel("cplex_model_1.lp");
        std::vector<Sortie> st;
        double obj = 0;
        double best_objective = IloInfinity;
        MyCallback myCallback(env, model, X, best_objective, 3, node_max_stage, D);
        cplex.use(&myCallback);
        cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
        // Solve the model
        if (!cplex.solve()) {
            // Check if the problem is infeasible
            if (cplex.getStatus() == IloAlgorithm::Infeasible) {
                // Handle infeasibility
                std::cout << "The problem is infeasible." << std::endl;
                std::cout << "Infeasibility at: " << cplex.getInfeasibility(c2) << std::endl;
                // You can also retrieve the infeasible constraints using cplex.getInfeasibility() method
            } else {
                // Handle other solver errors
                std::cerr << "Solver error: " << cplex.getStatus() << std::endl;
            }
        } else {
            std::cout << "Feasible solution found!" << std::endl;
            std::cout << "Truck nodes:" << std::endl;
            for (int k = 1; k <= node_max_stage; k++) {
                for (int i = 0; i <= D; i++) {
                    auto X_val = cplex.getValue(X[k][i]);
                    //std::cout << "k = " << k << ", i = " << i << ":" << X_val << std::endl;
                    if (abs(X_val - 1) < 1e-6) {
                        auto d_k = cplex.getValue(d[k]);
                        auto a_k = cplex.getValue(a[k]);
                        std::cout << "Stage " << k << " at customer " << i << " with arrival time is: " << a_k <<
                                  std::endl;
                        std::cout << "Stage " << k << " at customer " << i << " with departure time is: " << d_k <<
                                  std::endl;
                        break;
                    }
                }
            }
            std::cout << "Truck arcs:" << std::endl;
            std::map<int, std::pair<int, int> > map_stage_truck_arc;
            for (int k = 1; k <= arc_max_stage; k++) {
                for (int i = 0; i < D; i++)
                    for (int j = 1; j <= D; j++)
                        if (i != j) {
                            auto X_val = cplex.getValue(x[k][i][j]);
                            //std::cout << "k = " << k << ", i = " << i << ":" << X_val << std::endl;
                            if (abs(X_val - 1) < 1e-6) {
                                std::cout << "Arc " << k << " connecting " << i << " and " << j
                                          << " with cost " << tau[i][j] << " " << std::endl;
                                obj += tau[i][j];
                                map_stage_truck_arc[k] = std::make_pair(i, j);
                                break;
                            }
                        }
            }
            std::cout << "Drone served customers" << std::endl;
            //IloNumArray phi_val(env);
            //cplex.getValues(phi_val, phi);


            for (int k = 1; k < node_max_stage; k++)
                for (int kp = k + 1; kp <= node_max_stage; kp++) {
                    //std::cout << k << " " << kp << std::endl;
                    if (abs(cplex.getValue(z[k][kp]) - 1) < 1e-6) {
                        std::cout << "Drone flies from stage " << k << " to stage " << kp << std::endl;
                    }
                }
            for (int h: C)
                for (int k = 1; k < node_max_stage; k++)
                    for (int kp = k + 1; kp <= node_max_stage; kp++) {
                        //std::cout << k << " " << kp << std::endl;
                        if (abs(cplex.getValue(Z[h][k][kp]) - 1) < 1e-6) {
                            std::cout << "Drone flies from stage " << k << " to stage " << kp << " to serve customer "
                                      << h << std::endl;
                        }
                    }

            for (int h: C) {
                if (abs(cplex.getValue(phi[h]) - 1) < 1e-6) {
                    std::cout << "Customer " << h << " served by drone." << std::endl;
                    auto sortie = Sortie(h);
                    st.push_back(sortie);
                    int sv_i = -1, sv_j = -1, sv_k = -1, sv_kp = -1;
                    for (int k = 1; k <= node_max_stage; k++) {
                        for (int i = 0; i <= D; i++)
                            if (i != h) {
                                try {
                                    //std::cout << "from stage " << k << " at customer " << i << " to serve" << h << std::endl;
                                    auto Y_val = cplex.getValue(Y[k][i][h]);
                                    if (abs(Y_val - 1) < 1e-6) {
                                        sv_i = i;
                                        sv_k = k;
                                        //std::cout << "XXX from stage " << k << " at customer " << i << " to serve" << h << std::endl;
                                    }

                                    //std::cout << "to stage " << k << " at customer" << i << " from " << h << std::endl;
                                    auto W_val = cplex.getValue(W[k][i][h]);
                                    if (abs(W_val - 1) < 1e-6) {
                                        sv_j = i;
                                        sv_kp = k;
                                        //std::cout << "ZZZ to stage " << k << " at customer" << i << " from " << h << std::endl;
                                    }
                                } catch (...) {
                                }
                            }
                    }
                    ////assert(abs(cplex.getValue(z[sv_k][sv_kp]) - 1.0) < 1e-6);

                    std::cout << "Drone fly from " << sv_i << " at stage " << sv_k <<
                              " to serve " << h << " and then fly back to " << sv_j
                              << " at stage " << sv_kp << ". " << std::endl;
                    obj += (sl + sr);
                    if (sv_i == O) {
                        obj -= sl;
                    }
                    double drone_travel_time = tau_prime[sv_i][h] + tau_prime[h][sv_j];
                    double truck_travel_time = 0;
                    for (int k_start = sv_k; k_start <= sv_kp - 1; k_start++) {
                        truck_travel_time += tau[map_stage_truck_arc[k_start].first][map_stage_truck_arc[k_start].
                                second];
                    }
                    std::cout << "Truck travel time from stage " << sv_k << " to " << sv_kp << " is: " <<
                              truck_travel_time << std::endl;
                    if (drone_travel_time > truck_travel_time) {
                        obj += drone_travel_time - truck_travel_time;
                    }
                    auto drone_arrival_time = cplex.getValue(d[sv_k]) + drone_travel_time;
                    auto vehicle_departure_time = cplex.getValue(d[sv_kp]);
                    auto truck_arrival_time = cplex.getValue(a[sv_kp]);
                    std::cout << "Start arc cost: "
                              << tau_prime[sv_i][h] << ", end arc cost: " << tau_prime[h][sv_j] <<
                              ". Total drone travel time: " << drone_travel_time << std::endl;

                    std::cout << "Drone arrival time: " << drone_arrival_time << std::endl;
                    std::cout << "Truck arrival time: " << truck_arrival_time << std::endl;

                    std::cout << "Truck departure time = max(d/a, t/a) plus (sl): " << vehicle_departure_time <<
                              std::endl;
                    assert(drone_arrival_time <= vehicle_departure_time);
                    assert(abs(cplex.getValue(Z[h][sv_k][sv_kp]) - 1.0) < 1e-6);

                    assert(abs(cplex.getValue(z[sv_k][sv_kp]) - 1.0) < 1e-6);
                }
            }

            std::cout << "Done!" << std::endl;
            std::cout << "-------------------------Re-calculated objective-----------------------" << std::endl;
            std::cout << obj << std::endl;
            std::cout << "------------------------------------------------------------------------" << std::endl;
        }
        double c = cplex.populate();
        double time_spent = cplex.getCplexTime();
        env.end();
        cplex.end();
        model.end();
        std::cout << "OBJECTIVE VALUE: " << c << ", NUMBER OF SORTIES: " << st.size() << "." << std::endl;
        return Result{c, obj, time_spent, st};
    } catch (IloException &e) {
        std::cout << e.getMessage() << std::endl;
    }
}

Result Solver::OriginalSolverCPLEX(int n_thread, int e) {
    auto tau = instance->tau;
    // for (int i = 0; i < tau.size(); i++) {
    //     for (int j = 0; j < tau[i].size(); j++) {
    //         std::cout << tau[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
    auto d = instance->tau_prime;
    auto dtl = e;
    auto sl = 1, sr = 1;
    auto n = instance->num_node;
    auto s = 0, t = n;
    auto c_prime = instance->c_prime;
    std::vector<int> c_prime_0;
    c_prime_0.push_back(0);
    for (int i: c_prime) {
        c_prime_0.push_back(i);
    }
    c_prime_0.push_back(n);
    std::cout << "Printing number of nodes: " << n << std::endl;
    std::vector<int> C;
    std::vector<int> V;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0 || i == n) {
            V.push_back(i);
        } else {
            V.push_back(i);
            C.push_back(i);
        }
    }
    std::vector<int> c_s;
    std::vector<int> c_t;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0) {
            c_s.push_back(i);
        } else if (i == n) {
            c_t.push_back(i);
        } else {
            c_s.push_back(i);
            c_t.push_back(i);
        }
    }

    std::cout << std::endl;
    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    cplex.setParam(IloCplex::Param::Emphasis::MIP, CPX_MIPEMPHASIS_OPTIMALITY);
    cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
    cplex.setParam(IloCplex::Param::ClockType, 0);

    // y: (i, j) in A, truck route
    IloArray<IloBoolVarArray> y(env, n + 1);
    for (int i: c_s) {
        y[i] = IloBoolVarArray(env, n + 1);
        for (int j: c_t) {
            y[i][j] = IloBoolVar(env);
            if (i == j) {
                model.add(y[i][j] == 0);
            }
        }
    }
    model.add(y[s][t] == 0);

    IloArray<IloBoolVarArray> x(env, n + 1);
    for (int i: c_s) {
        x[i] = IloBoolVarArray(env, n + 1);
        for (int j: c_t) {
            x[i][j] = IloBoolVar(env);
            if (i == j) {
                model.add(x[i][j] == 0);
            }
        }
    }
    model.add(x[s][t] == 0);


    // gamma_h_ij
    IloArray<IloArray<IloBoolVarArray> > gamma(env, n + 1);
    for (int h: C) {
        gamma[h] = IloArray<IloBoolVarArray>(env, n + 1);
        for (int i: c_s) {
            gamma[h][i] = IloBoolVarArray(env, n + 1);
            for (int j: c_t) {
                gamma[h][i][j] = IloBoolVar(env);
                for (int heavy: instance->heavy) {
                    if (h == heavy) {
                        model.add(gamma[h][i][j] == 0);
                    }
                }
                if (i == j) {
                    model.add(gamma[h][i][j] == 0);
                }
            }
        }
        model.add(gamma[h][s][t] == 0);
    }

    IloBoolVarArray theta(env, n + 1);
    for (int h: V) {
        theta[h] = IloBoolVar(env);
        for (auto heavy: instance->heavy) {
            if (h == heavy) {
                model.add(theta[h] == 0);
            }
        }
        if (h == s || h == t) {
            model.add(theta[h] == 0);
        }
    }
    IloArray<IloBoolVarArray> omega(env, n + 1);
    for (int h: C) {
        omega[h] = IloBoolVarArray(env, n + 1);
        for (int i: V) {
            omega[h][i] = IloBoolVar(env);

            for (int heavy: instance->heavy) {
                if (h == heavy) {
                    model.add(omega[h][i] == 0);
                }
            }
            if (h == i || i == t) {
                model.add(omega[h][i] == 0);
            }
        }
    }
    IloArray<IloBoolVarArray> delta(env, n + 1);
    for (int h: C) {
        delta[h] = IloBoolVarArray(env, n + 1);
        for (int j: V) {
            delta[h][j] = IloBoolVar(env);
            for (int heavy: instance->heavy) {
                if (h == heavy) {
                    model.add(delta[h][j] == 0);
                }
            }
            if (h == j || j == s) {
                model.add(delta[h][j] == 0);
            }
        }
    }

    IloNumVarArray sigma(env, n + 1);
    for (int h: c_t) {
        sigma[h] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
        for (int heavy: instance->heavy) {
            if (h == heavy) {
                model.add(sigma[h] == 0);
            }
        }
    }

    IloExpr objective(env);
    for (int i: c_s) {
        for (int j: c_t) {
            objective += tau[i][j] * y[i][j];
        }
    }

    for (int h: C) {
        objective += (sl + sr) * theta[h];
        objective -= sl * omega[h][s];
    }
    for (int h: c_t) {
        objective += sigma[h];
    }
    IloExpr sum_theta(env);
    // Constraint 1
    IloExpr lhs_1(env), rhs_1(env);
    for (int j: c_t) {
        lhs_1 += y[s][j];
    }
    for (int i: c_s) {
        rhs_1 += y[i][t];
    }
    model.add(lhs_1 == 1);
    model.add(rhs_1 == 1);

    // Constraint 2
    for (int i: C) {
        IloExpr lhs_2(env), rhs_2(env);
        for (int j: c_t) {
            lhs_2 += y[i][j];
        }
        for (int j: c_s) {
            rhs_2 += y[j][i];
        }
        model.add(lhs_2 == rhs_2);
        model.add(lhs_2 <= 1);
        model.add(rhs_2 <= 1);
    }

    // Constraint 3
    auto setAndComps = generateSetsAndComplements(C);
    for (auto &set: setAndComps) {
        auto S = set.first;
        if (S.size() < 2) {
            continue;
        }
        if (S.size() == 2 && S[0] == s && S[1] == t) {
            continue;
        }
        IloExpr sum1(env), sum2(env);
        std::string cname = "C3";
        for (auto i: S) {
            cname += "_" + std::to_string(i);
            if (i != t) {
                for (auto j: S) {
                    if (j != s) {
                        sum1 += y[i][j];
                    }
                }
            }
        }

        for (auto h: S) {
            IloExpr sum3(env);
            for (auto k: S) {
                if (h == k || h == s || h == t) {
                    continue;
                } else {
                    sum3 += 1 - theta[k];
                }
            }

            model.add(sum1 <= sum3);
        }
    }
    // Constraint 4
    for (int h: C) {
        IloExpr lhs_4(env);
        std::string cname = "C4_h=" + std::to_string(h);
        for (int j: c_t) {
            lhs_4 += gamma[h][s][j];
        }
        model.add(lhs_4 == omega[h][s]);
    }

    // Constraint 5
    for (int h: C) {
        IloExpr lhs_5(env);
        std::string cname = "C5_h=" + std::to_string(h);

        for (int i: c_s) {
            lhs_5 += gamma[h][i][t];
        }
        model.add(lhs_5 == delta[h][t]);
    }
    // Constraint 6
    for (int i: C) {
        for (int h: C) {
            std::string cname = "C6_i=" + std::to_string(i) + "_h=" + std::to_string(h);
            IloExpr sum1(env), sum2(env);
            for (int j: c_t) {
                sum1 += gamma[h][i][j];
            }

            for (int j: c_s) {
                sum2 += gamma[h][j][i];
            }
            model.add(sum1 - sum2 == omega[h][i] - delta[h][i]);
        }
    }
    // Constraint 7
    for (int j: c_t) {
        std::string cname = "C7_s_j=" + std::to_string(j);
        model.add(y[s][j] + x[s][j] <= 1);
    }
    // Constraint 8
    for (int i: c_s) {
        std::string cname = "C8_i=" + std::to_string(i) + "_t";
        model.add(y[i][t] + x[i][t] <= 1);
    }
    // Constraint 9
    for (int i: C) {
        for (int j: C) {
            std::string cname = "C9_i=" + std::to_string(i) + "_j=" + std::to_string(j);
            model.add(y[i][j] + x[i][j] + x[j][i] <= 1);
        }
    }

    // Constraint 10
    for (int h: C) {
        IloExpr sum(env);
        std::string cname = "C10_h=" + std::to_string(h);
        for (int j: c_t) {
            sum += y[h][j];
        }
        model.add(sum + theta[h] == 1);
    }
    // Constraint 11
    for (int i: c_s) {
        for (int j: c_t) {
            std::string cname = "C11_i=" + std::to_string(i) + "_j=" + std::to_string(j);
            IloExpr sum(env);
            for (int h: c_prime) {
                sum += gamma[h][i][j];
            }
            model.add(sum <= y[i][j]);
        }
    }


    // Constraint 12
    for (int h: C) {
        IloExpr sum1(env), sum2(env);

        for (int i: V) {
            if (i != h && i != t) {
                sum1 += omega[h][i];
            }
        }
        for (int j: V) {
            if (j != s && j != h) {
                sum2 += delta[h][j];
            }
        }
        model.add(sum1 == theta[h]);
        model.add(sum2 == theta[h]);
    }
    // Constraint 13
    for (int i: c_s) {
        for (int j: c_t) {
            std::string cname = "C13_i=" + std::to_string(i) + "_j=" + std::to_string(j);
            model.add(x[i][j] <= theta[i] + theta[j]);
        }
    }
    //        // Constraint 14
    for (int i: c_s) {
        for (int j: c_t) {
            if (i != s && j != t) {
                std::string cname = "C14_i=" + std::to_string(i) + "_j=" + std::to_string(j);
                model.add(x[i][j] <= omega[j][i] + delta[i][j]);
            }
        }
    }
    // Constraint 15
    for (int i: c_s) {
        IloExpr sum1(env), sum2(env);
        std::string cname = "C15_i=" + std::to_string(i);
        for (int j: c_t) {
            sum1 += x[i][j];
        }
        for (int h: c_prime) {
            sum2 += omega[h][i];
        }
        sum2 += theta[i];
        model.add(sum1 == sum2);
        model.add(sum1 <= 1);
        model.add(sum2 <= 1);
    }
    // Constraint 16
    for (int j: c_t) {
        IloExpr sum1(env), sum2(env);
        for (int i: c_s) {
            sum1 += x[i][j];
        }

        for (int h: c_prime) {
            sum2 += delta[h][j];
        }
        sum2 += theta[j];
        model.add(sum1 == sum2);
        model.add(sum1 <= 1);
        model.add(sum2 <= 1);
    }
    // Constraint 17
    for (int h: c_prime) {
        IloExpr sum(env);
        std::string cname = "C17_h=" + std::to_string(h);
        for (int i: c_s) {
            for (int j: c_t) {
                sum += tau[i][j] * gamma[h][i][j];;
            }
        }
        model.add(sum <= (dtl - sr) * theta[h]);
    }
    // Constraint 18
    for (int h: c_prime) {
        IloExpr sum1(env);
        IloExpr sum2(env);
        IloExpr sum3(env);
        std::string c18_name = "C18_h=" + std::to_string(h);
        std::string c19_name = "C19_h=" + std::to_string(h);

        for (int i: c_s) {
            sum1 += d[i][h] * omega[h][i];
        }
        for (int j: c_t) {
            sum2 += d[h][j] * delta[h][j];
        }

        for (int i: c_s) {
            for (int j: c_t) {
                sum3 += tau[i][j] * gamma[h][i][j];
            }
        }
        model.add(sum1 + sum2 <= (dtl - sr) * theta[h]);
        model.add(sum1 + sum2 - sum3 <= sigma[h]);
    }
    model.add(IloMinimize(env, objective));
    cplex.solve();
    std::cout << "Truck arcs:" << std::endl;
    for (int i: c_s) {
        for (int j: c_t) {
            if (i != j) {
                if (cplex.getValue(y[i][j]) == 1) {
                    std::cout << i << " " << j << std::endl;
                }
            }
        }
    }
    for (int h: C) {
        if (cplex.getValue(theta[h]) == 1) {
            std::cout << "customer " << h << " is served by drone" << std::endl;
            for (int i: c_s) {
                if (h != i) {
                    if (cplex.getValue(omega[h][i]) == 1) {
                        std::cout << "start of this sortie: " << i << std::endl;
                    }
                }
            }
            for (int j: c_t) {
                if (j != h) {
                    if (cplex.getValue(delta[h][j]) == 1) {
                        std::cout << "end of this sortie: " << j << std::endl;
                    }
                }
            }
            std::cout << std::endl;
        }
    }
    std::cout << cplex.getObjValue() << std::endl;
    std::vector<Sortie> st;
    return Result{cplex.getObjValue(), 0, cplex.getCplexTime(), st};
}

Result Solver::SolverWithRandomTruckStageFixed(int n_thread, int e) {
    //    try {
    auto tau = instance->tau;
    auto tau_prime = instance->tau_prime;
    auto dtl = e;
    //dtl = 5;
    auto sl = 1, sr = 1;
    auto n = instance->num_node;

    auto c_prime = instance->c_prime;
    std::vector<int> c_prime_0;
    c_prime_0.push_back(0);
    for (int i: c_prime) {
        c_prime_0.push_back(i);
    }
    c_prime_0.push_back(n);

    std::cout << "Printing number of nodes: " << n << std::endl;
    std::vector<int> C;
    std::vector<int> V;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0 || i == n) {
            V.push_back(i);
        } else {
            V.push_back(i);
            C.push_back(i);
        }
    }

    // C_s : set C(customer) union s (source) ; source  = 0
    // C_t : set C(customer) union t (terminal); terminal = n
    // create.
    std::vector<int> c_s;
    std::vector<int> c_t;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0) {
            c_s.push_back(i);
        } else if (i == n) {
            c_t.push_back(i);
        } else {
            c_s.push_back(i);
            c_t.push_back(i);
        }
    }

    std::cout << std::endl;
    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    cplex.setParam(IloCplex::Param::ClockType, 0);
    auto K = C, k_prime = V;
    auto O = 0;
    auto D = n;
    auto node_max_stage = n + 1;
    auto arc_max_stage = node_max_stage - 1;

    // Khai bao bien
    //
    // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
    //đường đi của vehicle là i; k \in 1..n;
    //
    IloArray<IloBoolVarArray> X(env, node_max_stage + 1);
    for (int k = 1; k <= node_max_stage; k++) {
        X[k] = IloBoolVarArray(env, D + 1);

        for (int i = 0; i <= D; i++) {
            {
                X[k][i] = IloBoolVar(env);
                auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
                //std::cout << v_name << std::endl;
                X[k][i].setName(v_name.c_str());

                if (k > 1 && i == 0) X[k][0].setBounds(0, 0);
            }
        }
    }


    model.add(X[1][0] == 1).setName("start depot is the first node");
    model.add(X[1][D] == 0).setName("ending depot cannot be the first node");


    /*
     * Fix random truck route.
     * */
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis_3(n / 3, n / 2);

    auto n_random_stage = dis_3(gen);


    // Generate the random truck stage
    std::uniform_int_distribution<int> dis_truck(1, C.size());

    // Generate the random matrix
    std::unordered_set<int> seen;
    std::vector<int> truck_node;
    truck_node.reserve(n);

    for (int i = 0; i < n_random_stage; ++i) {
        int randomElement;
        do {
            randomElement = dis_truck(gen);
        } while (seen.count(randomElement) > 0);

        seen.insert(randomElement);
        truck_node.push_back(randomElement);
    }
    std::cout << "Chosen random node: ";
    for (int i: truck_node) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    std::cout << "Number of randomized stage: " << n_random_stage << std::endl;
    for (int k = 1; k <= n_random_stage; k++) {
        auto i = truck_node[k - 1];
        model.add(X[k + 1][i] == 1);
        std::cout << "Stage " << k + 1 << " assigned with node " << i << std::endl;
    }
    for (int k = 1; k <= node_max_stage; k++) {
        IloExpr sum(env);
        for (int i = 0; i <= D; i++)
            sum += X[k][i];
        model.add(sum <= 1).setName(("C20_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
    }

    IloExpr arrival_depot(env);
    for (int k = 1; k <= node_max_stage; k++) {
        arrival_depot += X[k][D];
    }
    model.add(arrival_depot == 1).setName("C21_arrival_depot_once");

    // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
    // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
    IloArray<IloArray<IloBoolVarArray> > x(env, arc_max_stage + 1);
    for (int k = 1; k <= arc_max_stage; k++) {
        x[k] = IloArray<IloBoolVarArray>(env, D);
        for (int i = 0; i < D; i++) {
            x[k][i] = IloBoolVarArray(env, D + 1);

            for (int j = 1; j <= D; j++)
                if (i != j) {
                    x[k][i][j] = IloBoolVar(env);
                    auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
                    x[k][i][j].setName(v_name.c_str());
                }
        }
    }


    //// phi^h equals to 1 if customer h is served by the drone
    IloBoolVarArray phi(env, n);
    for (int h: C) {
        phi[h] = IloBoolVar(env);
        auto v_name = "phi_" + std::to_string(h);
        phi[h].setName(v_name.c_str());
    }

    for (int heavy: instance->heavy) {
        model.add(phi[heavy] == 0);
    }
    IloArray<IloArray<IloBoolVarArray> > Y(env, node_max_stage + 1), W(env, node_max_stage + 1);
    for (int k = 1; k <= node_max_stage; k++) {
        Y[k] = IloArray<IloBoolVarArray>(env, D + 1);
        W[k] = IloArray<IloBoolVarArray>(env, D + 1);

        for (int i = 0; i <= D; i++) {
            {
                Y[k][i] = IloBoolVarArray(env, C.size() + 1);
                W[k][i] = IloBoolVarArray(env, C.size() + 1);
                for (int h: C)
                    if (i != h) {
                        Y[k][i][h] = IloBoolVar(env);
                        Y[k][i][h].setName(("Y_" + std::to_string(k) + "_"
                                            + std::to_string(i) + "_" + std::to_string(h)).c_str());

                        W[k][i][h] = IloBoolVar(env);
                        W[k][i][h].setName(("W_" + std::to_string(k) + "_"
                                            + std::to_string(i) + "_" + std::to_string(h)).c_str());

                        if (i == 0 && k > 1) Y[k][i][h].setBounds(0, 0);
                        if (i == D && k == 1) W[k][i][h].setBounds(0, 0);
                        if (tau_prime[i][h] > dtl - sr) Y[k][i][h].setBounds(0, 0);
                        if (tau_prime[h][i] > dtl - sr) W[k][i][h].setBounds(0, 0);
                    }
            }
        }
    }


    //// C17 - $X^k_i \geq \sum_h X^k_{ih}$ (C17) - chỉ bay drone ở nơi mà xe ở đó
    for (int k = 1; k <= node_max_stage; k++) {
        for (int i = 0; i <= D; i++) {
            IloExpr expr(env);
            for (int h: C)
                if (h != i)
                    if (tau_prime[i][h] <= dtl - sr) {
                        expr += Y[k][i][h];
                    }
            model.add(expr <= X[k][i]).setName(("C17_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
        }
    }
    // $X^k_i \geq \sum_h Y^k_{ih}$ (C17p) : chỉ bay drone tới nơi mà xe ở đó
    for (int k = 1; k <= node_max_stage; k++) {
        for (int i = 0; i <= D; i++) {
            IloExpr expr(env);
            for (int h: C)
                if (h != i)
                    if (tau_prime[h][i] <= dtl - sr) {
                        expr += W[k][i][h];
                    }
            model.add(expr <= X[k][i]).setName(("C17p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
        }
    }

    // $\sum_{i} X^k_{ih}\tau^D_{ih} + \sum_{i} Y^{k'}_{ih}\tau^D_{hi} \leq D_d$
    //- drone duration constraint cho mỗi $h$ (C19)
    for (int h: C) {
        IloExpr expr(env);

        for (int k = 1; k <= node_max_stage; k++) {
            for (int i = 0; i <= D; i++) {
                if (i != h && i != D && tau_prime[i][h] <= dtl - sr)
                    expr += Y[k][i][h] * tau_prime[i][h];

                if (i != h && i != 0 && tau_prime[h][i] <= dtl - sr)
                    expr += W[k][i][h] * tau_prime[h][i];
            }
        }

        model.add(expr <= (dtl - sr) * phi[h]).setName(("C19_" + std::to_string(h)).c_str());
    }
    //exit(0);

    //// aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
    IloArray<IloBoolVarArray> z(env, node_max_stage);
    for (int k = 1; k <= node_max_stage - 1; k++) {
        z[k] = IloBoolVarArray(env, node_max_stage + 1);
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            {
                z[k][k_p] = IloBoolVar(env);
                auto v_name = "z_" + std::to_string(k) + "_" + std::to_string(k_p);
                z[k][k_p].setName(v_name.c_str());
            }
        }
    }

    //// aux var Z_{k, k_p, h}: sortie launch from k and rendezvous at k_p.
    IloArray<IloArray<IloBoolVarArray> > Z(env, node_max_stage);
    for (int h: C) {
        Z[h] = IloArray<IloBoolVarArray>(env, node_max_stage + 1);
        for (int k = 1; k <= node_max_stage - 1; k++) {
            Z[h][k] = IloBoolVarArray(env, node_max_stage + 1);
            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                {
                    Z[h][k][k_p] = IloBoolVar(env);
                    auto v_name = "Z_" + std::to_string(h) + "_" + std::to_string(k) + "_" + std::to_string(k_p);
                    Z[h][k][k_p].setName(v_name.c_str());
                }
            }
        }
    }

    // $Z_{kk'} = \sum_{h}Z^h_{kk'}$: mỗi cặp (k,k') chỉ phục vụ tối đa một khách hàng (C22)
    for (int k = 1; k < node_max_stage; k++)
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            IloExpr expr(env);
            for (int h: C) {
                expr += Z[h][k][k_p];
            }
            model.add(expr == z[k][k_p]).setName(("C22_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
        }


    //C20:$\sum_{k'>k}Z_{kk'} = \sum_{i,h}Y^k_{ih}$ : với mỗi $k$,
    //ràng buộc liên kết drone đi ra từ stage $k$ và đoạn mà oto di chuyển không có drone. (C20)

    for (int h: C) {
        for (int k = 1; k <= node_max_stage - 1; k++) {
            IloExpr expr(env);
            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                {
                    expr += Z[h][k][k_p];
                }
            }

            for (int i = 0; i < D; i++) {
                if (i != h && tau_prime[i][h] <= dtl - sr) {
                    expr -= Y[k][i][h];
                }
            }
            model.add(expr == 0).setName(("C20_" + std::to_string(k) + "_" + std::to_string(h)).c_str());
        }
    }

    for (int h: C) {
        for (int k_p = 2; k_p <= node_max_stage; k_p++) {
            IloExpr expr(env);
            for (int k = 1; k < k_p; k++) {
                {
                    expr += Z[h][k][k_p];
                }
            }

            for (int i = 1; i <= D; i++) {
                if (i != h && tau_prime[h][i] <= dtl - sr) {
                    expr -= W[k_p][i][h];
                }
            }
            model.add(expr == 0).setName(("C20p_" + std::to_string(k_p)
                                          + "_" + std::to_string(h)).c_str());
        }
    }

    // arrival\departure variables a and d.
    IloNumVarArray a(env, node_max_stage + 1);
    IloNumVarArray d(env, node_max_stage + 1);
    for (int k = 1; k <= node_max_stage; k++) {
        a[k] = IloNumVar(env);
        auto v_name = "a_" + std::to_string(k);
        a[k].setName(v_name.c_str());
        d[k] = IloNumVar(env);
        v_name = "d_" + std::to_string(k);
        d[k].setName(v_name.c_str());
        model.add(d[k] >= a[k]).setName(("C13_" + std::to_string(k)).c_str());
    }

    model.add(a[1] == 0).setName("arrival to depot at time 0");
    model.add(d[1] == 0).setName("depart from depot at time 0");;

    ////////// Constraint C1
    for (int k = 1; k < node_max_stage; k++) {
        for (int i = 0; i < D; i++) {
            {
                // i == D => khong co i -> j.
                IloExpr sum(env);
                for (int j = 1; j <= D; j++) {
                    if (i != j) {
                        sum += x[k][i][j];
                    }
                }

                // neu node i la node_stage k, i khac D thi kieu gi cung co canh i, j.
                model.add(X[k][i] == sum).setName(("C1_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
            }
        }
    }

    for (int k = 2; k <= node_max_stage; k++) {
        for (int i = 1; i <= D; i++) {
            {
                IloExpr sum(env);
                for (int j = 0; j < D; j++) {
                    if (i != j) {
                        sum += x[k - 1][j][i];
                    }
                }
                // arcs entering i at stage k.
                model.add(X[k][i] == sum).setName(("C1p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
            }
        }
    }

    //////////// C2 - depart from the depot
    IloExpr C2(env);
    for (int i = 1; i <= D; i++) {
        C2 += x[1][O][i];
    }

    IloConstraint c2(C2 == 1);
    model.add(c2).setName("C2");
    // , "C2"

    ///////////// C3: arc_stage
    IloExpr C3(env);
    for (int k = 2; k <= node_max_stage; k++) {
        {
            C3 += X[k][D];
        }
    }
    // , "C3"
    model.add(C3 == 1).setName("C3"); // arrival to depot

    // $R_{k} = \sum_{k'}Z_{kk'}$: các đoạn bắt đầu từ k (C23)
    IloBoolVarArray R(env, node_max_stage + 1);
    for (int k = 1; k < node_max_stage; k++) {
        R[k].setName(("R_" + std::to_string(k)).c_str());
    }

    for (int k = 1; k < node_max_stage; k++) {
        IloExpr expr(env);
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            expr += z[k][k_p];
        }

        model.add(R[k] == expr).setName(("C23_" + std::to_string(k)).c_str());
    }

    // modified C7
    for (int k = 1; k <= node_max_stage - 1; k++) {
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            for (int l = k + 1; l < k_p; l++) {
                {
                    // tranh drone bay cac doan giao nhau.
                    if (k < l) {
                        model.add(z[k][k_p] + R[l] <= 1).setName(("C7m_" + std::to_string(k)
                                                                  + "_" + std::to_string(k_p) + "_" + std::to_string(l))
                                                                         .c_str());
                    } else {
                        //                        model.add(R[k] <= 1).setName(("C7k_" + std::to_string(k);
                        //    + "_" + std::to_string(k_p) + "_" + std::to_string(l)).c_str());
                    }
                }
            }
        }
    }


    for (int i = 0; i < D; i++) {
        for (int k = 1; k <= node_max_stage - 1; k++) {
            IloExpr lhs(env);

            for (int h: C) {
                if (i != h) {
                    lhs += Y[k][i][h];
                }
            }
            model.add(lhs <= X[k][i]).setName(("C8_launch_" + std::to_string(i) + "_" + std::to_string(k)).c_str());
        }
    }

    for (int j = 1; j <= D; j++) {
        for (int k_p = 2; k_p <= node_max_stage; k_p++) {
            IloExpr lhs(env);

            for (int h: C) {
                if (h != j) {
                    lhs += W[k_p][j][h];
                }
            }

            model.add(lhs <= X[k_p][j]).setName(
                    ("C8_rendezvous_" + std::to_string(j) + "_" + std::to_string(k_p)).c_str());
        }
    }


    // $\phi_h = \sum_{k,i}X^k_{ih} = \sum_{k,i}Y^k_{ih}$
    // - chỉ có duy nhất một điểm xuất phát và môt điểm đích cho
    //mỗi khách hàng $h$ được phục vụ bởi drone (C18)
    for (int h: C) {
        IloExpr rhs(env);
        for (int i = 0; i < D; i++) {
            if (i != h && tau_prime[i][h] <= dtl - sr)
                for (int k = 1; k <= node_max_stage - 1; k++) {
                    rhs += Y[k][i][h];
                }
        }
        // consistency constraint cho sortie phuc vu h.
        model.add(phi[h] == rhs).setName(("C18_" + std::to_string(h)).c_str());
    }


    for (int h: C) {
        IloExpr rhs(env);
        for (int i = 1; i <= D; i++) {
            if (i != h && tau_prime[h][i] <= dtl - sr)
                for (int k = 2; k <= node_max_stage; k++) {
                    rhs += W[k][i][h];
                }
        }
        // consistency constraint cho sortie phuc vu h.
        model.add(phi[h] == rhs).setName(("C18p_" + std::to_string(h)).c_str());
    }

    //////////// C10: node_stage
    for (int h: C) {
        IloExpr sum_k(env);
        for (int k = 2; k < node_max_stage; k++) {
            sum_k += X[k][h];
        }
        // phuc vu h it nhat 1 lan.
        model.add(phi[h] + sum_k >= 1).setName(("C10_" + std::to_string(h)).c_str());
    }


    /////////// C14: node_stage
    for (int k = 1; k <= arc_max_stage; k++) {
        IloExpr sum(env);
        for (int i = 0; i < D; i++) {
            for (int j = 1; j <= D; j++) {
                if (i != j) {
                    sum += x[k][i][j] * tau[i][j];
                }
            }
        }

        // o to toi k+1 >= o to den k + thoi gian di chuyen tu k->k+1 (tau_(ij)). (old model).
        // em nghĩ nên đổi thành bằng nhau, a_(k+1) chắc chắn bằng departure stage trước + di chuyển.
        // d[k+1] mới >= d[k]. trường hợp lớn hơn xảy ra khi truck phải đợi drone bay đến.
        model.add(a[k + 1] == d[k] + sum).setName(("C14_" + std::to_string(k) + "_" + std::to_string(k + 1)).c_str());
    }

    ////////// C15: node_stage
    auto M = 1e5;
    for (int k = 1; k <= node_max_stage - 1; k++) {
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            if (k < k_p) {
                // o to phai den k_p tu k trong khoang thoi gian <= dtl.
                // , "C15_" + std::to_string(k) + "_" + std::to_string(k_p)
                model.add(a[k_p] - d[k] <= z[k][k_p] * (dtl - sr) + (1 - z[k][k_p]) * M).setName(
                        ("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());

                IloExpr rhs(env);
                for (int i = 0; i <= D; i++) {
                    for (int h: C) {
                        if (h != i && i != D) {
                            if (tau_prime[i][h] <= dtl - sr) {
                                rhs += Y[k][i][h] * tau_prime[i][h];
                            }
                        }

                        if (h != i && i != 0) {
                            if (tau_prime[h][i] <= dtl - sr) {
                                rhs += W[k_p][i][h] * tau_prime[h][i];
                            }
                        }
                    }
                }
                // vehicle phai doi drone truoc khi di chuyen.
                //                model.add(d[k_p] - d[k] + (1 - z[k][k_p]) * 2 * (dtl-sr) >= rhs).setName(("C21_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
                //                model.add(d[k_p] - d[k] >= z[k][k_p] * sr - (1-z[k][k_p]) * 100000 + rhs);
            }
        }
    }

    for (int k = 1; k <= node_max_stage - 1; k++) {
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            if (k < k_p) {
                IloExpr sum_length_y_k(env);
                IloExpr sum_length_w_k_p(env);
                IloExpr sum_w_k_p(env);
                IloExpr sum_y_k_p(env);

                for (int i = 0; i < D; i++) {
                    for (int h: C) {
                        if (i != h) {
                            sum_length_y_k += Y[k][i][h] * tau_prime[i][h];
                            sum_y_k_p += Y[k_p][i][h];
                        }
                    }
                }
                for (int j = 1; j <= D; j++) {
                    for (int h: C) {
                        if (h != j) {
                            sum_length_w_k_p += W[k_p][j][h] * tau_prime[h][j];
                            sum_w_k_p += W[k_p][j][h];
                        }
                    }
                }
                model.add(
                        d[k_p] >= d[k] + sum_length_y_k + sum_length_w_k_p + sl * sum_y_k_p + sr * sum_w_k_p - (
                                                                                                                       1 -
                                                                                                                       z[k][k_p]) *
                                                                                                               M);
            }
        }
    }
    for (int k = 2; k <= node_max_stage; k++) {
        IloExpr sum(env);
        IloExpr sum_w_K(env);
        IloExpr sum_y_K(env);
        for (int i = 0; i < D; i++) {
            for (int j = 1; j <= D; j++) {
                if (i != j) {
                    sum += x[k - 1][i][j] * tau[i][j];
                }
            }
        }
        for (int i = 0; i < D; i++) {
            for (int h: C) {
                if (i != h && k != node_max_stage) {
                    sum_y_K += sl * Y[k][i][h];
                }
            }
        }

        for (int i = 1; i <= D; i++) {
            for (int h: C) {
                if (i != h) {
                    sum_w_K += sr * W[k][i][h];
                }
            }
        }
        auto constr_name = "CC_d_" + std::to_string(k) + "_constr";
        model.add(d[k] >= d[k - 1] + sum + sum_w_K + sum_y_K).setName(constr_name.c_str());
    }

    // - sum_sl_at_O * sl + sum_sl_sr)
    for (int k = 1; k <= node_max_stage - 1; k++) {
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            IloExpr sum(env);
            for (int k_p_p = k; k_p_p <= k_p - 1; k_p_p++) {
                for (int i = 0; i < D; i++) {
                    for (int j = 1; j <= D; j++) {
                        if (i != j) {
                            sum += x[k_p_p][i][j] * tau[i][j];
                        }
                    }
                }
            }
            //            model.add(sum - M * (1 - z[k][k_p]) <= dtl-sr);
        }
    }
    model.add(IloMinimize(env, d[node_max_stage]));
    cplex.exportModel("cplex_model_1.lp");
    std::vector<Sortie> st;
    double obj = 0;
    // Solve the model
    if (!cplex.solve()) {
        // Check if the problem is infeasible
        if (cplex.getStatus() == IloAlgorithm::Infeasible) {
            // Handle infeasibility
            std::cout << "The problem is infeasible." << std::endl;
            std::cout << "Infeasibility at: " << cplex.getInfeasibility(c2) << std::endl;
            // You can also retrieve the infeasible constraints using cplex.getInfeasibility() method
        } else {
            // Handle other solver errors
            std::cerr << "Solver error: " << cplex.getStatus() << std::endl;
        }
    } else {
        std::cout << "Feasible solution found!" << std::endl;
        std::cout << "Truck nodes:" << std::endl;
        for (int k = 1; k <= node_max_stage; k++) {
            for (int i = 0; i <= D; i++) {
                auto X_val = cplex.getValue(X[k][i]);
                //std::cout << "k = " << k << ", i = " << i << ":" << X_val << std::endl;
                if (abs(X_val - 1) < 1e-6) {
                    auto d_k = cplex.getValue(d[k]);
                    auto a_k = cplex.getValue(a[k]);
                    std::cout << "Stage " << k << " at customer " << i << " with arrival time is: " << a_k << std::endl;
                    std::cout << "Stage " << k << " at customer " << i << " with departure time is: " << d_k <<
                              std::endl;
                    break;
                }
            }
        }
        std::cout << "Truck arcs:" << std::endl;
        std::map<int, std::pair<int, int> > map_stage_truck_arc;
        for (int k = 1; k <= arc_max_stage; k++) {
            for (int i = 0; i < D; i++)
                for (int j = 1; j <= D; j++)
                    if (i != j) {
                        auto X_val = cplex.getValue(x[k][i][j]);
                        //std::cout << "k = " << k << ", i = " << i << ":" << X_val << std::endl;
                        if (abs(X_val - 1) < 1e-6) {
                            std::cout << "Arc " << k << " connecting " << i << " and " << j
                                      << " with cost " << tau[i][j] << " " << std::endl;
                            obj += tau[i][j];
                            map_stage_truck_arc[k] = std::make_pair(i, j);
                            break;
                        }
                    }
        }
        std::cout << "Drone served customers" << std::endl;
        //IloNumArray phi_val(env);
        //cplex.getValues(phi_val, phi);


        for (int k = 1; k < node_max_stage; k++)
            for (int kp = k + 1; kp <= node_max_stage; kp++) {
                //std::cout << k << " " << kp << std::endl;
                if (abs(cplex.getValue(z[k][kp]) - 1) < 1e-6) {
                    std::cout << "Drone flies from stage " << k << " to stage " << kp << std::endl;
                }
            }
        for (int h: C)
            for (int k = 1; k < node_max_stage; k++)
                for (int kp = k + 1; kp <= node_max_stage; kp++) {
                    //std::cout << k << " " << kp << std::endl;
                    if (abs(cplex.getValue(Z[h][k][kp]) - 1) < 1e-6) {
                        std::cout << "Drone flies from stage " << k << " to stage " << kp << " to serve customer " << h
                                  << std::endl;
                    }
                }

        for (int h: C) {
            if (abs(cplex.getValue(phi[h]) - 1) < 1e-6) {
                std::cout << "Customer " << h << " served by drone." << std::endl;
                auto sortie = Sortie(h);
                st.push_back(sortie);
                int sv_i = -1, sv_j = -1, sv_k = -1, sv_kp = -1;
                for (int k = 1; k <= node_max_stage; k++) {
                    for (int i = 0; i <= D; i++)
                        if (i != h) {
                            try {
                                //std::cout << "from stage " << k << " at customer " << i << " to serve" << h << std::endl;
                                auto Y_val = cplex.getValue(Y[k][i][h]);
                                if (abs(Y_val - 1) < 1e-6) {
                                    sv_i = i;
                                    sv_k = k;
                                    //std::cout << "XXX from stage " << k << " at customer " << i << " to serve" << h << std::endl;
                                }

                                //std::cout << "to stage " << k << " at customer" << i << " from " << h << std::endl;
                                auto W_val = cplex.getValue(W[k][i][h]);
                                if (abs(W_val - 1) < 1e-6) {
                                    sv_j = i;
                                    sv_kp = k;
                                    //std::cout << "ZZZ to stage " << k << " at customer" << i << " from " << h << std::endl;
                                }
                            } catch (...) {
                            }
                        }
                }
                ////assert(abs(cplex.getValue(z[sv_k][sv_kp]) - 1.0) < 1e-6);

                std::cout << "Drone fly from " << sv_i << " at stage " << sv_k <<
                          " to serve " << h << " and then fly back to " << sv_j
                          << " at stage " << sv_kp << ". " << std::endl;
                obj += (sl + sr);
                if (sv_i == O) {
                    obj -= sl;
                }
                double drone_travel_time = tau_prime[sv_i][h] + tau_prime[h][sv_j];
                double truck_travel_time = 0;
                for (int k_start = sv_k; k_start <= sv_kp - 1; k_start++) {
                    truck_travel_time += tau[map_stage_truck_arc[k_start].first][map_stage_truck_arc[k_start].second];
                }
                std::cout << "Truck travel time from stage " << sv_k << " to " << sv_kp << " is: " << truck_travel_time
                          << std::endl;
                if (drone_travel_time > truck_travel_time) {
                    obj += drone_travel_time - truck_travel_time;
                }
                auto drone_arrival_time = cplex.getValue(d[sv_k]) + drone_travel_time;
                auto vehicle_departure_time = cplex.getValue(d[sv_kp]);
                auto truck_arrival_time = cplex.getValue(a[sv_kp]);
                std::cout << "Start arc cost: "
                          << tau_prime[sv_i][h] << ", end arc cost: " << tau_prime[h][sv_j] <<
                          ". Total drone travel time: " << drone_travel_time << std::endl;

                std::cout << "Drone arrival time: " << drone_arrival_time << std::endl;
                std::cout << "Truck arrival time: " << truck_arrival_time << std::endl;

                std::cout << "Truck departure time = max(d/a, t/a) plus (sl): " << vehicle_departure_time << std::endl;
                assert(drone_arrival_time <= vehicle_departure_time);
                assert(abs(cplex.getValue(Z[h][sv_k][sv_kp]) - 1.0) < 1e-6);

                assert(abs(cplex.getValue(z[sv_k][sv_kp]) - 1.0) < 1e-6);
            }
        }

        std::cout << "Done!" << std::endl;
        std::cout << "-------------------------Re-calculated objective-----------------------" << std::endl;
        std::cout << obj << std::endl;
        std::cout << "------------------------------------------------------------------------" << std::endl;
    }
    double c = cplex.getObjValue();
    double time_spent = cplex.getCplexTime();
    env.end();
    cplex.end();
    model.end();
    std::cout << "OBJECTIVE VALUE: " << c << ", NUMBER OF SORTIES: " << st.size() << "." << std::endl;
    return Result{c, obj, time_spent, st};
}

// Paper: Exact methods for the traveling salesman problem with drone.
// Link: https://drive.google.com/file/d/1aglcxNadkpMoxdb_o-bYVVgc7PAANxK6/view?usp=sharing
Result Solver::Roberti2020(int n_thread, int e) {
    auto tau = instance->tau;
    auto tau_prime = instance->tau_prime;
    auto dtl = e;
    //dtl = 5;
    auto sl = 1, sr = 1;
    auto n = instance->num_node;

    auto c_prime = instance->c_prime;
    std::vector<int> c_prime_0;
    c_prime_0.push_back(0);
    for (int i: c_prime) {
        c_prime_0.push_back(i);
    }
    c_prime_0.push_back(n);

    std::cout << "Printing number of nodes: " << n << std::endl;
    std::vector<int> C;
    std::vector<int> V;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0 || i == n) {
            V.push_back(i);
        } else {
            V.push_back(i);
            C.push_back(i);
        }
    }

    // C_s : set C(customer) union s (source) ; source  = 0
    // C_t : set C(customer) union t (terminal); terminal = n
    // create.
    std::vector<int> c_s;
    std::vector<int> c_t;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0) {
            c_s.push_back(i);
        } else if (i == n) {
            c_t.push_back(i);
        } else {
            c_s.push_back(i);
            c_t.push_back(i);
        }
    }

    std::cout << std::endl;
    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
    // notations
    auto N = C; // N: set of customers
    auto N_0 = c_s; // N_0 = s unions N
    auto N_0_p = c_t; // N_0' = t unions N
    auto s = 0;
    auto t = n;

    IloArray<IloBoolVarArray> xt(env, n + 1);
    for (int i: c_s) {
        xt[i] = IloBoolVarArray(env, n + 1);
        for (int j: c_t) {
            if (i != j) {
                xt[i][j] = IloBoolVar(env);
                if (i == s && j == t) {
                    model.add(xt[i][j] == 0);
                }
            }
        }
    }

    IloArray<IloBoolVarArray> xd(env, n + 1);
    for (int i: c_s) {
        xd[i] = IloBoolVarArray(env, n + 1);
        for (int j: c_t) {
            if (i != j) {
                xd[i][j] = IloBoolVar(env);
                if (i == s && j == t) {
                    model.add(xd[i][j] == 0);
                }
            }
        }
    }

    IloBoolVarArray yt(env, n);
    IloBoolVarArray yd(env, n);
    IloBoolVarArray yc(env, n);
    IloBoolVarArray l(env, n);
    for (int i: N) {
        yt[i] = IloBoolVar(env);
        yd[i] = IloBoolVar(env);
        yc[i] = IloBoolVar(env);
        l[i] = IloBoolVar(env);
    }

    // Variable z
    IloArray<IloBoolVarArray> z(env, n + 1);
    for (int i: c_s) {
        z[i] = IloBoolVarArray(env, n + 1);
        for (int j: N) {
            if (i != j) {
                z[i][j] = IloBoolVar(env);
            }
        }
    }
    IloNumVarArray a(env, n + 1);
    for (int i: V) {
        a[i] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
    }


    // Constraints::

    // 1b
    for (int i: N) {
        IloExpr s1(env), s2(env);
        for (int j: c_t) {
            if (i != j) {
                s1 += xt[i][j];
            }
        }
        for (int j: c_s) {
            if (i != j) {
                s2 += xt[j][i];
            }
        }
        model.add(s1 == s2);
    }

    // 1c
    for (int i: N) {
        IloExpr s1(env);
        for (int j: c_t) {
            if (i != j) {
                s1 += xt[i][j];
            }
        }
        model.add(s1 == yt[i] + yc[i]);
    }

    // 1d
    IloExpr s1_1d(env), s2_1d(env);
    for (int j: c_t) {
        s1_1d += xt[s][j];
    }
    for (int i: c_s) {
        s2_1d += xt[i][t];
    }
    model.add(s1_1d == 1);
    model.add(s1_1d == s2_1d);

    // 1e
    for (int i: N) {
        IloExpr s1(env), s2(env);
        for (int j: c_t) {
            if (i != j) {
                s1 += xd[i][j];
            }
        }
        for (int j: c_s) {
            if (i != j) {
                s2 += xd[j][i];
            }
        }
        model.add(s1 == s2);
    }

    // 1f
    for (int i: N) {
        IloExpr s1(env);
        for (int j: c_t) {
            if (i != j) {
                s1 += xd[i][j];
            }
        }
        model.add(s1 == yd[i] + yc[i]);
    }

    // 1g
    IloExpr s1_1g(env), s2_1g(env);
    for (int j: c_t) {
        s1_1g += xd[s][j];
    }
    for (int i: c_s) {
        s2_1g += xd[i][t];
    }
    model.add(s1_1g == 1);
    model.add(s1_1g == s2_1g);

    // 1h
    for (int i: N) {
        model.add(yt[i] + yd[i] + yc[i] == 1);
    }

    int M = 1e5;
    // 1i
    for (int i: c_s) {
        for (int j: c_t) {
            if (i != j) {
                model.add(a[i] + tau[i][j] <= a[j] + M * (1 - xt[i][j]));
            }
        }
    }

    // 1j
    for (int i: c_s) {
        for (int j: c_t) {
            if (i != j) {
                model.add(a[i] + tau_prime[i][j] <= a[j] + M * (1 - xd[i][j]));
            }
        }
    }

    // 1k
    for (int i: N) {
        for (int j: N) {
            if (i < j) {
                model.add(xd[i][j] + xd[j][i] <= yc[i] + yc[j]);
            }
        }
    }

    // 1l
    for (int i: N) {
        model.add(xd[s][i] + xd[i][t] <= 1);
    }

    // Valid inequalities
    IloExpr sum_t(env), sum_d(env);
    for (int i: c_s) {
        for (int j: c_t) {
            if (i != j) {
                sum_t += xt[i][j] * tau[i][j];
                sum_d += xd[i][j] * tau_prime[i][j];
            }
        }
    }
    model.add(sum_t <= a[t]);
    model.add(sum_d <= a[t]);

    // Incompatible drone customer(s)
    for (int heavy: instance->heavy) {
        model.add(yd[heavy] == 0);
        for (int j: c_s) {
            if (heavy != j) {
                model.add(z[j][heavy] == 0);
            }
        }
    }

    // Drone endurance
    // 5a
    for (int i: c_s) {
        for (int j: c_t) {
            if (i != j) {
                if (tau_prime[i][j] >= dtl - sr) {
                    model.add(xd[i][j] <= xt[i][j]);
                }
            }
        }
    }

    // 5b
    for (int j: N) {
        if (tau_prime[s][j] + tau_prime[j][t] > dtl - sr) {
            model.add(z[s][j] == 0);
        }
    }

    // 5c
    for (int i: N) {
        for (int j: N) {
            if (i != j) {
                if (tau_prime[i][j] + tau_prime[j][i] > dtl - sr) {
                    model.add(z[i][j] == 0);
                }
            }
        }
    }

    // 5d
    for (int i: N) {
        IloExpr s1(env), s2(env), s3(env);
        for (int j: c_s) {
            if (j != i) {
                s1 += tau_prime[j][i] * xd[j][i];
            }
        }
        for (int j: c_t) {
            if (j != i) {
                s2 += tau_prime[i][j] * xd[i][j];
            }
        }
        for (int j: N) {
            if (i != j) {
                s3 += (tau_prime[j][i] + tau_prime[i][j]) * z[j][i];
            }
        }
        model.add(s1 + s2 + (tau_prime[s][i] + tau_prime[i][t]) * z[s][i] + s3 <= dtl - sr + M * (1 - yd[i]));
    }

    // Drone cannot land and wait
    // 7a
    for (int j: c_s) {
        for (int k: c_t) {
            for (int i: N) {
                if (i != j && j != k && i != k) {
                    if (tau_prime[j][i] + tau_prime[i][k] <= dtl - sr) {
                        model.add(a[k] - a[j] <= dtl - sr + M * (2 - xd[j][i] - xd[i][k]) + M * (1 - yd[i]));
                    }
                }
            }
        }
    }

    // Launch and rendezvous time
    // c9
    for (int i: N) {
        model.add(l[i] >= yd[i] - xd[s][i]);
    }

    IloExpr objective(env);
    objective += a[t];
    for (int j: N) {
        objective += (tau_prime[s][j] + tau_prime[j][t] + sr) * z[s][j];
    }
    for (int i: N) {
        for (int j: N) {
            if (i != j) {
                objective += (tau_prime[i][j] + tau_prime[j][i] + sl + sr) * z[i][j];
            }
        }
        objective += sl * l[i];
        objective += sr * yd[i];
    }
    // Objective definition
    model.add(IloMinimize(env, objective));
    auto startTime = std::chrono::high_resolution_clock::now();
    cplex.solve();
    auto endTime = std::chrono::high_resolution_clock::now();
    // Calculate the elapsed time
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Total time: " << cplex.getObjValue() << std::endl;
    std::cout << "Truck arcs:" << std::endl;
    for (int i: c_s) {
        for (int j: c_t) {
            if (i != j) {
                if (cplex.getValue(xt[i][j]) == 1) {
                    std::cout << i << " " << j << std::endl;
                }
            }

        }
    }
    std::cout << "Drone served customers:" << std::endl;
    for (int i: N) {
        if (cplex.getValue(yd[i]) == 1) {
            std::cout << "Customer " << i << " served by drone." << std::endl;
        }
    }
    std::cout << "Combined customers:" << std::endl;
    for (int i: N) {
        if (cplex.getValue(yc[i]) == 1) {
            std::cout << "Customer " << i << " is a combined customer." << std::endl;
        }
    }
    std::cout << "Drone arcs:" << std::endl;
    for (int i: c_s) {
        for (int j: c_t) {
            if (i != j) {
                if (cplex.getValue(xd[i][j]) == 1) {
                    std::cout << i << " " << j << " " << cplex.getValue(xd[i][j]) << std::endl;
                }
            }

        }
    }
    std::vector<Sortie> st;
    return Result(cplex.getObjValue(), 0, duration.count()/1000, st);
}

Result Solver::Amico2021_3Index(int n_thread, int e) {
    auto tau = instance->tau;
    auto tau_prime = instance->tau_prime;
    auto dtl = e;
    //dtl = 5;
    auto sl = 1, sr = 1;
    auto n = instance->num_node;

    auto c_prime = instance->c_prime;
    std::vector<int> c_prime_0;
    c_prime_0.push_back(0);
    for (int i: c_prime) {
        c_prime_0.push_back(i);
    }
    c_prime_0.push_back(n);

    std::cout << "Printing number of nodes: " << n << std::endl;
    std::vector<int> C;
    std::vector<int> V;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0 || i == n) {
            V.push_back(i);
        } else {
            V.push_back(i);
            C.push_back(i);
        }
    }

    // C_s : set C(customer) union s (source) ; source  = 0
    // C_t : set C(customer) union t (terminal); terminal = n
    // create.
    std::vector<int> c_s;
    std::vector<int> c_t;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0) {
            c_s.push_back(i);
        } else if (i == n) {
            c_t.push_back(i);
        } else {
            c_s.push_back(i);
            c_t.push_back(i);
        }
    }

    // paper notation synchronization
    auto N = V;
    auto N0 = c_s;
    auto N_p = c_t;
    auto M = 1e5;
    std::cout << std::endl;
    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);

    IloArray<IloBoolVarArray> x(env, n+1);
    for (int i:N0) {
        x[i] = IloBoolVarArray(env, n+1);
        for (int j:N_p) {
            if (i != j) {
                x[i][j] = IloBoolVar(env);
            }
        }
    }
    IloArray<IloArray<IloBoolVarArray>> y(env, n+1);
    for (int i:N0) {
        y[i] = IloArray<IloBoolVarArray>(env, n+1);
        for (int j:c_prime) {
            if (i != j) {
                y[i][j] = IloBoolVarArray(env, n+1);
                for (int k:N_p) {
                    if (i != k && j != k) {
                        if (tau_prime[i][j] + tau_prime[j][k] + sr <= dtl) {
                            y[i][j][k] = IloBoolVar(env);
                        }
                    }
                }
            }
        }
    }

    IloNumVarArray w(env, n+1), t(env, n+1);
    for (int i:N) {
        w[i] = IloNumVar(env, 0, IloInfinity);
        t[i] = IloNumVar(env, 0, IloInfinity);
    }

    IloBoolVarArray z(env, n+1);
    for (int i:N) {
        z[i] = IloBoolVar(env);
    }

    IloExpr objective(env);

    // Objective's first term
    for (int i:N0) {
        for (int j:N_p) {
            if (i != j) {
                objective += x[i][j] * tau[i][j];
            }
        }
    }

    // Objective's second term
    IloExpr obj_2nd_term(env);
    for (int j:c_prime) {
        for (int k:N_p) {
            if (j != k) {
                if (tau_prime[0][j] + tau_prime[j][k] + sr <= dtl) {
                    obj_2nd_term += y[0][j][k];
                }
            }
        }
    }
    objective += sr * obj_2nd_term;

    // Objective's third term
    IloExpr obj_3rd_term(env);
    for (int i:N0) {
        if (i != 0) {
            for (int j:c_prime) {
                if (i != j) {
                    for (int k:N_p) {
                        if (i != k && j != k) {
                            if (tau_prime[i][j] + tau_prime[j][k] + sr <= dtl) {
                                obj_3rd_term += y[i][j][k];
                            }
                        }
                    }
                }
            }
        }
    }
    objective += (sl + sr) * obj_3rd_term;

    // Fourth term
    for (int i:N_p) {
        objective += w[i];
    }

    // Constraints:

    // C2
    for (int j:C) {
        IloExpr s1(env), s2(env);
        for (int i:N0) {
            if (i != j) {
                s1 += x[i][j];
            }
        }
        if (exist(c_prime, j)) {
            for (int i:N0) {
                for (int k:N_p) {
                    if (i != j && i != k && j != k) {
                        if (tau_prime[i][j] + tau_prime[j][k] + sr <= dtl) {
                            s2 += y[i][j][k];
                        }
                    }
                }
            }
        } else {
            std::cout << "customer " << j << " can't be served by drone!" << std::endl;
            s2 = nullptr;
            model.add(s1 == 1);
            continue;
        }
        model.add(s1 + s2 == 1);
    }

    // C3
    IloExpr c3_s1(env), c3_s2(env);
    for (int j:N_p) {
        c3_s1 += x[0][j];
    }
    for (int i:N0) {
        c3_s2 += x[i][n];
    }
    model.add(c3_s1 == 1);
    model.add(c3_s2 == 1);

    // C4
    for (int j:C) {
        IloExpr c4_s1(env), c4_s2(env);
        for (int i:N0) {
            if (i != j) {
                c4_s1 += x[i][j];
            }
        }
        for (int i:N_p) {
            if (i != j) {
                c4_s2 += x[j][i];
            }
        }
        model.add(c4_s1 == c4_s2);
    }

    // C5
    for (int i:N0) {
        for (int j:N_p) {
            if (i != j) {
                model.add(t[j] >= t[i] + tau[i][j] - M * (1 - x[i][j]));
            }
        }
    }

    // C6
    for (int i:N0) {
        for (int j:N_p) {
            if (i != j) {
                IloExpr c6_s1(env);
                for (int k:c_prime) {
                    if (i != j && i != k && j != k) {
                        if (tau_prime[i][k] + tau_prime[k][j] + sr <= dtl) {
                            c6_s1 += (M + tau_prime[i][k] + tau_prime[k][j]) * y[i][k][j];
                        }
                    }
                }
                model.add(t[j] >= t[i] + c6_s1 - M);
            }
        }
    }

    // C7
    for (int i:N0) {
        for (int j:N_p) {
            if (i != j) {
                model.add(w[j] >= t[j] - t[i] - tau[i][j] - M * (1 - x[i][j]));
            }
        }
    }

    // C8
    for (int i:N0) {
        for (int j:N_p) {
            if (i != j) {
                IloExpr c8_s1(env);
                for (int k:c_prime) {
                    if (i != j && i != k && j != k) {
                        if (tau_prime[i][k] + tau_prime[k][j] + sr <= dtl) {
                            c8_s1 += y[i][k][j];
                        }
                    }
                }
                model.add(t[j] - t[i] + sr - M * (1 - c8_s1) <= dtl);
            }
        }
    }

    // C9
    for (int i:N_p) {
        IloExpr c9_s1(env);
        for (int j:N0) {
            if (i != j) {
                c9_s1 += x[j][i];
            }
        }
        model.add(z[i] <= c9_s1);
    }

    // C10
    for (int i:N0) {
        IloExpr c10_s1(env);
        for (int j:c_prime) {
            for (int k:N_p) {
                if (i != j && j != k && i != k) {
                    if (tau_prime[i][j] + tau_prime[j][k] + sr <= dtl) {
                        c10_s1 += y[i][j][k];
                    }
                }
            }
        }
        model.add(c10_s1 <= z[i]);
    }

    // C11
    for (int i:N0) {
        for (int j:N_p) {
            if (i != j) {
                IloExpr c11_s1(env), c11_s2(env);
                for (int l:N0) {
                    for (int k:c_prime) {
                        if (l != k && l != j && k != j) {
                            if (tau_prime[l][k] + tau_prime[k][j] + sr <= dtl) {
                                c11_s1 += y[l][k][j];
                            }

                        }
                    }
                }
                for (int k:c_prime) {
                    for (int l:N_p) {
                        if (i != k && i != l && l != k) {
                            if (tau_prime[i][k] + tau_prime[k][l] + sr <= dtl) {
                                c11_s2 += y[i][k][l];
                            }
                        }
                    }
                }
                model.add(z[j] <= z[i] - x[i][j] + c11_s1 - c11_s2 + 1);
            }
        }
    }

    // Add constraint
    for (int j:c_prime) {
        IloExpr sum(env);
        for (int i:c_s) {
            for (int k:c_t) {
                if (i != j && j != k && i != k) {
                    sum += y[i][j][k];
                }
            }
        }
        for (int i:c_prime) {
            for (int k:c_t) {
                if (i != j && j != k && i != k) {
                    sum += y[j][i][k];
                }
            }
        }
        for (int k:c_s) {
            for (int i:c_prime) {
                if (i != j && j != k && i != k) {
                    sum += y[k][i][j];
                }
            }
        }
        model.add(sum <= 1);
    }
    // C12
    model.add(t[0] == 0);

    // Valid inequalities

//    // C17
    IloExpr c17_s1(env);
    for (int j:N_p) {
        c17_s1 += x[0][j];
    }
    model.add(z[0] <= c17_s1);

    // C18
    for (int i:N_p) {
        IloExpr c18_s1(env);
        for (int k:N0) {
            for (int j:c_prime) {
                if (i != j && j != k && i != k) {
                    if (tau_prime[k][j] + tau_prime[j][i] + sr <= dtl) {
                        c18_s1 += y[k][j][i];
                    }
                }
            }
        }
        model.add(c18_s1 <= z[i]);
    }

    // C19
    for (int i:c_prime) {
        IloExpr c19_s1(env);
        for (int j:N0) {
            for (int k:N_p) {
                if (i != j && j != k && i != k) {
                    if (tau_prime[j][i] + tau_prime[i][k] + sr <= dtl) {
                        c19_s1 += y[j][i][k];
                    }
                }
            }
        }
        model.add(c19_s1 <= 1 - z[i]);
    }

    // C20
    for (int j:N_p) {
        IloExpr c20_s1(env);
        for (int i:N0) {
            for (int k:c_prime) {
                if (i != j && j != k && i != k) {
                    if (tau_prime[i][k] + tau_prime[k][j] + sr <= dtl) {
                        c20_s1 += y[i][k][j];
                    }
                }
            }
        }
        model.add(w[j] <= dtl * c20_s1);
    }

    // C21
    for (int j:c_prime) {
        IloExpr c21_s1(env);
        for (int i:N0) {
            for (int k:N_p) {
                if (i != j && j != k && i != k) {
                    if (tau_prime[i][j] + tau_prime[j][k] + sr <= dtl) {
                        c21_s1 += y[i][j][k];
                    }
                }
            }
        }
        model.add(w[j] <= dtl * (1 - c21_s1));
    }
    model.add(IloMinimize(env, objective));

    auto startTime = std::chrono::high_resolution_clock::now();
    cplex.solve();
    auto endTime = std::chrono::high_resolution_clock::now();
    // Calculate the elapsed time
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    if (cplex.getStatus() == IloAlgorithm::Infeasible) {
        // Handle infeasibility
        std::cout << "The problem is infeasible." << std::endl;
        // You can also retrieve the infeasible constraints using cplex.getInfeasibility() method
    } else {
        // Handle other solver errors
        std::cerr << "Solver error: " << cplex.getStatus() << std::endl;
    }
    for (int i:c_s) {
        for (int j:c_t) {
            if (i != j) {
                if (cplex.getValue(x[i][j]) == 1) {
                    std::cout << i << " " << j << std::endl;
                }
            }
        }
    }
    std::cout << "Drone sorties:" << std::endl;
    for (int i:N0) {
        for (int j:c_prime) {
            for (int k:N_p) {
                if (i != j && i != k && j != k) {
                    if (tau_prime[i][j] + tau_prime[j][k] <= e - sr) {
                        if (cplex.getValue(y[i][j][k]) == 1) {
                            std::cout << i << " " << j << " " << k << std::endl;
                        }
                    }
                }
            }
        }
    }
    std::cout << cplex.getObjValue() << std::endl;
    std::vector<Sortie> st;
    return Result(cplex.getObjValue(), 0, duration.count()/1000, st);
}

Result Solver::Amico2021_2Index(int n_thread, int e) {
    auto tau = instance->tau;
    auto tau_prime = instance->tau_prime;
    auto dtl = e;
    //dtl = 5;
    auto sl = 1, sr = 1;
    auto n = instance->num_node;

    auto c_prime = instance->c_prime;
    std::vector<int> c_prime_0;
    c_prime_0.push_back(0);
    for (int i: c_prime) {
        c_prime_0.push_back(i);
    }
    c_prime_0.push_back(n);

    std::cout << "Printing number of nodes: " << n << std::endl;
    std::vector<int> C;
    std::vector<int> V;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0 || i == n) {
            V.push_back(i);
        } else {
            V.push_back(i);
            C.push_back(i);
        }
    }

    // C_s : set C(customer) union s (source) ; source  = 0
    // C_t : set C(customer) union t (terminal); terminal = n
    // create.
    std::vector<int> c_s;
    std::vector<int> c_t;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0) {
            c_s.push_back(i);
        } else if (i == n) {
            c_t.push_back(i);
        } else {
            c_s.push_back(i);
            c_t.push_back(i);
        }
    }

    // paper notation synchronization
    auto N = V;
    auto N0 = c_s;
    auto N_p = c_t;
    auto M = 1e5;
    std::cout << std::endl;
    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);

    IloNumVarArray w(env, n+1), t(env, n+1);
    for (int i:N) {
        w[i] = IloNumVar(env, 0, IloInfinity);
        t[i] = IloNumVar(env, 0, IloInfinity);
    }

    IloBoolVarArray z(env, n+1);
    for (int i:N) {
        z[i] = IloBoolVar(env);
    }

    IloArray<IloBoolVarArray> x(env, n+1);
    for (int i:N0) {
        x[i] = IloBoolVarArray(env, n+1);
        for (int j:N_p) {
            if (i != j) {
                x[i][j] = IloBoolVar(env);
            }
        }
    }
    IloArray<IloBoolVarArray> gl(env, n+1);
    for (int i:N0) {
        gl[i] = IloBoolVarArray(env, n+1);
        for (int j:c_prime) {
            if (i != j) {
                gl[i][j] = IloBoolVar(env);
                if (tau_prime[i][j] > e) {
                    model.add(gl[i][j] == 0);
                }
            }
        }
    }
    IloArray<IloBoolVarArray> gr(env, n+1);
    for (int j:c_prime) {
        gr[j] = IloBoolVarArray(env, n+1);
        for (int k:N_p) {
            if (j != k) {
                gr[j][k] = IloBoolVar(env);
                if (tau_prime[j][k] > e) {
                    model.add(gr[j][k] == 0);
                }
            }
        }
    }

    // Objective
    IloExpr objective(env);

    // First term
    for (int i:N0) {
        for (int j:N_p) {
            if (i != j) {
                objective += x[i][j] * tau[i][j];
            }
        }
    }

    // Second term
    IloExpr obj_2nd_term(env);
    for (int i:N0) {
        if (i != 0) {
            for (int j:c_prime) {
                if (i != j) {
                    obj_2nd_term += gl[i][j];
                }
            }
        }
    }
    objective += sl * obj_2nd_term;

    // Third term
    IloExpr obj_3rd_term(env);
    for (int j:c_prime) {
        for (int k:N_p) {
            if (j != k) {
                objective += gr[j][k];
            }
        }
    }
    objective += sr * obj_3rd_term;

    // Fourth term
    for (int i:N_p) {
        objective += w[i];
    }

    // C23
    for (int j:C) {
        IloExpr c23_s1(env), c23_s2(env);
        for (int i:N0) {
            if (i != j) {
                c23_s1 += x[i][j];
                c23_s2 += gl[i][j];
            }
        }
        model.add(c23_s1 + c23_s2 == 1);
    }

    // C24
    for (int i:N0) {
        for (int j:N_p) {
            if (i != j) {
                model.add(t[j] >= t[i] + tau_prime[i][j] - M * (1 - gl[i][j]));
            }
        }
    }

    // C25
    for (int j:c_prime) {
        for (int k:N_p) {
            if (j != k) {
                model.add(t[k] >= t[j] + tau_prime[j][k] - M * (1 - gr[j][k]));
            }
        }
    }

    // C26
    for (int i:N0) {
        for (int j:c_prime) {
            for (int k:N_p) {
                if (i != j && i != k && j != k) {
                    if (tau_prime[i][j] + tau_prime[j][k] + sr <= dtl) {
                        model.add(t[k] - t[i] + sr - M * (2 - gl[i][j] - gr[j][k]) <= e);
                    }
                }
            }
        }
    }

    // C27
    for (int j:c_prime) {
        IloExpr s1(env), s2(env);
        for (int i:N0) {
            if (i != j) {
                s1 += gl[i][j];
            }
        }
        for (int k:N_p) {
            if (j != k) {
                s2 += gr[j][k];
            }
        }
        model.add(s1 == s2);
    }

    // C28
    for (int i:N0) {
        IloExpr s(env);
        for (int j:c_prime) {
            if (i != j) {
                s += gl[i][j];
            }
        }
        model.add(s <= z[i]);
    }

    // C29
    for (int i:N0) {
        for (int j:N_p) {
            if (i != j) {
                IloExpr s(env);
                for (int k:c_prime) {
                    if (k != j && i != k) {
                        s += gr[k][j];
                        s -= gl[i][k];
                    }
                }
                model.add(z[j] <= z[i] - x[i][j] + s + 1);
            }
        }
    }

    // C30
    for (int i:c_prime) {
        for (int j:c_prime) {
            if (i != j) {
                model.add(gl[i][j] + gr[i][j] <= 1);
                model.add(gl[i][j] + gr[j][i] <= 1);
            }
        }
    }

    // C3
    IloExpr c3_s1(env), c3_s2(env);
    for (int j:N_p) {
        c3_s1 += x[0][j];
    }
    for (int i:N0) {
        c3_s2 += x[i][n];
    }
    model.add(c3_s1 == 1);
    model.add(c3_s2 == 1);

    // C4
    for (int j:C) {
        IloExpr c4_s1(env), c4_s2(env);
        for (int i:N0) {
            if (i != j) {
                c4_s1 += x[i][j];
            }
        }
        for (int i:N_p) {
            if (i != j) {
                c4_s2 += x[j][i];
            }
        }
        model.add(c4_s1 == c4_s2);
    }
    // C5
    for (int i:N0) {
        for (int j:N_p) {
            if (i != j) {
                model.add(t[j] >= t[i] + tau[i][j] - M * (1 - x[i][j]));
            }
        }
    }
    // C7
    for (int i:N0) {
        for (int j:N_p) {
            if (i != j) {
                model.add(w[j] >= t[j] - t[i] - tau[i][j] - M * (1 - x[i][j]));
            }
        }
    }
    // C9
    for (int i:N_p) {
        IloExpr c9_s1(env);
        for (int j:N0) {
            if (i != j) {
                c9_s1 += x[j][i];
            }
        }
        model.add(z[i] <= c9_s1);
    }

    model.add(IloMinimize(env, objective));
    cplex.solve();
    std::cout << cplex.getObjValue() << std::endl;
    for (int i:c_s) {
        for (int j:c_t) {
            if (i != j) {
                if (cplex.getValue(x[i][j]) == 1) {
                    std::cout << i << " " << j << std::endl;
                }
            }
        }
    }
    std::cout << "gl:" << std::endl;
    for (int i:N0) {
        for (int j:c_prime) {
            if (i != j) {
                if (tau_prime[i][j] <= e) {
                    if (cplex.getValue(gl[i][j]) == 1) {
                        std::cout << i << " " << j << std::endl;
                    }
                }
            }
        }
    }

    std::cout << "gr:" << std::endl;
    for (int j:c_prime) {
        for (int k:N_p) {
            if (k != j) {
                if (tau_prime[j][k] <= e) {
                    if (cplex.getValue(gr[j][k]) == 1) {
                        std::cout << j << " " << k << std::endl;
                    }
                }
            }
        }
    }

    std::cout << cplex.getObjValue() << std::endl;
    return Result();
}
