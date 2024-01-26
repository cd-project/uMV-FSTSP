//
// Created by cuong on 18/01/2024.
//
#include "../include/solver.h"
#include <iostream>
#include <vector>
#include <algorithm>
std::vector<std::pair<std::vector<int>, std::vector<int>>> generateSetsAndComplements(const std::vector<int>& elements) {
    auto n = elements.size();
    std::vector<std::pair<std::vector<int>, std::vector<int>>> result;

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
bool containsSOrT(const std::vector<int>& inputVector, int s, int t) {
    return (std::find(inputVector.begin(), inputVector.end(), s) != inputVector.end()) ||
           (std::find(inputVector.begin(), inputVector.end(), t) != inputVector.end());
}
void setPrint(std::vector<int> &set) {
    for (int i:set) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
}

//std::vector<std::unordered_set<int>> generateAllSubsets(const std::vector<int>& originalSet) {
//    int n = originalSet.size();
//    int totalSubsets = 1 << n;
//
//    std::vector<std::unordered_set<int>> allSubsets;
//
//    for (int i = 0; i < totalSubsets; ++i) {
//        std::unordered_set<int> subset;
//        for (int j = 0; j < n; ++j) {
//            if (i & (1 << j)) subset.insert(originalSet[j]);
//        }
//        allSubsets.push_back(subset);
//    }
//
//    return allSubsets;
//}
// Paper: https://drive.google.com/file/d/1CpYCse--JWmrnBY566Obe8IMpcClTpnI/view?usp=sharing
void Solver::OriginalSolver(int n_thread) {
//    try {
        auto tau = instance->tau;
        auto d = instance->tau_prime;
        auto dtl = instance->e;
        dtl = 20;
        auto sl = 1, sr = 1;
        auto n = instance->num_node;
        auto s = 0, t = n;
        auto c_prime = instance->c_prime;
        std::vector<int> c_prime_0;
        c_prime_0.push_back(0);
        for (int i : c_prime) {
            c_prime_0.push_back(i);
        }
        c_prime_0.push_back(n);
        std::cout << "Printing number of nodes: " << n << std::endl;
        std::vector<int> C;
        std::vector<int> V;
        for (int i = 0; i < n+1; i++) {
            if (i == 0 || i == n) {
                V.push_back(i);
            } else {
                V.push_back(i);
                C.push_back(i);
            }
        }
        std::vector<int> c_s;
        std::vector<int> c_t;
        for (int i = 0; i < n+1; i++) {
            if (i == 0) {
                c_s.push_back(i);
            } else if (i == n){
                c_t.push_back(i);
            } else {
                c_s.push_back(i);
                c_t.push_back(i);
            }
        }

        std::cout << std::endl;
        GRBEnv env;
        GRBModel model(env);
        // y: (i, j) in A, truck route
        auto** y = new GRBVar * [n+1];
        for (int i:c_s) {
            y[i] = reinterpret_cast<GRBVar *>(new GRBVar *[n+1]);
            for (int j:c_t) {
                y[i][j] = model.addVar(0, 1, 0.0, GRB_BINARY, "y_" + std::to_string(i) + "_" + std::to_string(j));
                if (i == j) {
                    model.addConstr(y[i][j] == 0);
                }
            }
        }
        model.addConstr(y[s][t] == 0);


        auto** x = new GRBVar * [n+1];
        for (int i:c_s) {
            x[i] = reinterpret_cast<GRBVar *>(new GRBVar *[n+1]);
            for (int j:c_t) {
                x[i][j] = model.addVar(0, 1, 0.0, GRB_BINARY, "x_" + std::to_string(i) + "_" + std::to_string(j));
                if (i == j) {
                    model.addConstr(x[i][j] == 0);
                }

            }
        }

        model.addConstr(x[s][t] == 0);

        // gamma_h_ij
        auto*** gamma = new GRBVar ** [n+1];
        for (int h:C){
            gamma[h] = reinterpret_cast<GRBVar **>(new GRBVar **[n+1]);
            for (int i:c_s) {
                gamma[h][i] = reinterpret_cast<GRBVar *>(new GRBVar *[n+1]);
                for (int j:c_t) {
                    gamma[h][i][j] = model.addVar(0, 1, 0.0, GRB_BINARY, "gamma_" + std::to_string(h)+"_"+std::to_string(i)+"_"+std::to_string(j));
                    for (int heavy : instance->heavy) {
                        if (h == heavy) {
                            model.addConstr(gamma[h][i][j] == 0);
                        }
                    }
                    if (i == j) {
                        model.addConstr(gamma[h][i][j] == 0);
                    }
                }
            }
            model.addConstr(gamma[h][s][t] == 0);
        }

        std::vector<GRBVar> theta(n+1);
        for (int h:V){
            theta[h] = model.addVar(0, 1, 0.0, GRB_BINARY, "theta_" + std::to_string(h));
            for (auto heavy : instance->heavy) {
                if (h == heavy) {
                    model.addConstr(theta[h] == 0);
                }
            }
            if (h == s || h == t){
                model.addConstr(theta[h] == 0);
            }
        }
        auto** omega = new GRBVar * [n+1];
        for (int h:C) {
            omega[h] = reinterpret_cast<GRBVar *>(new GRBVar *[n+1]);
            for(int i:V) {
                omega[h][i] = model.addVar(0, 1, 0.0, GRB_BINARY, "omega_" + std::to_string(h) + "_" + std::to_string(i));

                for (int heavy : instance->heavy) {
                    if (h == heavy) {
                        model.addConstr(omega[h][i] == 0);
                    }
                }
                if (h == i || i == t) {
                    model.addConstr(omega[h][i] == 0);
                }
            }
        }
        auto** delta = new GRBVar * [n+1];
        for (int h:C) {
            delta[h] = reinterpret_cast<GRBVar *>(new GRBVar *[n+1]);
            for(int j:V) {
                delta[h][j] = model.addVar(0, 1, 0.0, GRB_BINARY, "delta_" + std::to_string(h) + "_" + std::to_string(j));
                for (int heavy : instance->heavy) {
                    if (h == heavy) {
                        model.addConstr(delta[h][j] == 0);
                    }
                }
                if (h == j || j == s) {
                    model.addConstr(delta[h][j] == 0);
                }
            }
        }

        std::vector<GRBVar> sigma(n+1);
        for (int h:c_t){
            sigma[h] = model.addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "sigma_" + std::to_string(h));

            for (int heavy : instance->heavy) {
                if (h == heavy) {
                    model.addConstr(sigma[h] == 0);
                }
            }
        }
        GRBLinExpr objective;
        for (int i:c_s){
            for(int j:c_t){
                objective += tau[i][j]*y[i][j];
            }
        }

        for(int h:C){
            objective += (sl+sr)*theta[h];
            objective -= sl*omega[h][s];
        }
        for(int h:c_t){
            objective += sigma[h];
        }
        GRBLinExpr sum_theta;
        // Constraint 1
        GRBLinExpr lhs_1, rhs_1;
        for (int j:c_t) {
            lhs_1 += y[s][j];
        }
        for (int i:c_s) {
            rhs_1 += y[i][t];
        }
        model.addConstr(lhs_1, GRB_EQUAL, 1, "C1_LHS");
        model.addConstr(rhs_1, GRB_EQUAL, 1, "C1_RHS");

        // Constraint 2
        for (int i:C) {
            GRBLinExpr lhs_2, rhs_2;
            for (int j:c_t) {
                lhs_2 += y[i][j];
            }
            for (int j:c_s) {
                rhs_2 += y[j][i];

            }
            model.addConstr(lhs_2, GRB_EQUAL, rhs_2, "C2_EQUAL_i="+ std::to_string(i));
            model.addConstr(lhs_2, GRB_LESS_EQUAL, 1, "C2_LHS_LEQ_1_i="+ std::to_string(i));
            model.addConstr(rhs_2, GRB_LESS_EQUAL, 1, "C2_RHS_LEQ_1_i="+ std::to_string(i));
        }

        // Constraint 3
        auto setAndComps = generateSetsAndComplements(C);
        for (auto &set:setAndComps){
            auto S = set.first;
            if (S.size() < 2) {
                continue;
            }
            if (S.size() == 2 && S[0] == s && S[1] == t) {
                continue;
            }
            GRBLinExpr sum1, sum2;
            std::string cname = "C3";
            for (auto i:S) {
                cname += "_" + std::to_string(i);
                if (i != t) {
                    for (auto j: S) {
                        if (j != s) {
                            sum1 += y[i][j];
                        }
                    }
                }
            }

            for (auto h:S) {
                GRBLinExpr sum3;
                for (auto k:S) {
                    if (h == k || h == s || h == t) {
                        continue;
                    } else {
                        sum3 += 1 - theta[k];
                    }
                }

                model.addConstr(sum1, GRB_LESS_EQUAL, sum3);
            }
//            if (containsSOrT(S, s, t)) {
//                std::cout << "Print set: ";
//                setPrint(S);
//                model.addConstr(sum1, GRB_LESS_EQUAL, sum2, cname);
//            } else {
//                model.addConstr(sum1, GRB_LESS_EQUAL, sum2, cname);
//
//            }
//            model.addConstr(sum1, GRB_LESS_EQUAL, sum2, cname);
        }
        // Constraint 4
        for (int h:C) {
            GRBLinExpr lhs_4;
            std::string cname = "C4_h=" + std::to_string(h);
            for (int j:c_t) {
                lhs_4 += gamma[h][s][j];
            }
            model.addConstr(lhs_4, GRB_EQUAL, omega[h][s], cname);
        }

        // Constraint 5
        for (int h:C) {
            GRBLinExpr lhs_5;
            std::string cname = "C5_h=" + std::to_string(h);

            for (int i:c_s) {
                lhs_5 += gamma[h][i][t];
            }
            model.addConstr(lhs_5, GRB_EQUAL, delta[h][t], cname);
        }
        // Constraint 6
        for (int i:C) {
            for (int h:C) {
                std::string cname = "C6_i=" + std::to_string(i) + "_h=" + std::to_string(h);
                GRBLinExpr sum1, sum2;
                for (int j:c_t){
                    sum1 += gamma[h][i][j];
                }

                for (int j:c_s){
                    sum2 += gamma[h][j][i];
                }
                model.addConstr(sum1-sum2, GRB_EQUAL, omega[h][i] - delta[h][i], cname);
            }
        }
        // Constraint 7
        for (int j:c_t) {
            std::string cname = "C7_s_j=" + std::to_string(j);
            model.addConstr(y[s][j] + x[s][j], GRB_LESS_EQUAL, 1, cname);
        }
        // Constraint 8
        for (int i:c_s) {
            std::string cname = "C8_i=" + std::to_string(i) + "_t";
            model.addConstr(y[i][t] + x[i][t], GRB_LESS_EQUAL, 1, cname);
        }
        // Constraint 9
        for (int i:C) {
            for (int j:C) {
                std::string cname = "C9_i=" + std::to_string(i) + "_j=" + std::to_string(j);
                model.addConstr(y[i][j] + x[i][j] + x[j][i], GRB_LESS_EQUAL, 1, cname);
            }
        }

        // Constraint 10
        for (int h:C) {
            GRBLinExpr sum;
            std::string cname = "C10_h=" + std::to_string(h);
            for (int j:c_t) {
                sum += y[h][j];
            }
            model.addConstr(sum + theta[h], GRB_EQUAL, 1, cname);
        }
        //Constraint 11
        for (int i:c_s) {
            for (int j:c_t) {
                std::string cname = "C11_i=" + std::to_string(i) + "_j=" + std::to_string(j);
                GRBLinExpr sum;
                for (int h:c_prime) {
                    sum += gamma[h][i][j];
                }
                model.addConstr(sum, GRB_LESS_EQUAL, y[i][j], cname);
            }
        }


        // Constraint 12
        for (int h:C) {
            GRBLinExpr sum1, sum2;

            for (int i:V) {
                if (i != h && i != t) {
                    sum1 += omega[h][i];
                }
            }
            for (int j:V) {
                if (j != s && j != h) {
                    sum2 += delta[h][j];
                }
            }
            model.addConstr(sum1, GRB_EQUAL, theta[h], "C12_RHS_EQUAL_h=" + std::to_string(h));
            model.addConstr(sum2, GRB_EQUAL, theta[h], "C12_LHS_EQUAL_h=" + std::to_string(h));
            model.addConstr(sum1 == sum2, "C12_EQUAL_h=" + std::to_string(h));
        }
        // Constraint 13
        for (int i:c_s) {
            for (int j:c_t) {
                std::string cname = "C13_i=" + std::to_string(i) + "_j=" + std::to_string(j);
                model.addConstr(x[i][j],  GRB_LESS_EQUAL, theta[i] + theta[j], cname);
            }
        }
//        // Constraint 14
        for (int i:c_s) {
            for (int j:c_t) {
                if (i != s && j != t) {
                    std::string cname = "C14_i=" + std::to_string(i) + "_j=" + std::to_string(j);
                    model.addConstr(x[i][j], GRB_LESS_EQUAL, omega[j][i] + delta[i][j], cname);
                }
            }
        }
        // Constraint 15
        for (int i:c_s) {
            GRBLinExpr sum1, sum2;
            std::string cname = "C15_i=" + std::to_string(i);
            for (int j:c_t){
                sum1 += x[i][j];
            }
            for (int h:c_prime) {
                sum2 += omega[h][i];
            }
            sum2 += theta[i];
            model.addConstr(sum1, GRB_EQUAL, sum2, "C15_LHS_RHS_EQUAL_i=" + std::to_string(i));
            model.addConstr(sum1, GRB_LESS_EQUAL, 1, "C15_LHS_LEQ_1_i="+ std::to_string(i));
            model.addConstr(sum2, GRB_LESS_EQUAL, 1, "C15_RHS_LEQ_1_i="+ std::to_string(i));
        }
        // Constraint 16
        for (int j:c_t) {
            GRBLinExpr sum1, sum2;
            for (int i:c_s){
                sum1 += x[i][j];
            }

            for (int h:c_prime) {
                sum2 += delta[h][j];
            }
            sum2 += theta[j];
            model.addConstr(sum1, GRB_EQUAL, sum2, "C16_LHS_RHS_EQUAL_j=" + std::to_string(j));
            model.addConstr(sum1, GRB_LESS_EQUAL, 1, "C16_LHS_LEQ_1_i="+ std::to_string(j));
            model.addConstr(sum2, GRB_LESS_EQUAL, 1, "C16_RHS_LEQ_1_i="+ std::to_string(j));
        }
        // Constraint 17
        for (int h:c_prime) {
            GRBLinExpr sum;
            std::string cname = "C17_h=" + std::to_string(h);
            for (int i:c_s) {
                for (int j:c_t){
                    sum += tau[i][j] * gamma[h][i][j];;
                }
            }
            model.addConstr(sum, GRB_LESS_EQUAL, (dtl - sr)*theta[h], cname);
        }
        // Constraint 18
        for(int h:c_prime) {
            GRBLinExpr sum1;
            GRBLinExpr sum2;
            GRBLinExpr sum3;
            std::string c18_name = "C18_h=" + std::to_string(h);
            std::string c19_name = "C19_h=" + std::to_string(h);

            for (int i:c_s){
                sum1 += d[i][h] * omega[h][i];
            }
            for (int j:c_t){
                sum2 += d[h][j] * delta[h][j];
            }

            for (int i:c_s) {
                for (int j:c_t){
                    sum3 += tau[i][j] * gamma[h][i][j];
                }
            }
            model.addConstr(sum1 + sum2, GRB_LESS_EQUAL, (dtl - sr)*theta[h], c18_name);
            model.addConstr(sum1 + sum2 - sum3, GRB_LESS_EQUAL, sigma[h], c19_name);
        }

    model.setObjective(objective, GRB_MINIMIZE);
        model.set(GRB_IntParam_Threads, n_thread);
    model.update();
    model.write("model.lp");
    model.optimize();
    std::cout << "Truck arcs: " << std::endl;
    for (int i:c_s) {
        for (int j:c_t) {
            if (y[i][j].get(GRB_DoubleAttr_X) == 1) {
                std::cout << i << " " << j << " " << tau[i][j] << std::endl;
            }
        }
    }
    int theta_cnt = 0;
    std::cout << "Theta:" << std::endl;
    for(int h:C) {
        if (theta[h].get(GRB_DoubleAttr_X) == 1) {
            theta_cnt++;
            std::cout << "Theta_" << h << " = " << theta[h].get(GRB_DoubleAttr_X) << std::endl;
        }
    }
    std::cout << "Drone arcs:" << std::endl;
    for (int i:c_s) {
        for (int j:c_t) {
            if (i != j && x[i][j].get(GRB_DoubleAttr_X) == 1) {
                std::cout << i << " " << j << std::endl;
            }
        }
    }
    std::cout << "Gamma:" << std::endl;
    for (int h:C) {
        for(int i:c_s) {
            for (int j:c_t) {
                if (gamma[h][i][j].get(GRB_DoubleAttr_X) == 1) {
                    std::cout << "gamma_" << h << "_" << i << "_" << j << "= 1" << std::endl;
                }
            }
        }
    }
    std::cout << "Sigma:" << std::endl;
    for (int h:c_t){
        std::cout << "Sigma_" << h << " = " << sigma[h].get(GRB_DoubleAttr_X) << std::endl;
    }
    std::cout << "Objective: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
    std::cout << "Number of sorties: " << theta_cnt << std::endl;
//    } catch (GRBException e) {
//        std::cout << "Error code: " << e.getErrorCode() << std::endl;
//        std::cout << e.getMessage() << std::endl;
//    }



}

