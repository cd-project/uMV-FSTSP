//
// Created by cuong on 18/01/2024.
//
#include "../include/solver.h"
#include <iostream>
#include <vector>
#include <algorithm>
Sortie::Sortie(int t, int la, int re, std::vector<int>& middle) {
    target = t;
    l = la;
    r = re;
    phi = middle;
}
Result::Result(double c, std::vector<Sortie> &st) {
    cost = c;
    sortie = st;
}
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
Result Solver::OriginalSolver(int n_thread, int e) {
//    try {
        auto tau = instance->tau;
        auto d = instance->tau_prime;
        auto dtl = e;
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
    auto** phi = new GRBVar *[n+1];
    for (int h:C) {
        phi[h] = reinterpret_cast<GRBVar *>(new GRBVar* [n+1]);
        for (int i:c_s) {
            phi[h][i] = model.addVar(0, 1, 0.0, GRB_BINARY, "phi_" + std::to_string(h) + "_" + std::to_string(i));
            for (int heavy:instance->heavy) {
                if (heavy == h) {
                    model.addConstr(phi[h][i] == 0);
                }
            }
            if (h == i) {
                model.addConstr(phi[h][i] == 0);
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
//    auto setAndCompss = generateSetsAndComplements(C);
//    for (auto &pair:setAndCompss) {
//        auto S = pair.first;
//        auto S_comp = pair.second;
//        S_comp.push_back(s);
//        S_comp.push_back(t);
//        GRBLinExpr lhs, rhs;
//        std::string cname = "C3";
//        for (int i:S) {
//            cname += "_" + std::to_string(i);
//            for (int j:S_comp) {
//                if (j != s && i != t) {
//                    lhs += y[i][j];
//                }
//            }
//        }
//        for (int h:S) {
//            rhs += theta[h];
//        }
//        cname += "_(";
//        for (int j:S_comp) {
//            cname += "_" + std::to_string(j);
//        }
//        cname += ")";
//        model.addConstr(S.size() * lhs >= S.size() - rhs, cname);
//    }
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
        // Constraint 11
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
//    auto s_c_V = generateSetsAndComplements(V);
//    for (auto &pair:s_c_V) {
//        auto S = pair.first;
//        if (S.size() < 2) {
//            continue;
//        } else {
//            for (int h: C) {
//                GRBLinExpr s1, s2;
//                for (int i:S) {
//                    if (i != t) {
//                        s2 += omega[h][i];
//                    }
//                    if (i != s) {
//                        s2 += delta[h][i];
//                    }
//                    if (i != s && i != t) {
//                        s2 += phi[h][i];
//                    }
//                    for (int j: S) {
//                        if (i != j && i != t && j != s) {
//                            s1 += gamma[h][i][j];
//                        }
//                    }
//                }
////                model.addConstr(s1 == 1);
////                model.addConstr(s2 == 1);
//                model.addConstr(s1 <= s2); // render the model infeasible. what can be the fix?
//            }
//        }
//    }
//    // phi constraint
//    for (int h:C) {
//        for (int i:c_s) {
//            GRBLinExpr sum;
//            for (int j:c_t) {
//                sum += gamma[h][i][j];
//            }
//            model.addConstr(sum == phi[h][i] + omega[h][i]);
//            model.addConstr(phi[h][i] + omega[h][i] <= 1);
//            model.addConstr(sum <= 1);
//        }
//    }

    model.setObjective(objective, GRB_MINIMIZE);
    model.set(GRB_IntParam_Threads, n_thread);
    model.update();
    model.write("model.lp");
    model.optimize();
    std::vector<Sortie> st;
    for (int h:C) {
        if (theta[h].get(GRB_DoubleAttr_X) == 1) {
//            auto trip = Sortie(h, _, _, _);
//            st.push_back(trip);
        }
    }
    return Result{model.get(GRB_DoubleAttr_ObjVal), st};
}

Result Solver::uMVFSTSPSolver(int n_thread, int e) {
    try {
        auto tau = instance->tau;
        auto d = instance->tau_prime;
        auto dtl = e;
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
            for (int j:V) {
                y[i][j] = model.addVar(0, 1, 0.0, GRB_INTEGER, "y_" + std::to_string(i) + "_" + std::to_string(j));
                if (i == j) {
                    model.addConstr(y[i][j] == 0);
                }
            }
        }
        model.addConstr(y[s][t] == 0);


        auto** x = new GRBVar * [n+1];
        for (int i:c_s) {
            x[i] = reinterpret_cast<GRBVar *>(new GRBVar *[n+1]);
            for (int j:V) {
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
                for (int j:V) {
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
        auto** phi = new GRBVar *[n+1];
        for (int h:C) {
            phi[h] = reinterpret_cast<GRBVar *>(new GRBVar* [n+1]);
            for (int i:c_s) {
                phi[h][i] = model.addVar(0, 1, 0.0, GRB_BINARY, "phi_" + std::to_string(h) + "_" + std::to_string(i));
                for (int heavy:instance->heavy) {
                    if (heavy == h) {
                        model.addConstr(phi[h][i] == 0);
                    }
                }
                if (h == i) {
                    model.addConstr(phi[h][i] == 0);
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
        model.addConstr(lhs_1, GRB_GREATER_EQUAL, 1, "C1_LHS");
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
//        model.addConstr(lhs_2, GRB_LESS_EQUAL, 1, "C2_LHS_LEQ_1_i="+ std::to_string(i));
//        model.addConstr(rhs_2, GRB_LESS_EQUAL, 1, "C2_RHS_LEQ_1_i="+ std::to_string(i));
        }
        auto setAndCompss = generateSetsAndComplements(C);
        for (auto &pair:setAndCompss) {
            auto S = pair.first;
            auto S_comp = pair.second;
            S_comp.push_back(s);
            S_comp.push_back(t);
            GRBLinExpr lhs, rhs;
            std::string cname = "C3";
            for (int i:S) {
                cname += "_" + std::to_string(i);
                for (int j:S_comp) {
                    if (j != s && i != t) {
                        lhs += y[i][j];
                    }
                }
            }
            for (int h:S) {
                rhs += theta[h];
            }
            cname += "_(";
            for (int j:S_comp) {
                cname += "_" + std::to_string(j);
            }
            cname += ")";
            model.addConstr(S.size() * lhs >= S.size() - rhs, cname);
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
            std::string cname = "C3";
            GRBLinExpr sum3;
            for (auto h:S) {

                if (h == s || h == t) {
                    continue;
                } else {
                    sum3 += 1 - theta[h];
                }

            }
            for (int h2:C) {
                GRBLinExpr sum_gamma;
                for (auto i:S) {
                    cname += "_" + std::to_string(i);
                    if (i != t) {
                        for (auto j: S) {
                            sum_gamma += gamma[h2][i][j];
                        }
                    }
//                    model.addConstr(sum_gamma <= sum3);
                }
            }
        }
//         Constraint 4
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
        for (int i:c_s) {
            for (int h:C) {
                std::string cname = "C6_i=" + std::to_string(i) + "_h=" + std::to_string(h);
                GRBLinExpr sum1, sum2;
                for (int j:V){
                    sum1 += gamma[h][i][j];
                }

                for (int j:c_s){
                    sum2 += gamma[h][j][i];
                }
                model.addConstr(sum1-sum2, GRB_EQUAL, omega[h][i] - delta[h][i], cname);
            }
        }
        // Constraint 7
//    for (int j:c_t) {
//        std::string cname = "C7_s_j=" + std::to_string(j);
//        model.addConstr(y[s][j] + x[s][j], GRB_LESS_EQUAL, 1, cname);
//    }
        // Constraint 8
        for (int i:c_s) {
            std::string cname = "C8_i=" + std::to_string(i) + "_t";
            model.addConstr(y[i][t] + x[i][t], GRB_LESS_EQUAL, 1, cname);
        }
        // Constraint 9
        for (int i:C) {
            for (int j:C) {
                std::string cname = "C9_i=" + std::to_string(i) + "_j=" + std::to_string(j);
                model.addConstr(x[i][j] + x[j][i], GRB_LESS_EQUAL, 1, cname);
            }
        }

        // Constraint 10
        for (int h:C) {
            GRBLinExpr sum;
            std::string cname = "C10_h=" + std::to_string(h);
            for (int j:c_t) {
                sum += y[h][j];
            }
            model.addConstr(sum + theta[h], GRB_GREATER_EQUAL, 1, cname);
        }
        // Constraint 11
        for (int i:c_s) {
            for (int j:V) {
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
            for (int j:V) {
                std::string cname = "C13_i=" + std::to_string(i) + "_j=" + std::to_string(j);
                model.addConstr(x[i][j],  GRB_LESS_EQUAL, theta[i] + theta[j], cname);
            }
        }
//        // Constraint 14
        for (int i:c_s) {
            for (int j:V) {
                GRBLinExpr sum;
                if (j != s && i != t && j != t) {
                    sum += omega[j][i];
                }
                if (i != t && i != s) {
                    sum += delta[i][j];
                }

                std::string cname = "C14_i=" + std::to_string(i) + "_j=" + std::to_string(j);
                model.addConstr(x[i][j] <= sum, cname);
            }
        }
        // Constraint 15
        for (int i:c_s) {
            GRBLinExpr sum1, sum2;
            std::string cname = "C15_i=" + std::to_string(i);
            for (int j:V){
                sum1 += x[i][j];
            }
            for (int h:c_prime) {
                sum2 += omega[h][i];
            }
            sum2 += theta[i];
            model.addConstr(sum1, GRB_EQUAL, sum2, "C15_LHS_RHS_EQUAL_i=" + std::to_string(i));
//        model.addConstr(sum1, GRB_LESS_EQUAL, 1, "C15_LHS_LEQ_1_i="+ std::to_string(i));
//        model.addConstr(sum2, GRB_LESS_EQUAL, 1, "C15_RHS_LEQ_1_i="+ std::to_string(i));
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
//        model.addConstr(sum1, GRB_LESS_EQUAL, 1, "C16_LHS_LEQ_1_i="+ std::to_string(j));
//        model.addConstr(sum2, GRB_LESS_EQUAL, 1, "C16_RHS_LEQ_1_i="+ std::to_string(j));
        }
        // Constraint 17
        for (int h:c_prime) {
            GRBLinExpr sum;
            std::string cname = "C17_h=" + std::to_string(h);
            for (int i:c_s) {
                for (int j:V){
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
            for (int j:V){
                sum2 += d[h][j] * delta[h][j];
            }

            for (int i:c_s) {
                for (int j:V){
                    sum3 += tau[i][j] * gamma[h][i][j];
                }
            }
            model.addConstr(sum1 + sum2, GRB_LESS_EQUAL, (dtl - sr)*theta[h], c18_name);
            model.addConstr(sum1 + sum2 - sum3, GRB_LESS_EQUAL, sigma[h], c19_name);
        }
        // sum phi constraint
        for (int h:C) {
            for (int i:c_s) {
                for (int j:c_s) {
                    model.addConstr(gamma[h][i][j] + gamma[h][j][i] <= 1);
                }
            }
        }
        for (int h:C) {
            GRBLinExpr lhs, rhs;
            for (int i:c_s) {
                for (int j:V) {
                    lhs += gamma[h][i][j];
                }
            }
            for (int i:V) {
                if (i != t) {
                    rhs += omega[h][i];
                }
                if (i != s && i != t) {
                    rhs += phi[h][i];
                }
                rhs += delta[h][i];
            }
            model.addConstr(lhs <= rhs-1);
        }
        auto s_c_V = generateSetsAndComplements(V);
        for (auto &pair:s_c_V) {
            auto S = pair.first;
            if (S.size() < 2) {
                continue;
            } else {
                for (int h: C) {
                    GRBLinExpr s1, s2;
                    for (int i:S) {
                        if (i != t) {
                            s2 += omega[h][i];
                        }
                        if (i != s) {
                            s2 += delta[h][i];
                        }
                        if (i != s && i != t) {
                            s2 += phi[h][i];
                        }
                        for (int j:S) {
                            if (i != j && i != t && j != s) {
                                s1 += gamma[h][i][j];
                            }
                        }
                    }
//                model.addConstr(s1 <= s2); // render the model infeasible. what can be the fix?
                }
            }
        }
        // phi constraint
        for (int h:C) {
            for (int i:c_s) {
                GRBLinExpr sum;
                for (int j:c_t) {
                    sum += gamma[h][i][j];
                }
                model.addConstr(sum == phi[h][i] + omega[h][i]);
                model.addConstr(phi[h][i] + omega[h][i] <= 1);
                model.addConstr(sum <= 1);
            }
        }
        for (int h:C) {
            GRBLinExpr sum_gamma, sum_sortie;
            for (int i:c_s) {
                for (int j:V) {
                    sum_gamma += gamma[h][i][j];
                }
            }
            for (int i:V) {
                if (i != t) {
                    sum_sortie += omega[h][i];
                }
                if (i != s) {
                    sum_sortie += delta[h][i];
                }
                if (i != s && i != t) {
                    sum_sortie += phi[h][i];
                }
            }
//        model.addConstr(sum_gamma <= sum_sortie - 1);
        }
        model.setObjective(objective, GRB_MINIMIZE);
//    model.set(GRB_IntParam_Threads, n_thread);
        model.update();
        model.write("model.lp");
        model.optimize();
        std::cout << "Truck arcs: " << std::endl;
        for (int i:c_s) {
            for (int j:c_t) {
                if (y[i][j].get(GRB_DoubleAttr_X) >= 1) {
                    std::cout << i << " " << j << " " << y[i][j].get(GRB_DoubleAttr_X) << std::endl;
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
        for (int h:C) {
            if (theta[h].get(GRB_DoubleAttr_X) == 1) {
                std::cout << "middle of " << h << " sortie are: ";
                for (int i:c_s) {
                    if (phi[h][i].get(GRB_DoubleAttr_X) == 1) {
                        std::cout << i << " ";
                    }
                }
                std::cout << std::endl;
            }
        }
        std::cout << "Objective: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
        std::cout << "Number of sorties: " << theta_cnt << std::endl;
        std::vector<Sortie> st;
        for (int h:C) {
            if (theta[h].get(GRB_DoubleAttr_X) == 1) {
                int start, end;
                for (int i:c_s) {
                    if (omega[h][i].get(GRB_DoubleAttr_X) == 1) {
                        start = i;
                    }
                }
                for (int j:V) {
                    if (delta[h][j].get(GRB_DoubleAttr_X) == 1) {
                        end = j;
                    }
                }
                std::vector<int> middle;
                for (int i:c_s) {
                    if (phi[h][i].get(GRB_DoubleAttr_X) == 1) {
                        middle.push_back(i);
                    }
                }
                auto trip = Sortie(h, start, end, middle);
                st.push_back(trip);
            }
        }
        for (auto& sortie:st) {
            std::cout << "---------------------------" << std::endl;
            std::cout << "Sortie of " << sortie.target << " {" << std::endl;
            std::cout << "l/r: (" << sortie.l << ", " << sortie.r  << ")" << std::endl;
            std::cout << "Truck routes: ";
            for (int i:c_s) {
                for (int j:V) {
                    if (gamma[sortie.target][i][j].get(GRB_DoubleAttr_X) == 1) {
                        std::cout << i << "->" << j << ", ";
                    }
                }
            }
            std::cout << std::endl;
        }
        return Result{model.get(GRB_DoubleAttr_ObjVal), st};
    } catch (GRBException &e) {
        std::cout << e.getMessage() << std::endl;
    }

}

Result Solver::mvdSolver(int n_thread, int e) {
    auto tau = instance->tau;
    auto tau_prime = instance->tau_prime;
    auto dtl = e;
    auto sl = 1, sr = 1;
    auto n = instance->num_node;

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
    auto K = C, k_prime = V;
    auto O = 0;
    auto D = n;
    auto max_stage = k_prime.size();

    // X^i_k: truck at node i\in{V} at stage k.
    // max(k) = n+1, as in there is no sortie.
    auto** X = new GRBVar* [n+1];
    for (int i = 0; i < n; i++) {
        X[i] = reinterpret_cast<GRBVar*> (new GRBVar * [n+1]);
        for (int k = 1; k <= max_stage; k++) {
            X[i][k] = model.addVar(0, 1, 0.0, GRB_BINARY, "X_" + std::to_string(i) + "_" + std::to_string(k));
        }
    }


    // x^k_ij: truck from node i at stage k moves to node j.
    auto*** x = new GRBVar ** [n+1];
    for (int k = 1; k <= max_stage; k++){
        x[k] = reinterpret_cast<GRBVar **>(new GRBVar **[n+1]);
        for (int i = 0; i <= D; i++) {
            x[k][i] = reinterpret_cast<GRBVar *>(new GRBVar *[n+1]);
            for (int j = 0; j <= D; j++) {
                if (i != j) {
                    x[k][i][j] = model.addVar(0, 1, 0.0, GRB_BINARY, "x_" + std::to_string(k)+"_"+std::to_string(i)+"_"+std::to_string(j));
                }
            }
        }
    }


    // phi^h equals to 1 if customer h is served by the drone
    std::vector<GRBVar> phi(n);
    for (int h:C) {
        phi[h] = model.addVar(0, 1, 0.0, GRB_BINARY, "phi_" + std::to_string(h));
    }

    auto***** Z = new GRBVar ****[n+1];
    for (int k = 1; k <= max_stage; k++) {
        Z[k] = reinterpret_cast<GRBVar ****>(new GRBVar **** [n+1]);
        for (int k_p = 1; k_p <= max_stage; k_p++) {
            if (k < k_p) {
                Z[k][k_p] = reinterpret_cast<GRBVar ***>(new GRBVar *** [n+1]);
                for (int i = O; i <= D; i++) {
                    Z[k][k_p][i] = reinterpret_cast<GRBVar **>(new GRBVar **[n + 1]);
                    for (int j = O; j <= D; j++) {
                        if (i != j) {
                            Z[k][k_p][i][j] = reinterpret_cast<GRBVar *>(new GRBVar *[n + 1]);
                            for (int h:C) {
                                if (h != i && h != j) {
                                    Z[k][k_p][i][j][h] = model.addVar(0, 1, 0.0, GRB_BINARY,
                                                                      "Z_" + std::to_string(k) + "_" + std::to_string(k_p) +
                                                                      "_" + std::to_string(i) + "_" + std::to_string(j) + "_" +
                                                                      std::to_string(h));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    auto** z = new GRBVar *[n+1];
    for (int k = 1; k <= max_stage; k++) {
        z[k] = reinterpret_cast<GRBVar *>(new GRBVar *[n+1]);
        for (int k_p = 1; k_p <= max_stage; k_p++) {
            if (k < k_p) {
                z[k][k_p] = model.addVar(0, 1, 0.0, GRB_BINARY, "z_" + std::to_string(k) + "_" +std::to_string(k_p));
            }
        }
    }

    // arrival\departure variables a and d.
    std::vector<GRBVar> a(n+1);
    std::vector<GRBVar> d(n+1);
    for (int k = 1; k <= max_stage; k++) {
        a[k] = model.addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,"t_" + std::to_string(k));
        d[k] = model.addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,"t_" + std::to_string(k));
        model.addConstr(d[k] >= a[k], "C13_" + std::to_string(k));
    }


    for (int k:V) {
        for (int i:V) {
            GRBLinExpr sum;
            for (int j:V) {
                sum += x[k][i][j];
            }
        }
    }

    //////////// C2
    GRBLinExpr C2;
    for (int i = 1; i <= D; i++) {
        C2 += x[1][O][i];
    }
    model.addConstr(C2 == 1, "C2");

    ///////////// C3
    GRBLinExpr C3;
    for (int k = 1; k <= max_stage; k++) {
        for (int i = 0; i < D; i++) {
            C3 += x[k][i][D];
        }
    }
    model.addConstr(C3 == 1, "C3");

    ///////////// C4
    for (int k = 1; k <= max_stage-1; k++) {
        for (int i = 0; i < D; i++) {
            GRBLinExpr lhs, rhs;
            for (int j = 0; j <= D; j++) {
                if (i != j) {
                    lhs += x[k][i][j];
                }
            }
            for (int j = 0; j < D; j++) {
                if (i != j) {
                    rhs += x[k+1][j][i];
                }
            }
            model.addConstr(lhs == rhs, "C4_" + std::to_string(k) + "_" + std::to_string(i));
        }
    }

    ////////////// C6
    for (int k = 1; k <= max_stage; k++) {
        for (int k_p = 1; k_p <= max_stage; k_p++) {
            if (k < k_p) {
                GRBLinExpr rhs;
                for (int i = 0; i < D; i++) { // start can't include D.
                    for (int j = 0; j <= D; j++) { // end can include D.
                        if (i != j) {
                            for (int h:C) {
                                if (h != i && h != j) {
                                    rhs += Z[k][k_p][i][j][h];
                                }
                            }
                        }
                    }
                }
                model.addConstr(z[k][k_p] == rhs, "C6_" + std::to_string(k) + "_to_" + std::to_string(k_p));
            }
        }
    }

    ////////// C7
    for (int k = 1; k <= max_stage; k++) {
        for (int k_p = 1; k_p <= max_stage; k_p++) {
            if (k < k_p) {
                for (int l = 1; l <= max_stage; l++) {
                    for (int l_p = 1; l_p <= max_stage; l_p++) {
                        if (k <= l && l < k_p) {
                            model.addConstr(z[k][k_p] + z[l][l_p] <= 1, "C7_" + std::to_string(k) + "_" + std::to_string(k_p) + "_" + std::to_string(l) + "_" + std::to_string(l_p));
                        }
                    }
                }
            }
        }
    }

    /////////// C8
    for (int i = 0; i < D; i++) {
        for (int k = 1; k <= max_stage; k++) {
            for (int k_p = 1; k_p <= max_stage; k_p++) {
                if (k < k_p) {
                    for (int j = 0; j <= D; j++) {
                        if (i != j) {
                            for (int h:C) {
                                if (i != h && h != j) {
                                    model.addConstr(Z[k][k_p][i][j][h] <= X[i][k], "C8_launch_NEED_REVIEW_COMBINE" + std::to_string(i) + "_" + std::to_string(k));
                                    model.addConstr(Z[k][k_p][i][j][h] <= X[j][k_p], "C8_rendezvous_NEED_REVIEW_COMBINE" + std::to_string(j)+ "_" + std::to_string(k_p));
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    //////// C9
    for (int h:C) {
        GRBLinExpr rhs;
        for (int i = 0; i < D; i++) {
            for (int j = 0; j <= D; j++) {
                if (i != j && i != h && j != h) {
                    for (int k = 1; k <= max_stage; k++) {
                        for (int k_p = 1; k_p <= max_stage; k_p++) {
                            if (k < k_p) {
                                rhs += Z[k][k_p][i][j][h];
                            }
                        }
                    }
                }

            }
        }
        model.addConstr(phi[h] == rhs, "C9_" + std::to_string(h));
    }

    //////////// C10
    for (int h:C) {
        GRBLinExpr sum_k;
        for (int k = 1; k <= max_stage; k++) {
            sum_k += X[h][k];
        }
        model.addConstr(phi[h] + sum_k >= 1, "C10_" + std::to_string(h));
    }

    //////////////// C11
    for (int h:C) {
        GRBLinExpr sum;
        for (int k = 1; k <= max_stage; k++) {
            for (int k_p = 1; k_p <= max_stage; k_p++) {
                if (k < k_p) {
                    for (int i = 0; i < D; i++) {
                        for (int j = 0; j <= D; j++) {
                            if (i != j &&i != h && j != h) {
                                sum += Z[k][k_p][i][j][h] * (tau_prime[i][h] + tau_prime[h][j]);
                            }
                        }
                    }
                }
            }
        }
        model.addConstr(sum <= dtl, "C11_" + std::to_string(h));
    }

    /////////// C14
    for (int k = 1; k <= max_stage; k++) {
        GRBLinExpr sum;
        for (int i = 0; i < D; i++) {
            for (int j = 0; j <= D; j++) {
                if (i != j) {
                    sum += x[k][i][j] * tau[i][j];
                }
            }
        }
        model.addConstr(a[k+1] >= d[k] + sum, "C14_" + std::to_string(k) + "_" + std::to_string(k+1));
    }

    ////////// C15
    auto M = 100000;
    for (int k = 1; k <= max_stage; k++) {
        for (int k_p = 1; k_p <= max_stage; k_p++) {
            if (k < k_p) {
                model.addConstr(a[k_p] - d[k] <= z[k][k_p] * dtl + (1-z[k][k_p]) * M, "C15_" + std::to_string(k) + "_" + std::to_string(k_p));
                GRBLinExpr rhs;
                for (int i = 0; i < D; i++) {
                    for (int j = 0; j <= D; j++) {
                        if (i != j) {
                            for (int h:C) {
                                if (h != i && h != j) {
                                    rhs += Z[k][k_p][i][j][h] * (tau_prime[i][h] + tau_prime[h][j]);
                                }
                            }
                        }
                    }
                }
                model.addConstr(d[k_p]- d[k] >= rhs, "C16_" + std::to_string(k) + "_" + std::to_string(k_p));
            }
        }
    }

    GRBVar max = model.addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "objective");
//    model.addConstr(max > 1);
    GRBLinExpr objective;
    for (int k = 1; k <= max_stage; k++) {
        model.addConstr(max >= a[k], "C_objective>=arrival_of_" + std::to_string(k));
    }
    objective += max;
    model.setObjective(objective, GRB_MINIMIZE);
    model.update();
    model.write("model.lp");
    model.optimize();
//    return Result(0, std::vector());
}

Result Solver::mvdSolverCPLEX(int n_thread, int e) {
    auto tau = instance->tau;
    auto tau_p = instance->tau_prime;
    auto dtl = e;
    auto sl = 1, sr = 1;
    auto n = instance->num_node;
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

    auto K = C, k_prime = V;
    auto O = 0;
    auto D = n;
    auto max_stage = n+1;
    IloEnv env;
    IloModel model(env);

    // X^i_k
    // (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
    // đường đi của vehicle là i; k \in {1..n}
    // đỉnh thứ k của truck tour là node i.
    // k max = n+1: toàn bộ đi boi oto.
    IloArray<IloBoolVarArray> X(env, n+1);
    for (int i = 0; i < D; i++) {
        X[i] = IloBoolVarArray(env, n+1);
        for (int k = 1; k <= max_stage; k++) {
            X[i][k] = IloBoolVar(env);
        }
    }

    IloArray<IloArray<IloBoolVarArray>> x(env, n+1);
    for (int k = 1; k <= max_stage; k++) {
        x[k] = IloArray<IloBoolVarArray>(env, n+1);
        for (int i:V) {
            x[k][i] = IloBoolVarArray(env, n+1);
            for (int j:V) {
                if (j != i) {
                    x[k][i][j] = IloBoolVar(env);
                }
            }
        }
    }
    IloNumVarArray phi(env, n);
    for (int h = 1; h < n; h++) {
        phi[h] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
    }
    IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>> Z(env, n+1);
    for (int k = 1; k <= max_stage; k++) {
        Z[k] = IloArray<IloArray<IloArray<IloBoolVarArray>>>(env, n+1);
        for (int k_p:V) {
            if (k < k_p) {
                Z[k][k_p] = IloArray<IloArray<IloBoolVarArray>>(env, n+1);
                for (int i:V) {
                    Z[k][k_p][i] = IloArray<IloBoolVarArray>(env, n+1);
                    for (int j:V) {
                        if (i != j) {
                            Z[k][k_p][i][j] = IloBoolVarArray(env, n+1);
                            for (int h:C) {
                                Z[k][k_p][i][j][h] = IloBoolVar(env);
                            }
                        }
                    }
                }
            }
        }
    }
    IloArray<IloBoolVarArray> z(env, n+1);
    for (int k = 1; k <= max_stage; k++) {
        z[k] = IloBoolVarArray(env, n+1);
        for (int k_p:V) {
            if (k < k_p) {
                z[k][k_p] = IloBoolVar(env);
            }
        }
    }
    IloNumVarArray a(env, n+1);
    IloNumVarArray d(env, n+1);
    IloExpr objective(env);
    for (int k = 1; k <= max_stage; k++) {
        a[k] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
        d[k] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
        model.add(d[k] >= a[k]);
        if (k == 0) {
            model.add(a[k] == 0);
            model.add(d[k] == 0);
        }
        model.add(objective >= a[k]);
    }
    model.add(IloMinimize(env, objective));
    IloCplex cplex(model);
    cplex.solve();
}




