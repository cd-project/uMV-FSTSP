//
// Created by cuong on 18/01/2024.
//
#include "../include/solver.h"
#include <iostream>
#include <vector>
Sortie::Sortie(int t, int la, int re, std::vector<int>& middle) {
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
Result::Result(double c, std::vector<Sortie>& st) {
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
            }
            else {
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
void setPrint(std::vector<int>& set) {
    for (int i : set) {
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
    for (int i = 0; i < n + 1; i++) {
        if (i == 0 || i == n) {
            V.push_back(i);
        }
        else {
            V.push_back(i);
            C.push_back(i);
        }
    }
    std::vector<int> c_s;
    std::vector<int> c_t;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0) {
            c_s.push_back(i);
        }
        else if (i == n) {
            c_t.push_back(i);
        }
        else {
            c_s.push_back(i);
            c_t.push_back(i);
        }
    }

    std::cout << std::endl;
    GRBEnv env;
    GRBModel model(env);
    // y: (i, j) in A, truck route
    auto** y = new GRBVar * [n + 1];
    for (int i : c_s) {
        y[i] = reinterpret_cast<GRBVar*>(new GRBVar * [n + 1]);
        for (int j : c_t) {
            y[i][j] = model.addVar(0, 1, 0.0, GRB_BINARY, "y_" + std::to_string(i) + "_" + std::to_string(j));
            if (i == j) {
                model.addConstr(y[i][j] == 0);
            }
        }
    }
    model.addConstr(y[s][t] == 0);


    auto** x = new GRBVar * [n + 1];
    for (int i : c_s) {
        x[i] = reinterpret_cast<GRBVar*>(new GRBVar * [n + 1]);
        for (int j : c_t) {
            x[i][j] = model.addVar(0, 1, 0.0, GRB_BINARY, "x_" + std::to_string(i) + "_" + std::to_string(j));
            if (i == j) {
                model.addConstr(x[i][j] == 0);
            }

        }
    }

    model.addConstr(x[s][t] == 0);

    // gamma_h_ij
    auto*** gamma = new GRBVar * *[n + 1];
    for (int h : C) {
        gamma[h] = reinterpret_cast<GRBVar**>(new GRBVar * *[n + 1]);
        for (int i : c_s) {
            gamma[h][i] = reinterpret_cast<GRBVar*>(new GRBVar * [n + 1]);
            for (int j : c_t) {
                gamma[h][i][j] = model.addVar(0, 1, 0.0, GRB_BINARY, "gamma_" + std::to_string(h) + "_" + std::to_string(i) + "_" + std::to_string(j));
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

    std::vector<GRBVar> theta(n + 1);
    for (int h : V) {
        theta[h] = model.addVar(0, 1, 0.0, GRB_BINARY, "theta_" + std::to_string(h));
        for (auto heavy : instance->heavy) {
            if (h == heavy) {
                model.addConstr(theta[h] == 0);
            }
        }
        if (h == s || h == t) {
            model.addConstr(theta[h] == 0);
        }
    }
    auto** omega = new GRBVar * [n + 1];
    for (int h : C) {
        omega[h] = reinterpret_cast<GRBVar*>(new GRBVar * [n + 1]);
        for (int i : V) {
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
    auto** delta = new GRBVar * [n + 1];
    for (int h : C) {
        delta[h] = reinterpret_cast<GRBVar*>(new GRBVar * [n + 1]);
        for (int j : V) {
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

    std::vector<GRBVar> sigma(n + 1);
    for (int h : c_t) {
        sigma[h] = model.addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "sigma_" + std::to_string(h));

        for (int heavy : instance->heavy) {
            if (h == heavy) {
                model.addConstr(sigma[h] == 0);
            }
        }
    }
    auto** phi = new GRBVar * [n + 1];
    for (int h : C) {
        phi[h] = reinterpret_cast<GRBVar*>(new GRBVar * [n + 1]);
        for (int i : c_s) {
            phi[h][i] = model.addVar(0, 1, 0.0, GRB_BINARY, "phi_" + std::to_string(h) + "_" + std::to_string(i));
            for (int heavy : instance->heavy) {
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
    for (int i : c_s) {
        for (int j : c_t) {
            objective += tau[i][j] * y[i][j];
        }
    }

    for (int h : C) {
        objective += (sl + sr) * theta[h];
        objective -= sl * omega[h][s];
    }
    for (int h : c_t) {
        objective += sigma[h];
    }
    GRBLinExpr sum_theta;
    // Constraint 1
    GRBLinExpr lhs_1, rhs_1;
    for (int j : c_t) {
        lhs_1 += y[s][j];
    }
    for (int i : c_s) {
        rhs_1 += y[i][t];
    }
    model.addConstr(lhs_1, GRB_EQUAL, 1, "C1_LHS");
    model.addConstr(rhs_1, GRB_EQUAL, 1, "C1_RHS");

    // Constraint 2
    for (int i : C) {
        GRBLinExpr lhs_2, rhs_2;
        for (int j : c_t) {
            lhs_2 += y[i][j];
        }
        for (int j : c_s) {
            rhs_2 += y[j][i];

        }
        model.addConstr(lhs_2, GRB_EQUAL, rhs_2, "C2_EQUAL_i=" + std::to_string(i));
        model.addConstr(lhs_2, GRB_LESS_EQUAL, 1, "C2_LHS_LEQ_1_i=" + std::to_string(i));
        model.addConstr(rhs_2, GRB_LESS_EQUAL, 1, "C2_RHS_LEQ_1_i=" + std::to_string(i));
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
    for (auto& set : setAndComps) {
        auto S = set.first;
        if (S.size() < 2) {
            continue;
        }
        if (S.size() == 2 && S[0] == s && S[1] == t) {
            continue;
        }
        GRBLinExpr sum1, sum2;
        std::string cname = "C3";
        for (auto i : S) {
            cname += "_" + std::to_string(i);
            if (i != t) {
                for (auto j : S) {
                    if (j != s) {
                        sum1 += y[i][j];
                    }
                }
            }
        }

        for (auto h : S) {
            GRBLinExpr sum3;
            for (auto k : S) {
                if (h == k || h == s || h == t) {
                    continue;
                }
                else {
                    sum3 += 1 - theta[k];
                }
            }

            model.addConstr(sum1, GRB_LESS_EQUAL, sum3);
        }
    }
    // Constraint 4
    for (int h : C) {
        GRBLinExpr lhs_4;
        std::string cname = "C4_h=" + std::to_string(h);
        for (int j : c_t) {
            lhs_4 += gamma[h][s][j];
        }
        model.addConstr(lhs_4, GRB_EQUAL, omega[h][s], cname);
    }

    // Constraint 5
    for (int h : C) {
        GRBLinExpr lhs_5;
        std::string cname = "C5_h=" + std::to_string(h);

        for (int i : c_s) {
            lhs_5 += gamma[h][i][t];
        }
        model.addConstr(lhs_5, GRB_EQUAL, delta[h][t], cname);
    }
    // Constraint 6
    for (int i : C) {
        for (int h : C) {
            std::string cname = "C6_i=" + std::to_string(i) + "_h=" + std::to_string(h);
            GRBLinExpr sum1, sum2;
            for (int j : c_t) {
                sum1 += gamma[h][i][j];
            }

            for (int j : c_s) {
                sum2 += gamma[h][j][i];
            }
            model.addConstr(sum1 - sum2, GRB_EQUAL, omega[h][i] - delta[h][i], cname);
        }
    }
    // Constraint 7
    for (int j : c_t) {
        std::string cname = "C7_s_j=" + std::to_string(j);
        model.addConstr(y[s][j] + x[s][j], GRB_LESS_EQUAL, 1, cname);
    }
    // Constraint 8
    for (int i : c_s) {
        std::string cname = "C8_i=" + std::to_string(i) + "_t";
        model.addConstr(y[i][t] + x[i][t], GRB_LESS_EQUAL, 1, cname);
    }
    // Constraint 9
    for (int i : C) {
        for (int j : C) {
            std::string cname = "C9_i=" + std::to_string(i) + "_j=" + std::to_string(j);
            model.addConstr(y[i][j] + x[i][j] + x[j][i], GRB_LESS_EQUAL, 1, cname);
        }
    }

    // Constraint 10
    for (int h : C) {
        GRBLinExpr sum;
        std::string cname = "C10_h=" + std::to_string(h);
        for (int j : c_t) {
            sum += y[h][j];
        }
        model.addConstr(sum + theta[h], GRB_EQUAL, 1, cname);
    }
    // Constraint 11
    for (int i : c_s) {
        for (int j : c_t) {
            std::string cname = "C11_i=" + std::to_string(i) + "_j=" + std::to_string(j);
            GRBLinExpr sum;
            for (int h : c_prime) {
                sum += gamma[h][i][j];
            }
            model.addConstr(sum, GRB_LESS_EQUAL, y[i][j], cname);
        }
    }


    // Constraint 12
    for (int h : C) {
        GRBLinExpr sum1, sum2;

        for (int i : V) {
            if (i != h && i != t) {
                sum1 += omega[h][i];
            }
        }
        for (int j : V) {
            if (j != s && j != h) {
                sum2 += delta[h][j];
            }
        }
        model.addConstr(sum1, GRB_EQUAL, theta[h], "C12_RHS_EQUAL_h=" + std::to_string(h));
        model.addConstr(sum2, GRB_EQUAL, theta[h], "C12_LHS_EQUAL_h=" + std::to_string(h));
        model.addConstr(sum1 == sum2, "C12_EQUAL_h=" + std::to_string(h));
    }
    // Constraint 13
    for (int i : c_s) {
        for (int j : c_t) {
            std::string cname = "C13_i=" + std::to_string(i) + "_j=" + std::to_string(j);
            model.addConstr(x[i][j], GRB_LESS_EQUAL, theta[i] + theta[j], cname);
        }
    }
    //        // Constraint 14
    for (int i : c_s) {
        for (int j : c_t) {
            if (i != s && j != t) {
                std::string cname = "C14_i=" + std::to_string(i) + "_j=" + std::to_string(j);
                model.addConstr(x[i][j], GRB_LESS_EQUAL, omega[j][i] + delta[i][j], cname);
            }
        }
    }
    // Constraint 15
    for (int i : c_s) {
        GRBLinExpr sum1, sum2;
        std::string cname = "C15_i=" + std::to_string(i);
        for (int j : c_t) {
            sum1 += x[i][j];
        }
        for (int h : c_prime) {
            sum2 += omega[h][i];
        }
        sum2 += theta[i];
        model.addConstr(sum1, GRB_EQUAL, sum2, "C15_LHS_RHS_EQUAL_i=" + std::to_string(i));
        model.addConstr(sum1, GRB_LESS_EQUAL, 1, "C15_LHS_LEQ_1_i=" + std::to_string(i));
        model.addConstr(sum2, GRB_LESS_EQUAL, 1, "C15_RHS_LEQ_1_i=" + std::to_string(i));
    }
    // Constraint 16
    for (int j : c_t) {
        GRBLinExpr sum1, sum2;
        for (int i : c_s) {
            sum1 += x[i][j];
        }

        for (int h : c_prime) {
            sum2 += delta[h][j];
        }
        sum2 += theta[j];
        model.addConstr(sum1, GRB_EQUAL, sum2, "C16_LHS_RHS_EQUAL_j=" + std::to_string(j));
        model.addConstr(sum1, GRB_LESS_EQUAL, 1, "C16_LHS_LEQ_1_i=" + std::to_string(j));
        model.addConstr(sum2, GRB_LESS_EQUAL, 1, "C16_RHS_LEQ_1_i=" + std::to_string(j));
    }
    // Constraint 17
    for (int h : c_prime) {
        GRBLinExpr sum;
        std::string cname = "C17_h=" + std::to_string(h);
        for (int i : c_s) {
            for (int j : c_t) {
                sum += tau[i][j] * gamma[h][i][j];;
            }
        }
        model.addConstr(sum, GRB_LESS_EQUAL, (dtl - sr) * theta[h], cname);
    }
    // Constraint 18
    for (int h : c_prime) {
        GRBLinExpr sum1;
        GRBLinExpr sum2;
        GRBLinExpr sum3;
        std::string c18_name = "C18_h=" + std::to_string(h);
        std::string c19_name = "C19_h=" + std::to_string(h);

        for (int i : c_s) {
            sum1 += d[i][h] * omega[h][i];
        }
        for (int j : c_t) {
            sum2 += d[h][j] * delta[h][j];
        }

        for (int i : c_s) {
            for (int j : c_t) {
                sum3 += tau[i][j] * gamma[h][i][j];
            }
        }
        model.addConstr(sum1 + sum2, GRB_LESS_EQUAL, (dtl - sr) * theta[h], c18_name);
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
    for (int h : C) {
        if (theta[h].get(GRB_DoubleAttr_X) == 1) {
            //            auto trip = Sortie(h, _, _, _);
            //            st.push_back(trip);
        }
    }
    return Result{ model.get(GRB_DoubleAttr_ObjVal), st };} catch (GRBException &exception) {
            std::cout << exception.getMessage() << std::endl;
        }
}

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
    for (int i : c_prime) {
        c_prime_0.push_back(i);
    }
    c_prime_0.push_back(n);

    std::cout << "Printing number of nodes: " << n << std::endl;
    std::vector<int> C;
    std::vector<int> V;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0 || i == n) {
            V.push_back(i);
        }
        else {
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
        }
        else if (i == n) {
            c_t.push_back(i);
        }
        else {
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
    IloArray<IloArray<IloBoolVarArray>> x(env, arc_max_stage + 1);
    for (int k = 1; k <= arc_max_stage; k++) {
        x[k] = IloArray<IloBoolVarArray>(env, D);
        for (int i = 0; i < D; i++) {
            x[k][i] = IloBoolVarArray(env, D + 1);

            for (int j = 1; j <= D; j++)
                if (i != j)
                {
                    x[k][i][j] = IloBoolVar(env);
                    auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
                    x[k][i][j].setName(v_name.c_str());
                }
        }
    }


    //// phi^h equals to 1 if customer h is served by the drone
    IloBoolVarArray phi(env, n);
    for (int h : C) {
        phi[h] = IloBoolVar(env);
        auto v_name = "phi_" + std::to_string(h);
        phi[h].setName(v_name.c_str());
    }
    for (auto heavy:instance->heavy) {
        model.add(phi[heavy] == 0);
    }

    IloArray<IloArray<IloArray<IloArray<IloBoolVarArray>>>> Z(env, node_max_stage);
    for (int k = 1; k <= node_max_stage - 1; k++) {
        Z[k] = IloArray<IloArray<IloArray<IloBoolVarArray>>>(env, node_max_stage + 1);
        for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
            if (k < k_p) {
                Z[k][k_p] = IloArray<IloArray<IloBoolVarArray>>(env, D);
                for (int i = 0; i < D; i++) {
                    Z[k][k_p][i] = IloArray<IloBoolVarArray>(env, D + 1);
                    for (int j = 1; j <= D; j++) {
                        if (i != j) {
                            Z[k][k_p][i][j] = IloBoolVarArray(env, D);
                            for (int h : C) {
                                if (h != i && h != j)
                                    if (tau_prime[i][h] + tau_prime[h][j] <= dtl)
                                    {
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
            { // i == D => khong co i -> j.
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
            { // cho chac luon.
                IloExpr rhs(env);
                for (int i = 0; i < D; i++) { // start can't include D.
                    for (int j = 1; j <= D; j++) { // end can include D.
                        if (i != j) {
                            for (int h : C) {
                                if (h != i && h != j)
                                    if (tau_prime[i][h] + tau_prime[h][j] <= dtl)
                                    {
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
                        model.add(z[k][k_p] + z[l][l_p] <= 1).setName(("C7_" + std::to_string(k) + "_" + std::to_string(k_p) + "_" + std::to_string(l) + "_" + std::to_string(l_p)).c_str());
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
        for (int k = 1; k <= node_max_stage - 1; k++)
        {
            IloExpr lhs(env);

            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                {
                    for (int j = 1; j <= D; j++) {
                        if (i != j) {
                            for (int h : C) {
                                if (i != h && h != j)
                                    if (tau_prime[i][h] + tau_prime[h][j] <= dtl)
                                    {
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
        for (int k_p = 2; k_p <= node_max_stage; k_p++)
        {
            IloExpr lhs(env);

            for (int k = 1; k < k_p; k++) {
                {
                    for (int i = 0; i < D; i++) {
                        if (i != j) {
                            for (int h : C) {
                                if (i != h && h != j)
                                    if (tau_prime[i][h] + tau_prime[h][j] <= dtl)
                                    {
                                        lhs += Z[k][k_p][i][j][h];
                                    }
                            }
                        }
                    }
                }
            }
            model.add(lhs <= X[k_p][j]).setName(("C8_rendezvous_" + std::to_string(j) + "_" + std::to_string(k_p)).c_str());

        }
    }

    ////////// C9: node_stage
    for (int h : C) {
        IloExpr rhs(env);
        for (int i = 0; i < D; i++) {
            for (int j = 1; j <= D; j++) {
                if (i != j && i != h && j != h)
                    if (tau_prime[i][h] + tau_prime[h][j] <= dtl)
                    {
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
    for (int h : C) {
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
                model.add(a[k_p] - d[k] <= z[k][k_p] * dtl + (1 - z[k][k_p]) * M).setName(("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());

                IloExpr rhs(env);
                for (int i = 0; i < D; i++) {
                    for (int j = 1; j <= D; j++) {
                        if (i != j) {
                            for (int h : C) {
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
                model.add(d[k_p] - d[k] >= rhs).setName(("C16_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
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
        }
        else {
            // Handle other solver errors
            std::cerr << "Solver error: " << cplex.getStatus() << std::endl;
        }
    }
    else {
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

        for (int h : C) {
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
    for (int i : c_prime) {
        c_prime_0.push_back(i);
    }
    c_prime_0.push_back(n);

    std::cout << "Printing number of nodes: " << n << std::endl;
    std::vector<int> C;
    std::vector<int> V;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0 || i == n) {
            V.push_back(i);
        }
        else {
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
        }
        else if (i == n) {
            c_t.push_back(i);
        }
        else {
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
    IloArray<IloArray<IloBoolVarArray>> x(env, arc_max_stage + 1);
    for (int k = 1; k <= arc_max_stage; k++) {
        x[k] = IloArray<IloBoolVarArray>(env, D);
        for (int i = 0; i < D; i++) {
            x[k][i] = IloBoolVarArray(env, D + 1);

            for (int j = 1; j <= D; j++)
                if (i != j)
                {
                    x[k][i][j] = IloBoolVar(env);
                    auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
                    x[k][i][j].setName(v_name.c_str());
                }
        }
    }


    //// phi^h equals to 1 if customer h is served by the drone
    IloBoolVarArray phi(env, n);
    for (int h : C) {
        phi[h] = IloBoolVar(env);
        auto v_name = "phi_" + std::to_string(h);
        phi[h].setName(v_name.c_str());
    }

    for (int heavy:instance->heavy) {
        model.add(phi[heavy] == 0);
    }
    IloArray<IloArray<IloBoolVarArray>> Y(env, node_max_stage + 1), W(env, node_max_stage + 1);
    for (int k = 1; k <= node_max_stage; k++) {

        Y[k] = IloArray<IloBoolVarArray>(env, D + 1);
        W[k] = IloArray<IloBoolVarArray>(env, D + 1);

        for (int i = 0; i <= D; i++) {
            {
                Y[k][i] = IloBoolVarArray(env, C.size() + 1);
                W[k][i] = IloBoolVarArray(env, C.size() + 1);
                for (int h : C)
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
            for (int h : C)
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
            for (int h : C)
                if (h != i)
                    if (tau_prime[h][i] <= dtl) {
                        expr += W[k][i][h];
                    }
            model.add(expr <= X[k][i]).setName(("C17p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
        }
    }

    // $\sum_{i} X^k_{ih}\tau^D_{ih} + \sum_{i} Y^{k'}_{ih}\tau^D_{hi} \leq D_d$
    //- drone duration constraint cho mỗi $h$ (C19)
    for (int h : C) {
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
    IloArray<IloArray<IloBoolVarArray>> Z(env, node_max_stage);
    for (int h : C) {
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
            for (int h : C) {
                expr += Z[h][k][k_p];
            }
            model.add(expr == z[k][k_p]).setName(("C22_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
        }


    //C20:$\sum_{k'>k}Z_{kk'} = \sum_{i,h}Y^k_{ih}$ : với mỗi $k$,
    //ràng buộc liên kết drone đi ra từ stage $k$ và đoạn mà oto di chuyển không có drone. (C20)

    for (int h : C) {
        for (int k = 1; k <= node_max_stage - 1; k++) {

            IloExpr expr(env);
            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                {
                    expr += Z[h][k][k_p];
                }
            }

            for (int i = 0; i < D; i++)
            {
                if (i != h && tau_prime[i][h] <= dtl) {
                    expr -= Y[k][i][h];
                }
            }
            model.add(expr == 0).setName(("C20_" + std::to_string(k) + "_" + std::to_string(h)).c_str());

        }

    }

    for (int h : C) {
        for (int k_p = 2; k_p <= node_max_stage; k_p++) {

            IloExpr expr(env);
            for (int k = 1; k < k_p; k++) {
                {
                    expr += Z[h][k][k_p];
                }
            }

            for (int i = 1; i <= D; i++)
            {
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
            { // i == D => khong co i -> j.
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

    for (int k = 1; k < node_max_stage; k++)
    {
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
                                                                  + "_" + std::to_string(k_p) + "_" + std::to_string(l)).c_str());
                    }
                    else {
                        //model.add(R[k] <= 1).setName(("C7k_" + std::to_string(k)
                        //    + "_" + std::to_string(k_p) + "_" + std::to_string(l)).c_str());
                    }

                }
            }
        }
    }



    for (int i = 0; i < D; i++) {
        for (int k = 1; k <= node_max_stage - 1; k++)
        {
            IloExpr lhs(env);

            for (int h : C) {
                if (i != h)
                {
                    lhs += Y[k][i][h];
                }


            }
            model.add(lhs <= X[k][i]).setName(("C8_launch_" + std::to_string(i) + "_" + std::to_string(k)).c_str());

        }
    }

    for (int j = 1; j <= D; j++) {
        for (int k_p = 2; k_p <= node_max_stage; k_p++)
        {
            IloExpr lhs(env);

            for (int h : C) {
                if (h != j)
                {
                    lhs += W[k_p][j][h];
                }
            }

            model.add(lhs <= X[k_p][j]).setName(("C8_rendezvous_" + std::to_string(j) + "_" + std::to_string(k_p)).c_str());
        }
    }


    // $\phi_h = \sum_{k,i}X^k_{ih} = \sum_{k,i}Y^k_{ih}$
    // - chỉ có duy nhất một điểm xuất phát và môt điểm đích cho
    //mỗi khách hàng $h$ được phục vụ bởi drone (C18)
    for (int h : C) {
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


    for (int h : C) {
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
    for (int h : C) {
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
                model.add(a[k_p] - d[k] <= z[k][k_p] * dtl + (1 - z[k][k_p]) * M).setName(("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());

                IloExpr rhs(env);
                for (int i = 0; i <= D; i++) {
                        for (int h : C) {
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
                model.add(d[k_p] - d[k] + (1 - z[k][k_p]) * 2 * dtl >= rhs).setName(("C21_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
            }
        }
    }
    for (int k = 1; k <= node_max_stage-1; k++) {
        for (int k_p = k+1; k_p <= node_max_stage; k_p++) {
            IloExpr sum_Y(env);
            IloExpr sum_W(env);
            for (int i = 0; i <= D; i++) {
                for (int h:C) {
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
    for (int k = 1; k <= node_max_stage-1; k++) {
        for (int k_p = k+1; k_p <= node_max_stage; k_p++) {
            IloExpr sum(env);
            for (int k_p_p = k; k_p_p <= k_p-1; k_p_p++) {
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

    // Solve the model
    if (!cplex.solve()) {
        // Check if the problem is infeasible
        if (cplex.getStatus() == IloAlgorithm::Infeasible) {
            // Handle infeasibility
            std::cout << "The problem is infeasible." << std::endl;
            std::cout << "Infeasibility at: " << cplex.getInfeasibility(c2) << std::endl;
            // You can also retrieve the infeasible constraints using cplex.getInfeasibility() method
        }
        else {
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
                //std::cout << "k = " << k << ", i = " << i << ":" << X_val << std::endl;
                if (abs(X_val - 1) < 1e-6) {
                    auto d_k = cplex.getValue(d[k]);
                    std::cout << "Stage " << k << " at customer " << i << " with departure time is: " << d_k << std::endl;
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
            for(int h:C)
                for (int k = 1; k < node_max_stage; k++)
                    for (int kp = k + 1; kp <= node_max_stage; kp++) {
                        //std::cout << k << " " << kp << std::endl;
                        if (abs(cplex.getValue(Z[h][k][kp]) - 1) < 1e-6) {
                            std::cout << "Drone flies from stage " << k << " to stage " << kp << " to serve customer " << h << std::endl;
                        }
                    }

            for (int h : C) {
                //std::cout << "AAAAAAAAAAA Customer " << h << " served by drone." << std::endl;

                if (abs(cplex.getValue(phi[h]) - 1) < 1e-6) {
                    std::cout << "Customer " << h << " served by drone." << std::endl;
                    int sv_i = -1, sv_j = -1, sv_k = -1, sv_kp = -1;
                    for (int k = 1; k <= node_max_stage; k++) {
                        for (int i = 0; i <= D; i++)
                            if (i != h)
                            {
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


                                }
                                catch (...) {

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
                              << tau_prime[sv_i][h] << ", end arc cost: " << tau_prime[h][sv_j] << ". Total drone travel time: " << travel_time << std::endl;

//                    std::cout << "Drone/Vehicle time: " << drone_arrival_time << " " << vehicle_departure_time << std::endl;
                    std::cout << "Drone arrival time: " << drone_arrival_time << std::endl;
                    std::cout << "Truck arrival time: " << truck_arrival_time << std::endl;
                    std::cout << "Truck departure time = max(d/a, t/a): " << vehicle_departure_time << std::endl;
                    assert(drone_arrival_time <= vehicle_departure_time);
                    assert(abs(cplex.getValue(Z[h][sv_k][sv_kp]) - 1.0) < 1e-6);

                    assert(abs(cplex.getValue(z[sv_k][sv_kp]) - 1.0) < 1e-6);

                }
            }
            std::cout << cplex.getValue(Z[1][2][4]);
            std::cout << "Done!" << std::endl;

            exit(0);
    }

    return Result();
}

Result Solver::mvdSolverWithLR(int n_thread, int e) {
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
    for (int i : c_prime) {
        c_prime_0.push_back(i);
    }
    c_prime_0.push_back(n);

    std::cout << "Printing number of nodes: " << n << std::endl;
    std::vector<int> C;
    std::vector<int> V;
    for (int i = 0; i < n + 1; i++) {
        if (i == 0 || i == n) {
            V.push_back(i);
        }
        else {
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
        }
        else if (i == n) {
            c_t.push_back(i);
        }
        else {
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
    IloArray<IloArray<IloBoolVarArray>> x(env, arc_max_stage + 1);
    for (int k = 1; k <= arc_max_stage; k++) {
        x[k] = IloArray<IloBoolVarArray>(env, D);
        for (int i = 0; i < D; i++) {
            x[k][i] = IloBoolVarArray(env, D + 1);

            for (int j = 1; j <= D; j++)
                if (i != j)
                {
                    x[k][i][j] = IloBoolVar(env);
                    auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
                    x[k][i][j].setName(v_name.c_str());
                }
        }
    }


    //// phi^h equals to 1 if customer h is served by the drone
    IloBoolVarArray phi(env, n);
    for (int h : C) {
        phi[h] = IloBoolVar(env);
        auto v_name = "phi_" + std::to_string(h);
        phi[h].setName(v_name.c_str());
    }

    for (int heavy:instance->heavy) {
        model.add(phi[heavy] == 0);
    }
    IloArray<IloArray<IloBoolVarArray>> Y(env, node_max_stage + 1), W(env, node_max_stage + 1);
    for (int k = 1; k <= node_max_stage; k++) {

        Y[k] = IloArray<IloBoolVarArray>(env, D + 1);
        W[k] = IloArray<IloBoolVarArray>(env, D + 1);

        for (int i = 0; i <= D; i++) {
            {
                Y[k][i] = IloBoolVarArray(env, C.size() + 1);
                W[k][i] = IloBoolVarArray(env, C.size() + 1);
                for (int h : C)
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
            for (int h : C)
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
            for (int h : C)
                if (h != i)
                    if (tau_prime[h][i] <= dtl) {
                        expr += W[k][i][h];
                    }
            model.add(expr <= X[k][i]).setName(("C17p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
        }
    }

    // $\sum_{i} X^k_{ih}\tau^D_{ih} + \sum_{i} Y^{k'}_{ih}\tau^D_{hi} \leq D_d$
    //- drone duration constraint cho mỗi $h$ (C19)
    for (int h : C) {
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
    IloArray<IloArray<IloBoolVarArray>> Z(env, node_max_stage);
    for (int h : C) {
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
            for (int h : C) {
                expr += Z[h][k][k_p];
            }
            model.add(expr == z[k][k_p]).setName(("C22_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
        }


    //C20:$\sum_{k'>k}Z_{kk'} = \sum_{i,h}Y^k_{ih}$ : với mỗi $k$,
    //ràng buộc liên kết drone đi ra từ stage $k$ và đoạn mà oto di chuyển không có drone. (C20)

    for (int h : C) {
        for (int k = 1; k <= node_max_stage - 1; k++) {

            IloExpr expr(env);
            for (int k_p = k + 1; k_p <= node_max_stage; k_p++) {
                {
                    expr += Z[h][k][k_p];
                }
            }

            for (int i = 0; i < D; i++)
            {
                if (i != h && tau_prime[i][h] <= dtl) {
                    expr -= Y[k][i][h];
                }
            }
            model.add(expr == 0).setName(("C20_" + std::to_string(k) + "_" + std::to_string(h)).c_str());

        }

    }

    for (int h : C) {
        for (int k_p = 2; k_p <= node_max_stage; k_p++) {

            IloExpr expr(env);
            for (int k = 1; k < k_p; k++) {
                {
                    expr += Z[h][k][k_p];
                }
            }

            for (int i = 1; i <= D; i++)
            {
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
            { // i == D => khong co i -> j.
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

    for (int k = 1; k < node_max_stage; k++)
    {
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
                                                                  + "_" + std::to_string(k_p) + "_" + std::to_string(l)).c_str());
                    }
                    else {
                        //model.add(R[k] <= 1).setName(("C7k_" + std::to_string(k)
                        //    + "_" + std::to_string(k_p) + "_" + std::to_string(l)).c_str());
                    }

                }
            }
        }
    }



    for (int i = 0; i < D; i++) {
        for (int k = 1; k <= node_max_stage - 1; k++)
        {
            IloExpr lhs(env);

            for (int h : C) {
                if (i != h)
                {
                    lhs += Y[k][i][h];
                }


            }
            model.add(lhs <= X[k][i]).setName(("C8_launch_" + std::to_string(i) + "_" + std::to_string(k)).c_str());

        }
    }

    for (int j = 1; j <= D; j++) {
        for (int k_p = 2; k_p <= node_max_stage; k_p++)
        {
            IloExpr lhs(env);

            for (int h : C) {
                if (h != j)
                {
                    lhs += W[k_p][j][h];
                }
            }

            model.add(lhs <= X[k_p][j]).setName(("C8_rendezvous_" + std::to_string(j) + "_" + std::to_string(k_p)).c_str());
        }
    }


    // $\phi_h = \sum_{k,i}X^k_{ih} = \sum_{k,i}Y^k_{ih}$
    // - chỉ có duy nhất một điểm xuất phát và môt điểm đích cho
    //mỗi khách hàng $h$ được phục vụ bởi drone (C18)
    for (int h : C) {
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


    for (int h : C) {
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
    for (int h : C) {
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
                model.add(a[k_p] - d[k] <= z[k][k_p] * dtl + (1 - z[k][k_p]) * M).setName(("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());

                IloExpr rhs(env);
                for (int i = 0; i <= D; i++) {
                    for (int h : C) {
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
                model.add(d[k_p] - d[k] - sr + (1 - z[k][k_p]) * 2 * dtl >= rhs).setName(("C21_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
            }
        }
    }

    for (int k = 2; k <= node_max_stage; k++) {
        IloExpr sum(env);
        IloExpr sum_w_K(env);
        for (int i = 0; i < D; i++) {
            for (int j = 1; j <= D; j++) {
                if (i != j) {
                   sum += x[k-1][i][j] * tau[i][j];
                }
            }
            for (int h:C) {
                if (i != h && i != O) {
                    sum_w_K += W[k][i][h] * sl;
                }
            }
        }

        model.add(d[k] >= d[k-1] + sum + sum_w_K);
    }

    for (int k = 1; k <= node_max_stage-1; k++) {
        for (int k_p = k+1; k_p <= node_max_stage; k_p++) {
            IloExpr sum_start(env);
            IloExpr sum_back(env);
            for (int i = 0; i < D; i++) {
                for (int h:C) {
                    if (i != h) {
                        sum_start += W[k][i][h] * tau_prime[i][h];
                    }
                }
            }
            for (int j = 1; j <= D; j++) {
                for (int h:C) {
                    if (h != j) {
                        sum_back += Y[k_p][j][h] * tau_prime[h][j];
                    }
                }
            }
            model.add(d[k_p] >= d[k] + sum_start + sum_back + sr);
        }
    }
//    IloExpr sum_sl_at_O(env);
//    IloExpr sum_sl_sr(env);
//    for (int k = 1; k <= node_max_stage-1; k++) {
//        for (int h:C) {
//            sum_sl_at_O += Y[k][O][h];
//        }
//    }
//    for (int h:C) {
//        sum_sl_sr += phi[h] * (sl+sr);
//    }

    // - sum_sl_at_O * sl + sum_sl_sr)
    for (int k = 1; k <= node_max_stage-1; k++) {
        for (int k_p = k+1; k_p <= node_max_stage; k_p++) {
            IloExpr sum(env);
            for (int k_p_p = k; k_p_p <= k_p-1; k_p_p++) {
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
    std::vector<Sortie> st;

    // Solve the model
    if (!cplex.solve()) {
        // Check if the problem is infeasible
        if (cplex.getStatus() == IloAlgorithm::Infeasible) {
            // Handle infeasibility
            std::cout << "The problem is infeasible." << std::endl;
            std::cout << "Infeasibility at: " << cplex.getInfeasibility(c2) << std::endl;
            // You can also retrieve the infeasible constraints using cplex.getInfeasibility() method
        }
        else {
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
                //std::cout << "k = " << k << ", i = " << i << ":" << X_val << std::endl;
                if (abs(X_val - 1) < 1e-6) {
                    auto d_k = cplex.getValue(d[k]);
                    std::cout << "Stage " << k << " at customer " << i << " with departure time is: " << d_k << std::endl;
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
                            obj += tau[i][j];
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
        for(int h:C)
            for (int k = 1; k < node_max_stage; k++)
                for (int kp = k + 1; kp <= node_max_stage; kp++) {
                    //std::cout << k << " " << kp << std::endl;
                    if (abs(cplex.getValue(Z[h][k][kp]) - 1) < 1e-6) {
                        std::cout << "Drone flies from stage " << k << " to stage " << kp << " to serve customer " << h << std::endl;
                    }
                }

        for (int h : C) {
            //std::cout << "AAAAAAAAAAA Customer " << h << " served by drone." << std::endl;

            if (abs(cplex.getValue(phi[h]) - 1) < 1e-6) {
                std::cout << "Customer " << h << " served by drone." << std::endl;
                auto sortie = Sortie(h);
                st.push_back(sortie);
                int sv_i = -1, sv_j = -1, sv_k = -1, sv_kp = -1;
                for (int k = 1; k <= node_max_stage; k++) {
                    for (int i = 0; i <= D; i++)
                        if (i != h)
                        {
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


                            }
                            catch (...) {

                            }
                        }

                }
                ////assert(abs(cplex.getValue(z[sv_k][sv_kp]) - 1.0) < 1e-6);

                std::cout << "Drone fly from " << sv_i << " at stage " << sv_k <<
                          " to serve " << h << " and then fly back to " << sv_j
                          << " at stage " << sv_kp << ". " << std::endl;
                obj += (sl+sr);
                if (sv_i == O) {
                    obj -= sl;
                }
                double travel_time = tau_prime[sv_i][h] + tau_prime[h][sv_j];
                auto drone_arrival_time = cplex.getValue(d[sv_k]) + travel_time;
                auto vehicle_departure_time = cplex.getValue(d[sv_kp]);
                auto truck_arrival_time = cplex.getValue(a[sv_kp]);
                std::cout << "Start arc cost: "
                          << tau_prime[sv_i][h] << ", end arc cost: " << tau_prime[h][sv_j] << ". Total drone travel time: " << travel_time << std::endl;

//                    std::cout << "Drone/Vehicle time: " << drone_arrival_time << " " << vehicle_departure_time << std::endl;
                std::cout << "Drone arrival time: " << drone_arrival_time << std::endl;
                std::cout << "Truck arrival time: " << truck_arrival_time << std::endl;
                if (truck_arrival_time < drone_arrival_time) {
                    obj += drone_arrival_time - truck_arrival_time;
                }
                std::cout << "Truck departure time = max(d/a, t/a): " << vehicle_departure_time << std::endl;
                assert(drone_arrival_time <= vehicle_departure_time);
                assert(abs(cplex.getValue(Z[h][sv_k][sv_kp]) - 1.0) < 1e-6);

                assert(abs(cplex.getValue(z[sv_k][sv_kp]) - 1.0) < 1e-6);

            }
        }

        std::cout << "Done!" << std::endl;
        std::cout << "-------------------------Re-calculated objective-----------------------" << std::endl;
        std::cout << obj << std::endl;
        std::cout <<"------------------------------------------------------------------------" << std::endl;
//        exit(0);
    }
//    std::cout << tau[0][8] + tau[8][1] + tau[1][5] + tau[5][6] + tau[6][7];
    double c = cplex.getObjValue();
    env.end();
    std::cout << "OBJECTIVE VALUE: " << c << ", NUMBER OF SORTIES: " << st.size() << "." << std::endl;
    return Result{c, st};
}
