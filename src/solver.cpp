//
// Created by cuong on 18/01/2024.
//
#include "../include/solver.h"
#include "../include/solver.h"

#include <chrono>
#include <functional>
#include <iostream>
#include <vector>
#include <unordered_set>

#include "../../../../../usr/include/complex.h"

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

Result::Result(double c, double re_c, double t, double rc, bool use_gap) {
    cost = c;
    recalculated_cost = re_c;
    time_spent = t;
    revisit_count = rc;
    used_stage_gap = use_gap;
}

inline bool exist(const std::vector<int> &vec, int element) {
    // Use std::find to search for the element in the vector
    return std::find(vec.begin(), vec.end(), element) != vec.end();
}
bool areAllElementsDistinct(const std::vector<int>& vec) {
    std::unordered_set<int> seen;

    for (int num : vec) {
        if (seen.count(num) > 0) {
            // Duplicate found
            return false;
        }
        seen.insert(num);
    }

    // No duplicates found
    return true;
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

void genSetsHelper_SortieGap(const std::vector<int> &nodes, int k, std::vector<int> &currentSet,
                  std::vector<std::vector<int> > &result) {
    // Base case: if currentSet has k nodes, add it to result
    if (currentSet.size() == k) {
        result.push_back(currentSet);
        return;
    }

    // Recursive case: iterate through all nodes
    for (int i = 0; i < nodes.size(); ++i) {
        // Add the current node to the set
        currentSet.push_back(nodes[i]);

        // Recur with the updated set
        genSetsHelper_SortieGap(nodes, k, currentSet, result);

        // Backtrack: remove the last node from the set
        currentSet.pop_back();
    }
}

std::vector<std::vector<int> > genSetForSortieGap(const std::vector<int> &nodes, int k, int D) {
    std::vector<std::vector<int> > result;
    std::vector<int> currentSet;
    genSetsHelper_SortieGap(nodes, k, currentSet, result);
    std::vector<std::vector<int> > result2;
    for (auto vec: result) {
        // perform test
        bool check = true;
        for (int i = 0; i < vec.size(); i++) {
            if (i < vec.size() - 1 && vec[i] == D) {
                check = false;
            }
            if (i < vec.size() - 1 && vec[i] == vec[i + 1]) {
                check = false;
            }
        }
        if (check) {
            result2.push_back(vec);
        }
    }

    return result2;
}

inline double calculate_tour(std::vector<std::vector<double> > &tau, std::vector<int> &tour) {
    double l = 0;
    for (int i = 0; i < tour.size() - 1; i++) {
        l += tau[tour[i]][tour[i + 1]];
    }
    return l;
}

inline double smallest_tour_length(int stage_gap, std::vector<std::vector<double> > &tau, std::vector<int> &V) {
    int D = V.size() - 1;
    auto sets = genSetForSortieGap(V, stage_gap + 1, D);
    double smallest = std::numeric_limits<double>::max();
    for (auto set: sets) {
        auto l = calculate_tour(tau, set);
        if (l < smallest) {
            smallest = l;
        }
    }
    return smallest;
}

IloNumArray Solver::TSP_MTZ(std::vector<std::vector<double> > &tau) {
    int n = tau.size() - 1;

    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
    IloArray<IloBoolVarArray> x(env, n);
    for (int i = 0; i < n; i++) {
        x[i] = IloBoolVarArray(env, n);
        for (int j = 0; j < n; j++) {
            model.add(x[i][j]);
        }
    }
    IloNumVarArray u(env, n);
    for (int i = 0; i < n; i++) {
        u[i] = IloNumVar(env, 0, n - 1, ILOFLOAT);
        model.add(u[i]);
    }
    IloRangeArray cons1(env, n);
    for (int j = 0; j < n; ++j) {
        IloExpr expr(env);
        for (int i = 0; i < n; ++i) {
            if (i != j) expr += x[i][j];
        }
        cons1[j] = IloRange(env, 1, expr, 1);
        model.add(cons1[j]);
    }

    IloRangeArray cons2(env, n);
    for (int i = 0; i < n; ++i) {
        IloExpr expr(env);
        for (int j = 0; j < n; ++j) {
            if (i != j) expr += x[i][j];
        }
        cons2[i] = IloRange(env, 1, expr, 1);
        model.add(cons2[i]);
    }

    model.add(IloRange(env, 0, u[0], 0));
    IloArray<IloRangeArray> cons3(env, n);
    for (int i = 0; i < n; ++i) {
        cons3[i] = IloRangeArray(env, n);
        for (int j = 1; j < n; ++j) {
            if (i == j) continue;
            cons3[i][j] = IloRange(env, u[i] - u[j] + n * x[i][j], n - 1);
            model.add(cons3[i][j]);
        }
    }
    IloExpr obj(env);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            obj += tau[i][j] * x[i][j];
        }
    }
    model.add(IloMinimize(env, obj));
    cplex.solve();
    std::cout << std::endl;
    IloNumArray order(env);
    for (int i = 0; i < n; i++) {
        for (int vertex = 0; vertex < n; vertex++) {
            if (abs(cplex.getValue(u[vertex]) - i) < 1e-5) {
                order.add(vertex);
                continue;
            }
        }
    }
    order.add(n);
    std::cout << "TSP solution for this instance: ";
    for (int i = 0; i < order.getSize(); i++) {
        std::cout << order[i] << " ";
    }
    std::cout << std::endl;
    cplex.end();
    model.end();
    env.end();
    return order;
}

IloNumArray Solver::RevisitTSP(std::vector<std::vector<double> > &tau) {
    int n = tau.size() - 1;

    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
    int D = n;
    int K = n * 1.5;
    IloArray<IloBoolVarArray> X(env, K + 1);
    for (int k = 1; k <= K; k++) {
        X[k] = IloBoolVarArray(env, D + 1);
        for (int i = 0; i <= D; i++) {
            X[k][i] = IloBoolVar(env);
            model.add(X[k][i]);
            auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
            //std::cout << v_name << std::endl;
            X[k][i].setName(v_name.c_str());
            if (k > 1 && i == 0) {
                model.add(X[k][0] == 0);
            }
        }
    }

    model.add(X[1][0] == 1).setName("start depot is the first node");
    model.add(X[1][D] == 0).setName("ending depot cannot be the first node");


    for (int k = 1; k <= K; k++) {
        IloExpr sum(env);
        for (int i = 0; i <= D; i++)
            sum += X[k][i];
        model.add(sum <= 1).setName(("C20_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
    }

    IloExpr arrival_depot(env);
    for (int k = 1; k <= K; k++) {
        arrival_depot += X[k][D];
    }
    model.add(arrival_depot == 1).setName("C21_arrival_depot_once");
    IloExpr obj(env);
    // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
    // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
    IloArray<IloArray<IloBoolVarArray> > x(env, K);
    for (int k = 1; k < K; k++) {
        x[k] = IloArray<IloBoolVarArray>(env, D);
        for (int i = 0; i < D; i++) {
            x[k][i] = IloBoolVarArray(env, D + 1);
            for (int j = 1; j <= D; j++) {
                if (i != j) {
                    x[k][i][j] = IloBoolVar(env);
                    auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
                    x[k][i][j].setName(v_name.c_str());
                    obj += x[k][i][j] * tau[i][j];
                    model.add(x[k][0][D] == 0);
                }
            }
        }
    }

    model.add(IloMinimize(env, obj));
    ////////// Constraint C1
    for (int k = 1; k < K; k++) {
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

    for (int k = 2; k <= K; k++) {
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
        C2 += x[1][0][i];
    }

    IloConstraint c2(C2 == 1);
    model.add(c2).setName("C2");

    ///////////// C3: arc_stage
    IloExpr C3(env);
    for (int k = 2; k <= K; k++) {
        {
            C3 += X[k][D];
        }
    }
    // , "C3"
    model.add(C3 == 1).setName("C3"); // arrival to depot

    for (int h = 1; h < D; h++) {
        IloExpr sum(env);
        for (int k = 2; k < K; k++) {
            sum += X[k][h];
        }
        model.add(sum >= 1);
    }
    model.add(X[5][6] == 1);
    model.add(X[10][6] == 1);
    cplex.solve();
    IloNumArray order(env);

    for (int k = 1; k <= K; k++) {
        for (int i = 0; i <= D; i++) {
            if (abs(cplex.getValue(X[k][i]) - 1) < 1e-5) {
                std::cout << "Stage " << k << " at " << i << std::endl;
            }
        }
    }
    for (int k = 1; k < K; k++) {
        for (int i = 0; i < D; i++) {
            for (int j = 1; j <= D; j++) {
                if (i != j) {
                    if (abs(cplex.getValue(x[k][i][j]) - 1) < 1e-5) {
                        std::cout << i << " " << j << std::endl;
                        order.add(i);
                    }
                }
            }
        }
    }

    order.add(D);
    std::cout << "Revisit TSP solution for this instance: ";
    for (int i = 0; i < order.getSize(); i++) {
        std::cout << order[i] << " ";
    }
    std::cout << std::endl;
    return order;
}

// Result Solver::StageBasedNoRevisit(int n_thread, int e) const {
//
// }
Result Solver::mvdSolverWithLR(int n_thread, double dtl, double sl, double sr, bool use_tsp_as_warmstart) const {
    auto tau = instance->tau;
    auto tau_prime = instance->tau_prime;
    auto n = instance->num_node;
    std::vector<int> C;
    std::vector<int> V;
    for (int i = 0; i < n + 1; i++) {
        if (i != 0 && i != n) {
            C.push_back(i);
        }
        V.push_back(i);
    }

    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    // cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);
    // cplex.setParam(IloCplex::Param::TimeLimit, 200.0);
    auto O = 0;
    auto D = n;
    int K = n+1;
    auto K_arc = K - 1;

    // Variable declaration
    // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
    //đường đi của vehicle là i; k \in 1..n;
    IloArray<IloBoolVarArray> X(env, K + 1);
    for (int k = 1; k <= K; k++) {
        X[k] = IloBoolVarArray(env, D + 1);
        for (int i = 0; i <= D; i++) {
            X[k][i] = IloBoolVar(env);
            model.add(X[k][i]);
            auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
            X[k][i].setName(v_name.c_str());

        }
        if (k > 1) {
            model.add(X[k][0] == 0);
        }
    }

    model.add(X[1][0] == 1).setName("First stage must be source depot");

    // Single-visit constraint
    // for (int k = 1; k <= K; k++) {
    //     IloExpr sumK(env);
    //     for (int i = 0; i <= D; i++) {
    //         sumK += X[k][i];
    //     }
    //     model.add(sumK <= 1);
    // }


    // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
    // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
    IloArray<IloArray<IloBoolVarArray> > x(env, K_arc + 1);
    for (int k = 1; k <= K_arc; k++) {
        x[k] = IloArray<IloBoolVarArray>(env, D);
        for (int i = 0; i < D; i++) {
            x[k][i] = IloBoolVarArray(env, D + 1);
            for (int j = 1; j <= D; j++) {
                if (i != j) {
                    x[k][i][j] = IloBoolVar(env);
                    auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
                    x[k][i][j].setName(v_name.c_str());
                }
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
        if (heavy != D) {
            model.add(phi[heavy] == 0);
        }
    }

    IloArray<IloArray<IloBoolVarArray> > Y(env, K + 1), W(env, K + 1);
    for (int k = 1; k <= K; k++) {
        Y[k] = IloArray<IloBoolVarArray>(env, D + 1);
        W[k] = IloArray<IloBoolVarArray>(env, D + 1);

        for (int i = 0; i <= D; i++) {
            Y[k][i] = IloBoolVarArray(env, C.size() + 1);
            W[k][i] = IloBoolVarArray(env, C.size() + 1);
            for (int h: C) {
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
    }

    // arrival\departure variables a and d.
    IloNumVarArray a(env, K + 1);
    IloNumVarArray d(env, K + 1);
    for (int k = 1; k <= K; k++) {
        a[k] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
        auto v_name = "a_" + std::to_string(k);
        a[k].setName(v_name.c_str());
        d[k] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
        v_name = "d_" + std::to_string(k);
        d[k].setName(v_name.c_str());
        model.add(d[k] >= a[k]).setName(("C13_" + std::to_string(k)).c_str());
    }

    model.add(a[1] == 0).setName("arrival to depot at time 0");
    model.add(d[1] == 0).setName("depart from depot at time 0");;

    // $R_{k} = \sum_{k'}Z_{kk'}$: các đoạn bắt đầu từ k (C23)
    IloBoolVarArray R(env, K + 1);
    for (int k = 1; k < K; k++) {
        R[k].setName(("R_" + std::to_string(k)).c_str());
    }

    //// aux var Z_{k, k_p, h}: sortie launch from k and rendezvous at k_p.
    IloArray<IloArray<IloBoolVarArray> > Z(env, K);
    for (int h: C) {
        Z[h] = IloArray<IloBoolVarArray>(env, K + 1);
        for (int k = 1; k <= K - 1; k++) {
            Z[h][k] = IloBoolVarArray(env, K + 1);
            for (int k_p = k + 1; k_p <= K; k_p++) {
                Z[h][k][k_p] = IloBoolVar(env);
                auto v_name = "Z_" + std::to_string(h) + "_" + std::to_string(k) + "_" + std::to_string(k_p);
                Z[h][k][k_p].setName(v_name.c_str());
            }
        }
    }
    //// aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
    IloArray<IloBoolVarArray> z(env, K);
    for (int k = 1; k <= K - 1; k++) {
        z[k] = IloBoolVarArray(env, K + 1);
        for (int k_p = k + 1; k_p <= K; k_p++) {
            z[k][k_p] = IloBoolVar(env);
            auto v_name = "z_" + std::to_string(k) + "_" + std::to_string(k_p);
            z[k][k_p].setName(v_name.c_str());
        }
    }

    ////-----------------------------------------------------------------------------------------------
    // WARMSTART CONFIG: using original TSP solution.
    // Obtained with MTZ formulation.
    // On variable X.
    if (use_tsp_as_warmstart) {
        auto tsp_solution = TSP_MTZ(tau);
        std::cout << tsp_solution.getSize() << std::endl;
        IloNumVarArray X_warm_var(env);
        IloNumArray X_warm_val(env);
        for (int k = 1; k <= K - 1; k++) {
            for (int i = 0; i <= D; i++) {
                X_warm_var.add(X[k][i]);
                if (tsp_solution[k - 1] == i) {
                    X_warm_val.add(true);
                    std::cout << "Warmstart X[" << k << "][" << i << "] == 1" << std::endl;
                } else {
                    X_warm_val.add(false);
                }
            }
        }
        cplex.addMIPStart(X_warm_var, X_warm_val);
        X_warm_var.end();
        X_warm_val.end();
    }
    ////-----------------------------------------------------------------------------------------------

    //// Sortie maximum stage gap calculation.
    /// Find maximum stage gap that a sortie can start and rendezvous.
    /// For each stage gap in increasing order (k' - k), find the minimum tour length from stage k to k'.
    /// Revisit(s) are also considered.
    /// If the tour length > dtl - sr => there can be no sortie with stage gap greater or equal k' - k.
    bool used_stage_gap = false;
    int min_stage_gap = 0;
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "Calculate max stage gap for sortie.... " << std::endl;
    for (int k = 1; k < K; k++) {
        for (int kp = k + 1; kp <= K; kp++) {
            if (kp > k + 6) {
                goto after_z_cons;
            }
            double smallest_tour = smallest_tour_length(kp - k, tau, V);
            if (smallest_tour > dtl - sr) {
                std::cout << "Stage gap of " << kp - k << " with smallest length = " << smallest_tour <<
                        " violated endurance constraint!" << std::endl;
                std::cout << "---------------------------------------------" << std::endl;
                min_stage_gap = kp - k;
                goto after_z_cons;
            }
        }
    }
    after_z_cons:
        if (min_stage_gap == 0) {
            std::cout << "Stage gap calculation consumes too much memory. No constraint was added." << std::endl;
            std::cout << "---------------------------------------------" << std::endl;
        }
    if (min_stage_gap != 0) {
        used_stage_gap = true;
        for (int k = 1; k < K; k++) {
            for (int kp = k + min_stage_gap; kp <= K; kp++) {
                model.add(z[k][kp] == 0);
                std::cout << "Variable z[" << k << "][" << kp << "] was set to 0." << std::endl;
                for (int h:instance->c_prime) {
                    model.add(Z[h][k][kp] == 0);
                    IloExpr start_stage_k(env), end_stage_kp(env);
                    for (int i = 0; i < D; i++) {
                        if (i != h) {
                            start_stage_k += Y[k][i][h];
                            // model.add(Z[h][k][kp] + Y[k][i][h] <= 1);

                        }
                    }
                    for (int j = 1; j <= D; j++) {
                        if (j != h) {
                            end_stage_kp += W[kp][j][h];
                            // model.add(Z[h][k][kp] + W[kp][j][h] <= 1);
                        }
                    }
                    std::string cname = "Can't serve customer " + std::to_string(h) + " start at stage " + std::to_string(k) + ", end at stage " + std::to_string(kp);
                    // Looks like quicker lower bound improvements with this constraints.
                    model.add(start_stage_k + end_stage_kp <= 1).setName(cname.c_str());
                }
            }
        }
    }

    ////-----------------------------------------------------------------------------------------------

    // CONSTRAINTS DECLARATION.------------------------------------------------------------------------
    // Constraint C1
    for (int k = 1; k < K; k++) {
        for (int i = 0; i < D; i++) {
            IloExpr sum(env);
            for (int j = 1; j <= D; j++) {
                if (i != j) {
                    sum += x[k][i][j];
                }
            }
            model.add(X[k][i] == sum).setName(("C1_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
        }
    }

    for (int k = 2; k <= K; k++) {
        for (int i = 1; i <= D; i++) {
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

    // C2: truck must depart from the depot at stage 1.
    IloExpr C2(env);
    for (int i = 1; i <= D; i++) {
        C2 += x[1][O][i];
    }

    IloConstraint c2(C2 == 1);
    model.add(c2).setName("Must start from source");

    // C3: Terminal depot must be arrived only once.
    IloExpr C3(env);
    for (int k = 2; k <= K; k++) {
        C3 += X[k][D];
    }
    model.add(C3 == 1).setName("Must visit terminal depot once");

    // At most ONE node each stage.
    for (int k = 1; k <= K; k++) {
        IloExpr sum(env);
        for (int i = 0; i <= D; i++)
            sum += X[k][i];
        model.add(sum <= 1).setName(("C20_at_most_one_customer_at_stage_" + std::to_string(k)).c_str());
    }

    // $Z_{kk'} = \sum_{h}Z^h_{kk'}$: mỗi cặp (k,k') chỉ phục vụ tối đa một khách hàng (C22)
    for (int k = 1; k < K; k++) {
        for (int k_p = k + 1; k_p <= K; k_p++) {
            IloExpr expr(env);
            for (int h: C) {
                expr += Z[h][k][k_p];
            }
            model.add(expr == z[k][k_p]).setName(("C22_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
        }
    }

    //C20:$\sum_{k'>k}Z_{kk'} = \sum_{i,h}Y^k_{ih}$ : với mỗi $k$,
    //ràng buộc liên kết drone đi ra từ stage $k$ và đoạn mà oto di chuyển không có drone. (C20)
    for (int h: C) {
        for (int k = 1; k <= K - 1; k++) {
            IloExpr expr(env);
            for (int k_p = k + 1; k_p <= K; k_p++) {
                expr += Z[h][k][k_p];
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
        for (int k_p = 2; k_p <= K; k_p++) {
            IloExpr expr(env);
            for (int k = 1; k < k_p; k++) {
                expr += Z[h][k][k_p];
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

    for (int k = 1; k < K; k++) {
        IloExpr expr(env);
        for (int k_p = k + 1; k_p <= K; k_p++) {
            expr += z[k][k_p];
        }
        model.add(R[k] == expr).setName(("C23_" + std::to_string(k)).c_str());
    }

    // modified C7
    for (int k = 1; k <= K - 1; k++) {
        for (int k_p = k + 1; k_p <= K; k_p++) {
            for (int l = k + 1; l < k_p; l++) {
                // tranh drone bay cac doan giao nhau.
                if (k < l) {
                    model.add(z[k][k_p] + R[l] <= 1).setName(("C7m_" + std::to_string(k)
                                                              + "_" + std::to_string(k_p) + "_" + std::to_string(l))
                        .c_str());
                }
                // else {
                //                        model.add(R[k] <= 1);
                // }
            }
        }
    }
    //// C17 - $X^k_i \geq \sum_h X^k_{ih}$ (C17) - chỉ bay drone ở nơi mà xe ở đó
    for (int k = 1; k <= K; k++) {
        for (int i = 0; i <= D; i++) {
            IloExpr expr(env);
            for (int h: C) {
                if (h != i) {
                    if (tau_prime[i][h] <= dtl - sr) {
                        expr += Y[k][i][h];
                    }
                }
            }
            model.add(expr <= X[k][i]).setName(("C17_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
        }
    }

    // $X^k_i \geq \sum_h Y^k_{ih}$ (C17p) : chỉ bay drone tới nơi mà xe ở đó
    for (int k = 1; k <= K; k++) {
        for (int i = 0; i <= D; i++) {
            IloExpr expr(env);
            for (int h: C) {
                if (h != i) {
                    if (tau_prime[h][i] <= dtl - sr) {
                        expr += W[k][i][h];
                    }
                }
            }
            model.add(expr <= X[k][i]).setName(("C17p_" + std::to_string(k) + "_" + std::to_string(i)).c_str());
        }
    }

    // $\sum_{i} X^k_{ih}\tau^D_{ih} + \sum_{i} Y^{k'}_{ih}\tau^D_{hi} \leq D_d$
    //- drone duration constraint cho mỗi $h$ (C19)
    for (int h: C) {
        IloExpr expr(env);

        for (int k = 1; k <= K; k++) {
            for (int i = 0; i <= D; i++) {
                if (i != h && i != D && tau_prime[i][h] <= dtl - sr) {
                    expr += Y[k][i][h] * tau_prime[i][h];
                }

                if (i != h && i != 0 && tau_prime[h][i] <= dtl - sr) {
                    expr += W[k][i][h] * tau_prime[h][i];
                }
            }
        }
        model.add(expr <= (dtl - sr) * phi[h]).setName(("C19_" + std::to_string(h)).c_str());
    }

    // modified C7p - we can select at most one segment that contains the point l
    for (int l = 2; l < K; l++) {
        IloExpr expr(env);

        for (int k = 1; k < l; k++)
            for (int k_p = l; k_p <= K; k_p++)
                expr += z[k][k_p];
        model.add(expr <= 1).setName(("C7mm_" + std::to_string(l)).c_str());
    }

    for (int i = 0; i < D; i++) {
        for (int k = 1; k <= K - 1; k++) {
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
        for (int k_p = 2; k_p <= K; k_p++) {
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
            if (i != h && tau_prime[i][h] <= dtl - sr) {
                for (int k = 1; k <= K - 1; k++) {
                    rhs += Y[k][i][h];
                }
            }
        }
        // consistency constraint cho sortie phuc vu h.
        model.add(phi[h] == rhs).setName(("C18_" + std::to_string(h)).c_str());
    }


    for (int h: C) {
        IloExpr rhs(env);
        for (int i = 1; i <= D; i++) {
            if (i != h && tau_prime[h][i] <= dtl - sr) {
                for (int k = 2; k <= K; k++) {
                    rhs += W[k][i][h];
                }
            }
        }
        // consistency constraint cho sortie phuc vu h.
        model.add(phi[h] == rhs).setName(("C18p_" + std::to_string(h)).c_str());
    }


    // Assignment constraint
    //////////// C10: node_stage
    for (int h: C) {
        IloExpr sum_k(env);
        for (int k = 2; k < K; k++) {
            sum_k += X[k][h];
        }
        // phuc vu h it nhat 1 lan.
        model.add(phi[h] + sum_k >= 1).setName(("C10_" + std::to_string(h)).c_str());
    }

    /////////// C14: node_stage
    for (int k = 1; k <= K_arc; k++) {
        IloExpr sum(env);
        for (int i = 0; i < D; i++) {
            for (int j = 1; j <= D; j++) {
                if (i != j) {
                    sum += x[k][i][j] * tau[i][j];
                }
            }
        }

        model.add(a[k + 1] == d[k] + sum).setName(("C14_" + std::to_string(k) + "_" + std::to_string(k + 1)).c_str());
    }

    ////////// C15: node_stage
    ///// big M calculation
    double M = 0;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i < j) {
                M += tau[i][j];
            }
        }
    }

    for (int k = 1; k <= K - 1; k++) {
        for (int k_p = k + 1; k_p <= K; k_p++) {
            if (k < k_p) {
                model.add(a[k_p] - d[k] <= z[k][k_p] * (dtl - sr) + (1 - z[k][k_p]) * M).setName(
                    ("C15_" + std::to_string(k) + "_" + std::to_string(k_p)).c_str());
            }
        }
    }

    for (int k = 1; k <= K - 1; k++) {
        for (int k_p = k + 1; k_p <= K; k_p++) {
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

    for (int k = 2; k <= K; k++) {
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
                if (i != h && k != K) {
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

    // SLOW
    // (k,h): k chỉ có thể là đến hoặc đi của khách hàng h.
    int count_cons = 0;
    for (int h:instance->c_prime) {
        for (int i:C) {
            if (i != h) {
                for (int k = 1; k < K; k++) {
                    for (int kp = k+1; kp < K; kp++) {
                        model.add(Y[k][i][h] + W[kp][i][h] <= phi[h]);
                    }
                }
                // SLOW VARIANT.
            }
        }
    }
    std::cout << "Num constraint added = " << count_cons << std::endl;

    // LB constraint
    IloExpr lb_truck_tour(env);
    for (int k = 1; k < K; k++) {
        for (int i = 0; i < D; i++)
            for (int j = 1; j <= D; j++)
                if (i != j)
                    lb_truck_tour += x[k][i][j] * tau[i][j];
    }
    model.add(d[K] >= lb_truck_tour).setName("Lower_bound_obj");

    IloExpr lb_drone_tour(env);
    for (int k = 1; k < K; k++) {
        for (int i = 0; i < D; i++) {
            for (int h: C) {
                if (i != h) {
                    lb_drone_tour += Y[k][i][h] * tau_prime[i][h];
                }
            }
        }
    }
    for (int k = 2; k <= K; k++) {
        for (int j = 1; j <= D; j++) {
            for (int h: C) {
                if (j != h) {
                    lb_drone_tour += W[k][j][h] * tau_prime[h][j];
                }
            }
        }
    }
    model.add(d[K] >= lb_drone_tour);

    for (int k = 2; k < K / 2; k++) {
        model.add(X[k][D] == 0);
    }

    double bestObjective = IloInfinity;
    // BranchAndCutCallback bc(env, model, X, x, phi, Z, z, Y, W, K, D, bestObjective, C, tau, tau_prime);
    // cplex.use(&bc);
    model.add(IloMinimize(env, d[K]));
    cplex.exportModel("cplex_model_1.lp");
    std::vector<Sortie> st;
    double obj = 0;
    double revisit_count = 0;
    // Solve the model
    std::vector<int> rev(n + 1, 0);
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
    for (int k = 1; k <= K; k++) {
        for (int i = 0; i <= D; i++) {
            auto X_val = cplex.getValue(X[k][i]);
            //std::cout << "k = " << k << ", i = " << i << ":" << X_val << std::endl;
            if (X_val == 1) {
                rev[i]++;
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
    for (int k = 1; k <= K_arc; k++) {
        for (int i = 0; i < D; i++)
            for (int j = 1; j <= D; j++)
                if (i != j) {
                    auto X_val = cplex.getValue(x[k][i][j]);
                    if (X_val == 1) {
                        std::cout << "Arc " << k << " connecting " << i << " and " << j
                                << " with cost " << tau[i][j] << " " << std::endl;
                        obj += tau[i][j];
                        map_stage_truck_arc[k] = std::make_pair(i, j);
                        break;
                    }
                }
    }

    for (int h: C) {
        if (cplex.getValue(phi[h]) == 1) {
            std::cout << "Customer " << h << " served by drone." << std::endl;
            auto sortie = Sortie(h);
            st.push_back(sortie);
            int sv_i = -1, sv_j = -1, sv_k = -1, sv_kp = -1;
            for (int k = 1; k <= K; k++) {
                for (int i = 0; i <= D; i++)
                    if (i != h) {
                        try {
                            auto Y_val = cplex.getValue(Y[k][i][h]);
                            if (Y_val == 1) {
                                sv_i = i;
                                sv_k = k;
                            }
                            auto W_val = cplex.getValue(W[k][i][h]);
                            if (W_val == 1) {
                                sv_j = i;
                                sv_kp = k;
                            }
                        } catch (...) {
                        }
                    }
            }

            std::cout << "Drone fly from " << sv_i << " at stage " << sv_k <<
                    " to serve " << h << " and then fly back to " << sv_j
                    << " at stage " << sv_kp << ". " << std::endl;
            obj += (sl + sr);
            if (sv_i == O) {
                obj -= sl;
            }
            double drone_travel_time = tau_prime[sv_i][h] + tau_prime[h][sv_j];
            double truck_travel_time = 0;
            std::cout << "Truck arcs during this sortie: ";
            for (int k_start = sv_k; k_start <= sv_kp; k_start++) {
                if (k_start == sv_kp) {
                    std::cout << map_stage_truck_arc[k_start].first;
                } else {
                    std::cout << map_stage_truck_arc[k_start].first << "--->";
                }
            }
            std::cout << std::endl;
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
            std::cout << "Total drone travel time: " << drone_travel_time << std::endl;

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
    for (int i = 0; i < rev.size(); i++) {
        if (rev[i] > 1) {
            revisit_count += rev[i] - 1;
            std::cout << "Node " << i << " was revisited " << rev[i] - 1 << " times!" << std::endl;
        }
    }
    std::cout << "OBJECTIVE VALUE: " << c << ", NUMBER OF SORTIES: " << st.size() << "." << std::endl;
    std::cout << "Number of revisit: " << revisit_count << std::endl;
    return Result{c, obj, duration.count() / 1000.0, revisit_count, used_stage_gap};
}

// Link: https://drive.google.com/file/d/1CpYCse--JWmrnBY566Obe8IMpcClTpnI/view?usp=drive_link
Result Solver::OriginalSolverCPLEX(int n_thread, int e) {
    auto tau = instance->tau;

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
        if (!exist(instance->c_prime, h) || h == s || h == t) {
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
                    if (exist(C, k)) {
                        sum3 += 1 - theta[k];
                    }
                }
            }

            model.add(sum1 <= sum3);
        }
    }
    // Constraint 4
    for (int h: C) {
        IloExpr lhs_4(env);
        std::string cname = "C4_h_" + std::to_string(h);
        for (int j: c_t) {
            lhs_4 += gamma[h][s][j];
        }
        model.add(lhs_4 == omega[h][s]);
    }

    // Constraint 5
    for (int h: C) {
        IloExpr lhs_5(env);
        std::string cname = "C5_h_" + std::to_string(h);

        for (int i: c_s) {
            lhs_5 += gamma[h][i][t];
        }
        model.add(lhs_5 == delta[h][t]);
    }
    // Constraint 6
    for (int i: C) {
        for (int h: C) {
            std::string cname = "C6_i_" + std::to_string(i) + "_h_" + std::to_string(h);
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
        std::string cname = "C8_i_" + std::to_string(i) + "_t";
        model.add(y[i][t] + x[i][t] <= 1);
    }
    // Constraint 9
    for (int i: C) {
        for (int j: C) {
            std::string cname = "C9_i_" + std::to_string(i) + "_j=" + std::to_string(j);
            model.add(y[i][j] + x[i][j] + x[j][i] <= 1);
        }
    }

    // Constraint 10
    for (int h: C) {
        IloExpr sum(env);
        std::string cname = "C10_h_" + std::to_string(h);
        for (int j: c_t) {
            if (h != j) {
                sum += y[h][j];
            }
        }
        model.add(sum + theta[h] == 1);
    }
    // Constraint 11
    for (int i: c_s) {
        for (int j: c_t) {
            if (i != j) {
                std::string cname = "C11_i_" + std::to_string(i) + "_j=" + std::to_string(j);
                IloExpr sum(env);
                for (int h: c_prime) {
                    sum += gamma[h][i][j];
                }
                model.add(sum <= y[i][j]);
            }
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
            if (i != j) {
                std::string cname = "C13_i_" + std::to_string(i) + "_j=" + std::to_string(j);
                model.add(x[i][j] <= theta[i] + theta[j]);
            }
        }
    }
    //        // Constraint 14
    for (int i: c_s) {
        for (int j: c_t) {
            if (i != s && j != t && i != j) {
                std::string cname = "C14_i_" + std::to_string(i) + "_j=" + std::to_string(j);
                model.add(x[i][j] <= omega[j][i] + delta[i][j]);
            }
        }
    }
    // Constraint 15
    for (int i: c_s) {
        IloExpr sum1(env), sum2(env);
        std::string cname = "C15_i_" + std::to_string(i);
        for (int j: c_t) {
            if (i != j) {
                sum1 += x[i][j];
            }
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
        std::string cname = "C17_h_" + std::to_string(h);
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
        std::string c18_name = "C18_h_" + std::to_string(h);
        std::string c19_name = "C19_h_" + std::to_string(h);

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
    auto startTime = std::chrono::high_resolution_clock::now();
    cplex.solve();
    auto endTime = std::chrono::high_resolution_clock::now();
    // Calculate the elapsed time
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
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
    return Result{cplex.getObjValue(), 0, duration.count() / 1000.0, st};
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
    return Result(cplex.getObjValue(), 0, duration.count() / 1000, st);
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

    IloArray<IloBoolVarArray> x(env, n + 1);
    for (int i: N0) {
        x[i] = IloBoolVarArray(env, n + 1);
        for (int j: N_p) {
            if (i != j) {
                x[i][j] = IloBoolVar(env);
            }
        }
    }
    IloArray<IloArray<IloBoolVarArray> > y(env, n + 1);
    for (int i: N0) {
        y[i] = IloArray<IloBoolVarArray>(env, n + 1);
        for (int j: c_prime) {
            if (i != j) {
                y[i][j] = IloBoolVarArray(env, n + 1);
                for (int k: N_p) {
                    if (i != k && j != k) {
                        if (tau_prime[i][j] + tau_prime[j][k] + sr <= dtl) {
                            y[i][j][k] = IloBoolVar(env);
                        }
                    }
                }
            }
        }
    }

    IloNumVarArray w(env, n + 1), t(env, n + 1);
    for (int i: N) {
        w[i] = IloNumVar(env, 0, IloInfinity);
        t[i] = IloNumVar(env, 0, IloInfinity);
    }

    IloBoolVarArray z(env, n + 1);
    for (int i: N) {
        z[i] = IloBoolVar(env);
    }

    IloExpr objective(env);

    // Objective's first term
    for (int i: N0) {
        for (int j: N_p) {
            if (i != j) {
                objective += x[i][j] * tau[i][j];
            }
        }
    }

    // Objective's second term
    IloExpr obj_2nd_term(env);
    for (int j: c_prime) {
        for (int k: N_p) {
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
    for (int i: N0) {
        if (i != 0) {
            for (int j: c_prime) {
                if (i != j) {
                    for (int k: N_p) {
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
    for (int i: N_p) {
        objective += w[i];
    }

    // Constraints:

    // C2
    for (int j: C) {
        IloExpr s1(env), s2(env);
        for (int i: N0) {
            if (i != j) {
                s1 += x[i][j];
            }
        }
        if (exist(c_prime, j)) {
            for (int i: N0) {
                for (int k: N_p) {
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
    for (int j: N_p) {
        c3_s1 += x[0][j];
    }
    for (int i: N0) {
        c3_s2 += x[i][n];
    }
    model.add(c3_s1 == 1);
    model.add(c3_s2 == 1);

    // C4
    for (int j: C) {
        IloExpr c4_s1(env), c4_s2(env);
        for (int i: N0) {
            if (i != j) {
                c4_s1 += x[i][j];
            }
        }
        for (int i: N_p) {
            if (i != j) {
                c4_s2 += x[j][i];
            }
        }
        model.add(c4_s1 == c4_s2);
    }

    // C5
    for (int i: N0) {
        for (int j: N_p) {
            if (i != j) {
                model.add(t[j] >= t[i] + tau[i][j] - M * (1 - x[i][j]));
            }
        }
    }

    // C6
    for (int i: N0) {
        for (int j: N_p) {
            if (i != j) {
                IloExpr c6_s1(env);
                for (int k: c_prime) {
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
    for (int i: N0) {
        for (int j: N_p) {
            if (i != j) {
                model.add(w[j] >= t[j] - t[i] - tau[i][j] - M * (1 - x[i][j]));
            }
        }
    }

    // C8
    for (int i: N0) {
        for (int j: N_p) {
            if (i != j) {
                IloExpr c8_s1(env);
                for (int k: c_prime) {
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
    for (int i: N_p) {
        IloExpr c9_s1(env);
        for (int j: N0) {
            if (i != j) {
                c9_s1 += x[j][i];
            }
        }
        model.add(z[i] <= c9_s1);
    }

    // C10
    for (int i: N0) {
        IloExpr c10_s1(env);
        for (int j: c_prime) {
            for (int k: N_p) {
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
    for (int i: N0) {
        for (int j: N_p) {
            if (i != j) {
                IloExpr c11_s1(env), c11_s2(env);
                for (int l: N0) {
                    for (int k: c_prime) {
                        if (l != k && l != j && k != j) {
                            if (tau_prime[l][k] + tau_prime[k][j] + sr <= dtl) {
                                c11_s1 += y[l][k][j];
                            }
                        }
                    }
                }
                for (int k: c_prime) {
                    for (int l: N_p) {
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
    for (int j: c_prime) {
        IloExpr sum(env);
        for (int i: c_s) {
            for (int k: c_t) {
                if (i != j && j != k && i != k) {
                    sum += y[i][j][k];
                }
            }
        }
        for (int i: c_prime) {
            for (int k: c_t) {
                if (i != j && j != k && i != k) {
                    sum += y[j][i][k];
                }
            }
        }
        for (int k: c_s) {
            for (int i: c_prime) {
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
    for (int j: N_p) {
        c17_s1 += x[0][j];
    }
    model.add(z[0] <= c17_s1);

    // C18
    for (int i: N_p) {
        IloExpr c18_s1(env);
        for (int k: N0) {
            for (int j: c_prime) {
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
    for (int i: c_prime) {
        IloExpr c19_s1(env);
        for (int j: N0) {
            for (int k: N_p) {
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
    for (int j: N_p) {
        IloExpr c20_s1(env);
        for (int i: N0) {
            for (int k: c_prime) {
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
    for (int j: c_prime) {
        IloExpr c21_s1(env);
        for (int i: N0) {
            for (int k: N_p) {
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
    for (int i: c_s) {
        for (int j: c_t) {
            if (i != j) {
                if (cplex.getValue(x[i][j]) == 1) {
                    std::cout << i << " " << j << std::endl;
                }
            }
        }
    }
    std::cout << "Drone sorties:" << std::endl;
    for (int i: N0) {
        for (int j: c_prime) {
            for (int k: N_p) {
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
    return Result(cplex.getObjValue(), 0, duration.count() / 1000, st);
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

    IloNumVarArray w(env, n + 1), t(env, n + 1);
    for (int i: N) {
        w[i] = IloNumVar(env, 0, IloInfinity);
        t[i] = IloNumVar(env, 0, IloInfinity);
    }

    IloBoolVarArray z(env, n + 1);
    for (int i: N) {
        z[i] = IloBoolVar(env);
    }

    IloArray<IloBoolVarArray> x(env, n + 1);
    for (int i: N0) {
        x[i] = IloBoolVarArray(env, n + 1);
        for (int j: N_p) {
            if (i != j) {
                x[i][j] = IloBoolVar(env);
            }
        }
    }
    IloArray<IloBoolVarArray> gl(env, n + 1);
    for (int i: N0) {
        gl[i] = IloBoolVarArray(env, n + 1);
        for (int j: c_prime) {
            if (i != j) {
                gl[i][j] = IloBoolVar(env);
                if (tau_prime[i][j] < e) {
                    model.add(gl[i][j] == 0);
                }
            }
        }
    }
    IloArray<IloBoolVarArray> gr(env, n + 1);
    for (int j: c_prime) {
        gr[j] = IloBoolVarArray(env, n + 1);
        for (int k: N_p) {
            if (j != k) {
                gr[j][k] = IloBoolVar(env);
                if (tau_prime[j][k] < e) {
                    model.add(gr[j][k] == 0);
                }
            }
        }
    }

    // Objective
    IloExpr objective(env);

    // First term
    for (int i: N0) {
        for (int j: N_p) {
            if (i != j) {
                objective += x[i][j] * tau[i][j];
            }
        }
    }

    // Second term
    IloExpr obj_2nd_term(env);
    for (int i: N0) {
        if (i != 0) {
            for (int j: c_prime) {
                if (i != j) {
                    if (tau_prime[i][j] < e)
                        obj_2nd_term += gl[i][j];
                }
            }
        }
    }
    objective += sl * obj_2nd_term;

    // Third term
    IloExpr obj_3rd_term(env);
    for (int j: c_prime) {
        for (int k: N_p) {
            if (j != k) {
                if (tau_prime[j][k] < e)
                    objective += gr[j][k];
            }
        }
    }
    objective += sr * obj_3rd_term;

    // Fourth term
    for (int i: N_p) {
        objective += w[i];
    }

    // C23
    for (int j: C) {
        IloExpr c23_s1(env), c23_s2(env);
        for (int i: N0) {
            if (i != j) {
                if (tau_prime[i][j] < e)
                    c23_s1 += x[i][j];
                c23_s2 += gl[i][j];
            }
        }
        model.add(c23_s1 + c23_s2 == 1);
    }

    // C24
    for (int i: N0) {
        for (int j: N_p) {
            if (i != j) {
                if (tau_prime[i][j] < e)
                    model.add(t[j] >= t[i] + tau_prime[i][j] - M * (1 - gl[i][j]));
            }
        }
    }

    // C25
    for (int j: c_prime) {
        for (int k: N_p) {
            if (j != k) {
                if (tau_prime[j][k] < e)

                    model.add(t[k] >= t[j] + tau_prime[j][k] - M * (1 - gr[j][k]));
            }
        }
    }

    // C26
    for (int i: N0) {
        for (int j: c_prime) {
            for (int k: N_p) {
                if (i != j && i != k && j != k) {
                    if (tau_prime[i][j] + tau_prime[j][k] + sr <= dtl) {
                        model.add(t[k] - t[i] + sr - M * (2 - gl[i][j] - gr[j][k]) <= e);
                    }
                }
            }
        }
    }

    // C27
    for (int j: c_prime) {
        IloExpr s1(env), s2(env);
        for (int i: N0) {
            if (i != j) {
                if (tau_prime[i][j] < e)
                    s1 += gl[i][j];
            }
        }
        for (int k: N_p) {
            if (j != k) {
                if (tau_prime[j][k] < e)
                    s2 += gr[j][k];
            }
        }
        model.add(s1 == s2);
    }

    // C28
    for (int i: N0) {
        IloExpr s(env);
        for (int j: c_prime) {
            if (i != j) {
                if (tau_prime[i][j] < e)

                    s += gl[i][j];
            }
        }
        model.add(s <= z[i]);
    }

    // C29
    for (int i: N0) {
        for (int j: N_p) {
            if (i != j) {
                IloExpr s(env);
                for (int k: c_prime) {
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
    for (int i: c_prime) {
        for (int j: c_prime) {
            if (i != j) {
                if (tau_prime[i][j] < e)

                    model.add(gl[i][j] + gr[i][j] <= 1);
                model.add(gl[i][j] + gr[j][i] <= 1);
            }
        }
    }

    // C3
    IloExpr c3_s1(env), c3_s2(env);
    for (int j: N_p) {
        c3_s1 += x[0][j];
    }
    for (int i: N0) {
        c3_s2 += x[i][n];
    }
    model.add(c3_s1 == 1);
    model.add(c3_s2 == 1);

    // C4
    for (int j: C) {
        IloExpr c4_s1(env), c4_s2(env);
        for (int i: N0) {
            if (i != j) {
                c4_s1 += x[i][j];
            }
        }
        for (int i: N_p) {
            if (i != j) {
                c4_s2 += x[j][i];
            }
        }
        model.add(c4_s1 == c4_s2);
    }
    // C5
    for (int i: N0) {
        for (int j: N_p) {
            if (i != j) {
                model.add(t[j] >= t[i] + tau[i][j] - M * (1 - x[i][j]));
            }
        }
    }
    // C7
    for (int i: N0) {
        for (int j: N_p) {
            if (i != j) {
                model.add(w[j] >= t[j] - t[i] - tau[i][j] - M * (1 - x[i][j]));
            }
        }
    }
    // C9
    for (int i: N_p) {
        IloExpr c9_s1(env);
        for (int j: N0) {
            if (i != j) {
                c9_s1 += x[j][i];
            }
        }
        model.add(z[i] <= c9_s1);
    }

    model.add(IloMinimize(env, objective));
    cplex.solve();
    std::cout << cplex.getObjValue() << std::endl;
    for (int i: c_s) {
        for (int j: c_t) {
            if (i != j) {
                if (cplex.getValue(x[i][j]) == 1) {
                    std::cout << i << " " << j << std::endl;
                }
            }
        }
    }
    std::cout << "gl:" << std::endl;
    for (int i: N0) {
        for (int j: c_prime) {
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
    for (int j: c_prime) {
        for (int k: N_p) {
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

Result Solver::mFSTSPSolve(int n_thread, double dtl, double sl, double sr) const {
    auto tau = instance->tau;
    auto tau_prime = instance->tau_prime;
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

    const auto O = 0;
    const auto D = n;
    const auto K = n + 1;

    // cs: points that from it vehicle can leave.
    std::vector<int> c_s;

    // ce: points that the truck can enter.
    std::vector<int> c_t;

    for (int i = O; i <= D; i++) {
        if (i != D) {
            c_s.push_back(i);
            c_t.push_back(i);
        } else {
            c_t.push_back(i);
        }
    }

    IloArray<IloBoolVarArray> y(env, n + 1);
    for (int i: c_s) {
        y[i] = IloBoolVarArray(env, n + 1);
        for (int j: c_t) {
            if (i != j) {
                y[i][j] = IloBoolVar(env);
            }
        }
    }
    int nd = 3;
    std::vector<int> d = {0, 1, 2};
    IloArray<IloArray<IloBoolVarArray> > x(env, nd);
    for (int v: d) {
        x[v] = IloArray<IloBoolVarArray>(env, n + 1);
        for (int i: c_s) {
            x[v][i] = IloBoolVarArray(env, n + 1);
            for (int j: c_t) {
                x[v][i][j] = IloBoolVar(env);
            }
        }
    }
    IloArray<IloArray<IloArray<IloBoolVarArray> > > gamma(env, n);
    for (int h: C) {
        gamma[h] = IloArray<IloArray<IloBoolVarArray> >(env, nd);
        for (int v: d) {
            gamma[h][v] = IloArray<IloBoolVarArray>(env, n + 1);
            for (int i: c_s) {
                gamma[h][v][i] = IloBoolVarArray(env, n + 1);
                for (int j: c_t) {
                    if (i != j) {
                        gamma[h][v][i][j] = IloBoolVar(env);
                    }
                }
            }
        }
    }

    IloArray<IloBoolVarArray> theta(env, n + 1);
    for (int h: C) {
        theta[h] = IloBoolVarArray(env, n + 1);
        for (int v: d) {
            theta[h][v] = IloBoolVar(env);
        }
    }
    IloArray<IloArray<IloBoolVarArray> > omega(env, n + 1);
    for (int h: C) {
        omega[h] = IloArray<IloBoolVarArray>(env, nd);
        for (int v: d) {
            omega[h][v] = IloBoolVarArray(env, n + 1);
            for (int i: c_s) {
                if (h != i) {
                    omega[h][v][i] = IloBoolVar(env);
                }
            }
        }
    }
    IloArray<IloArray<IloBoolVarArray> > delta(env, n + 1);
    for (int h: C) {
        delta[h] = IloArray<IloBoolVarArray>(env, nd);
        for (int v: d) {
            delta[h][v] = IloBoolVarArray(env, n + 1);
            for (int j: c_t) {
                if (h != j) {
                    delta[h][v][j] = IloBoolVar(env);
                }
            }
        }
    }

    IloNumVarArray w(env, n);
    for (int i: c_t) {
        w[i] = IloNumVar(env, 0, IloInfinity);
    }

    IloArray<IloArray<IloBoolVarArray> > z(env, n + 1);
    for (int i: c_s) {
        z[i] = IloArray<IloBoolVarArray>(env, nd);
        for (int v: d) {
            z[i][v] = IloBoolVarArray(env, nd);
            for (int w: d) {
                if (v != w) {
                    z[i][v][w] = IloBoolVar(env);
                }
            }
        }
    }
    auto s = 0;
    auto t = n;
    // C1
    IloExpr c1s1(env), c1s2(env);
    for (int j: c_t) {
        c1s1 += y[s][j];
    }
    for (int i: c_s) {
        c1s2 += y[i][t];
    }
    model.add(c1s1 == c1s2);
    model.add(c1s1 == 1);

    // C2
    for (int i: C) {
        IloExpr c2s1(env), c2s2(env);
        for (int j: c_t) {
            if (i != j) {
                c2s1 += y[i][j];
            }
        }
        for (int j: c_s) {
            if (i != j) {
                c2s2 += y[j][i];
            }
        }
        model.add(c2s1 == c2s2);
        model.add(c2s1 <= 1);
    }

    // C3
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
                    if (exist(C, k)) {
                        sum3 += 1;
                        for (int v: d) {
                            sum3 -= theta[k][v];
                        }
                    }
                }
            }

            model.add(sum1 <= sum3);
        }
    }

    // C4
    for (int h: C) {
        for (int v: d) {
            IloExpr c4(env);
            for (int j: c_t) {
                c4 += gamma[h][v][s][j];
            }
            model.add(c4 == omega[h][v][s]);
        }
    }

    // C5
    for (int h: C) {
        for (int v: d) {
            IloExpr c5(env);
            for (int i: c_s) {
                c5 += gamma[h][v][i][t];
            }
            model.add(c5 == delta[h][v][t]);
        }
    }

    // C6
    for (int i: C) {
        for (int h: C) {
            if (i != h) {
                for (int v: d) {
                    IloExpr c6s1(env), c6s2(env);
                    for (int j: c_t) {
                        c6s1 += gamma[h][v][i][j];
                    }

                    for (int j: c_s) {
                        c6s2 += gamma[h][v][j][i];
                    }
                    model.add(c6s1 - c6s2 == omega[h][v][i] - delta[h][v][i]);
                }
            }
        }
    }

    // C7
    for (int j: c_t) {
        IloExpr c7s1(env);
        for (int v: d) {
            c7s1 += x[v][s][j];
        }
        model.add(y[s][j] + c7s1 <= 1);
    }

    // C8
    for (int i: c_s) {
        IloExpr c8s1(env);
        for (int v: d) {
            c8s1 += x[v][i][t];
        }
        model.add(y[i][t] + c8s1 <= 1);
    }

    // C9
    for (int i: C) {
        for (int j: C) {
            if (i != j) {
                IloExpr c9s1(env);
                for (int v: d) {
                    c9s1 += x[v][i][j] + x[v][j][i];
                }
                model.add(y[i][j] + c9s1 <= 1);
            }
        }
    }

    // C10
    for (int h: C) {
        IloExpr c10s1(env), c10s2(env);
        for (int j: c_t) {
            if (h != j) {
                c10s1 += y[h][j];
            }
        }
        for (int v: d) {
            c10s2 += theta[h][v];
        }
        model.add(c10s1 + c10s2 == 1);
    }

    // C11
    for (int i: c_s) {
        for (int j: c_t) {
            if (i != j) {
                IloExpr c11s1(env);
                for (int h: C) {
                    if (h != i && h != j) {
                        for (int v: d) {
                            c11s1 += gamma[h][v][i][j];
                        }
                    }
                }
                model.add(c11s1 <= y[i][j]);
            }
        }
    }

    // C12
    for (int h: C) {
        for (int v: d) {
            IloExpr c12s1(env), c12s2(env);
            for (int i: c_s) {
                if (i != h) {
                    c12s1 += omega[h][v][i];
                }
            }
            for (int j: c_t) {
                if (j != h) {
                    c12s2 += delta[h][v][j];
                }
            }
            model.add(c12s1 == theta[h][v]);
            model.add(c12s2 == theta[h][v]);
        }
    }

    // C13
    for (int i: c_s) {
        for (int j: c_t) {
            if (i != j) {
                for (int v: d) {
                    model.add(x[v][i][j] <= theta[i][v] + theta[j][v]);
                }
            }
        }
    }

    // C14
    for (int i: c_s) {
        for (int j: c_t) {
            if (i != j) {
                for (int v: d) {
                    model.add(x[v][i][j] <= omega[j][v][i] + delta[i][v][j]);
                }
            }
        }
    }

    // C15
    for (int i: C) {
        for (int v: d) {
            IloExpr c15s1(env), c15s2(env);
            for (int j: c_t) {
                if (i != j) {
                    c15s1 += x[v][i][j];
                }
            }
            for (int h: C) {
                if (h != i) {
                    c15s2 += omega[h][v][i];
                }
            }
            model.add(c15s1 == c15s2 + theta[i][v]);
            model.add(c15s1 <= 1);
            model.add(c15s2 + theta[i][v] <= 1);
        }
    }

    // C16
    for (int j: C) {
        for (int v: d) {
            IloExpr c16s1(env), c16s2(env);
            for (int i: c_s) {
                if (i != j) {
                    c16s1 += x[v][i][j];
                }
            }
            for (int h: C) {
                if (h != j) {
                    c16s2 += delta[h][v][j];
                }
            }
            model.add(c16s1 == c16s2 + theta[j][v]);
            model.add(c16s1 <= 1);
            model.add(c16s2 + theta[j][v] <= 1);
        }
    }

    // C18
    for (int h: C) {
        for (int v: d) {
            IloExpr c17(env);
            for (int i: c_s) {
                for (int j: c_t) {
                    if (i != j) {
                        c17 += gamma[h][v][i][j] * tau[i][j];
                    }
                }
            }
            model.add(c17 <= (dtl - sr) * theta[h][v]);
            IloExpr c18s1(env), c18s2(env);
            for (int i: c_s) {
                if (i != h) {
                    c18s1 += omega[h][v][i] * tau_prime[i][h];
                }
            }
            for (int j: c_t) {
                if (j != h) {
                    c18s2 += delta[h][v][j] * tau_prime[h][j];
                }
            }
            model.add(c18s1 + c18s2 <= (dtl - sr) * theta[h][v]);
        }
    }

    // Waiting time constraint
    auto M = 1e5;
    for (int h: instance->c_prime) {
        for (int v: d) {
            for (int j: c_t) {
                if (h != j) {
                    IloExpr sum_d_launch(env), sum_d_rend(env), sum_truck(env);
                    // model.add(w[j] <= )
                }
            }
        }
    }
}

struct MultivisitTuple {
    int index;
    int start_node;
    int end_node;
    std::vector<int> serve_customer;
    std::vector<int> visit_seq;
    double trip_l;
};

inline void generateCombinationsHelper(const std::vector<int>& C, int L, int start, std::vector<int>& current, std::vector<std::vector<int>>& result) {
    // Base case: If current combination is of size L, add it to the result
    if (current.size() == L) {
        result.push_back(current);
        return;
    }

    // Recursive case: Generate combinations starting from each index in C
    for (int i = start; i < C.size(); ++i) {
        current.push_back(C[i]);
        generateCombinationsHelper(C, L, i + 1, current, result);
        current.pop_back();
    }
}

inline std::vector<std::vector<int>> generateCombinations(const std::vector<int>& C, int L) {
    std::vector<std::vector<int>> result;
    std::vector<int> current;
    generateCombinationsHelper(C, L, 0, current, result);
    return result;
}

inline std::vector<std::vector<int>> generatePermutations(const std::vector<int>& nums) {
    std::vector<std::vector<int>> result;

    // Sort the input vector to ensure all permutations are unique
    std::vector<int> sortedNums = nums;
    std::sort(sortedNums.begin(), sortedNums.end());

    // Generate permutations using std::next_permutation
    do {
        result.push_back(sortedNums);
    } while (std::next_permutation(sortedNums.begin(), sortedNums.end()));

    return result;
}

inline std::vector<MultivisitTuple> shortest_L_visit_sorties(int start, int rendezvous, int L, const std::vector<int>& C, std::vector<std::vector<double>> &tau_prime, double &dtl, double &sr) {
    auto combination_of_L_from_C = generateCombinations(C, L);
    std::vector<MultivisitTuple> result;
    for (auto &combo:combination_of_L_from_C) {
        double shortest = std::numeric_limits<double>::max();
        bool got = false;
        MultivisitTuple mt;
        auto permutations = generatePermutations(combo);
        for (auto &per:permutations) {
            std::vector<int> temp;
            temp.push_back(start);
            for (int &elem:per) {
                temp.push_back(elem);
            }
            temp.push_back(rendezvous);
            if (areAllElementsDistinct(temp)) {
                if (double l = calculate_tour(tau_prime, temp); l < shortest && l <= dtl-sr) {
                    shortest = l;
                    mt.visit_seq = temp;
                    mt.trip_l = l;
                    mt.serve_customer = per;
                    got = true;
                }
            }
        }
        if (got) {
            mt.start_node = start;
            mt.end_node = rendezvous;
            result.push_back(mt);
        }
    }
    return result;
}
std::vector<MultivisitTuple> getSortiesFromLR(int l, int r, std::vector<MultivisitTuple> &mt) {
    std::vector<MultivisitTuple> res;
    // both are provided
    if (l >= 0 && r > 0 && l != r) {
        for (auto &m:mt) {
            if (m.start_node == l && m.end_node == r) {
                res.push_back(m);
            }
        }
        return res;
    } else if (l >= 0 && r == -1) {
        // only provide start. end = -1
        for (auto &m:mt) {
            if (m.start_node == l) {
                res.push_back(m);
            }
        }
        return res;
    } else if (l == -1 && r > 0) {
        // only provide end. start = -1
        for (auto &m:mt) {
            if (m.end_node == r) {
                res.push_back(m);
            }
        }
        return res;
    } else {
        std::cout << "No sorties with options was found. Provided: launch = " << l << ", rendezvous = " << r << std::endl;
        return res;
    }

}
Result Solver::RV_FSTSP_MVD(int n_thread, double dtl, double sl, double sr, int L, bool use_tsp_as_warmstart) const {
    auto tau = instance->tau;
    auto tau_prime = instance->tau_prime;
    auto n = instance->num_node;
    std::vector<int> C;
    std::vector<int> V;
    std::vector<int> c_s;
    std::vector<int> c_t;
    for (int i = 0; i < n + 1; i++) {
        if (i != 0 && i != n) {
            C.push_back(i);
        }
        if (i != 0) {
            c_t.push_back(i);
        }
        if (i != n) {
            c_s.push_back(i);
        }
        V.push_back(i);
    }

    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);

    auto O = 0;
    auto D = n;
    auto K = n + 1;
    auto K_arc = K - 1;

    std::vector<MultivisitTuple> all_sortie;

    // get set D = {i, Y, j}: {i \in c_s, j \in c_t, Y subset of C', |Y| <= L}
    for (int n_customer = 1; n_customer <= L; n_customer++) {
        for (int i = 0; i < D; i++) {
            for (int j = 1; j <= D; j++) {
                if (i != j) {
                    auto mt = shortest_L_visit_sorties(i, j, n_customer, instance->c_prime, tau_prime, dtl, sr);
                    for (auto &m:mt) {
                        all_sortie.push_back(m);
                    }
                }
            }
        }
    }

    all_sortie.shrink_to_fit();
    int s_index = 0;
    for (auto &H:all_sortie) {
        H.index = s_index;
        s_index++;
    }

    auto S = all_sortie.size();
    std::cout << "Number of possible sorties: " << S << std::endl;
    std::vector<std::vector<int>> start_with_i(D), end_with_j(D+1);
    for (int i = 0; i <= D; i++) {
        if (i < D) {
            auto s_i = getSortiesFromLR(i, -1, all_sortie);
            for (auto &s:s_i) {
                start_with_i[i].push_back(s.index);
            }
        }
        if (i > 0) {
            auto s_j = getSortiesFromLR(i, -1, all_sortie);
            for (auto &s:s_j) {
                end_with_j[i].push_back(s.index);
            }
        }
    }

    std::unordered_map<std::pair<int, int>, std::vector<MultivisitTuple>, PairHash> start_i_end_j;
    for (int i = 0; i < D; i++) {
        for (int j = 1; j <= D; j++) {
            if (i != j) {
                start_i_end_j[std::make_pair(i, j)] = getSortiesFromLR(i, j, all_sortie);
            }
        }
    }
    std::vector<std::vector<int>> serve_h(D);
    for (auto &H:all_sortie) {
        for (int h:instance->c_prime) {
            if (exist(H.serve_customer, h)) {
                serve_h[h].push_back(H.index);
            }
        }
    }



    /// Variable declaration
    // X^i_k (binary variable) và nhận giá trị một tương ứng với đỉnh thứ k của
    //đường đi của vehicle là i; k \in 1..n;
    IloArray<IloBoolVarArray> X(env, K + 1);
    for (int k = 1; k <= K; k++) {
        X[k] = IloBoolVarArray(env, D + 1);
        for (int i = 0; i <= D; i++) {
            X[k][i] = IloBoolVar(env);
            model.add(X[k][i]);
            auto v_name = "X_" + std::to_string(k) + "_" + std::to_string(i);
            //std::cout << v_name << std::endl;
            X[k][i].setName(v_name.c_str());

        }
        if (k > 1) {
            model.add(X[k][0] == 0);
        }
    }

    model.add(X[1][0] == 1).setName("First stage must be source depot");

    // x^k_(ij) (binary variable) và nhận giá trị một nếu Xk
    // mô ta cạnh nối 2 đỉnh liên tiếp trên đường đi.
    IloArray<IloArray<IloBoolVarArray> > x(env, K_arc + 1);
    for (int k = 1; k <= K_arc; k++) {
        x[k] = IloArray<IloBoolVarArray>(env, D);
        for (int i = 0; i < D; i++) {
            x[k][i] = IloBoolVarArray(env, D + 1);
            for (int j = 1; j <= D; j++) {
                if (i != j) {
                    x[k][i][j] = IloBoolVar(env);
                    auto v_name = "x_" + std::to_string(k) + "_" + std::to_string(i) + "_" + std::to_string(j);
                    x[k][i][j].setName(v_name.c_str());
                }
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
        if (heavy != D) {
            model.add(phi[heavy] == 0);
        }
    }

    IloArray<IloBoolVarArray> W(env, K+1);
    for (int kp = 2; kp <= K; kp++) {
        W[kp] = IloBoolVarArray(env, all_sortie.size()+1);
        for (auto &H:all_sortie) {
                W[kp][H.index] = IloBoolVar(env);
        }
    }

    IloArray<IloBoolVarArray> Y(env, K+1);
    for (int k = 1; k < K; k++) {
        Y[k] = IloBoolVarArray(env, all_sortie.size()+1);
        for (auto &H:all_sortie) {
            Y[k][H.index] = IloBoolVar(env);
        }
    }

    // arrival\departure variables a and d.
    IloNumVarArray a(env, K + 1);
    IloNumVarArray d(env, K + 1);
    for (int k = 1; k <= K; k++) {
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

    // $R_{k} = \sum_{k'}Z_{kk'}$: các đoạn bắt đầu từ k (C23)
    IloBoolVarArray R(env, K + 1);
    for (int k = 1; k < K; k++) {
        R[k].setName(("R_" + std::to_string(k)).c_str());
    }

    //// aux var Z_{k, k_p, h}: sortie launch from k and rendezvous at k_p.
    IloArray<IloArray<IloBoolVarArray> > Z(env, S+1);
    for (int H = 0; H < S; H++) {
        Z[H] = IloArray<IloBoolVarArray>(env, K);
        for (int k = 1; k < K; k++) {
            Z[H][k] = IloBoolVarArray(env, K + 1);
            for (int k_p = k + 1; k_p <= K; k_p++) {
                Z[H][k][k_p] = IloBoolVar(env);
                auto v_name = "Z_" + std::to_string(H) + "_" + std::to_string(k) + "_" + std::to_string(k_p);
                Z[H][k][k_p].setName(v_name.c_str());
            }
        }
    }
    //// aux var z_{k, k_p}: sortie launch from k and rendezvous at k_p.
    IloArray<IloBoolVarArray> z(env, K);
    for (int k = 1; k < K; k++) {
        z[k] = IloBoolVarArray(env, K + 1);
        for (int k_p = k + 1; k_p <= K; k_p++) {
            z[k][k_p] = IloBoolVar(env);
            auto v_name = "z_" + std::to_string(k) + "_" + std::to_string(k_p);
            z[k][k_p].setName(v_name.c_str());
        }
    }

    ////-----------------------------------------------------------------------------------------------
    // WARMSTART CONFIG: using original TSP solution.
    // Obtained with MTZ formulation.
    // On variable X.
    if (use_tsp_as_warmstart) {
        auto tsp_solution = TSP_MTZ(tau);
        std::cout << tsp_solution.getSize() << std::endl;
        IloNumVarArray X_warm_var(env);
        IloNumArray X_warm_val(env);
        for (int k = 1; k <= K - 1; k++) {
            for (int i = 0; i <= D; i++) {
                X_warm_var.add(X[k][i]);
                if (tsp_solution[k - 1] == i) {
                    X_warm_val.add(true);
                    std::cout << "Warmstart X[" << k << "][" << i << "] == 1" << std::endl;
                } else {
                    X_warm_val.add(false);
                }
            }
        }
        cplex.addMIPStart(X_warm_var, X_warm_val);
        X_warm_var.end();
        X_warm_val.end();
    }
    ////-----------------------------------------------------------------------------------------------

    //// Sortie maximum stage gap calculation.
    /// Find maximum stage gap that a sortie can start and rendezvous.
    /// For each stage gap in increasing order (k' - k), find the minimum tour length from stage k to k'.
    /// Revisit(s) are also considered.
    /// If the tour length > dtl - sr => there can be no sortie with stage gap greater or equal k' - k.
    bool used_stage_gap = false;
    int min_stage_gap = 0;
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "Calculate max stage gap for sortie.... " << std::endl;
    for (int k = 1; k < K; k++) {
        for (int kp = k + 1; kp <= K; kp++) {
            if (kp > k + 6) {
                goto after_z_cons;
            }
            double smallest_tour = smallest_tour_length(kp - k, tau, V);
            if (smallest_tour > dtl - sr) {
                std::cout << "Stage gap of " << kp - k << " with smallest length = " << smallest_tour <<
                        " violated endurance constraint!" << std::endl;
                std::cout << "---------------------------------------------" << std::endl;
                min_stage_gap = kp - k;
                goto after_z_cons;
            }
        }
    }
    after_z_cons:
        if (min_stage_gap == 0) {
            std::cout << "Stage gap calculation consumes too much memory. No constraint was added." << std::endl;
            std::cout << "---------------------------------------------" << std::endl;
        }
    if (min_stage_gap != 0) {
        used_stage_gap = true;
        for (int k = 1; k < K; k++) {
            for (int kp = k + min_stage_gap; kp <= K; kp++) {
                model.add(z[k][kp] == 0);
                std::cout << "Variable z[" << k << "][" << kp << "] was set to 0." << std::endl;
                for (int H = 0; H < S; H++) {
                    model.add(Z[H][k][kp] == 0);
                    // IloExpr start_stage_k(env), end_stage_kp(env);
                    // for (int i = 0; i < D; i++) {
                    //     if (i != h) {
                    //         start_stage_k += Y[k][i][h];
                    //         // model.add(Z[h][k][kp] + Y[k][i][h] <= 1);
                    //
                    //     }
                    // }
                    // for (int j = 1; j <= D; j++) {
                    //     if (j != h) {
                    //         end_stage_kp += W[kp][j][h];
                    //         // model.add(Z[h][k][kp] + W[kp][j][h] <= 1);
                    //     }
                    // }
                    // std::string cname = "Can't serve customer " + std::to_string(h) + " start at stage " + std::to_string(k) + ", end at stage " + std::to_string(kp);
                    // // Looks like quicker lower bound improvements with this constraints.
                    // model.add(start_stage_k + end_stage_kp <= 1).setName(cname.c_str());
                }
            }
        }
    }

    // Constraint definition
    model.add(X[1][O] == 1);
    IloExpr c2(env);
    for (int k = 2; k <= K; k++) {
        c2 += X[k][D];
    }
    model.add(c2 == 1);

    //  C3
    for (int k = 1; k < K; k++) {
        for (int i:c_s) {
            IloExpr c3_1(env);
            for (int j:c_t) {
                if (i != j) {
                    c3_1 += x[k][i][j];
                }
            }
            model.add(X[k][i] == c3_1);
        }
    }

    // C3_2
    for (int k = 2; k <= K; k++) {
        for (int i:c_t) {
            IloExpr c4_2(env);
            for (int j:c_s) {
                if (i != j) {
                    c4_2 += x[k-1][j][i];
                }
            }
            model.add(X[k][i] == c4_2);
        }
    }

    // C4
    for (int k = 1; k < K; k++) {
        IloExpr lhs(env), rhs(env);
        for (auto &H:all_sortie) {
            for (int kp = k+1; kp <= K; kp++) {
                lhs += Z[H.index][k][kp];
            }
            rhs += Y[k][H.index];
        }
        model.add(lhs == rhs);
    }

    // C5
    for (int kp = 2; kp <= K; kp++) {
        IloExpr lhs(env), rhs(env);
        for (auto &H:all_sortie) {
            for (int k = 1; k < K; k++) {
                lhs += Z[H.index][k][kp];
            }
            for (int i:c_s) {
                rhs += W[kp][H.index];
            }
        }
        model.add(lhs == rhs);
    }

    // C6
    for (int h_i = 0; h_i < all_sortie.size(); h_i++) {
        IloExpr start_H(env), end_H(env);
        for (int k = 1; k < K; k++) {
            start_H += Y[k][h_i];
        }
        for (int kp = 2; kp <= K; kp++) {
            end_H += W[kp][h_i];
        }
        model.add(start_H == end_H);
        model.add(start_H <= 1);
    }

    for (int k = 1; k < K; k++) {
        IloExpr sum_z(env);
        for (int kp = k+1; kp <= K; kp++) {
            IloExpr sum_Z(env);
            for (auto &H:all_sortie) {
                sum_Z += Z[H.index][k][kp];
            }
            sum_z += z[k][kp];
            model.add(z[k][kp] == sum_Z);
        }
        model.add(sum_z <= 1);
    }

    for (int h:C) {
        IloExpr sumY(env), sumW(env);
        for (int k = 1; k < K; k++) {
            for (int h_i:serve_h[h]) {
                sumY += Y[k][h_i];
            }
        }
        for (int kp = 2; kp <= K; kp++) {
            for (int h_i:serve_h[h]) {
                sumY += W[kp][h_i];
            }
        }
        model.add(phi[h] == sumY);
        model.add(phi[h] == sumW);

        // all must be served constraint
        IloExpr sumX(env);
        for (int k = 2; k < K; k++) {
            sumX += X[k][h];
        }
        model.add(sumX + phi[h] >= 1);
    }

    for (int k = 1; k < K; k++) {
        for (int i:c_s) {
            IloExpr sumY(env);
            for (auto h_i:start_with_i[i]) {
                sumY += Y[k][h_i];
            }
            model.add(X[k][i] >= sumY);
        }
    }
    for (int kp = 2; kp <= K; kp++) {
        for (int j:c_t) {
            IloExpr sumW(env);
            for (auto h_i:end_with_j[j]) {
                sumW += W[kp][h_i];
            }
            model.add(X[kp][j] >= sumW);
        }
    }

    for (int k = 1; k < K; k++) {
        IloExpr sumX(env), sum_z(env);
        for (int i:c_s) {
            sumX += X[k][i];
        }
        for (int kp = k+1; kp <= K; kp++) {
            sum_z += z[k][kp];
        }
        model.add(sumX >= sum_z);
    }
    for (int kp = 2; kp < K; kp++) {
        IloExpr sumX(env), sum_z(env);
        for (int j:c_t) {
            sumX += X[kp][j];
        }
        for (int k = 1; k < kp; k++) {
            sum_z += z[k][kp];
        }
        model.add(sumX >= sum_z);
    }

    // crossing constraint?
    for (int k = 1; k < K; k++) {
        IloExpr expr(env);
        for (int k_p = k + 1; k_p <= K; k_p++) {
            expr += z[k][k_p];
        }
        model.add(R[k] == expr).setName(("C23_" + std::to_string(k)).c_str());
    }

    // modified C7
    for (int k = 1; k <= K - 1; k++) {
        for (int k_p = k + 1; k_p <= K; k_p++) {
            for (int l = k + 1; l < k_p; l++) {
                // tranh drone bay cac doan giao nhau.
                if (k < l) {
                    model.add(z[k][k_p] + R[l] <= 1).setName(("C7m_" + std::to_string(k)
                                                              + "_" + std::to_string(k_p) + "_" + std::to_string(l))
                        .c_str());
                }
            }
        }
    }

    // time synchronization constraint
    for (int k = 1; k < K; k++) {
        IloExpr sum_l(env);
        for (int i:c_s) {
            for (int j:c_t) {
                if (i != j) {
                    sum_l += x[k][i][j] * tau[i][j];
                }
            }
        }
        model.add(a[k+1] >= d[k] + sum_l);
    }

    for (int k = 1; k < K; k++) {
        for (int kp = k+1; kp <= K; kp++) {

        }
    }

    model.add(IloMinimize(env, d[K]));


    cplex.solve();
    return Result{cplex.getObjValue(), 0, 1000.0, 0, used_stage_gap};
}
Result Solver::stage_based_fstsp(int n_thread, double dtl, double sl, double sr, bool use_tsp_as_warmstart) const {
    try {
        auto tau = instance->tau;
        auto tau_prime = instance->tau_prime;
        auto n = instance->num_node;
        std::vector<int> C;
        std::vector<int> V;
        std::vector<int> c_s;
        std::vector<int> c_t;
        for (int i = 0; i < n + 1; i++) {
            if (i != 0 && i != n) {
                C.push_back(i);
            }
            if (i != 0) {
                c_t.push_back(i);
            }
            if (i != n) {
                c_s.push_back(i);
            }
            V.push_back(i);
        }

        IloEnv env;
        IloModel model(env);
        IloCplex cplex(model);
        cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 0);

        auto O = 0;
        auto D = n;
        auto K = n + 1;

        IloArray<IloBoolVarArray> X(env, K + 1);
        for (int k = 1; k <= K; k++) {
            X[k] = IloBoolVarArray(env, D + 1);
            for (int i = 0; i <= D; i++) {
                X[k][i] = IloBoolVar(env);
                auto v = "X_" + std::to_string(k) + "_" + std::to_string(i);
                X[k][i].setName(v.c_str());
            }
            if (k > 1) {
                std::string cname = "X_k_" + std::to_string(k) + "_O == 0";
                model.add(X[k][O] == 0).setName(cname.c_str());
            }
        }
        model.add(X[K][D] == 1).setName("X_K_D==1");

        IloArray<IloBoolVarArray> x(env, D);
        for (int i:c_s) {
            x[i] = IloBoolVarArray(env, D+1);
            for (int j:c_t) {
                if (i != j) {
                    auto v_name = "x_" + std::to_string(i) + "_" + std::to_string(j);
                    x[i][j] = IloBoolVar(env);
                    x[i][j].setName(v_name.c_str());
                }
            }
        }

        IloArray<IloBoolVarArray> Y(env, K);
        for (int k = 1; k < K; k++) {
            Y[k] = IloBoolVarArray(env, D);
            for (int h:C) {
                Y[k][h] = IloBoolVar(env);
                auto v_name = "Y_" + std::to_string(k) + "_" + std::to_string(h);
                Y[k][h].setName(v_name.c_str());
                if (exist(instance->heavy, h)) {
                    std::string cname = "Y_k_" + std::to_string(k) + "_h_" + std::to_string(h) + "==0";
                    model.add(Y[k][h] == 0).setName(cname.c_str());
                }
            }
        }

        IloArray<IloBoolVarArray> W(env, K+1);
        for (int kp = 2; kp <= K; kp++) {
            W[kp] = IloBoolVarArray(env, D);
            for (int h:C) {
                W[kp][h] = IloBoolVar(env);
                auto v_name = "W_" + std::to_string(kp) + "_" + std::to_string(h);
                W[kp][h].setName(v_name.c_str());
                if (exist(instance->heavy, h)) {
                    std::string cname = "W_kp_" + std::to_string(kp) + "_h_" + std::to_string(h) + "==0";
                    model.add(W[kp][h] == 0).setName(cname.c_str());
                }
            }
        }
        IloArray<IloBoolVarArray> A(env, D);
        for (int i:c_s) {
            A[i] = IloBoolVarArray(env, D);
            for (int h:C) {
                if (i != h) {
                    A[i][h] = IloBoolVar(env);
                    auto v_name = "A_" + std::to_string(i) + "_" +  std::to_string(h);
                    A[i][h].setName(v_name.c_str());
                    if (exist(instance->heavy, h)) {
                        std::string cname = "A_i_" + std::to_string(i) + "_h_" + std::to_string(h) + "==0_heavy";
                        model.add(A[i][h] == 0).setName(cname.c_str());
                    }
                    if (tau_prime[i][h] > dtl - sr) {
                        std::string cname = "A_i_" + std::to_string(i) + "_h_" + std::to_string(h) + "==0_>dtl-sr";
                        model.add(A[i][h] == 0).setName(cname.c_str());
                    }
                }

            }
        }
        IloArray<IloBoolVarArray> B(env, D+1);
        for (int j:c_t) {
            B[j] = IloBoolVarArray(env, D);
            for (int h:C) {
                if (j != h) {
                    B[j][h] = IloBoolVar(env);
                    auto v_name = "B_" + std::to_string(j) + "_" + std::to_string(h);
                    B[j][h].setName(v_name.c_str());
                    if (exist(instance->heavy, h)) {
                        std::string cname = "B_j=" + std::to_string(j) + "_h_" + std::to_string(h) + "==0_heavy";
                        model.add(B[j][h] == 0);
                    }
                    if (tau_prime[h][j] > dtl - sr) {
                        std::string cname = "B_j=" + std::to_string(j) + "_h_" + std::to_string(h) + "==0_>dtl-sr";
                        model.add(B[j][h] == 0).setName(cname.c_str());
                    }
                }
            }
        }

        IloBoolVarArray phi(env, D);
        for (int h:C) {
            phi[h] = IloBoolVar(env);
            auto v_name = "phi_" + std::to_string(h);
            phi[h].setName(v_name.c_str());
            if (exist(instance->heavy, h)) {
                std::string cname = "h_" + std::to_string(h) + "_to_heavy";
                model.add(phi[h] == 0).setName(cname.c_str());
            }
        }
        IloArray<IloBoolVarArray> z(env, K);
        for (int k = 1; k < K; k++) {
            z[k] = IloBoolVarArray(env, K+1);
            for (int kp = k+1; kp <= K; kp++) {
                auto v_name = "z_" + std::to_string(k) + std::to_string(kp);
                z[k][kp].setName(v_name.c_str());
                z[k][kp] = IloBoolVar(env);
            }
        }

        IloNumVarArray b(env, D+1);
        IloNumVarArray e(env, D+1);
        for (int i = 0; i <= D; i++) {
            auto v_name = "b_" + std::to_string(i);
            b[i] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
            b[i].setName(v_name.c_str());
            e[i] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
            v_name = "e_" + std::to_string(i);
            e[i].setName(v_name.c_str());
        }

        // Constraint 2
        model.add(X[1][O] == 1).setName("start_at_O");

        // Constraint 3
        IloExpr c3(env);
        for (int k = 2; k <= K; k++) {
            c3 += X[k][D];
        }
        model.add(c3 == 1).setName("back_to_D");

        // Constraint 4
        for (int h:instance->c_prime) {
            IloExpr sum_A(env), sum_B(env);
            for (int i:c_s) {
                if (i != h) {
                    sum_A += A[i][h];
                }
            }
            for (int j:c_t) {
                if (j != h) {
                    sum_B += B[j][h];
                }
            }
            std::string c_name1 = "C4_lhs_h_" + std::to_string(h);
            model.add(sum_A == phi[h]).setName(c_name1.c_str());
            c_name1 = "C4_rhs_h_" + std::to_string(h);
            model.add(sum_B == phi[h]).setName(c_name1.c_str());
        }

        // Constraint 5
        for (int h:instance->c_prime) {
            for (int i:C) {
                if (i != h) {
                    std::string cname = "C5_h_" + std::to_string(h) + "_i_" + std::to_string(i);
                    model.add(A[i][h] + B[i][h] <= phi[h]).setName(cname.c_str());
                }
            }
        }

        // Constraint 6
        for (int h:instance->c_prime) {
            IloExpr sum_Y(env), sum_W(env);
            for (int k = 1; k < K; k++) {
                sum_Y += Y[k][h];
            }
            for (int kp = 2; kp <= K; kp++) {
                sum_W += W[kp][h];
            }
            std::string cname = "C6_lhs_h_" + std::to_string(h);
            model.add(sum_Y == phi[h]).setName(cname.c_str());
            cname = "C6_rhs_h_" + std::to_string(h);
            model.add(sum_W == phi[h]).setName(cname.c_str());
        }

        // Constraint 7
        for (int h:instance->c_prime) {
            for (int k = 2; k < K; k++) {
                std::string cname = "C7_h_" + std::to_string(h) + "_k_" + std::to_string(k);
                model.add(Y[k][h] + W[k][h] <= phi[h]).setName(cname.c_str());
            }
        }

        // Constraint 8
        for (int i:c_s) {
            IloExpr lhs(env), rhs(env);
            for (int k = 1; k < K; k++) {
                lhs += X[k][i];
            }
            for (int h:instance->c_prime) {
                if (i != h) {
                    rhs += A[i][h];
                }
            }
            std::string cname = "C8_i_" +std::to_string(i);
            model.add(lhs >= rhs).setName(cname.c_str());
        }

        // Constraint 9
        for (int j:c_t) {
            IloExpr lhs(env), rhs(env);
            for (int kp = 2; kp <= K; kp++) {
                lhs += X[kp][j];
            }
            for (int h:instance->c_prime) {
                if (j != h) {
                    rhs += B[j][h];
                }
            }
            std::string cname = "C9_j=" +std::to_string(j);
            model.add(lhs >= rhs).setName(cname.c_str());
        }

        // Constraint 10
        for (int k = 1; k < K; k++) {
            IloExpr lhs(env), rhs(env);
            for (int i:c_s) {
                lhs += X[k][i];
            }
            for (int h:instance->c_prime) {
                rhs += Y[k][h];
            }
            std::string cname = "C10_k_" + std::to_string(k);
            model.add(lhs >= rhs).setName(cname.c_str());
        }

        // Constraint 11
        for (int kp = 2; kp <= K; kp++) {
            IloExpr lhs(env), rhs(env);
            for (int j:c_t) {
                lhs += X[kp][j];
            }
            for (int h:instance->c_prime) {
                rhs += W[kp][h];
            }
            std::string cname = "C11_kp_" + std::to_string(kp);
            model.add(lhs >= rhs).setName(cname.c_str());
        }

        // Constraint 12
        for (int h:instance->c_prime) {
            IloExpr s1(env), s2(env);
            for (int i:c_s) {
                if (i != h) {
                    s1 += A[i][h] * tau_prime[i][h];
                }
            }
            for (int j:c_t) {
                if (j != h) {
                    s2 += B[j][h] * tau_prime[h][j];
                }
            }
            std::string cname = "C12_h_" + std::to_string(h);
            model.add(s1 + s2 <= (dtl - sr) * phi[h]).setName(cname.c_str());
        }

        // Constraint 13
        for (int k = 1; k < K; k++) {
            IloExpr lhs(env), rhs(env);
            for (int kp = k+1; kp <= K; kp++) {
                lhs += z[k][kp];
            }
            for (int h:instance->c_prime) {
                rhs += Y[k][h];
            }
            std::string cname = "C13_k_" + std::to_string(k) + "_lhs=rhs";
            model.add(lhs == rhs).setName(cname.c_str());
            cname = "C13_k_" + std::to_string(k) + "<=1";
            model.add(rhs <= 1);
        }

        // Constraint 14
        for (int kp = 2; kp <= K; kp++) {
            IloExpr lhs(env), rhs(env);
            for (int k = 1; k < kp; k++) {
                lhs += z[k][kp];
            }
            for (int h:instance->c_prime) {
                rhs += W[kp][h];
            }
            std::string cname = "C14_kp_" + std::to_string(kp) + "_lhs=rhs";
            model.add(lhs == rhs).setName(cname.c_str());
            cname = "C14_kp_" + std::to_string(kp) + "<=1";
            model.add(rhs <= 1);
        }

        // Constraint 15
        for (int l = 1; l < K; l++) {
            IloExpr sum(env);
            for (int k = 1; k <= l; k++) {
                for (int kp = k+1; kp <= K; kp++) {
                    if (k <= l && l < kp) {
                        sum += z[k][kp];
                    }
                }
            }
            std::string cname = "C15_l=" + std::to_string((l));
            model.add(sum <= 1).setName(cname.c_str());
        }

        // Constraint 16
        for (int l = 2; l <= K; l++) {
            IloExpr sum(env);
            for (int k = 1; k < l; k++) {
                for (int kp = l; kp <= K; kp++) {
                    if (k < l && l <= kp) {
                        sum += z[k][kp];
                    }
                }
            }
            std::string cname = "C16_l=" + std::to_string((l));
            model.add(sum <= 1).setName(cname.c_str());
        }
        // Constraint 17
        for (int h:C) {
            IloExpr sum_X(env);
            for (int k = 2; k < K; k++) {
                sum_X += X[k][h];
            }
            std::string cname = "C17_h_" + std::to_string((h));
            model.add(sum_X + phi[h] == 1).setName(cname.c_str());
        }

        // Constraint 18
        // Tighter
        for (int k = 1; k <= K; k++) {
            IloExpr sum_X(env);
            for (int i = 0; i <= D; i++) {
                sum_X += X[k][i];
            }
            std::string cname = "C18_k_" +std::to_string(k);
            model.add(sum_X <= 1).setName(cname.c_str());
        }

        // Constraint 19/20 rewrite
        // Leaving i
        for (int i:C) {
            IloExpr sum_leave(env), sum_enter(env);
            IloExpr lhs(env);
            for (int j:c_t) {
                if (i != j) {
                    sum_leave += x[i][j];
                }
            }
            for (int j:c_s) {
                if (i != j) {
                    sum_enter += x[j][i];
                }
            }
            for (int k = 2; k < K; k++) {
                lhs += X[k][i];
            }
            std::string eq_deg = "C19_20_i_" + std::to_string(i);
            model.add(sum_leave == sum_enter).setName(eq_deg.c_str());
            eq_deg = "C19_20_i_" + std::to_string(i) + "_deg<=1";
            model.add(sum_leave <= 1).setName(eq_deg.c_str());
            eq_deg = "C19_20_i_" + std::to_string(i) + "_stage_presence";
            model.add(sum_leave == lhs).setName(eq_deg.c_str());
        }

        // Constraint 21
        for (int k = 1; k < K; k++) {
            for (int i:c_s) {
                for (int j:c_t) {
                    if (i != j) {
                        std::string cname = "C21_k_" + std::to_string(k) + "_i_" + std::to_string(i) + "_j=" + std::to_string(j);
                        model.add(X[k+1][j] + 1 - x[i][j] >= X[k][i]).setName(cname.c_str());
                    }
                }
            }
        }

        // Constraint 23
        model.add(b[O] == 0);
        model.add(e[O] == 0);

        // Constraint 24
        model.add(IloMinimize(env, e[D])).setName("Objective");

        // Constraint 25
        for (int i = 1; i <= D; i++) {
            model.add(b[i] <= e[i]);
        }
        auto M = 1e5;
        // Constraint 26
        for (int i:c_s) {
            for (int j:c_t) {
                if (i != j) {
                    std::string cname = "C26_i_" + std::to_string(i) + "_j=" + std::to_string(j);
                    model.add(b[j] + M * (1 - x[i][j]) >= e[i] + tau[i][j] * x[i][j]).setName(cname.c_str());
                }
            }
        }

        // Constraint 27
        for (int i:c_s) {
            for (int h:instance->c_prime) {
                if (i != h && tau_prime[i][h] <= dtl - sr) {
                    std::string cname = "C27_i_" + std::to_string(i) + "_h_" + std::to_string(h);
                    model.add(b[h] + M * (1 - A[i][h]) >= e[i] + tau_prime[i][h] * A[i][h]).setName(cname.c_str());
                }
            }
        }

        // Constraint 28
        for (int j:c_t) {
            for (int h:instance->c_prime) {
                if (j != h && tau_prime[h][j] <= dtl - sr) {
                    std::string cname = "C28_j=" + std::to_string(j) + "_h_" + std::to_string(h);
                    model.add(e[j] + M * (1 - B[j][h]) >= e[h] + tau_prime[h][j] * B[j][h] + sr).setName(cname.c_str());
                }
            }
        }

        // Constraint 29
        for (int i:c_s) {
            for (int j:c_t) {
                if (i != j) {
                    IloExpr sum1(env), sum2(env);
                    for (int h:instance->c_prime) {
                        if (tau_prime[i][h] <= dtl - sr && tau_prime[h][j] <= dtl - sr) {
                            sum1 += tau_prime[i][h] * A[i][h] + tau_prime[h][j] * B[j][h];
                            sum2 += A[i][h] + B[j][h];
                        }
                    }
                    std::string cname = "C29_i_" + std::to_string(i) + "_j=" + std::to_string(j);
                    model.add(b[j] <= e[i] + sum1 + M * (2 - sum2)).setName(cname.c_str());
                }
            }
        }

        IloExpr lb_truck(env), lb_drone(env);
        for (int i:c_s) {
            for (int j:c_t) {
                if (i != j) {
                    lb_truck += x[i][j] * tau[i][j];
                }
            }
        }

        for (int i = 0; i <= D; i++) {
            for (int h:instance->c_prime) {
                if (i != h) {
                    if (i < D) {
                        lb_drone += A[i][h] * tau_prime[i][h];
                    }
                    if (i > 0) {
                        lb_drone += B[i][h] * tau_prime[h][i];
                    }
                }
            }
        }
        model.add(e[D] >= lb_truck).setName("e[D]>=truck_tour");
        model.add(e[D] >= lb_drone).setName("e[D]>=drone_tour");
        cplex.exportModel("model.lp");
        cplex.solve();
        std::cout << "Total travel time: " << cplex.getObjValue() << std::endl;
        std::cout << "Truck stages:" << std::endl;
        for (int k = 1; k <= K; k++) {
            for (int i = 0; i <= D; i++) {
                if (cplex.getValue(X[k][i]) == 1) {
                    std::cout << "Stage " << k << " at " << i << std::endl;
                }
            }
        }
        std::cout << "Truck arcs:" << std::endl;
        for (int i:c_s) {
            for (int j:c_t) {
                if (i != j) {
                    if (abs(cplex.getValue(x[i][j]) - 1) < 1e-5) {
                        std::cout << "Truck moves from " <<  i << " to " << j << ", cost = " << tau[i][j] << std::endl;
                    }
                }
            }
        }
        std::cout << "Drone served customers:" << std::endl;
        for (int h:instance->c_prime) {
            if (abs(cplex.getValue(phi[h]) - 1) < 1e-5) {
                std::cout << "Customer " << h << " served by drone." << std::endl;
                for (int i:c_s) {
                    if (cplex.getValue(A[i][h]) == 1) {
                        for (int k = 1; k < K; k++) {
                            if (cplex.getValue(Y[k][h]) == 1) {
                                std::cout << "Launch at stage " << k << " at node " << i << std::endl;
                            }
                        }
                    }
                }
                for (int j:c_t) {
                    if (cplex.getValue(B[j][h]) == 1) {
                        for (int k = 2; k <= K; k++) {
                            if (cplex.getValue(W[k][h]) == 1) {
                                std::cout << "Rendezvous at stage " << k << " at node " << j << std::endl;
                            }
                        }
                    }
                }
            }
        }
        std::cout << "Timing: " << std::endl;
        for (int i = 0; i <= D; i++) {
            std::cout << "Arrival at " << i << " is: " << cplex.getValue(b[i]) << std::endl;
            std::cout << "Departure at " << i << " is: " << cplex.getValue(e[i]) << std::endl;
        }
        return Result{cplex.getObjValue(), 0, 1000.0, 0, true};
    } catch (IloException &e) {
        std::cout << "Exception: " << e.getMessage() << std::endl;
    }
}
