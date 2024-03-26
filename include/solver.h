//
// Created by cuong on 18/01/2024.
//

#ifndef UMV_FSTSP_SOLVER_H
#define UMV_FSTSP_SOLVER_H

#include <memory>
#include "instance.h"
#include<ilcplex/ilocplex.h>
#include <utility>
#include<cassert>
#include <map>
#include <random>
#include <unordered_set>

// On a new feasible solution callback.
class BranchAndCutCallback : public IloCplex::UserCutCallbackI {
public:
    // Constructor
    BranchAndCutCallback(IloEnv env, IloModel &model, IloArray<IloBoolVarArray> &X,
                         IloArray<IloArray<IloBoolVarArray> > &x, IloBoolVarArray &phi,
                         IloArray<IloArray<IloBoolVarArray> > &Z,
                         IloArray<IloBoolVarArray> &z,
                         IloArray<IloArray<IloBoolVarArray> > &Y,
                         IloArray<IloArray<IloBoolVarArray> > &W,
                         int &K,
                         int &D,
                         double &best_objective,
                         std::vector<int> &C,
                         std::vector<std::vector<double>> &tau,
                         std::vector<std::vector<double>> &tau_prime)
        : IloCplex::UserCutCallbackI(env), model(model), X(X), x(x), phi(phi), Z(Z), z(z), Y(Y), W(W), K(K), D(D), best_objective(best_objective), C(C), tau(tau), tau_prime(tau_prime) {
    }

protected:
    // Override the main callback method
    void main() override {
        if (hasIncumbent() && getIncumbentObjValue() < best_objective) {
            // Retrieve all values.
            int aK = K-1;
            IloNumArray2 X_val(getEnv(), K+1);
            for (int k = 1; k <= K; k++) {
                X_val[k] = IloNumArray(getEnv(), D+1);
                for (int i = 0; i <= D; i++) {
                    X_val[k][i] = getIncumbentValue(X[k][i]);
                }
            }
            IloNumArray3 x_val(getEnv(), K);
            for (int k = 1; k <= aK; k++) {
                x_val[k] = IloNumArray2(getEnv(), D);
                for (int i = 0; i < D; i++) {
                    x_val[k][i] = IloNumArray(getEnv(), D + 1);
                    for (int j = 1; j <= D; j++) {
                        if (i != j) {
                            x_val[k][i][j] = getIncumbentValue(x[k][i][j]);
                        }
                    }
                }
            }
            for (int k = 1; k < K; k++) {
                for (int kp = k+1; kp <= K; kp++) {
                    // for each couple of (k, k')
                    // calculate the tour length
                    // if tour length >= dtl - sr then there can't be no sortie performed
                    // in this particular tour.
                    double tour_length = 0;
                    for (int k_start = k; k_start < kp; k_start++) {
                        for (int i = 0; i < D; i++) {
                            for (int j = 0; j <= D; j++) {
                                if (i != j) {
                                    if (abs(x_val[k_start][i][j] - 1) < 1e-5) {
                                        tour_length += tau[i][j];
                                    }

                                }
                            }
                        }
                    }
                    if (tour_length >= 19) {
                        std::cout << "----------------Added z["<< k << "][" << kp << "] == 0" << std::endl;
                        model.add(z[k][kp] == 0);
                    }

                }
            }

            IloNumArray phi_val(getEnv(), D);
            for (int h: C) {
                phi_val[h] = getIncumbentValue(phi[h]);
            }

            IloArray<IloArray<IloNumArray> > Z_val(getEnv(), K);
            for (int h: C) {
                Z_val[h] = IloArray<IloNumArray>(getEnv(), K + 1);
                for (int k = 1; k <= K - 1; k++) {
                    Z_val[h][k] = IloNumArray(getEnv(), K + 1);
                    for (int k_p = k + 1; k_p <= K; k_p++) {
                            Z_val[h][k][k_p] = getIncumbentValue(Z[h][k][k_p]);
                            if (abs(Z_val[h][k][k_p] - 1) < 1e-5) {

                            }
                    }
                }
            }
            IloArray<IloNumArray> z_val(getEnv(), K);
            for (int k = 1; k <= K - 1; k++) {
                z_val[k] = IloNumArray(getEnv(), K + 1);
                for (int k_p = k + 1; k_p <= K; k_p++) {
                    z_val[k][k_p] = getIncumbentValue(z[k][k_p]);
                }
            }
            IloArray<IloArray<IloNumArray> > Y_val(getEnv(), K + 1), W_val(getEnv(), K + 1);
            for (int k = 1; k <= K; k++) {
                Y_val[k] = IloArray<IloNumArray>(getEnv(), D + 1);
                W_val[k] = IloArray<IloNumArray>(getEnv(), D + 1);

                for (int i = 0; i <= D; i++) {
                    Y_val[k][i] = IloNumArray(getEnv(), C.size() + 1);
                    W_val[k][i] = IloNumArray(getEnv(), C.size() + 1);
                    for (int h: C) {
                        if (i != h) {
                            Y_val[k][i][h] = getIncumbentValue(Y[k][i][h]);
                            W_val[k][i][h] = getIncumbentValue(W[k][i][h]);
                        }
                    }
                }
            }


        }
    }

    // Implement duplicateCallback method
    [[nodiscard]] IloCplex::CallbackI *duplicateCallback() const override {
        return new(getEnv()) BranchAndCutCallback(*this);
    }

private:
    IloArray<IloBoolVarArray> X;
    IloArray<IloArray<IloBoolVarArray> > x;
    IloBoolVarArray phi;
    IloArray<IloArray<IloBoolVarArray> > Z;
    IloArray<IloBoolVarArray> z;
    IloArray<IloArray<IloBoolVarArray> > Y;
    IloArray<IloArray<IloBoolVarArray> > W;
    IloModel model;
    double best_objective;
    int numProcedures;
    int K;
    int D;
    std::vector<std::vector<double>> tau;
    std::vector<std::vector<double>> tau_prime;
    std::vector<int> C;
};

class Sortie {
public:
    int target;
    double flight_time;
    int l;
    int r;
    std::vector<int> phi;
    std::vector<std::string> routes;

    Sortie(int target, int l, int r, std::vector<int> &phi);

    explicit Sortie(int target);
};

class Result {
public:
    double cost;
    std::vector<Sortie> sortie;
    double recalculated_cost;
    double time_spent;
    double revisit_count;
    bool used_stage_gap;

    Result() {
    }

    Result(double c, std::vector<Sortie> &st);

    Result(double c, double re_c, double time_spent, std::vector<Sortie> &st);

    Result(double c, double re_c, double time_spent, double revisit_count, bool used_stage_gap);
};
// Hash function for pairs of integers
struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        // Combine hashes of the pair's components
        std::hash<T1> hash1;
        std::hash<T2> hash2;
        return hash1(pair.first) ^ hash2(pair.second);
    }
};

class Solver {
public:
    std::shared_ptr<Instance> instance;

    explicit Solver(const std::shared_ptr<Instance> &inst) {
        instance = inst;
    }


    Result OriginalSolverCPLEX(int n_thread, int e);

    [[nodiscard]] Result mvdSolverWithLR(int n_thread, double e, double sl, double sr, bool use_tsp_as_warmstart) const;
    [[nodiscard]] Result stage_based_fstsp(int n_thread, double e, double sl, double sr, bool use_tsp_as_warmstart) const;
    Result Roberti2020(int n_thread, int e);

    Result Amico2021_3Index(int n_thread, int e);

    Result Amico2021_2Index(int n_thread, int e);

    [[nodiscard]] Result mFSTSPSolve(int n_thread, double e, double sl, double sr) const;

    [[nodiscard]] Result RV_FSTSP_MVD(int n_thread, double dtl, double sl, double sr, int L, bool use_tsp_as_warmstart) const;

    static IloNumArray TSP_MTZ(std::vector<std::vector<double> > &tau);

    static IloNumArray RevisitTSP(std::vector<std::vector<double> > &tau);
};
#endif //UMV_FSTSP_SOLVER_H
