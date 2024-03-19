//
// Created by cuong on 18/01/2024.
//

#ifndef UMV_FSTSP_SOLVER_H
#define UMV_FSTSP_SOLVER_H

#include <memory>
#include "instance.h"
#include<ilcplex/ilocplex.h>
#include <utility>

// On a new feasible solution callback.
class MyCallback : public IloCplex::UserCutCallbackI {
public:
    // Constructor
    MyCallback(IloEnv env, IloModel &model, IloArray<IloBoolVarArray> X, double &best_objective,int numProcedures, int node_max_stage, int D)
            : IloCplex::UserCutCallbackI(env), model(model), X(X), best_objective(IloInfinity), numProcedures(numProcedures), node_max_stage(node_max_stage), D(D) {
        fixed = std::vector<int>(node_max_stage);
    }
protected:
    // Override the main callback method
    void main() override {
        if (hasIncumbent()) {
            double current_obj = getIncumbentObjValue();
            if (current_obj < best_objective) {
                best_objective = current_obj;
                // copy the current state
                IloArray<IloNumArray> X_val(getEnv(), node_max_stage+1);
                for (int k = 1; k <= node_max_stage; k++) {
                    X_val[k] = IloNumArray(getEnv(), D);
                    getIncumbentValues(X_val[k], X[k]);
                    for (int i = 0; i <= D; i++) {
                        if (X_val[k][i] == 1) {
                            std::cout << "Value of X[" << k << "][" << i << "] = " << X_val[k][i] << " " << std::endl;
                        }
                    }
                }


            }
        }
    }

    // Helper function to solve with fixed variables
    void solveFixedVariables() {
        try {
            IloEnv env = getEnv();
            IloCplex cplex(env);
            cplex.solve();
        } catch (IloException& e) {
            std::cerr << "Error: " << e << std::endl;
            throw;
        }
    }
    // Implement duplicateCallback method
    [[nodiscard]] IloCplex::CallbackI * duplicateCallback() const override {
        return new (getEnv()) MyCallback(*this);
    }

private:
    IloArray<IloBoolVarArray> X;
    IloModel &model;
    double best_objective;
    int numProcedures;
    int node_max_stage;
    int D;
    std::vector<int> fixed;
};

class Sortie {
public:
    int target;
    double flight_time;
    int l;
    int r;
    std::vector<int> phi;
    std::vector<std::string> routes;
    Sortie(int target, int l, int r, std::vector<int>& phi);
    explicit Sortie(int target);
};
class Result {
public:
    double cost;
    std::vector<Sortie> sortie;
    double recalculated_cost;
    double time_spent;
    double revisit_count;
    Result() {}
    Result(double c, std::vector<Sortie>& st);
    Result(double c, double re_c, double time_spent, std::vector<Sortie> &st);
    Result(double c, double re_c, double time_spent, double revisit_count);

};
class Solver {
public:

    std::shared_ptr<Instance> instance;
    explicit Solver(const std::shared_ptr<Instance>& inst) {
        instance = inst;
    }
    double sl = {1};
    double sr = {1};
    double dtl = {20};
    Result OriginalSolverCPLEX(int n_thread, int e);
//    Result mvdSolver(int n_thread, int e);
    Result mvdSolverCPLEX(int n_thread, int e);
    Result mvdSolverCPLEXFewerVariables(int n_thread, int e);
    Result mvdSolverWithLR(int n_thread, int e);
    Result HeuristicFixCallback(int n_thread, int e);
    Result SolverWithRandomTruckStageFixed(int n_thread, int e);
    Result Roberti2020(int n_thread, int e);
    Result Amico2021_3Index(int n_thread, int e);
    Result Amico2021_2Index(int n_thread, int e);
    Result mvdSolverRevisitDepotLRLoop(int n_thread, int e, bool replaceOnDepot);
    Result mFSTSPSolve(int n_thread, int e);

};
#endif //UMV_FSTSP_SOLVER_H