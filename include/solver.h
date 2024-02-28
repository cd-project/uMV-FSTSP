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
class MyCallback : public IloCplex::MIPCallbackI {
public:
    // Constructor
    MyCallback(IloEnv env, IloArray<IloBoolVarArray&> variables, int numProcedures)
            : IloCplex::MIPCallbackI(env), variables(variables), numProcedures(numProcedures) {}

protected:
    // Override the main callback method
    void main() override {
        for (int procedure = 0; procedure < numProcedures; ++procedure) {
            // Obtain current solution
            IloNumArray currentSolution(getEnv());
            getIncumbentValues(currentSolution, variables);

            // Do some processing on the current solution
            for (int i = 0; i < variables.getSize(); ++i) {
                if (rand() % 2 == 0) { // Randomly fix some variables to their current values
                    variables[i].setLB(currentSolution[i]); // Fix the lower bound
                    variables[i].setUB(currentSolution[i]); // Fix the upper bound
                } else { // Reset other variables
                    variables[i].setLB(-IloInfinity); // Reset lower bound
                    variables[i].setUB(IloInfinity);  // Reset upper bound
                }
            }

            // Solve for another new solution
            if (procedure < numProcedures - 1) {
                // Let the solver solve for another new solution
                solveFixedVariables();
            }

            // End of a procedure
            currentSolution.end();
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
    IloArray<IloBoolVarArray&> variables;
    int numProcedures;
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
    Result() {}
    Result(double c, std::vector<Sortie>& st);
    Result(double c, double re_c, std::vector<Sortie> &st);

};
class Solver {
public:
    std::shared_ptr<Instance> instance;
    explicit Solver(std::shared_ptr<Instance>& inst) {
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
};
#endif //UMV_FSTSP_SOLVER_H