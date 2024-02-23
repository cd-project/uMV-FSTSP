//
// Created by cuong on 18/01/2024.
//

#ifndef UMV_FSTSP_SOLVER_H
#define UMV_FSTSP_SOLVER_H

#include <memory>
#include "instance.h"
#include<ilcplex/ilocplex.h>
#include <utility>
class MyCutCallback : public IloCplex::UserCutCallbackI {
public:
    MyCutCallback(IloEnv env) : IloCplex::UserCutCallbackI(env) {}

    // Override the main callback method
//    void main() override {
//        // Get current incumbent solution
//        IloNumArray vals(getEnv());
//        getValues(vals, x); // Assuming x is your decision variable array
//
//        // Evaluate current solution and add cutting planes as needed
//        if (/* condition for adding cutting planes */) {
//            addCut(/* cutting plane constraint */);
//        }
//
//        vals.end();
//    }
    void addCut1();
};


//// On a new feasible solution callback.
//class NewSolutionCallback : public IloCplex::MIPCallbackI {
//public:
//    // Constructor
//    NewSolutionCallback(IloEnv env) : IloCplex::MIPCallbackI(env) {}
//
//    // Override the main callback method
//    void main() override {
//        // Execute custom code every time a new solution is found
//        if (hasIncumbent()) {
//            // Get the incumbent solution
//            // Get all stage solution.
//            // Fixed a part of the solution with addMIPStart.
//            // Solve the flexible node stage value.
//
//            IloNumArray X_val(getEnv());
//            getIncumbentValues(X_val, X); // Assuming x is your decision variable array
//
//            // Process the solution, e.g., print it
//            std::cout << "New incumbent solution found: ";
//            for (int i = 0; i < vals.getSize(); ++i) {
//                std::cout << vals[i] << " ";
//            }
//            std::cout << std::endl;
//
//            vals.end();
//        }
//    }
//};

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
};
#endif //UMV_FSTSP_SOLVER_H