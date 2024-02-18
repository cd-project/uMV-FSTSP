//
// Created by cuong on 18/01/2024.
//

#ifndef UMV_FSTSP_SOLVER_H
#define UMV_FSTSP_SOLVER_H

#include <memory>
#include "instance.h"
#include <gurobi_c++.h>
#include<ilcplex/ilocplex.h>
class Sortie {
public:
    int target;
    double flight_time;
    int l;
    int r;
    std::vector<int> phi;
    std::vector<std::string> routes;
    explicit Sortie(int target, int l, int r, std::vector<int>& phi);
};
class Result {
public:
    double cost;
    std::vector<Sortie> sortie;
    Result() {}
    Result(double c, std::vector<Sortie>& st);
};
class Solver {
public:
    std::shared_ptr<Instance> instance;
    explicit Solver(std::shared_ptr<Instance>& inst) {
        instance = inst;
    }
    Result OriginalSolver(int n_thread, int e);
    Result uMVFSTSPSolver(int n_thread, int dtl);
    Result mvdSolver(int n_thread, int e);
    Result mvdSolverCPLEX(int n_thread, int e);
    Result mvdSolverCPLEXFewerVariables(int n_thread, int e);

};
#endif //UMV_FSTSP_SOLVER_H