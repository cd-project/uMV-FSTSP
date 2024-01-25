//
// Created by cuong on 18/01/2024.
//

#ifndef UMV_FSTSP_SOLVER_H
#define UMV_FSTSP_SOLVER_H

#include <memory>
#include "instance.h"
#include <gurobi_c++.h>

class Solver {
public:
    std::shared_ptr<Instance> instance;
    explicit Solver(std::shared_ptr<Instance> &inst) {
        instance = inst;
    }
    void OriginalSolver(int n_thread);
};
#endif //UMV_FSTSP_SOLVER_H
