#include <iostream>
#include "../include/instance.h"
#include "../include/solver.h"
#include <memory>
int main(int argc, char**argv) {
    std::string folder_path = argv[1];
    std::cout << "Instance name: " << folder_path << std::endl;
    folder_path = "/home/cuong/CLionProjects/uMV-FSTSP/Murray_Chu_2015_test_data/FSTSP/FSTSP_10_customer_problems/" + folder_path;
    auto instance = std::make_shared<Instance>(folder_path);
    auto solver = std::make_shared<Solver>(instance);
    solver->OriginalSolver(8);
    return 0;
}
