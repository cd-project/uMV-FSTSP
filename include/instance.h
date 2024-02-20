//
// Created by cuong on 18/01/2024.
//

#ifndef UMV_FSTSP_INSTANCE_H
#define UMV_FSTSP_INSTANCE_H
#include <vector>
#include <memory>
#include <fstream>
#include <iostream>
#include <sstream>
class Instance {
public:
    int num_node;
    std::vector<std::vector<double>> tau;
    std::vector<std::vector<double>> tau_prime;
    double drone_speed;
    double e;
    double sl = 1;
    double sr = 1;
    std::vector<int> c_prime;
    std::vector<int> heavy;

    Instance(const std::string folder_path, bool roundDist);
//    Instance(const std::string &folder_path, string &dist_type);
//    static std::vector<std::string> SplitStringWithDelimiter(const std::string& s, const std::string& delimiter);
};
#endif //UMV_FSTSP_INSTANCE_H
