#include <iostream>
#include "../include/instance.h"
#include "../include/solver.h"
#include <memory>
static std::vector <std::string> SplitStringWithDelimiter(const std::string& s, const std::string& delimiter) {
    std::vector<std::string> returnValue;
    std::string::size_type start = 0;
    std::string::size_type end = s.find(delimiter);

    while(end != std::string::npos) {
        returnValue.push_back(s.substr(start, end-start));
        start = end + 1;
        end = s.find(delimiter, start);
    }

    returnValue.push_back(s.substr(start));
    return returnValue;
}

int main(int argc, char**argv) {
    std::string folder_path;
    bool write = false;
    if (argc == 2) {
        folder_path = "/mnt/c/Users/ORG/CLionProjects/uMV-FSTSP/Murray_Chu_2015_test_data/FSTSP/FSTSP_10_customer_problems/" + std::string(argv[1]);
        write = true;
    } else {
        folder_path ="/mnt/c/Users/ORG/CLionProjects/uMV-FSTSP/Murray_Chu_2015_test_data/FSTSP/FSTSP_10_customer_problems/20140810T123437v5";
    }
    std::cout << "Instance name: " << folder_path << std::endl;

    auto instance = std::make_shared<Instance>(folder_path);
    auto solver = std::make_shared<Solver>(instance);
                    auto result = solver->mvdSolverCPLEX(
            20, 20);
    if (write) {
        auto i_name_split = SplitStringWithDelimiter(folder_path, "/");
        auto i_name = i_name_split[i_name_split.size()-1];
        std::ofstream out("/mnt/c/Users/ORG/CLionProjects/uMV-FSTSP/result_comp.csv", std::ios::app);
        out << i_name <<"," << result.cost << "," << result.sortie.size() << "\n";
    }
    return 0;
}
