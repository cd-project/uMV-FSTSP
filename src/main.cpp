#include <iostream>
#include "../include/instance.h"
#include "../include/solver.h"
#include "../include/gen_instance.h"
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
    if (argc == 4) {
        std::string n_c = argv[1];
        std::string dtl_str = argv[2];
        std::string v_str = argv[3];

        auto n_customer = stoi(n_c);
        auto dtl = stod(dtl_str);
        auto v = stoi(v_str);

        auto g = GenInstance(n_customer, dtl, v, true, 200);
        return EXIT_SUCCESS;
    }

    std::string folder_path;

    bool write = false;
    if (argc == 2) {
        folder_path ="/home/cuong/CLionProjects/uMV-FSTSP/50_dis_rand_ins/" + std::string(argv[1]);
        write = true;
    } else {
        folder_path ="/home/cuong/CLionProjects/uMV-FSTSP/50_dis_rand_ins/v23";
    }

    std::cout << "Instance name: " << folder_path << std::endl;
    std::cout << "Write arg val: " << write << std::endl;

    auto instance = std::make_shared<Instance>(folder_path, false, 0);
    auto solver = std::make_shared<Solver>(instance);
    auto result = solver->mvdSolverWithLR(20, 20, false);
    if (write) {
        std::cout << "In write mode" << std::endl;
        auto i_name_split = SplitStringWithDelimiter(folder_path, "/");
        auto i_name = i_name_split[i_name_split.size()-1];
        std::ofstream out("/home/cuong/CLionProjects/uMV-FSTSP/100_dis_rand_nows.csv", std::ios::app);
        if(!out.is_open()) {
            std::cout << "Error opening file!" << std::endl;
        }
        out << i_name <<"," << result.cost << "," << result.recalculated_cost << "," << result.time_spent << "," << result.revisit_count << "\n";
    }
}
