//
// Created by cuong on 18/01/2024.
//
#include <cmath>
#include "../include/instance.h"

static std::vector <std::string> SplitStringWithDelimiter(const std::string& s, const std::string& delimiter) {
    std::vector<std::string> returnValue;
    std::string::size_type start = 0;
    std::string::size_type end = s.find(delimiter);

    while (end != std::string::npos) {
        returnValue.push_back(s.substr(start, end - start));
        start = end + 1;
        end = s.find(delimiter, start);
    }

    returnValue.push_back(s.substr(start));
    return returnValue;
}

Instance::Instance(const std::string file_path, bool roundDist) {
    std::string c_prime_path = file_path + "/Cprime.csv";
    std::string nodes_path = file_path + "/nodes.csv";
    std::string tau_path = file_path + "/tau.csv";
    std::string tau_prime_path = file_path + "/tauprime.csv";

    std::ifstream c_ifs(c_prime_path);
    std::ifstream n_ifs(nodes_path);
    std::ifstream t_ifs(tau_path);
    std::ifstream t_prime_ifs(tau_prime_path);

    std::string str;
    getline(c_ifs, str);
    auto c_split_str = SplitStringWithDelimiter(str, ",");
    for (auto& s : c_split_str) {
        c_prime.push_back(stoi(s));
    }

    std::cout << std::endl;
    int n = 0;
    while (getline(n_ifs, str)) {
        if (n == 0) {
            auto n_split_str = SplitStringWithDelimiter(str, ",");
            drone_speed = stod(n_split_str[n_split_str.size() - 1]);
            std::cout << "drone speed: " << drone_speed << " miles/minute" << std::endl;
        }
        else {
            auto n_split_str = SplitStringWithDelimiter(str, ",");
            int x = stoi(n_split_str[n_split_str.size() - 1]);
            if (x == 1) {
                heavy.push_back(stoi(n_split_str[0]));
            }
        }
        n++;
    }
    std::cout << "uav non-eligible customer: ";
    for (auto x : heavy) {
        std::cout << x << " ";
    }
    std::cout << std::endl;
    getline(n_ifs, str);
    n -= 1;
    //n = 20;

    num_node = n;
    tau.resize(num_node + 1);
    tau_prime.resize(num_node + 1);
    for (int i = 0; i < tau.size(); i++) {
        tau[i].resize(num_node + 1);
        tau_prime[i].resize(num_node + 1);
    }
    double d;
    char c;
    roundDist = false;

    for (int i = 0; i < n + 1; i++) {
        getline(t_prime_ifs, str);
        std::istringstream iss(str);
        for (int j = 0; j < n + 1; j++) {
            iss >> d >> c;
            if (roundDist) {
                tau_prime[i][j] = (int)d;
            }
            else {
                tau_prime[i][j] = d;
            }
        }
    }


    for (int i = 0; i < n + 1; i++) {
        getline(t_ifs, str);
        std::istringstream iss(str);
        for (int j = 0; j < n + 1; j++) {
            iss >> d >> c;
            if (roundDist) {
                tau[i][j] = (int)d;
            }
            else {
                tau[i][j] = d;
            }
        }
    }

}
