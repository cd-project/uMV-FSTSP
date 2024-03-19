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


//Instance::Instance(const std::string &folder_path, std::string &dist_type) {
//    std::string c_prime_path = folder_path + "/Cprime.csv";
//    std::string nodes_path = folder_path + "/nodes.csv";
//    std::string tau_path = folder_path + "/tau.csv";
//    std::string tau_prime_path = folder_path + "/tauprime.csv";
//
//    std::ifstream c_ifs(c_prime_path);
//    std::ifstream n_ifs(nodes_path);
//    std::ifstream t_ifs(tau_path);
//    std::ifstream t_prime_ifs(tau_prime_path);
//
//    std::string str;
//    getline(c_ifs, str);
//    auto c_split_str = SplitStringWithDelimiter(str, ",");
//    for (auto& s : c_split_str) {
//        c_prime.push_back(stoi(s));
//    }
//    std::vector<std::pair<double, double>> coord;
//    std::cout << std::endl;
//    int n = 0;
//    while (getline(n_ifs, str)) {
//        if (n == 0) {
//            auto n_split_str = SplitStringWithDelimiter(str, ",");
//            drone_speed = stod(n_split_str[n_split_str.size() - 1]);
//            std::cout << "drone speed: " << drone_speed << " miles/minute" << std::endl;
//            coord.emplace_back(stod(n_split_str[1]), stod(n_split_str[2]));
//        }
//        else {
//            auto n_split_str = SplitStringWithDelimiter(str, ",");
//            int x = stoi(n_split_str[n_split_str.size() - 1]);
//            if (x == 1) {
//                heavy.emplace_back(stoi(n_split_str[0]));
//            }
//            coord.emplace_back(stod(n_split_str[1]), stod(n_split_str[2]));
//        }
//        n++;
//    }
//    std::cout << "uav non-eligible customer: ";
//    for (auto x : heavy) {
//        std::cout << x << " ";
//    }
//    std::cout << std::endl;
//    getline(n_ifs, str);
//    n -= 1;
//    //n = 20;
//
//    num_node = n;
//    tau.resize(num_node + 1);
//    tau_prime.resize(num_node + 1);
//    for (int i = 0; i < tau.size(); i++) {
//        tau[i].resize(num_node + 1);
//        tau_prime[i].resize(num_node + 1);
//    }
//    double d;
//    char c;
////    roundDist = false;
//    if (dist_type == "original") {
//        for (int i = 0; i < n + 1; i++) {
//            getline(t_prime_ifs, str);
//            std::istringstream iss(str);
//            for (int j = 0; j < n + 1; j++) {
//                iss >> d >> c;
//                tau_prime[i][j] = d;
//            }
//        }
//
//
//        for (int i = 0; i < n + 1; i++) {
//            getline(t_ifs, str);
//            std::istringstream iss(str);
//            for (int j = 0; j < n + 1; j++) {
//                iss >> d >> c;
//                tau[i][j] = d;
//            }
//        }
//    } else if (dist_type == "geo") {
//        double PI = 3.141592, RRR = 6378.388;
//    }
//
//}
inline double manhattanDistance(double x1, double y1, double x2, double y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}
inline double euclideanDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}
Instance::Instance(const std::string folder_path, bool roundDist, double gamma) {
    std::string c_prime_path = folder_path + "/Cprime.csv";
    std::string nodes_path = folder_path + "/nodes.csv";
    std::string tau_path = folder_path + "/tau.csv";
    std::string tau_prime_path = folder_path + "/tauprime.csv";

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
    std::vector<double> X_coord;
    std::vector<double> Y_coord;
    std::cout << std::endl;
    int n = 0;
    while (getline(n_ifs, str)) {
        if (n == 0) {
            auto n_split_str = SplitStringWithDelimiter(str, ",");
            drone_speed = stod(n_split_str[n_split_str.size() - 1]);
            std::cout << "drone speed: " << drone_speed << " miles/minute" << std::endl;
            X_coord.push_back(stod(n_split_str[1]));
            Y_coord.push_back(stod(n_split_str[2]));
        }
        else {
            auto n_split_str = SplitStringWithDelimiter(str, ",");
            int x = stoi(n_split_str[n_split_str.size() - 1]);
            if (x == 1) {
                heavy.push_back(stoi(n_split_str[0]));
            }
            X_coord.push_back(stod(n_split_str[1]));
            Y_coord.push_back(stod(n_split_str[2]));
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
    if (gamma != 0) {
        // we calculate the distance instead of using instance's provided data
        // truck distance will be calculated using manhattan distance.
        // drone distance will be calculated using euclidean distance.
        // then drone distance will be scaled off/up by a scale of gamma.
        std::cout << "Travel time is in self-calculate mode. tau/tau_prime provided from instance file will be disregarded." << std::endl;
        for (int i = 0; i < n+1; i++) {
            for (int j = 0; j < n+1; j++) {
                tau[i][j] = euclideanDistance(X_coord[i], Y_coord[i], X_coord[j], Y_coord[j]);
                tau_prime[i][j] = euclideanDistance(X_coord[i], Y_coord[i], X_coord[j], Y_coord[j]) / gamma;
                if (i == n) {
                    tau[i][j] = 0;
                    tau_prime[i][j] = 0;
                }
            }
        }

    } else {
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
                    tau_prime[i][j] = (d);
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
                    tau[i][j] = (d);
                }
            }
        }
    }
    std::cout << "Printing tau:" << std::endl;
    for (int i = 0; i < n+1; i++) {
        for (int j = 0; j < n+1; j++) {
            std::cout << tau[i][j] << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "Printing tau_prime:" << std::endl;
    for (int i = 0; i < n+1; i++) {
        for (int j = 0; j < n+1; j++) {
            std::cout << tau_prime[i][j] << " ";
        }
        std::cout << std::endl;
    }

    // for (int i = 0; i < n; i++) {
    //     for (int j = i+1; j < n; j++) {
    //         for (int k = j+1; k < n; k++) {
    //             if (tau[i][j] + tau[j][k] < tau[i][k]) {
    //                 std::cout << "Triangle inequalities violated!!" << std::endl;
    //                 std::cout << "On vertices " << i << " " << j << " " << k << std::endl;
    //                 std::cout << "Order: (" << i << ", " << j << ") + (" << j << ", " << k << ") < (" << i << ", " << k << ")" << std::endl;
    //                 std::cout << tau[i][j] << " " << tau[j][k] << " " << tau[i][k] << std::endl;
    //             }
    //             if (tau[i][k] + tau[k][j] < tau[i][j]) {
    //                 std::cout << "Triangle inequalities violated!!" << std::endl;
    //                 std::cout << "On vertices " << i << " " << j << " " << k << std::endl;
    //                 std::cout << "Order: (" << i << ", " << k << ") + (" << k << ", " << j << ") < (" << i << ", " << j << ")" << std::endl;
    //                 std::cout << tau[i][k] << " " << tau[j][k] << " " << tau[i][j] << std::endl;
    //
    //             }
    //             if (tau[i][j] + tau[i][k] < tau[j][k]) {
    //                 std::cout << "Triangle inequalities violated!!" << std::endl;
    //                 std::cout << "On vertices " << i << " " << j << " " << k << std::endl;
    //                 std::cout << "Order: (" << i << ", " << j << ") + (" << i << ", " << k << ") < (" << j << ", " << k << ")" << std::endl;
    //                 std::cout << tau[i][j] << " " << tau[i][k] << " " << tau[j][k] << std::endl;
    //
    //             }
    //         }
    //     }
    // }
}
