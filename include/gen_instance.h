//
// Created by ORG on 23/02/2024.
//

#ifndef UMV_FSTSP_GEN_INSTANCE_H
#define UMV_FSTSP_GEN_INSTANCE_H

#include <vector>
#include <random>
#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;
std::vector<std::vector<double>> generateSymmetricMatrix(int rows, int cols, double upperBound) {
    std::vector<std::vector<double>> matrix(rows, std::vector<double>(cols, 0.0));

    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(1.2, upperBound);

    // Fill the upper triangular part of the matrix with random values
    for (int i = 0; i < rows; ++i) {
        for (int j = i + 1; j < cols; ++j) {
            double randomValue = dis(gen);
            matrix[i][j] = randomValue;
            matrix[j][i] = randomValue; // Ensures symmetry
        }
    }

    // Set diagonal elements, last element of the first row, and last row to 0
    for (int i = 0; i < rows; ++i) {
        matrix[i][i] = 0.0; // Diagonal
        matrix[0][cols - 1] = 0.0; // Last element of the first row
        matrix[rows - 1][i] = 0.0; // Last row
    }

    return matrix;
}
std::vector<int> generate1DMatrix(int n) {
    // Calculate the minimum number of elements (at least 0.5*n, rounded up)
    int minElements = static_cast<int>(std::ceil(0.5 * n))+1;

    // Create a shuffled list of numbers from 1 to n
    std::vector<int> shuffledList(n);
    std::iota(shuffledList.begin(), shuffledList.end(), 1);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(shuffledList.begin(), shuffledList.end(), gen);

    // Generate a random number between minElements and n for the size of the matrix
    std::uniform_int_distribution<int> dis(minElements, n);
    int size = dis(gen);

    // Select the required number of elements from the shuffled list
    std::vector<int> matrix(shuffledList.begin(), shuffledList.begin() + size);

    return matrix;
}
class GenInstance {
public:
    int n_customer;
    double dtl;
    std::vector<std::vector<double>> tau;
    std::vector<std::vector<double>> tau_prime;
    std::vector<int> c_prime;
    std::vector<int> heavy;

    GenInstance(int n_customer, double dtl, int v);
};

class path;
void write_c_prime_to_csv(const std::vector<int>& matrix, const std::string& filename, const std::string& folderPath) {
    // Write the vector to a CSV file
    std::ofstream file(filename);
    if (file.is_open()) {
        for (size_t i = 0; i < matrix.size(); ++i) {
            file << matrix[i];
            if (i != matrix.size() - 1) {
                file << ",";
            }
        }
        file.close();
        std::cout << "Vector has been written to file '" << filename << "' successfully." << std::endl;
    } else {
        std::cerr << "Unable to open file '" << filename << "' for writing." << std::endl;
        return;
    }

    // Copy the file to the specified folder path
    fs::path srcPath(filename);
    fs::path destPath = fs::path(folderPath) / srcPath.filename();
    try {
        fs::copy_file(srcPath, destPath, fs::copy_options::overwrite_existing);
        std::cout << "File copied to folder path: " << destPath << std::endl;
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error copying file: " << e.what() << std::endl;
    }
}
void write_tau_to_csv(const std::vector<std::vector<double>>& matrix, const std::string& filename, const std::string& folderPath) {
    // Write the matrix to a CSV file
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const auto& row : matrix) {
            for (size_t i = 0; i < row.size(); ++i) {
                file << row[i];
                if (i != row.size() - 1) {
                    file << ",";
                }
            }
            file << std::endl;
        }
        file.close();
        std::cout << "Matrix has been written to file '" << filename << "' successfully." << std::endl;
    } else {
        std::cerr << "Unable to open file '" << filename << "' for writing." << std::endl;
        return;
    }

    // Copy the file to the specified folder path
    fs::path srcPath(filename);
    fs::path destPath = fs::path(folderPath) / srcPath.filename();
    try {
        fs::copy_file(srcPath, destPath, fs::copy_options::overwrite_existing);
        std::cout << "File copied to folder path: " << destPath << std::endl;
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error copying file: " << e.what() << std::endl;
    }
}
void create_and_write_node_file(int n_customer, std::vector<int>& c_prime, const std::string& filename, const std::string& folderPath) {
    std::ofstream file(filename);
    if(file.is_open()) {
        for (int i = 0; i < n_customer+2; i++) {
            file << i << "," << 1 << "," << 1 << ",";
            if (i == 0) {
                file << 1 << "\n";
                continue;
            }

            bool check = false;
            for (int c:c_prime) {
                if (c == i) {
                    check = true;
                }
            }

            if (check || i == n_customer+1) {
                file << 0 << "\n";
            } else {
                file << 1 << "\n";
            }
        }
        file.close();
    } else {
        std::cerr << "Unable to open file '" << filename << "' for writing." << std::endl;
        return;
    }
    fs::path srcPath(filename);
    fs::path destPath = fs::path(folderPath) / srcPath.filename();
    try {
        fs::copy_file(srcPath, destPath, fs::copy_options::overwrite_existing);
        std::cout << "File copied to folder path: " << destPath << std::endl;
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error copying file: " << e.what() << std::endl;
    }
}
void createDirectory(const std::string& directoryPath, const std::string& directoryName) {
    fs::path dirPath = fs::path(directoryPath) / directoryName;
    try {
        fs::create_directory(dirPath);
        std::cout << "Directory '" << directoryName << "' created successfully in '" << directoryPath << "'." << std::endl;
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error creating directory: " << e.what() << std::endl;
    }
}
// v: version number. just for naming purpose.
GenInstance::GenInstance(int n_customer, double dtl, int v) {
    createDirectory("/mnt/c/Users/ORG/CLionProjects/uMV-FSTSP/rand_generated_instances", "v"+std::to_string(v));
    c_prime = generate1DMatrix(n_customer);
    tau = generateSymmetricMatrix(n_customer+2, n_customer+2, 20);
    tau_prime = generateSymmetricMatrix(n_customer+2, n_customer+2, 20);
//    std::vector<std::tuple<int, double, double, double>> nodes(n_customer+2);
    write_c_prime_to_csv(c_prime, "Cprime.csv", "/mnt/c/Users/ORG/CLionProjects/uMV-FSTSP/rand_generated_instances/v" +std::to_string(v));
    write_tau_to_csv(tau, "tau.csv", "/mnt/c/Users/ORG/CLionProjects/uMV-FSTSP/rand_generated_instances/v" +std::to_string(v));
    write_tau_to_csv(tau_prime, "tau_prime.csv", "/mnt/c/Users/ORG/CLionProjects/uMV-FSTSP/rand_generated_instances/v" +std::to_string(v));
    create_and_write_node_file(n_customer, c_prime, "nodes.csv", "/mnt/c/Users/ORG/CLionProjects/uMV-FSTSP/rand_generated_instances/v" +std::to_string(v));
}

#endif //UMV_FSTSP_GEN_INSTANCE_H
