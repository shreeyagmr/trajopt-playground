#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include <nlohmann/json.hpp>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief Saves a joint trajectory to a CSV file
 *
 * @param joint_trajectory Vector of joint positions at each waypoint
 * @param file_path Path where the CSV file will be saved
 */
void saveTrajectory(std::vector<Eigen::VectorXd>& joint_path, const std::string& file_path) {
    std::ofstream f(file_path);

    for (size_t i = 0; i < joint_path.size(); ++i) {
        const Eigen::VectorXd& vec = joint_path[i];
        for (Eigen::Index j = 0; j < vec.size(); ++j) {
            f << vec(j) << ",";
        }
        f << std::endl;
    }

    f.close();
}

/**
 * @brief Reads a Cartesian tool path from a CSV file
 *
 * @param cartesian_path Vector to store the parsed transformation matrices
 * @param file_path Path to the CSV file containing the tool path
 */
void readCartesianPath(std::vector<Eigen::Matrix4d>& path, const std::string& file_path) {
    std::ifstream f(file_path);

    path = std::vector<Eigen::Matrix4d>();
    std::string line;
    while (std::getline(f, line)) {
        std::stringstream line_stream(line);
        std::string cell;

        Eigen::Matrix4d matrix;
        matrix.setIdentity();

        std::vector<double> values;
        while (std::getline(line_stream, cell, ',')) {
            values.push_back(std::stod(cell));
        }

        matrix.block<3, 1>(0, 3) << values[0], values[1], values[2];
        matrix.block<3, 1>(0, 0) << values[3], values[4], values[5];
        matrix.block<3, 1>(0, 1) << values[6], values[7], values[8];
        matrix.block<3, 1>(0, 2) << values[9], values[10], values[11];

        path.push_back(matrix);
    }

    f.close();
}

/**
 * @brief Loads a Descartes solution from a JSON file
 *
 * @param file_path Path to the JSON file containing the Descartes solution
 * @return std::vector<Eigen::VectorXd> Joint trajectory from the Descartes
 * solution
 * @throws std::runtime_error If the file cannot be found or parsed
 */
std::vector<Eigen::VectorXd> loadDescartesSolution(const std::string& file_path) {
    std::cout << "Loading Descartes solution from: " << file_path << std::endl;

    // Check if file exists
    if (!std::filesystem::exists(file_path)) {
        throw std::runtime_error("Descartes solution file not found: " + file_path);
    }

    // Read the JSON file
    std::ifstream file(file_path);
    nlohmann::json json_data;
    file >> json_data;

    // Create the joint trajectory
    std::vector<Eigen::VectorXd> trajectory;

    // Process each waypoint
    for (const auto& waypoint : json_data["trajectory"]) {
        // Extract joint values - match the order from the robot model
        Eigen::VectorXd joint_values(7); // 7 joints total

        // Get the extender joint value first
        if (waypoint.contains("ext")) {
            joint_values[0] = static_cast<double>(waypoint["ext"]);
        } else {
            joint_values[0] = 0.0; // Default value if not specified
        }

        // Then get the 6 robot joints
        for (int i = 0; i < 6; ++i) {
            joint_values[i + 1] = static_cast<double>(waypoint["joint"][i]);
        }

        trajectory.push_back(joint_values);
    }
    return trajectory;
}
