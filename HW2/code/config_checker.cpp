#include "utils.h"
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <map_file> <num_dofs> <config>" << std::endl;
        return -1;
    }

    std::string map_file = argv[1];
    int num_dofs = std::stoi(argv[2]);
    std::string config_str = argv[3];

    // Load the map
    double* map;
    int x_size, y_size;
    std::tie(map, x_size, y_size) = loadMap(map_file);

    // Convert config string to array
    std::vector<double> angles;
    std::stringstream ss(config_str);
    std::string token;
    while (std::getline(ss, token, ',')) {
        angles.push_back(std::stod(token));
    }

    if (angles.size() != num_dofs) {
        std::cerr << "Error: Number of DOFs does not match provided configuration." << std::endl;
        return -1;
    }

    // Check if the configuration is valid
    if (IsValidArmConfiguration(angles.data(), num_dofs, map, x_size, y_size)) {
        return 0;
    } else {
        return 1;
    }
}