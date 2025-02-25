#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>
#include "./generate_map.h"

/*
 * Return vector of coords from CSV row; each vector in outer vector corresponds to a polygon.
 * Assume last value is NOT NaN.
 */
std::vector<std::vector<double> > get_coords(const std::string& line) {
    std::vector<std::vector<double> > coords;
    coords.push_back(std::vector<double>()); // First polygon.

    std::stringstream ss(line);
    std::string x;
    char* end;

    while (getline(ss, x, ',')) {
        if(x == "NaN") {
            coords.push_back(std::vector<double>());
            continue;
        }
        int y = std::strtod(x.c_str(), &end);
        coords.back().push_back(y);
    }

    return coords;
}

// Return vector of polygons from CSV file: x in 1st row, y in 2nd row, polygons divided by NaN
std::vector<std::vector<std::vector<double> > > get_polygons(const std::string& filename) {
    std::vector<std::vector<std::vector<double> > > polygons;
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cout << "Error opening " << filename << std::endl;
        return polygons;
    }

    // Assume two rows of data
    std::string x_line;
    std::getline(file, x_line);
    
    std::string y_line;
    std::getline(file, y_line);

    std::vector<std::vector<double> > x_coords = get_coords(x_line);
    std::vector<std::vector<double> > y_coords = get_coords(y_line);

    // Assume x_coords and y_coords have the same size.
    for (size_t i = 0; i < x_coords.size(); i++) {
        polygons.push_back(std::vector<std::vector<double> >());
        for (size_t j = 0; j < x_coords[i].size(); j++) {
            std::vector<double> point;
            point.push_back(x_coords[i][j]);
            point.push_back(y_coords[i][j]);
            polygons.back().push_back(point);
        }
    }

    file.close();
    return polygons;
}

int main() {
    std::string filename = "../poly_riskpercentage_mass_5_frequency_1.6111111111111112.csv";
    std::vector<std::vector<std::vector<double> > > polygons = get_polygons(filename);
    std::cout << "num of polygons: " << polygons.size() << std::endl;
    // Print the vertices
    for (std::vector<std::vector<double> > polygon : polygons) {
        for (std::vector<double> point: polygon)
            std::cout << "(" << point[0] << ", " << point[1] << ")" << std::endl;
        std::cout << std::endl;
    }

    return 0;
}