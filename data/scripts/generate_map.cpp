#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>
#include "./generate_map.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

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

/* 
 * Return vector of polygons from CSV file: x in 1st row, y in 2nd row, polygons divided by NaN.
 * Assumes specified "filename" includes path to ament package share directory.
 */
std::vector<std::vector<std::vector<double> > > get_polygons(const std::string& filename) {
    std::vector<std::vector<std::vector<double> > > polygons;
    std::ifstream file(filename);
    std::cout << "received filed " <<  filename << std::endl;

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

    /* Assumptions:
     * x_coords and y_coords have the same size.
     * Polygon 1 (square containing the other obstacles) should be ignored
     * First vertex of each polygon must be manually duplicated
     */
    for (size_t i = 1; i < x_coords.size(); i++) {
        polygons.push_back(std::vector<std::vector<double> >());
        std::vector<double> first_point;
        for (size_t j = 0; j < x_coords[i].size(); j++) {
            std::vector<double> point;
            point.push_back(x_coords[i][j]);
            point.push_back(y_coords[i][j]);
            polygons.back().push_back(point);
            if (j == 0) first_point = point;
        }
        //polygons.back().push_back(first_point);
    }

    std::cout << "num of polygons: " << polygons.size() << std::endl;
    // Print the vertices
    for (std::vector<std::vector<double> > polygon : polygons) {
        for (std::vector<double> point: polygon)
            std::cout << "(" << point[0] << ", " << point[1] << ")" << std::endl;
        std::cout << std::endl;
    }

    file.close();

    return polygons;
}

/* 
 * Creates "model.sdf" for Gazebo visualization. Stores in /models/polygons/model.sdf
 */
void generate_sdf(std::vector<std::vector<std::vector<double> > > polygons) {
    std::string share_directory = ament_index_cpp::get_package_share_directory("semnav");
    std::string sdf_filename = share_directory + "/models/polygons/model.sdf";

    std::ofstream sdf_file(sdf_filename);
    if (!sdf_file.is_open()) {
        std::cerr << "Error opening " << sdf_filename << std::endl;
        return;
    }

    sdf_file << R"(<sdf version='1.6'>
	<model name='polygons'>
)";
    
    for (size_t i = 0; i < polygons.size(); i++) {
        sdf_file << R"(
        <link name='link )" << i << "'>" << R"(
            <visual name='poly )" << i << "'>" << R"(
                <geometry>
                    <polyline>
                        <height>0.1</height>)";
        for (size_t j = 0; j < polygons[i].size(); j++) {
            sdf_file << R"(
                        <point>)" << polygons[i][j][0] << " " << polygons[i][j][1] << "</point>";
        }
        sdf_file << R"(
                    </polyline>
                </geometry>
            </visual>
        </link>)";
    }

    sdf_file << R"(
    </model>
</sdf>
)";
    sdf_file.close();
    std::cout << "sdf file complete" << std::endl;
}

/*
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
*/