#ifndef GENERATE_MAP_H
#define GENERATE_MAP_H

std::vector<std::vector<std::vector<double> > > get_polygons(const std::string& filename);

void generate_sdf(std::vector<std::vector<std::vector<double> > > polygons);

#endif // GENERATE_MAP_H