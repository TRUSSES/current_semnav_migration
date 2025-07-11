// Runs the planner at points on a grid and visualizes the generated velocity vector field.

#include <reactive_planner_lib.h>
#include "matplotlibcpp.h"
#include <vector>
#include <cmath>
#include <cstdlib>

namespace plt = matplotlibcpp;

// Define properties for dilations
double ObstacleDilation_ = 0.3;
const int points_per_circle = 5;
bg::strategy::buffer::join_miter join_strategy_input;
bg::strategy::buffer::end_flat end_strategy_input;
bg::strategy::buffer::point_circle point_strategy_input;
bg::strategy::buffer::side_straight side_strategy_input;

// Global variables for obstacles and diffeo trees.
std::vector<polygon> PolygonList_;
std::vector<std::vector<PolygonClass>> DiffeoTreeArray_;

// Global variables for robot state.
point RobotPosition_;
double RobotOrientation_ = 0;

// Numerical parameters.
double AllowableRange_ = 4.0;
double CutoffRange_ = 0.15;

double ForwardLinCmdLimit_ = 0.3;
double BackwardLinCmdLimit_ = 0.0;
double AngCmdLimit_ = 0.7;

double RFunctionExponent_ = 20.0;
double Epsilon_ = 1.0;
double VarEpsilon_ = 1.0;
double Mu1_ = 0.8;
double Mu2_ = 0.05;
DiffeoParamsClass DiffeoParams_ = DiffeoParamsClass(RFunctionExponent_, Epsilon_, VarEpsilon_, Mu1_, Mu2_, 
    {{-100.0, -100.0}, {300.0, -100.0}, {300.0, 300.0}, {-100.0, 300.0}, {-100.0, -100.0}});

double LinearGain_ = 0.2;
double AngularGain_ = 0.4;

double Goal_x_ = 0.0;
double Goal_y_ = 2.0;
point Goal_;
double Tolerance_ = 0.4;

// For plots
std::vector<double> nan_x_, nan_y_;
double target_x, target_y;
polygon target_LF_;
point target_LGL_, target_LGA_;

void set_goal() {
    // Set goal point based on coordinates.
    Goal_.set<0>(Goal_x_);
    Goal_.set<1>(Goal_y_);
}

std::vector<polygon> create_polygon_list() {
    // Create list of Boost polygons.
    std::vector<polygon> polygon_list;

    // Unit square
    std::vector<point> square_points = {
        point(0.0, 0.0),
        point(1.0, 0.0),
        point(1.0, 1.0),
        point(0.0, 1.0),
        point(0.0, 0.0)  // repeat first point to close the polygon
    };

    std::vector<point> square1_points = {
        point(0.0, 1.0),
        point(1.0, 1.0),
        point(1.0, 2.0),
        point(0.0, 2.0),
        point(0.0, 1.0)  // repeat first point to close the polygon
    };

    std::vector<point> square2_points = {
        point(0.0, -3.0),
        point(1.0, -3.0),
        point(1.0, -2.0),
        point(0.0, -1.0),
        point(0.0, -3.0)  // repeat first point to close the polygon
    };

    std::vector<point> square3_points = {
        point(6.0, 0.0),
        point(8.0, 0.0),
        point(8.0, 2.0),
        point(6.0, 2.0),
        point(6.0, 0.0)  // repeat first point to close the polygon
    };

    // Convert to Boost polygon
    //polygon_list.push_back(BoostPointToBoostPoly(square_points));
    //polygon_list.push_back(BoostPointToBoostPoly(square1_points));
    //polygon_list.push_back(BoostPointToBoostPoly(square2_points));
    //polygon_list.push_back(BoostPointToBoostPoly(square3_points));
    
    return polygon_list;
}

void diffeo_tree_update(const std::vector<polygon>& semantic_map_data) {
    std::vector<polygon> polygon_list;
	std::vector<polygon> polygon_list_merged;

    // Dilate the polygon by the robot radius and append it to the polygon list
    for (const auto& polygon_in: semantic_map_data) {
        multi_polygon output;
        bg::strategy::buffer::distance_symmetric<double> distance_strategy(ObstacleDilation_);
        bg::buffer(polygon_in, output, distance_strategy, side_strategy_input, join_strategy_input, end_strategy_input, point_strategy_input);
        polygon_list.push_back(output.front());
    }

    std::cout << "Received " << polygon_list.size() << " polygons." << std::endl;

    multi_polygon output_union;
    if (polygon_list.size() >= 1) {
        output_union.push_back(polygon_list.back());
        polygon_list.pop_back();
        while (!polygon_list.empty()) {
            polygon next_polygon = polygon_list.back();
            polygon_list.pop_back();
            multi_polygon temp_result;
            bg::union_(output_union, next_polygon, temp_result);
            output_union = temp_result;
        }
    }

    std::cout << "Found unions. output_union.size(): " << output_union.size() << std::endl;

    for (size_t i = 0; i < output_union.size(); i++) {
        // polygon ch_component;
        // bg::convex_hull(output_union[i], ch_component);
        polygon simplified_component;
        bg::simplify(output_union[i], simplified_component, 0.2);
        polygon_list_merged.push_back(simplified_component);
        std::cout << "Merged polygon added." << std::endl;
    }

    // Find diffeomorphism trees for all merged polygons
    std::vector<std::vector<PolygonClass>> localDiffeoTreeArray;
    for (size_t i = 0; i < polygon_list_merged.size(); i++) {
        std::cout << bg::dsv(polygon_list_merged[i]) << std::endl;
        std::vector<PolygonClass> tree;
        auto points_std = BoostPointToStd(BoostPolyToBoostPoint(polygon_list_merged[i]));
        std::cout << "boostpointtostd completed" << std::endl;
        diffeoTreeConvex(points_std, DiffeoParams_, &tree);
        localDiffeoTreeArray.push_back(tree);
        std::cout << "Found diffeo trees for polygon " << i << std::endl;

    }

    DiffeoTreeArray_.clear();
    PolygonList_.clear();
    DiffeoTreeArray_.assign(localDiffeoTreeArray.begin(), localDiffeoTreeArray.end());
    PolygonList_.assign(polygon_list_merged.begin(), polygon_list_merged.end());

    std::cout << "Updated diffeo trees." << std::endl;
}

std::shared_ptr<sensor_msgs::msg::LaserScan> create_fake_laserscan() {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();

    // Typical LIDAR setup: 360 degrees with 1-degree resolution
    scan->angle_min = -M_PI;
    scan->angle_max = M_PI;
    scan->angle_increment = M_PI / 180.0;  // 1 degree
    scan->range_min = 0.1;     // LIDAR's minimum distance
    scan->range_max = 10.0;    // LIDAR's maximum distance

    // Number of beams
    int num_readings = static_cast<int>((scan->angle_max - scan->angle_min) / scan->angle_increment);

    // Fill all beams with max range (no obstacles detected)
    scan->ranges = std::vector<float>(num_readings, scan->range_max);

    std::cout << "Created fake laser scan." << std::endl;
    return scan;
}

std::vector<double> get_cmd(double robot_pose_x, double robot_pose_y, const sensor_msgs::msg::LaserScan::ConstPtr& lidar_data) {
    // Make local copies
    std::vector<polygon> localPolygonList;
    std::vector<std::vector<PolygonClass>> localDiffeoTreeArray;
    localPolygonList.assign(PolygonList_.begin(), PolygonList_.end());
    localDiffeoTreeArray.assign(DiffeoTreeArray_.begin(), DiffeoTreeArray_.end());

    RobotPosition_.set<0>(robot_pose_x);
    RobotPosition_.set<1>(robot_pose_y);
    double RobotPitch_ = 0.0;

    // Construct LIDAR object
    LIDARClass LIDAR;
    constructLIDAR2D(lidar_data, CutoffRange_, AllowableRange_, RobotPitch_, &LIDAR);

    // Complete LIDAR readings
    completeLIDAR2D(&LIDAR);

    // Set the LIDAR rays that hit known obstacles (in semantic map) to the LIDAR range
    for (size_t i = 0; i < localPolygonList.size(); i++) {
        compensateObstacleLIDAR2D(RobotPosition_, RobotOrientation_, localPolygonList[i], &LIDAR);
    }

    // Find list of polygon objects in the model layer based on the known obstacles
    std::vector<polygon> KnownObstaclesModel;
    for (size_t i = 0; i < localDiffeoTreeArray.size(); i++) {
        std::vector<double> theta = linspace(-M_PI, M_PI, 15);
        std::vector<std::vector<double>> model_polygon_coords;
        point root_center = localDiffeoTreeArray[i].back().get_center();
        double root_radius = localDiffeoTreeArray[i].back().get_radius();
        for (size_t j = 0; j < theta.size(); j++) {
            model_polygon_coords.push_back({root_center.get<0>()+root_radius*cos(theta[j]), root_center.get<1>()+root_radius*sin(theta[j])});
        }
        KnownObstaclesModel.push_back(BoostPointToBoostPoly(StdToBoostPoint(model_polygon_coords)));
    }

    // Find the diffeomorphism and its jacobian at the robot position, along with the necessary second derivatives
    std::vector<double> RobotPositionTransformed = {RobotPosition_.get<0>(), RobotPosition_.get<1>()};
    std::vector<std::vector<double>> RobotPositionTransformedD = {{1.0, 0.0}, {0.0, 1.0}};
    std::vector<double> RobotPositionTransformedDD = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (size_t i = 0; i < localDiffeoTreeArray.size(); i++) {
        OutputStructVector TempTransformation = polygonDiffeoConvex(RobotPositionTransformed, localDiffeoTreeArray[i], DiffeoParams_);

        std::vector<double> TempPositionTransformed = TempTransformation.Value;
        std::vector<std::vector<double>> TempPositionTransformedD = TempTransformation.Jacobian;
        std::vector<double> TempPositionTransformedDD = TempTransformation.JacobianD;

        double res1 = TempPositionTransformedD[0][0]*RobotPositionTransformedDD[0] + TempPositionTransformedD[0][1]*RobotPositionTransformedDD[4] + RobotPositionTransformedD[0][0]*(TempPositionTransformedDD[0]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[1]*RobotPositionTransformedD[1][0]) + RobotPositionTransformedD[1][0]*(TempPositionTransformedDD[2]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[3]*RobotPositionTransformedD[1][0]);
        double res2 = TempPositionTransformedD[0][0]*RobotPositionTransformedDD[1] + TempPositionTransformedD[0][1]*RobotPositionTransformedDD[5] + RobotPositionTransformedD[0][0]*(TempPositionTransformedDD[0]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[1]*RobotPositionTransformedD[1][1]) + RobotPositionTransformedD[1][0]*(TempPositionTransformedDD[2]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[3]*RobotPositionTransformedD[1][1]);
        double res3 = TempPositionTransformedD[0][0]*RobotPositionTransformedDD[2] + TempPositionTransformedD[0][1]*RobotPositionTransformedDD[6] + RobotPositionTransformedD[0][1]*(TempPositionTransformedDD[0]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[1]*RobotPositionTransformedD[1][0]) + RobotPositionTransformedD[1][1]*(TempPositionTransformedDD[2]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[3]*RobotPositionTransformedD[1][0]);
        double res4 = TempPositionTransformedD[0][0]*RobotPositionTransformedDD[3] + TempPositionTransformedD[0][1]*RobotPositionTransformedDD[7] + RobotPositionTransformedD[0][1]*(TempPositionTransformedDD[0]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[1]*RobotPositionTransformedD[1][1]) + RobotPositionTransformedD[1][1]*(TempPositionTransformedDD[2]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[3]*RobotPositionTransformedD[1][1]);
        double res5 = TempPositionTransformedD[1][0]*RobotPositionTransformedDD[0] + TempPositionTransformedD[1][1]*RobotPositionTransformedDD[4] + RobotPositionTransformedD[0][0]*(TempPositionTransformedDD[4]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[5]*RobotPositionTransformedD[1][0]) + RobotPositionTransformedD[1][0]*(TempPositionTransformedDD[6]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[7]*RobotPositionTransformedD[1][0]);
        double res6 = TempPositionTransformedD[1][0]*RobotPositionTransformedDD[1] + TempPositionTransformedD[1][1]*RobotPositionTransformedDD[5] + RobotPositionTransformedD[0][0]*(TempPositionTransformedDD[4]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[5]*RobotPositionTransformedD[1][1]) + RobotPositionTransformedD[1][0]*(TempPositionTransformedDD[6]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[7]*RobotPositionTransformedD[1][1]);
        double res7 = TempPositionTransformedD[1][0]*RobotPositionTransformedDD[2] + TempPositionTransformedD[1][1]*RobotPositionTransformedDD[6] + RobotPositionTransformedD[0][1]*(TempPositionTransformedDD[4]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[5]*RobotPositionTransformedD[1][0]) + RobotPositionTransformedD[1][1]*(TempPositionTransformedDD[6]*RobotPositionTransformedD[0][0] + TempPositionTransformedDD[7]*RobotPositionTransformedD[1][0]);
        double res8 = TempPositionTransformedD[1][0]*RobotPositionTransformedDD[3] + TempPositionTransformedD[1][1]*RobotPositionTransformedDD[7] + RobotPositionTransformedD[0][1]*(TempPositionTransformedDD[4]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[5]*RobotPositionTransformedD[1][1]) + RobotPositionTransformedD[1][1]*(TempPositionTransformedDD[6]*RobotPositionTransformedD[0][1] + TempPositionTransformedDD[7]*RobotPositionTransformedD[1][1]);

        RobotPositionTransformedDD[0] = res1;
        RobotPositionTransformedDD[1] = res2;
        RobotPositionTransformedDD[2] = res3;
        RobotPositionTransformedDD[3] = res4;
        RobotPositionTransformedDD[4] = res5;
        RobotPositionTransformedDD[5] = res6;
        RobotPositionTransformedDD[6] = res7;
        RobotPositionTransformedDD[7] = res8;

        RobotPositionTransformedD = MatrixMatrixMultiplication(TempPositionTransformedD, RobotPositionTransformedD);

        RobotPositionTransformed = TempPositionTransformed;
    }

    // Make a point for the transformed robot position
    point RobotPositionTransformedPoint = point(RobotPositionTransformed[0],RobotPositionTransformed[1]);

    // Find alpha1, alpha2, beta1, beta2
    double alpha1 = -(RobotPositionTransformedD[1][0]*cos(RobotOrientation_) + RobotPositionTransformedD[1][1]*sin(RobotOrientation_));
    double beta1 = RobotPositionTransformedDD[0]*pow(cos(RobotOrientation_),2) + (RobotPositionTransformedDD[1]+RobotPositionTransformedDD[2])*sin(RobotOrientation_)*cos(RobotOrientation_) + RobotPositionTransformedDD[3]*pow(sin(RobotOrientation_),2);
    double alpha2 = RobotPositionTransformedD[0][0]*cos(RobotOrientation_) + RobotPositionTransformedD[0][1]*sin(RobotOrientation_);
    double beta2 = RobotPositionTransformedDD[4]*pow(cos(RobotOrientation_),2) + (RobotPositionTransformedDD[5]+RobotPositionTransformedDD[6])*sin(RobotOrientation_)*cos(RobotOrientation_) + RobotPositionTransformedDD[7]*pow(sin(RobotOrientation_),2);

    // Find transformed orientation
    double RobotOrientationTransformed = atan2(RobotPositionTransformedD[1][0]*cos(RobotOrientation_)+RobotPositionTransformedD[1][1]*sin(RobotOrientation_), 
        RobotPositionTransformedD[0][0]*cos(RobotOrientation_) +
        RobotPositionTransformedD[0][1]*sin(RobotOrientation_));

    // Read LIDAR data in the model space to account for the known obstacles
    LIDARClass LIDARmodel_known;
    LIDARmodel_known.RangeMeasurements = LIDAR.RangeMeasurements;
    LIDARmodel_known.Angle = LIDAR.Angle;
    LIDARmodel_known.Range = LIDAR.Range;
    LIDARmodel_known.Infinity = LIDAR.Infinity;
    LIDARmodel_known.MinAngle = LIDAR.MinAngle;
    LIDARmodel_known.MaxAngle = LIDAR.MaxAngle;
    LIDARmodel_known.Resolution = LIDAR.Resolution;
    LIDARmodel_known.NumSample = LIDAR.NumSample;
    readLIDAR2D(point(RobotPositionTransformed[0], RobotPositionTransformed[1]), 
            RobotOrientationTransformed, KnownObstaclesModel, LIDAR.Range, LIDAR.MinAngle, LIDAR.MaxAngle, LIDAR.NumSample, &LIDARmodel_known);

    // Translate LIDAR data from the unknown obstacles to the transformed robot state
    LIDARClass LIDARmodel_unknown;
    LIDARmodel_unknown.RangeMeasurements = LIDAR.RangeMeasurements;
    LIDARmodel_unknown.Angle = LIDAR.Angle;
    LIDARmodel_unknown.Range = LIDAR.Range;
    LIDARmodel_unknown.Infinity = LIDAR.Infinity;
    LIDARmodel_unknown.MinAngle = LIDAR.MinAngle;
    LIDARmodel_unknown.MaxAngle = LIDAR.MaxAngle;
    LIDARmodel_unknown.Resolution = LIDAR.Resolution;
    LIDARmodel_unknown.NumSample = LIDAR.NumSample;
    translateLIDAR2D(RobotPosition_, RobotOrientation_, point(RobotPositionTransformed[0], RobotPositionTransformed[1]), RobotOrientationTransformed, ObstacleDilation_, &LIDARmodel_unknown);

    // Build final model LIDAR object
    std::vector<double> newRangeMeasurements(LIDAR.RangeMeasurements.size(), 0.0);
    for (size_t i = 0; i < LIDAR.RangeMeasurements.size(); i++) {
        newRangeMeasurements[i] = std::min(LIDARmodel_known.RangeMeasurements[i], LIDARmodel_unknown.RangeMeasurements[i]);
    }
    LIDARClass LIDARmodel(newRangeMeasurements, LIDAR.Range-bg::distance(RobotPositionTransformedPoint, RobotPosition_), 
            LIDAR.Infinity, LIDAR.MinAngle, LIDAR.MaxAngle, LIDAR.Resolution);
    // std::cout << "Constructed model space LIDAR with " << LIDARmodel.RangeMeasurements.size() << " rays and " << LIDARmodel.Angle.size() << " angles." << std::endl;

    // Find local freespace; the robot radius can be zero because we have already dilated the obstacles
    polygon LF_model = localfreespaceLIDAR2D(RobotPositionTransformedPoint, RobotOrientationTransformed, 0.0, &LIDARmodel);

    // Find projected goal
    point LGL_model = localgoal_linearLIDAR2D(RobotPositionTransformedPoint, RobotOrientationTransformed, LF_model, Goal_);
    point LGA1_model = localgoalLIDAR2D(LF_model, Goal_);
    point LGA2_model = localgoal_angularLIDAR2D(RobotPositionTransformedPoint, RobotOrientationTransformed, LF_model, Goal_);
    point LGA_model(LGA1_model.get<0>(), LGA1_model.get<1>()); // avoid division by zero
    //std::cout << "Computed model space projections. " << bg::dsv(LGA_model) << std::endl;
    if ((robot_pose_x == target_x) && (robot_pose_y == target_y)) {
        target_LF_ = LF_model;
        target_LGL_ = LGL_model;
        target_LGA_ = LGA_model;
    }

    // Compute the basis for the virtual control inputs
    double tV = (LGL_model.get<0>()-RobotPositionTransformed[0])*cos(RobotOrientationTransformed) + (LGL_model.get<1>()-RobotPositionTransformed[1])*sin(RobotOrientationTransformed);
    double tW1 = (LGA_model.get<0>()-RobotPositionTransformed[0])*cos(RobotOrientationTransformed) + (LGA_model.get<1>()-RobotPositionTransformed[1])*sin(RobotOrientationTransformed);
    double tW2 = -(LGA_model.get<0>()-RobotPositionTransformed[0])*sin(RobotOrientationTransformed) + (LGA_model.get<1>()-RobotPositionTransformed[1])*cos(RobotOrientationTransformed);

    // Compute the basis for transforming to actual control inputs
    double e_norm = sqrt(pow((RobotPositionTransformedD[0][0]*cos(RobotOrientation_)+RobotPositionTransformedD[0][1]*sin(RobotOrientation_)),2) + pow((RobotPositionTransformedD[1][0]*cos(RobotOrientation_)+RobotPositionTransformedD[1][1]*sin(RobotOrientation_)),2));
    double dksi_dpsi = MatrixDeterminant(RobotPositionTransformedD)/pow(e_norm,2);
    double DksiCosSin = (alpha1*beta1 + alpha2*beta2)/pow(e_norm,2);

    // Compute commands by accounting for limits
    double LinearCtrlGain, AngularCtrlGain;
    std::vector<double> vector_to_check_1 = {LinearGain_, ForwardLinCmdLimit_*e_norm/fabsf(tV), 0.4*AngCmdLimit_*dksi_dpsi*e_norm/(fabsf(tV*DksiCosSin))};
    LinearCtrlGain = *std::min_element(vector_to_check_1.begin(), vector_to_check_1.end());

    std::vector<double> vector_to_check_2 = {AngularGain_, 0.6*AngCmdLimit_*dksi_dpsi/(fabsf(atan2(tW2,tW1)))};
    AngularCtrlGain = *std::min_element(vector_to_check_2.begin(), vector_to_check_2.end());

    // Compute virtual and actual inputs
    double dV_virtual = LinearCtrlGain*tV;
    double LinearCmd = dV_virtual/e_norm;
    double dW_virtual = AngularCtrlGain*atan2(tW2,tW1);
    double AngularCmd = (dW_virtual-LinearCmd*DksiCosSin)/dksi_dpsi;

    std::vector<double> cmd = {LinearCmd, AngularCmd};
    return cmd;
}

double compute_next_x(double v, double w, double t) {
    // v = linear velocity, w = angular velocity, t = time step
    double r = v / w;

    double res1 = 4*pow(r, 3)*cos(RobotOrientation_)*sin(t*(2*r*w+v)/(2*r)) / (2*r*w + v);
    double res2 = 4*pow(r, 3)*sin(RobotOrientation_)*cos(t*(2*r*w+v)/(2*r)) / (2*r*w + v);
    double res3 = 4*pow(r, 3)*sin(RobotOrientation_) / (2*r*w + v);

    double res = res1 + res2 - res3;

    return res;
}

double compute_next_y(double v, double w, double t) {
    // v = linear velocity, w = angular velocity, t = time step
    double r = v / w;

    double res1 = -cos(-RobotOrientation_ + v*t/(2*r) - t*w) / (v - 2*r*w);
    double res2 = -2*cos(RobotOrientation_ + v*t/(2*r) + t*w) / (2*r*w + v);
    double res3 = cos(RobotOrientation_ + 3*v/(2*r) + t*w) / (2*r*w + 3*v);
    double res4 = -cos(-RobotOrientation_) / (v - 2*r*w);
    double res5 = -2*cos(RobotOrientation_) / (2*r*w + v);
    double res6 = cos(RobotOrientation_) / (2*r*w + 3*v);

    double res = 2*pow(r, 3)*(res1 + res2 + res3 - res4 - res5 - res6);

    return res;
}

void plot_polygon(polygon poly, std::string format) {
    std::vector<double> x_poly, y_poly;
    for (const auto& point: BoostPointToStd(BoostPolyToBoostPoint(poly))) {
        x_poly.push_back(point[0]);
        y_poly.push_back(point[1]);
    }
    plt::plot(x_poly, y_poly, format);
}

void plot_point(double x, double y, std::string format) {
    std::vector<double> x_coord = {x};
    std::vector<double> y_coord = {y};
    plt::plot(x_coord, y_coord, format);
    std::cout << "Plotted point (" << x << "," << y << ")" << std::endl;
}

std::vector<double> get_vector(double xi, double yi, double vel_linear, double vel_angular) {
    std::vector<double> vec;

    std::cout << "Generating for (" << xi << ", " << yi << ")" << std::endl;
    std::cout << "Cmd: " << vel_linear << ", " << vel_angular << "\n" << std::endl;
    
    // Time step required for diff drive calcs
    double update_frequency = 10; // Hz
    double t = 1 / update_frequency; // s

    // Calculate robot pose in next time step
    double next_x = compute_next_x(vel_linear, vel_angular, t);
    double next_y = compute_next_y(vel_linear, vel_angular, t);

    // Vector shows pose difference, normalized
    double mag = sqrt(pow(next_x - xi, 2) + pow(next_y - yi, 2));
    vec.push_back((next_x - xi) / (2*mag));
    vec.push_back((next_y - yi) / (2*mag));

    return vec;
}

int main(int argc, char** argv) {
    // Get command arguments
    if ((argc != 8) && (argc != 5)) {
        std::cerr << "Usage: ros2 run semnav vector_field_plot -- "
         << "[goal x] [goal y] [min x] [max x] [min y] [max y] [grid n]\n" 
         << "or -- [goal x] [goal y] [target x] [target y]" << std::endl;
        return 1;
    }

    // Initialize environment and robot state
    Goal_x_ = std::atof(argv[1]);
    Goal_y_ = std::atof(argv[2]);
    set_goal();

    const std::vector<polygon> polygon_list = create_polygon_list();
    diffeo_tree_update(polygon_list);

    auto lidar_data = create_fake_laserscan();

    std::vector<double> x, y, u, v;

    // Common plots: obstacles, goal
    plt::figure();

    for (const auto& poly: polygon_list) {
        plot_polygon(poly, "b");
    }

    plot_point(Goal_x_, Goal_y_, "ro");

    if ((argc == 5)) { // single grid point with intermediate planner calcs

        target_x = std::atof(argv[3]);
        target_y = std::atof(argv[4]);

        x.push_back(target_x);
        y.push_back(target_y);

        // Twist command
        std::vector<double> cmd = get_cmd(target_x, target_y, lidar_data);
        double vel_linear = cmd[0];
        double vel_angular = cmd[1];

        // Vector from twist command
        std::vector<double> vec = get_vector(target_x, target_y, vel_linear, vel_angular); 
        u.push_back(vec[0]);
        v.push_back(vec[1]);

        // Plot local freespace and local goals
        plot_polygon(target_LF_, "y");
        plot_point(target_LGL_.get<0>(), target_LGL_.get<1>(), "m*");
        plot_point(target_LGA_.get<0>(), target_LGA_.get<1>(), "c*");

    } else { // all grid points

        const double min_x = std::atof(argv[3]);
        const double max_x = std::atof(argv[4]);
        const double min_y = std::atof(argv[5]);
        const double max_y = std::atof(argv[6]);
        const double grid_n = std::atof(argv[7]);

        for (int i = 0; i < grid_n; i++) {
            double xi = min_x + (max_x - min_x) * i / grid_n;
            for (int j = 0; j < grid_n; j++) {
                double yi = min_y + (max_y - min_y) * j / grid_n;
                x.push_back(xi);
                y.push_back(yi);

                // Twist command
                std::vector<double> cmd = get_cmd(xi, yi, lidar_data);
                double vel_linear = cmd[0];
                double vel_angular = cmd[1];

                // Points with invalid commands
                if (isnan(vel_linear) || isinf(vel_linear)
                    || isnan(vel_angular) || isinf(vel_angular)) {
                    std::cout << "No cmd for (" << xi 
                        << ", " << yi << ")" << std::endl;
                    nan_x_.push_back(xi);
                    nan_y_.push_back(yi);
                }

                // Vector from twist command
                std::vector<double> vec = get_vector(xi, yi, vel_linear, vel_angular); 
                u.push_back(vec[0]);
                v.push_back(vec[1]);
            }
        }

        // Plot points with NaN/inf cmds
        plt::plot(nan_x_, nan_y_, "y");
    }

    // Plot the vector field
    plt::quiver(x, y, u, v);

    plt::xlabel("X");
    plt::ylabel("Y");
    plt::title("Velocity Vector Field");

    plt::axis("equal");
    plt::grid(true);

    plt::show();

    return 0;
}