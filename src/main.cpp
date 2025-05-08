#include "utils.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include <console_bridge/console.h>
#include <nlohmann/json.hpp>
#include <tesseract_collision/core/types.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/stopwatch.h>
#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_motion_planner.h>
#include <tesseract_srdf/kinematics_information.h>
#include <trajopt/problem_description.hpp>
#include <trajopt_common/collision_types.h>
#include <trajopt_common/logging.hpp>

using namespace tesseract_common;
using namespace tesseract_environment;
using namespace tesseract_planning;

static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";
static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";

int main(int argc, char** argv) {
    //==========================================================================
    // PARSE COMMAND LINE ARGUMENTS
    //==========================================================================
    bool debug = false;
    bool use_ifopt = false; // Default to using TrajOptMotionPlanner
    std::size_t points_to_consider = 0; // Default to using all points (will be set after loading the path)
    std::string seed_method = "none"; // Default to using no seeding method

    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--debug" || arg == "-d") {
            debug = true;
            std::cout << "Debug mode enabled" << std::endl;
        } else if (arg == "--use_ifopt") {
            use_ifopt = true;
        } else if (arg == "--points" || arg == "-p") {
            if (i + 1 < argc) {
                try {
                    points_to_consider = std::stoul(argv[i + 1]);
                    std::cout << "Will consider " << points_to_consider << " points from the toolpath" << std::endl;
                    i++; // Skip the next argument since we've used it
                } catch (const std::exception& e) {
                    std::cerr << "Error parsing number of points: " << e.what() << std::endl;
                    return -1;
                }
            } else {
                std::cerr << "Error: --points/-p requires a number" << std::endl;
                return -1;
            }
        } else if (arg == "--seed_method" && i + 1 < argc) {
            seed_method = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0]
                      << " [--use_ifopt] [--points N] [--seed_method "
                         "none|waypoint|pci] [--debug]"
                      << std::endl;
            std::cout << "  --use_ifopt         : Use TrajOptIfoptMotionPlanner "
                         "instead of TrajOptMotionPlanner."
                      << std::endl;
            std::cout << "  --points N          : Consider only the first N points "
                         "from the toolpath (0=all)."
                      << std::endl;
            std::cout << "  --seed_method METHOD: Set the seeding method for "
                         "TrajOptMotionPlanner:"
                      << std::endl;
            std::cout << "      none (default)  : No explicit seeding." << std::endl;
            std::cout << "      waypoint        : Seed each waypoint individually "
                         "(requires Descartes solution)."
                      << std::endl;
            std::cout << "      pci             : Manually construct PCI with seed "
                         "(requires Descartes solution)."
                      << std::endl;
            std::cout << "  --debug             : Enable debug mode." << std::endl;
            return 0;
        }
    }

    // Validate seed_method argument
    if (seed_method != "none" && seed_method != "waypoint" && seed_method != "pci") {
        seed_method = "none"; // Default to no seeding
    }

    //==========================================================================
    // INITIALIZATION
    //==========================================================================
    std::cout << "Starting TrajOpt Playground..." << std::endl;
    if (debug) {
        console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
        trajopt_common::gLogLevel = trajopt_common::LevelInfo;
    } else {
        console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE);
        trajopt_common::gLogLevel = trajopt_common::LevelError;
    }

    //==========================================================================
    // ENVIRONMENT SETUP
    //==========================================================================
    std::cout << "\n=== Loading Environment ===" << std::endl;

    // Get executable directory to find resources relative to it
    std::filesystem::path executable_path = std::filesystem::canonical("/proc/self/exe").parent_path();
    std::filesystem::path source_dir = executable_path;

    // If we're in a build directory, go up to find the source
    if (source_dir.filename() == "build" || (source_dir.parent_path().filename() == "build" && source_dir.filename() == "bin")) {
        source_dir = source_dir.parent_path();
    }

    // Set paths relative to the determined base directory
    std::filesystem::path config_path = source_dir / "config" / "robots" / "fanuc_m710";
    std::string urdf_path = (config_path / "robot_actual_roe.urdf").string();
    std::string srdf_path = (config_path / "robot_actual_roe.srdf").string();

    std::cout << "Using config path: " << config_path << std::endl;

    auto env = std::make_shared<Environment>();

    // Create a GeneralResourceLocator with the config path
    auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>(std::vector<std::filesystem::path>{config_path});

    std::ifstream urdf_file(urdf_path), srdf_file(srdf_path);
    std::stringstream urdf_stream, srdf_stream;
    urdf_stream << urdf_file.rdbuf();
    srdf_stream << srdf_file.rdbuf();

    if (!env->init(urdf_stream.str(), srdf_stream.str(), locator)) {
        std::cerr << "Failed to initialize environment!" << std::endl;
        return -1;
    }
    std::cout << "Environment loaded successfully!" << std::endl;

    //==========================================================================
    // MANIPULATOR SETUP
    //==========================================================================
    std::cout << "\n=== Setting Up Manipulator ===" << std::endl;

    auto kin_info = env->getKinematicsInformation();
    std::cout << "Group TCP : " << kin_info.group_tcps["arm"]["tcp0"].matrix() << std::endl;
    std::cout << "Have group arm : " << kin_info.hasGroup("arm") << std::endl;
    std::cout << "Kin info fk size : " << kin_info.kinematics_plugin_info.fwd_plugin_infos.size() << std::endl;
    std::cout << "Kin info size : " << kin_info.kinematics_plugin_info.inv_plugin_infos.size() << std::endl;

    // Setup manipulator info
    tesseract_common::ManipulatorInfo manip_info;
    manip_info.manipulator = "arm";
    manip_info.working_frame = "y_prismatic";
    manip_info.tcp_frame = "tool0";

    Eigen::Isometry3d tcp;
    tcp.translation() = Eigen::Vector3d(-0.376, 0, 0.257953);
    tcp.linear() = Eigen::Quaterniond(0, 0.7071068, 0, -0.7071068).normalized().toRotationMatrix();
    manip_info.tcp_offset = tcp;

    std::cout << "Custom TCP transform: \n" << tcp.matrix() << std::endl;
    std::cout << "Custom TCP Translation: [" << tcp.translation().x() << ", " << tcp.translation().y() << ", " << tcp.translation().z() << "]"
              << std::endl;

    Eigen::Quaterniond tcp_rotation(tcp.rotation());
    std::cout << "Custom TCP Rotation (quaternion): [" << tcp_rotation.x() << ", " << tcp_rotation.y() << ", " << tcp_rotation.z() << ", "
              << tcp_rotation.w() << "]" << std::endl;

    //==========================================================================
    // FORWARD KINEMATICS TEST
    //==========================================================================
    std::cout << "\n=== Testing Forward Kinematics ===" << std::endl;

    // Get the kinematic group
    auto kin_group = env->getKinematicGroup("arm");
    if (kin_group == nullptr) {
        std::cerr << "Failed to get kinematic group" << std::endl;
        return -1;
    }

    // Print joint names to verify order
    std::cout << "Joint names in order: ";
    for (const auto& joint_name : kin_group->getJointNames()) {
        std::cout << joint_name << " ";
    }
    std::cout << std::endl;

    // Test 1: Forward kinematics for home position (0,0,0,0,0,0,0)
    std::cout << "\n--- Test 1: Home Position (0,0,0,0,0,0,0) ---" << std::endl;
    Eigen::VectorXd joint_values1(7);
    joint_values1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    std::cout << "Joint values: [";
    for (int i = 0; i < joint_values1.size(); ++i) {
        std::cout << joint_values1[i];
        if (i < joint_values1.size() - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    auto fk_results1 = kin_group->calcFwdKin(joint_values1);
    if (!fk_results1.empty()) {
        if (fk_results1.find("tool0") != fk_results1.end()) {
            std::cout << "tool0 transform:\n" << fk_results1["tool0"].matrix() << std::endl;
            std::cout << "tool0 position: [" << fk_results1["tool0"].translation().x() << ", " << fk_results1["tool0"].translation().y() << ", "
                      << fk_results1["tool0"].translation().z() << "]" << std::endl;

            Eigen::Quaterniond quat(fk_results1["tool0"].rotation());
            std::cout << "tool0 orientation (quaternion): [" << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << "]"
                      << std::endl;
        }

        if (fk_results1.find("base_link") != fk_results1.end()) {
            std::cout << "base_link transform:\n" << fk_results1["base_link"].matrix() << std::endl;
            std::cout << "base_link position: [" << fk_results1["base_link"].translation().x() << ", " << fk_results1["base_link"].translation().y()
                      << ", " << fk_results1["base_link"].translation().z() << "]" << std::endl;

            Eigen::Quaterniond quat(fk_results1["base_link"].rotation());
            std::cout << "base_link orientation (quaternion): [" << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << "]"
                      << std::endl;
        }
    }

    // Test 2: Forward kinematics for extender = 1m (1,0,0,0,0,0,0)
    std::cout << "\n--- Test 2: Extender = 1m (1,0,0,0,0,0,0) ---" << std::endl;
    Eigen::VectorXd joint_values2(7);
    joint_values2 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    std::cout << "Joint values: [";
    for (int i = 0; i < joint_values2.size(); ++i) {
        std::cout << joint_values2[i];
        if (i < joint_values2.size() - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    auto fk_results2 = kin_group->calcFwdKin(joint_values2);
    if (!fk_results2.empty()) {
        if (fk_results2.find("tool0") != fk_results2.end()) {
            std::cout << "tool0 transform:\n" << fk_results2["tool0"].matrix() << std::endl;
            std::cout << "tool0 position: [" << fk_results2["tool0"].translation().x() << ", " << fk_results2["tool0"].translation().y() << ", "
                      << fk_results2["tool0"].translation().z() << "]" << std::endl;

            Eigen::Quaterniond quat(fk_results2["tool0"].rotation());
            std::cout << "tool0 orientation (quaternion): [" << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << "]"
                      << std::endl;
        }

        if (fk_results2.find("base_link") != fk_results2.end()) {
            std::cout << "base_link transform:\n" << fk_results2["base_link"].matrix() << std::endl;
            std::cout << "base_link position: [" << fk_results2["base_link"].translation().x() << ", " << fk_results2["base_link"].translation().y()
                      << ", " << fk_results2["base_link"].translation().z() << "]" << std::endl;

            Eigen::Quaterniond quat(fk_results2["base_link"].rotation());
            std::cout << "base_link orientation (quaternion): [" << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << "]"
                      << std::endl;
        }
    }

    // Test 3: Forward kinematics for joint_1 = 90 degrees (0,1.5708,0,0,0,0,0)
    std::cout << "\n--- Test 3: Joint 1 = 90 degrees (0,1.5708,0,0,0,0,0) ---" << std::endl;
    Eigen::VectorXd joint_values3(7);
    joint_values3 << 0.0, 1.5708, 0.0, 0.0, 0.0, 0.0, 0.0;

    std::cout << "Joint values: [";
    for (int i = 0; i < joint_values3.size(); ++i) {
        std::cout << joint_values3[i];
        if (i < joint_values3.size() - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    auto fk_results3 = kin_group->calcFwdKin(joint_values3);
    if (!fk_results3.empty()) {
        if (fk_results3.find("tool0") != fk_results3.end()) {
            std::cout << "tool0 transform:\n" << fk_results3["tool0"].matrix() << std::endl;
            std::cout << "tool0 position: [" << fk_results3["tool0"].translation().x() << ", " << fk_results3["tool0"].translation().y() << ", "
                      << fk_results3["tool0"].translation().z() << "]" << std::endl;

            Eigen::Quaterniond quat(fk_results3["tool0"].rotation());
            std::cout << "tool0 orientation (quaternion): [" << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << "]"
                      << std::endl;
        }

        if (fk_results3.find("base_link") != fk_results3.end()) {
            std::cout << "base_link transform:\n" << fk_results3["base_link"].matrix() << std::endl;
            std::cout << "base_link position: [" << fk_results3["base_link"].translation().x() << ", " << fk_results3["base_link"].translation().y()
                      << ", " << fk_results3["base_link"].translation().z() << "]" << std::endl;

            Eigen::Quaterniond quat(fk_results3["base_link"].rotation());
            std::cout << "base_link orientation (quaternion): [" << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << "]"
                      << std::endl;
        }
    }

    // Test with TCP applied
    std::cout << "\n=== Testing Forward Kinematics with TCP Applied ===" << std::endl;

    // Apply TCP to each result
    std::cout << "\n--- Test 1 with TCP: Home Position ---" << std::endl;
    if (fk_results1.find("tool0") != fk_results1.end()) {
        Eigen::Isometry3d tool_with_tcp = fk_results1["tool0"] * tcp;
        std::cout << "tooltcp transform:\n" << tool_with_tcp.matrix() << std::endl;
        std::cout << "tooltcp position: [" << tool_with_tcp.translation().x() << ", " << tool_with_tcp.translation().y() << ", "
                  << tool_with_tcp.translation().z() << "]" << std::endl;

        Eigen::Quaterniond quat(tool_with_tcp.rotation());
        std::cout << "tooltcp orientation (quaternion): [" << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << "]"
                  << std::endl;
    }

    std::cout << "\n--- Test 2 with TCP: Extender = 1m ---" << std::endl;
    if (fk_results2.find("tool0") != fk_results2.end()) {
        Eigen::Isometry3d tool_with_tcp = fk_results2["tool0"] * tcp;
        std::cout << "tooltcp transform:\n" << tool_with_tcp.matrix() << std::endl;
        std::cout << "tooltcp position: [" << tool_with_tcp.translation().x() << ", " << tool_with_tcp.translation().y() << ", "
                  << tool_with_tcp.translation().z() << "]" << std::endl;

        Eigen::Quaterniond quat(tool_with_tcp.rotation());
        std::cout << "tooltcp orientation (quaternion): [" << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << "]"
                  << std::endl;
    }

    std::cout << "\n--- Test 3 with TCP: Joint 1 = 90 degrees ---" << std::endl;
    if (fk_results3.find("tool0") != fk_results3.end()) {
        Eigen::Isometry3d tool_with_tcp = fk_results3["tool0"] * tcp;
        std::cout << "tooltcp transform:\n" << tool_with_tcp.matrix() << std::endl;
        std::cout << "tooltcp position: [" << tool_with_tcp.translation().x() << ", " << tool_with_tcp.translation().y() << ", "
                  << tool_with_tcp.translation().z() << "]" << std::endl;

        Eigen::Quaterniond quat(tool_with_tcp.rotation());
        std::cout << "tooltcp orientation (quaternion): [" << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << "]"
                  << std::endl;
    }

    std::cout << "\nSetup complete! Ready for trajectory planning." << std::endl;

    //==========================================================================
    // READ TARGET PATH
    //==========================================================================
    std::cout << "\n=== Reading Target Path ===" << std::endl;

    // Read the target path from CSV
    std::filesystem::path input_csv_path = source_dir / "data" / "input.csv";
    std::vector<Eigen::Matrix4d> target_path;
    readCartesianPath(target_path, input_csv_path.string());
    std::cout << "Read " << target_path.size() << " waypoints from CSV file" << std::endl;

    // If points_to_consider wasn't specified or is too large, use all points
    if (points_to_consider <= 0 || points_to_consider > target_path.size()) {
        points_to_consider = target_path.size();
        std::cout << "Using all " << points_to_consider << " points from the toolpath" << std::endl;
    }

    //==========================================================================
    // LOAD DESCARTES SOLUTION
    //==========================================================================
    std::cout << "\n=== Loading Descartes Solution ===" << std::endl;

    // Load the Descartes solution
    std::filesystem::path descartes_solution_path = source_dir / "data" / "descartes_solution.json";
    std::vector<Eigen::VectorXd> descartes_trajectory_eigen;
    bool descartes_loaded = false; // Flag to track if loading was successful
    try {
        descartes_trajectory_eigen = loadDescartesSolution(descartes_solution_path.string());

        // Check if enough points were loaded to cover the points_to_consider
        if (descartes_trajectory_eigen.size() < points_to_consider) {
            std::cerr << "Warning: Loaded Descartes trajectory has only " << descartes_trajectory_eigen.size() << " points, but "
                      << points_to_consider << " are needed. Seed cannot be used." << std::endl;
            // descartes_loaded remains false
        } else {
            if (descartes_trajectory_eigen.size() > points_to_consider) {
                std::cout << "Successfully loaded Descartes solution with " << descartes_trajectory_eigen.size() << " points. Using the first "
                          << points_to_consider << " points as seed." << std::endl;
            } else { // Size exactly matches points_to_consider
                std::cout << "Successfully loaded Descartes solution with " << descartes_trajectory_eigen.size()
                          << " points (matching points_to_consider)." << std::endl;
            }
            descartes_loaded = true; // Set flag to true as we have enough points
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to load Descartes solution: " << e.what() << std::endl;
        std::cerr << "Continuing without Descartes seed..." << std::endl;
        // descartes_loaded remains false
    }

    if (seed_method == "pci" && use_ifopt) {
        std::cerr << "Warning: Seeding method 'pci' is not supported with "
                     "TrajOptIfopt. Seeding will be disabled."
                  << std::endl;
        seed_method = "none"; // Force disable seeding for this specific
                              // incompatible combination
    }
    // Check if Descartes solution is loaded if *any* seeding method requiring it
    // is selected
    if ((seed_method == "waypoint" || seed_method == "pci") && !descartes_loaded) {
        std::cerr << "Warning: Seeding method '" << seed_method
                  << "' requires a successfully loaded Descartes solution. Seeding "
                     "will be disabled."
                  << std::endl;
        seed_method = "none"; // Force disable seeding if required data isn't available
    }

    //==========================================================================
    // CREATE PLANNING PROFILES
    //==========================================================================
    std::cout << "\n=== Creating Planning Profiles ===" << std::endl;

    // Create a profile dictionary
    auto profiles = std::make_shared<ProfileDictionary>();

    if (use_ifopt) {
        // Create TrajOpt_Ifopt Profile
        auto trajopt_ifopt_plan_profile = std::make_shared<TrajOptIfoptDefaultPlanProfile>();
        trajopt_ifopt_plan_profile->joint_cost_config.enabled = false;
        trajopt_ifopt_plan_profile->cartesian_cost_config.enabled = false;
        trajopt_ifopt_plan_profile->cartesian_constraint_config.enabled = true;

        // Tool z-axis free
        trajopt_ifopt_plan_profile->cartesian_constraint_config.coeff = Eigen::VectorXd::Constant(6, 10);
        trajopt_ifopt_plan_profile->cartesian_constraint_config.coeff(5) = 0;

        auto trajopt_ifopt_composite_profile = std::make_shared<TrajOptIfoptDefaultCompositeProfile>();
        trajopt_ifopt_composite_profile->collision_constraint_config = nullptr;
        trajopt_ifopt_composite_profile->collision_cost_config->type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
        trajopt_ifopt_composite_profile->collision_cost_config->contact_manager_config = tesseract_collision::ContactManagerConfig(0.025);
        trajopt_ifopt_composite_profile->collision_cost_config->collision_coeff_data = trajopt_common::CollisionCoeffData(20);

        // Smoothing settings
        trajopt_ifopt_composite_profile->smooth_velocities = true;
        trajopt_ifopt_composite_profile->smooth_accelerations = true;
        trajopt_ifopt_composite_profile->smooth_jerks = false;
        Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(7);
        joint_weights[0] = 4;
        trajopt_ifopt_composite_profile->velocity_coeff = 5 * joint_weights;
        trajopt_ifopt_composite_profile->acceleration_coeff = 10 * joint_weights;

        auto trajopt_ifopt_solver_profile = std::make_shared<TrajOptIfoptOSQPSolverProfile>();
        trajopt_ifopt_solver_profile->opt_params.max_iterations = 200;
        trajopt_ifopt_solver_profile->opt_params.min_approx_improve = 1e-3;
        trajopt_ifopt_solver_profile->opt_params.min_trust_box_size = 1e-3;

        profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_ifopt_plan_profile);
        profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_ifopt_composite_profile);
        profiles->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_ifopt_solver_profile);
    } else {
        // Create TrajOpt Profile (non-IFOPT)
        auto trajopt_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
        trajopt_plan_profile->joint_cost_config.enabled = false;
        trajopt_plan_profile->cartesian_cost_config.enabled = false;
        trajopt_plan_profile->cartesian_constraint_config.enabled = true;

        // Tool z-axis free
        trajopt_plan_profile->cartesian_constraint_config.coeff = Eigen::VectorXd::Constant(6, 10);
        trajopt_plan_profile->cartesian_constraint_config.coeff(5) = 0;

        auto trajopt_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
        trajopt_composite_profile->collision_constraint_config.enabled = false;
        trajopt_composite_profile->collision_cost_config.enabled = true;
        trajopt_composite_profile->collision_cost_config.safety_margin = 0.025;
        trajopt_composite_profile->collision_cost_config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
        trajopt_composite_profile->collision_cost_config.coeff = 20;

        // Smoothing settings
        trajopt_composite_profile->smooth_velocities = true;
        trajopt_composite_profile->smooth_accelerations = true;
        trajopt_composite_profile->smooth_jerks = false;
        Eigen::VectorXd joint_weights = Eigen::VectorXd::Ones(7);
        joint_weights[0] = 4;
        trajopt_composite_profile->velocity_coeff = 5 * joint_weights;
        trajopt_composite_profile->acceleration_coeff = 10 * joint_weights;

        auto trajopt_solver_profile = std::make_shared<TrajOptOSQPSolverProfile>();
        trajopt_solver_profile->opt_params.num_threads = 0;
        trajopt_solver_profile->opt_params.max_iter = 200;
        trajopt_solver_profile->opt_params.min_approx_improve = 1e-3;
        trajopt_solver_profile->opt_params.min_trust_box_size = 1e-3;

        profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_plan_profile);
        profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite_profile);
        profiles->addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_solver_profile);
    }

    std::cout << "Profiles created successfully!" << std::endl;

    //==========================================================================
    // CREATE PROGRAM
    //==========================================================================
    std::cout << "\n=== Creating Trajectory Program ===" << std::endl;

    // Create main program
    CompositeInstruction program("DEFAULT", manip_info);

    // Get the joint names in the correct order
    std::vector<std::string> joint_names = kin_group->getJointNames(); // Get names from kin_group

    std::cout << "Adding " << points_to_consider << " waypoints with CARTESIAN motion type" << std::endl;
    for (std::size_t i = 0; i < points_to_consider; ++i) {
        // Convert Matrix4d to Isometry3d
        Eigen::Isometry3d current_pose;
        current_pose.matrix() = target_path[i];

        // Create a CartesianWaypoint for the current position
        CartesianWaypoint current_cartesian_waypoint(current_pose);

        // Set seed for the current waypoint if available
        if (seed_method == "waypoint") { // Check if Descartes data is loaded was done above
            if (i < descartes_trajectory_eigen.size()) { // Ensure index is valid
                if (descartes_trajectory_eigen[i].size() == joint_names.size()) {
                    tesseract_common::JointState current_seed_state;
                    current_seed_state.joint_names = joint_names;
                    current_seed_state.position = descartes_trajectory_eigen[i];
                    current_cartesian_waypoint.setSeed(current_seed_state);
                } else {
                    std::cerr << "Warning: Joint count mismatch for seed at waypoint " << i << ". Skipping seed." << std::endl;
                }
            } else {
                std::cerr << "Warning: Descartes trajectory index " << i << " out of bounds. Skipping seed." << std::endl;
            }
        }

        // Create a MoveInstruction with LINEAR motion type
        MoveInstruction linear_instruction(current_cartesian_waypoint, MoveInstructionType::LINEAR, "CARTESIAN");
        linear_instruction.setDescription("waypoint_" + std::to_string(i));

        // Add to the program
        program.push_back(linear_instruction);

        if (i % 100 == 0 || i == points_to_consider - 1) {
            std::cout << "Added waypoint " << i + 1 << " of " << points_to_consider << std::endl;
        }
    }

    std::cout << "Created program with " << program.size() << " instructions" << std::endl;

    // Prepare Planner Request
    PlannerRequest request;
    request.env = env;
    request.profiles = profiles;
    request.instructions = program; // Set the target instructions
    request.data = nullptr; // Initialize data pointer to null

    // Create and Assign TrajOpt Problem with Seed via request.data (if loaded and
    // using TrajOpt)
    if (seed_method == "pci") {
        std::cout << "Attempting manual construction of ProblemConstructionInfo "
                     "for TrajOpt."
                  << std::endl;
        try {
            // 1. Initialize PCI and Get Kinematics
            std::shared_ptr<trajopt::ProblemConstructionInfo> pci = std::make_shared<trajopt::ProblemConstructionInfo>(request.env);

            // Extract manipulator info from the composite instruction
            const auto& composite_mi = request.instructions.getManipulatorInfo();
            pci->kin = request.env->getKinematicGroup(composite_mi.manipulator);
            if (pci->kin == nullptr) {
                throw std::runtime_error("Manual PCI: Kinematic group '" + composite_mi.manipulator + "' not found.");
            }
            std::vector<std::string> joint_names = pci->kin->getJointNames(); // Use names from PCI kin group
            std::size_t n_joints = joint_names.size();
            std::vector<std::string> active_links = pci->kin->getActiveLinkNames();

            // 2. Set Basic Info
            std::size_t n_steps_required = points_to_consider; // Use the number of points we are actually
                                                               // planning for
            pci->basic_info.n_steps = static_cast<int>(n_steps_required);
            pci->basic_info.manip = composite_mi.manipulator;
            pci->basic_info.use_time = false; // Assuming no time parameterization here
            // Identify fixed steps (e.g., start/end). Start with none.
            std::vector<int> fixed_steps;
            // Example: fix the first point if needed: fixed_steps.push_back(0);
            pci->basic_info.fixed_timesteps = fixed_steps;

            // 3. Apply Solver Parameters from Profile
            std::string solver_profile_ns = TRAJOPT_DEFAULT_NAMESPACE; // Adjust if needed
            std::string solver_profile_key = request.instructions.getProfile(); // Get composite profile key
            if (solver_profile_key.empty())
                solver_profile_key = "DEFAULT"; // Default if empty

            // Use the free function tesseract_planning::getProfile
            // Pass the dictionary by reference (*request.profiles)
            // Provide a default profile instance
            TrajOptSolverProfile::ConstPtr solver_profile =
                tesseract_planning::getProfile<TrajOptSolverProfile>(solver_profile_ns, solver_profile_key, *request.profiles,
                                                                     std::make_shared<TrajOptOSQPSolverProfile>()); // Default

            // The helper function already handles the fallback logic internally, but
            // we can add an explicit check if needed.
            if (!solver_profile) {
                // This case should ideally not happen if a default is provided and
                // valid, but adding robustness.
                CONSOLE_BRIDGE_logError(
                    "Manual PCI: Failed to retrieve or create a default TrajOpt solver "
                    "profile.");
                throw std::runtime_error("Manual PCI: Solver profile retrieval failed.");
            }
            // Note: The warning/fallback logic previously here is now handled by the
            // helper or the check above.

            pci->basic_info.convex_solver = solver_profile->getSolverType();
            pci->basic_info.convex_solver_config = solver_profile->createSolverConfig();
            pci->opt_info = solver_profile->createOptimizationParameters();
            pci->callbacks = solver_profile->createOptimizationCallbacks();

            // 4. Set Initial Trajectory (Seed)
            // Verify joint count consistency in the loaded seed data
            if (static_cast<std::size_t>(descartes_trajectory_eigen[0].size()) != n_joints) {
                throw std::runtime_error("Manual PCI: Mismatch between Descartes joint state size ("
                                         + std::to_string(static_cast<long long>(descartes_trajectory_eigen[0].size()))
                                         + ") and kinematic group joint count (" + std::to_string(static_cast<long long>(n_joints))
                                         + ") at waypoint 0.");
            }

            // Convert the required subset of Descartes trajectory to TrajArray
            trajopt::TrajArray initial_trajectory(static_cast<Eigen::Index>(n_steps_required), static_cast<Eigen::Index>(n_joints));
            for (std::size_t i = 0; i < n_steps_required; ++i) {
                if (static_cast<std::size_t>(descartes_trajectory_eigen[i].size()) != n_joints) {
                    throw std::runtime_error(
                        "Manual PCI: Inconsistent number of joints in Descartes "
                        "trajectory at index "
                        + std::to_string(static_cast<long long>(i)) + ". Expected " + std::to_string(static_cast<long long>(n_joints)) + ", got "
                        + std::to_string(static_cast<long long>(descartes_trajectory_eigen[i].size())));
                }
                initial_trajectory.row(static_cast<Eigen::Index>(i)) = descartes_trajectory_eigen[i];
            }
            std::cout << "Manual PCI: Created seed TrajArray with " << initial_trajectory.rows() << " steps and " << initial_trajectory.cols()
                      << " joints." << std::endl;

            pci->init_info.type = trajopt::InitInfo::GIVEN_TRAJ;
            pci->init_info.data = initial_trajectory;

            // 5. Generate Costs and Constraints from Profiles
            // Flatten instructions to get individual MoveInstructions
            auto flat_instructions = request.instructions.flatten(tesseract_planning::moveFilter);
            if (flat_instructions.size() != n_steps_required) {
                throw std::runtime_error("Manual PCI: Number of flattened move instructions (" + std::to_string(flat_instructions.size())
                                         + ") does not match points_to_consider (" + std::to_string(n_steps_required) + ").");
            }

            // Process individual waypoint profiles (Plan Profiles)
            for (int i = 0; i < static_cast<int>(n_steps_required); ++i) {
                const auto& move_instruction = flat_instructions[static_cast<std::size_t>(i)].get().as<MoveInstructionPoly>();

                // Get Plan Profile (similar logic to solver profile retrieval)
                std::string plan_profile_ns = TRAJOPT_DEFAULT_NAMESPACE; // Adjust if needed
                std::string plan_profile_key = move_instruction.getProfile();
                if (plan_profile_key.empty())
                    plan_profile_key = "DEFAULT"; // Default if empty

                // Use the free function tesseract_planning::getProfile
                TrajOptPlanProfile::ConstPtr cur_plan_profile =
                    tesseract_planning::getProfile<TrajOptPlanProfile>(plan_profile_ns, plan_profile_key, *request.profiles,
                                                                       std::make_shared<TrajOptDefaultPlanProfile>()); // Default

                if (!cur_plan_profile) {
                    // This case should ideally not happen if a default is provided and
                    // valid.
                    CONSOLE_BRIDGE_logError(
                        "Manual PCI: Failed to retrieve or create a default TrajOpt plan "
                        "profile for instruction %d.",
                        i);
                    throw std::runtime_error("Manual PCI: Plan profile retrieval failed for instruction " + std::to_string(i));
                }
                // Note: The warning/fallback logic previously here is now handled by
                // the helper or the check above.

                // Create waypoint-specific term infos
                // Note: The seed from wp_info is ignored here as we set it globally
                // above
                TrajOptWaypointInfo wp_info = cur_plan_profile->create(move_instruction, composite_mi, request.env, active_links, i);

                if (!wp_info.term_infos.costs.empty())
                    pci->cost_infos.insert(pci->cost_infos.end(), wp_info.term_infos.costs.begin(), wp_info.term_infos.costs.end());

                if (!wp_info.term_infos.constraints.empty())
                    pci->cnt_infos.insert(pci->cnt_infos.end(), wp_info.term_infos.constraints.begin(), wp_info.term_infos.constraints.end());

                // Note: We are ignoring wp_info.fixed here and using the global
                // fixed_steps vector
            }

            // Process composite profile
            std::string composite_profile_ns = TRAJOPT_DEFAULT_NAMESPACE; // Adjust if needed
            std::string composite_profile_key = request.instructions.getProfile(); // Get composite profile key
            if (composite_profile_key.empty())
                composite_profile_key = "DEFAULT"; // Default if empty

            // Use the free function tesseract_planning::getProfile
            TrajOptCompositeProfile::ConstPtr composite_profile =
                tesseract_planning::getProfile<TrajOptCompositeProfile>(composite_profile_ns, composite_profile_key, *request.profiles,
                                                                        std::make_shared<TrajOptDefaultCompositeProfile>()); // Default

            if (!composite_profile) {
                // This case should ideally not happen if a default is provided and
                // valid.
                CONSOLE_BRIDGE_logError(
                    "Manual PCI: Failed to retrieve or create a default TrajOpt "
                    "composite profile.");
                throw std::runtime_error("Manual PCI: Composite profile retrieval failed.");
            }
            // Note: The warning/fallback logic previously here is now handled by the
            // helper or the check above.

            TrajOptTermInfos c_term_infos = composite_profile->create(composite_mi, request.env, fixed_steps, 0, pci->basic_info.n_steps - 1);

            if (!c_term_infos.costs.empty())
                pci->cost_infos.insert(pci->cost_infos.end(), c_term_infos.costs.begin(), c_term_infos.costs.end());

            if (!c_term_infos.constraints.empty())
                pci->cnt_infos.insert(pci->cnt_infos.end(), c_term_infos.constraints.begin(), c_term_infos.constraints.end());

            // 6. Assign the fully constructed PCI to request.data
            request.data = std::static_pointer_cast<void>(pci);
            std::cout << "Manual PCI construction successful. Assigned to request.data." << std::endl;

        } catch (const std::exception& e) {
            std::cerr << "Error during manual PCI construction: " << e.what() << std::endl;
            std::cerr << "Falling back to planner's internal problem generation (no "
                         "seed via request.data)."
                      << std::endl;
            request.data = nullptr; // Ensure data is null if manual construction failed
        }
    }

    //==========================================================================
    // SOLVE TRAJECTORY USING DIRECT PLANNER
    //==========================================================================
    std::cout << "\n=== Solving Trajectory Using Direct Planner ===" << std::endl;

    // Create planner based on use_ifopt flag
    PlannerResponse response;
    if (use_ifopt) {
        std::cout << "Using TrajOptIfoptMotionPlanner" << std::endl;
        TrajOptIfoptMotionPlanner ifopt_planner(TRAJOPT_IFOPT_DEFAULT_NAMESPACE);

        // Solve trajectory
        std::cout << "Starting trajectory optimization..." << std::endl;
        tesseract_common::Stopwatch stopwatch;
        stopwatch.start();
        response = ifopt_planner.solve(request);
        stopwatch.stop();

        std::cout << "Planning took " << stopwatch.elapsedSeconds() << " seconds." << std::endl;
    } else {
        std::cout << "Using TrajOptMotionPlanner" << std::endl;
        TrajOptMotionPlanner planner(TRAJOPT_DEFAULT_NAMESPACE);

        // Solve trajectory
        std::cout << "Starting trajectory optimization..." << std::endl;
        tesseract_common::Stopwatch stopwatch;
        stopwatch.start();
        response = planner.solve(request);
        stopwatch.stop();

        std::cout << "Planning took " << stopwatch.elapsedSeconds() << " seconds." << std::endl;
    }

    // Create output directory if it doesn't exist
    std::filesystem::path output_dir = source_dir / "output";
    if (!std::filesystem::exists(output_dir)) {
        std::filesystem::create_directories(output_dir);
    }

    // Try to extract and save trajectory even if planning failed
    if (response.successful) {
        try {
            // Extract the trajectory
            tesseract_common::JointTrajectory trajectory = toJointTrajectory(response.results);

            // Print trajectory information
            std::cout << "Trajectory has " << trajectory.size() << " waypoints" << std::endl;

            // Convert to vector of joint positions and velocities for saving
            std::vector<Eigen::VectorXd> joint_positions;
            std::vector<Eigen::VectorXd> joint_velocities;

            for (const auto& state : trajectory) {
                joint_positions.push_back(state.position);
                if (state.velocity.size() > 0) {
                    joint_velocities.push_back(state.velocity);
                }
            }

            // Define file name components
            std::string planner_type = use_ifopt ? "trajopt_ifopt" : "trajopt";
            std::string status = "success";

            // Save the results
            std::string joint_file = (output_dir / (planner_type + "_joint_solutions_" + status + ".csv")).string();
            saveTrajectory(joint_positions, joint_file);
            std::cout << "Joint positions saved to " << joint_file << std::endl;

            if (!joint_velocities.empty()) {
                std::string vel_file = (output_dir / (planner_type + "_joint_velocities_" + status + ".csv")).string();
                saveTrajectory(joint_velocities, vel_file);
                std::cout << "Joint velocities saved to " << vel_file << std::endl;
            }

        } catch (const std::exception& e) {
            std::cerr << "Error extracting trajectory data: " << e.what() << std::endl;

            // Save error information
            std::ofstream error_file(output_dir / "extraction_error.txt");
            error_file << "Error: " << e.what() << std::endl;
            error_file.close();
        }
    } else {
        std::cerr << "\n=== Planning Failed ===" << std::endl;
        std::cerr << "Error message: " << response.message << std::endl;

        // Save failure information with timestamp
        std::string planner_type = use_ifopt ? "trajopt_ifopt" : "trajopt";
        std::string status = "failed";
        std::ofstream error_file((output_dir / (planner_type + "_planning_" + status + ".txt")).string());
        error_file << "Planning failed: " << response.message << std::endl;
        error_file.close();
    }

    return 0;
}
