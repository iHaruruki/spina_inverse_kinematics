#include <iostream>
#include <Eigen/Dense>
#include <cmath>

// Convenience function to create a 4x4 homogeneous translation matrix.
Eigen::Matrix4d translate(double x, double y, double z) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = x;
    T(1, 3) = y;
    T(2, 3) = z;
    return T;
}

// Create a homogeneous rotation matrix around the X axis.
Eigen::Matrix4d rotX(double angle_rad) {
    Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
    double c = cos(angle_rad);
    double s = sin(angle_rad);
    R(1, 1) = c;  R(1, 2) = -s;
    R(2, 1) = s;  R(2, 2) = c;
    return R;
}

// Create a homogeneous rotation matrix around the Y axis.
Eigen::Matrix4d rotY(double angle_rad) {
    Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
    double c = cos(angle_rad);
    double s = sin(angle_rad);
    R(0, 0) = c;  R(0, 2) = s;
    R(2, 0) = -s; R(2, 2) = c;
    return R;
}

// Given a roll and pitch, compute the transformation for one module.
// The URDF defines for each module:
//   - A fixed translation before the roll joint: (0,0,0.075)
//   - A roll rotation about x axis.
//   - A pitch rotation about y axis (with no additional translation between the two joints)
//   - A fixed translation after the pitch joint: (0,0,0.075)
// Note: In your URDF sometimes the joint origin for pitch is (0,0,0) and then a fixed transform follows.
Eigen::Matrix4d computeModuleTransform(double roll, double pitch) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    // Fixed translation before roll joint
    T = translate(0, 0, 0.075) * T;
    // Apply roll rotation (about x-axis)
    T = rotX(roll) * T;
    // Apply pitch rotation (about y-axis)
    T = rotY(pitch) * T;
    // Fixed translation after the pitch joint to reach the upper link
    T = translate(0, 0, 0.075) * T;
    return T;
}

int main() {
    // In a real-world application these values would be received from joint_state_publisher.
    // Here we hard code some sample joint angles (in radians) for modules 1 through 6.
    struct ModuleAngles {
        double roll;
        double pitch;
    };

    ModuleAngles modules[6] = {
        {0.1, 0.2},   // module1
        {0.15, 0.1},  // module2
        {0.0, -0.2},  // module3
        {0.05, 0.0},  // module4
        {-0.1, 0.05}, // module5
        {0.2, -0.1}   // module6
    };

    // Start from the base_link.
    Eigen::Matrix4d T_total = Eigen::Matrix4d::Identity();

    // For module 1, the fixed joint from base_link to module1_lower_link is identity (as defined).
    // Now chain the transformations for modules 1 to 6.
    for (int i = 0; i < 6; ++i) {
        Eigen::Matrix4d T_module = computeModuleTransform(modules[i].roll, modules[i].pitch);
        T_total = T_total * T_module;
    }

    // The position of module6_upper_link is the translation part of the final transform.
    Eigen::Vector3d module6_position = T_total.block<3,1>(0, 3);

    std::cout << "Module6 position (x, y, z): "
              << module6_position.transpose() << std::endl;

    return 0;
}
