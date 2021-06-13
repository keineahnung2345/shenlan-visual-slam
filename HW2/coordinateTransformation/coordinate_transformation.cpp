#include <iostream>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

int main(int argc, char** argv){
    Eigen::Quaterniond q1(0.55, 0.3, 0.2, 0.2);
    Eigen::Quaterniond q2(-0.1,0.3,-0.7,0.2);

    Eigen::Vector3d t1(0.7, 1.1, 0.2);
    Eigen::Vector3d t2(-0.1, 0.4, 0.8);

    Eigen::Vector3d p_in_1(0.5, -0.1, 0.2);

    // they are required!
    q1.normalize();
    q2.normalize();

    // rotation matrix
    Eigen::Vector3d p_in_w = q1.toRotationMatrix().inverse() * (p_in_1 - t1);
    Eigen::Vector3d p_in_2 = q2.toRotationMatrix() * p_in_w + t2;
    cout << "p2: " << p_in_2.transpose() << endl;

    // quaternion overoaded * as rotation operator
    p_in_w = q1.inverse() * (p_in_1 - t1);
    p_in_2 = q2 * p_in_w + t2;
    cout << "p2: " << p_in_2.transpose() << endl;

    // quaternion, pure math
    Eigen::Quaterniond p1_quat(0, 0.5, -0.1, 0.2);
    Eigen::Quaterniond t1_quat(0, 0.7, 1.1, 0.2);
    Eigen::Quaterniond t2_quat(0, -0.1, 0.4, 0.8);
    Eigen::Quaterniond p_in_w_quat = Eigen::Quaterniond((q1.inverse() * p1_quat * q1).coeffs() - (q1.inverse() * t1_quat * q1).coeffs());
    Eigen::Quaterniond p_in_2_quat = Eigen::Quaterniond((q2 * p_in_w_quat * q2.inverse()).coeffs() + t2_quat.coeffs());
    cout << "p2: " << p_in_2_quat.coeffs().transpose() << endl;
    return 0;
}