//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.hpp>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;
    cout << "E= " << endl << E << endl;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    JacobiSVD<Matrix3d> svd(E, ComputeFullU | ComputeFullV);
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();
    Vector3d Sing = svd.singularValues();
    cout << "singular values: " << Sing.transpose() << endl;
    Eigen::DiagonalMatrix<double, 3> S((Sing[0]+Sing[1])/2, (Sing[0]+Sing[1])/2, 0);
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    // compute rotation matrix from Sophus::SO3d?
    // (x, y) -> (-y, x)
    Matrix3d Rz_p90 = AngleAxisd(M_PI/2.0, Vector3d(0, 0, 1)).toRotationMatrix();
    // (x, y) -> (y, -x)
    Matrix3d Rz_n90 = AngleAxisd(-M_PI/2.0, Vector3d(0, 0, 1)).toRotationMatrix();
    cout << "Rotate 90: " << endl << Rz_p90 << endl;
    cout << "Rotate -90: " << endl << Rz_n90 << endl;
    // t_wedge1 = t_wedge2 * (-1)
    Matrix3d t_wedge1 = U * Rz_p90 * S * U.transpose(); //上尖尖
    Matrix3d t_wedge2 = U * Rz_n90 * S * U.transpose();

    Matrix3d R1 = U * Rz_p90.transpose() * V.transpose();
    Matrix3d R2 = U * Rz_n90.transpose() * V.transpose();
    // END YOUR CODE HERE

    cout << "R1 = " << endl << R1 << endl;
    cout << "R2 = " << endl << R2 << endl;
    cout << "t1 = " << Sophus::SO3d::vee(t_wedge1).transpose() << endl;
    cout << "t2 = " << Sophus::SO3d::vee(t_wedge2).transpose() << endl;

    // check t^R=E up to scale
    Matrix3d tR1 = t_wedge1 * R1;
    Matrix3d tR2 = t_wedge1 * R2;
    Matrix3d tR3 = t_wedge2 * R1;
    Matrix3d tR4 = t_wedge2 * R2;
    // tR1 equals tR4 and tR2 equals tR3?
    // tR1 and tR4 equals to E
    cout << "t^R = " << endl << tR1 << endl;
    cout << "t^R = " << endl << tR2 << endl;
    cout << "t^R = " << endl << tR3 << endl;
    cout << "t^R = " << endl << tR4 << endl;

    return 0;
}