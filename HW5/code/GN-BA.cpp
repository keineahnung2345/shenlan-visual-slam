//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include <sophus/se3.hpp>

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector2d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "./p3d.txt";
string p2d_file = "./p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    std::ifstream infile(p3d_file);
    double x, y, z;
    while (infile >> x >> y >> z){
        p3d.emplace_back(x, y, z);
    }
    infile = ifstream(p2d_file);
    while (infile >> x >> y){
        p2d.emplace_back(x, y);
    }
    
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3d T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE 
            // world coord -> camera coord: P'
            Vector3d p3d_cam = T_esti.rotationMatrix() * p3d[i] + T_esti.translation(); //T_esti * p3d[i];?
            // depth: p3d_cam[2], not p3d[i][2]!
            Vector2d e = p2d[i] - (1/p3d_cam[2] * K * p3d_cam).block<2,1>(0,0);
            cost += pow(e.norm(), 2);
	        // END YOUR CODE HERE

	        // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE 
            double Xp = p3d_cam.coeff(0), Yp = p3d_cam.coeff(1), Zp = p3d_cam.coeff(2);
            J(0, 0) = fx / Zp;
            J(0, 1) = 0;
            J(0, 2) = -fx * Xp/ (Zp * Zp);
            J(0, 3) = -fx * Xp * Yp / (Zp * Zp);
            J(0, 4) = fx + fx * Xp * Xp / (Zp * Zp);
            J(0, 5) = -fx * Yp / Zp;
            J(1, 0) = 0;
            J(1, 1) = fy / Zp;
            J(1, 2) = -fy * Yp / (Zp * Zp);
            J(1, 3) = -fy - fy * Yp * Yp / (Zp * Zp);
            J(1, 4) = fy * Xp * Yp / (Zp * Zp);
            J(1, 5) = fy * Xp / Zp;
            J *= -1;
	        // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	    // solve dx 
        Vector6d dx;

        // START YOUR CODE HERE 
        dx = H.ldlt().solve(b);
        // cout << "dx: " << dx.transpose() << endl;
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        // xi: [translation, rotation]
        T_esti.translation() += dx.head(3);
        // T_esti.rotationMatrix() returns const, it can't be updated!
        T_esti.setRotationMatrix(Sophus::SE3d::exp(dx.tail(3)).rotationMatrix() * T_esti.rotationMatrix()); //?
        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
        // cout << "T_esti: " << endl << T_esti.matrix() << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
