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
#include <unordered_set>
#include <unistd.h>

#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

#include <opencv2/core/core.hpp>


using namespace std;

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 7, 1> Vector7d;
typedef vector<Vector7d, Eigen::aligned_allocator<Vector7d>> VecVector7d;
typedef vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> VecVector3f;

string traj_file = "./compare.txt";
vector<pair<int, int>> dummy_matches;

void DrawTwoTrajectories(vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>>&,
    vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>>&, 
    int iter = -1, vector<pair<int, int>>& matches = dummy_matches);

void pose_estimation_3d3d(
  const VecVector3f &pts1,
  const VecVector3f &pts2,
  Eigen::Matrix3f &R, Eigen::Vector3f &t
);

void write_pointcloud_file(const string &fname, 
    const vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>> &poses);

int main(int argc, char **argv) {

    vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>> traj_est;
    vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>> traj_gt;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    std::ifstream infile(traj_file);
    // quaternion order: (s, v) or (v, s)
    double te, xe, ye, ze, qxe, qye, qze, qse;
    double tg, xg, yg, zg, qxg, qyg, qzg, qsg;
    while (infile >> te >> xe >> ye >> ze >> qxe >> qye >> qze >> qse
                  >> tg >> xg >> yg >> zg >> qxg >> qyg >> qzg >> qsg){
        traj_est.push_back(Sophus::SE3f(Eigen::Quaternionf(qse, qxe, qye, qze), Eigen::Vector3f(xe, ye, ze)));
        traj_gt.push_back(Sophus::SE3f(Eigen::Quaternionf(qsg, qxg, qyg, qze), Eigen::Vector3f(xg, yg, zg)));
    }

    // END YOUR CODE HERE
    assert(traj_est.size() == traj_gt.size());
    // cout << "There are " << traj_gt.size() << " points." << endl;

    DrawTwoTrajectories(traj_gt, traj_est);

    write_pointcloud_file("traj_gt.txt", traj_gt);
    write_pointcloud_file("traj_est.txt", traj_est);

    VecVector3f pts1, pts2;
    for(size_t i = 0; i < traj_gt.size(); ++i){
        Eigen::Vector3f traj_gt_pt = traj_gt[i].translation();
        Eigen::Vector3f traj_est_pt = traj_est[i].translation();
        pts1.push_back(traj_gt_pt);
        pts2.push_back(traj_est_pt);
    }
    // 如果知道兩點雲間的匹配,可以用SVD一次就求出解,不需迭代(ICP)
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    pose_estimation_3d3d(pts1, pts2, R, t);
    cout << "ICP via SVD results: " << endl;
    // p1 = R*p2+t
    cout << "R = " << R << endl;
    cout << "t = " << t.transpose() << endl;
    // p2 = R^(-1) * (p1-t) = R^(-1) * p1 - R^(-1) * t
    cout << "R_inv = " << R.transpose() << endl;
    cout << "t_inv = " << (-R.transpose() * t).transpose() << endl;

    Sophus::SE3f T = Sophus::SE3f(Sophus::Matrix3f(R), Sophus::Vector3f(t));
    vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>> traj_est_aligned;
    for(Sophus::SE3f& pt : traj_est){
        traj_est_aligned.push_back((T * pt));
    }
    DrawTwoTrajectories(traj_gt, traj_est_aligned, 0);

    write_pointcloud_file("traj_est_aligned.txt", traj_est_aligned);

    return 0;
}

void DrawTwoTrajectories(vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>>& poses1,
    vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>>& poses2, 
    int iter, vector<pair<int, int>>& matches) {
    if (poses1.empty() || poses2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    if (poses1.size() != poses2.size()){
        cerr << "The two trajectories' size are not equal!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    string title = "Groundtruth: red, Estimated: blue";
    if(iter >= 0) title += " - " + to_string(iter);
    pangolin::CreateWindowAndBind(title, 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses1.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3f(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3f(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            p1 = poses2[i], p2 = poses2[i + 1];
            glVertex3f(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3f(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (const pair<int, int>& match : matches) {
            glColor3f(0.0f, 1.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = poses1[match.second], p2 = poses2[match.first];
            glVertex3f(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3f(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}

void pose_estimation_3d3d(const VecVector3f &pts1,
                          const VecVector3f &pts2,
                          Eigen::Matrix3f &R, Eigen::Vector3f &t) {
  Eigen::Vector3f p1 = Eigen::Vector3f::Zero(), p2 = Eigen::Vector3f::Zero();     // center of mass
  int N = pts1.size();
  for (int i = 0; i < N; i++) {
    p1 += pts1[i];
    p2 += pts2[i];
  }
  p1 /= N;
  p2 /= N;
  cout << "cloud 1 centroid: " << p1.transpose() << endl;
  cout << "cloud 2 centroid: " << p2.transpose() << endl;

  VecVector3f q1(N), q2(N); // remove the center
  for (int i = 0; i < N; i++) {
    q1[i] = pts1[i] - p1;
    q2[i] = pts2[i] - p2;
  }

  // compute q1*q2^T
  // W = sigma (q1i * q2i^T)
  Eigen::Matrix3f W = Eigen::Matrix3f::Zero();
  for (int i = 0; i < N; i++) {
    W += q1[i] * q2[i].transpose();
  }
  cout << "W=" << W << endl;

  // SVD on W
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f U = svd.matrixU();
  Eigen::Matrix3f V = svd.matrixV();

  cout << "U=" << U << endl;
  cout << "V=" << V << endl;

  // 用SVD求解R = U * V^T
  R = U * (V.transpose());
  // 如果R_的判別式小於0,則把它乘上-1,使它變為一個旋轉矩陣.
  // 還可以這樣?
  if (R.determinant() < 0) {
    R = -R;
  }
  /**
   * p1: target
   * p2: source
   * 取t = p1 - Rp2
   **/
  t = p1 - R * p2;

}

void write_pointcloud_file(const string &fname, 
    const vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>> &poses){
    // output point cloud file
    
    std::ofstream outfile(fname);
    for(const Sophus::SE3f& pose : poses){
        Sophus::Vector3f t = pose.translation();
        outfile << t.x() << ", " << t.y() << ", " << t.z() << endl;
    }
    outfile.close();
}