#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h> //usleep

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

//cannot include sophus before pangolin?!
#include <sophus/se3.hpp>

using namespace std;

// path to trajectory file
string trajectory_file = "./trajectory.txt";
string groundtruth_file = "./groundtruth.txt";
string estimated_file = "./estimated.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>>);
void DrawTwoTrajectories(vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>>,
    vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>>);

int main(int argc, char **argv) {

    vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>> poses;
    vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>> groundtruth;
    vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>> estimated;

    /// implement pose reading code
    // start your code here (5~10 lines)
    std::ifstream infile(trajectory_file);
    float t, tx, ty, tz, qx, qy, qz, qw;
    while(infile >> t >> tx >> ty >> tz >> qx >> qy >> qz >> qw){
        poses.push_back(Sophus::SE3f(Eigen::Quaternionf(qw, qx, qy, qz), Eigen::Vector3f(tx, ty, tz)));
    }
    cout << "There are " << poses.size() << " timestamps." << endl;
    // end your code here

    // draw trajectory in pangolin
    DrawTrajectory(poses);

    infile = std::ifstream(groundtruth_file);
    while(infile >> t >> tx >> ty >> tz >> qx >> qy >> qz >> qw){
        groundtruth.push_back(Sophus::SE3f(Eigen::Quaternionf(qw, qx, qy, qz), Eigen::Vector3f(tx, ty, tz)));
    }

    infile = std::ifstream(estimated_file);
    while(infile >> t >> tx >> ty >> tz >> qx >> qy >> qz >> qw){
        estimated.push_back(Sophus::SE3f(Eigen::Quaternionf(qw, qx, qy, qz), Eigen::Vector3f(tx, ty, tz)));
    }

    float error = 0.0f, rmse = 0.0f;
    for(size_t i = 0; i < groundtruth.size(); ++i){
        error = (groundtruth[i].inverse() * estimated[i]).log().norm();
        rmse += error * error;
    }
    rmse /= groundtruth.size();
    rmse = sqrt(rmse);
    cout << "rmse: " << rmse << endl;

    //groundtruth: red, estimated: blue
    DrawTwoTrajectories(groundtruth, estimated);

    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
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
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}

void DrawTwoTrajectories(vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>> poses1,
    vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>> poses2) {
    if (poses1.empty() || poses2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    if (poses1.size() != poses2.size()){
        cerr << "The two trajectories' size are not equal!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Groundtruth: red, Estimated: blue", 1024, 768);
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
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            p1 = poses2[i], p2 = poses2[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}