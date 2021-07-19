#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h> //g2o::CameraParameters
#include <g2o/core/parameter.h> //g2o::Parameter
#include <g2o/core/robust_kernel.h> //g2o::RobustKernelHuber
#include <g2o/core/robust_kernel_impl.h>
#include <sophus/se3.hpp>
#include <chrono>

using namespace std;
using namespace cv;

void find_feature_matches(
    const Mat &img_1, const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);

// BA by g2o
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

typedef struct {
    int cam_id;
    int point_id;
    Eigen::Vector2d p2d;
}observation;

typedef struct camera{
    Sophus::SO3d R;
    Eigen::Vector3d t;
    double f;
    double k1;
    double k2;

    camera() {
        t = Eigen::Vector3d::Zero();
        f = 0;
        k1 = 0;
        k2 = 0;
    };

    Eigen::Vector2d proj(const Eigen::Vector3d& p3d) const{
        /**
         * https://grail.cs.washington.edu/projects/bal/
         * P  =  R * X + t       (conversion from world to camera coordinates)
         * p  = -P / P.z         (perspective division)
         * p' =  f * r(p) * p    (conversion to pixel coordinates)
         * r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4.
         **/
        Eigen::Vector3d cam_p3d = R * p3d + t;
        cam_p3d /= cam_p3d(2);
        cam_p3d *= -1; // BAL coordinate system
        Eigen::Vector2d p2d = cam_p3d.head(2);
        double r2 = p2d(0) * p2d(0) + p2d(1) * p2d(1);
        double r4 = r2 * r2;
        p2d = f * (1 + k1 * r2 + k2 * r4) * p2d;
        return p2d;
    }
}camera;

void bundleAdjustmentG2O(
    const vector<camera> &cams,
    const vector<Eigen::Vector3d> &points,
    const vector<observation> &obvs,
    vector<camera> &poses,
    vector<Eigen::Vector3d> &positions
);

int main(int argc, char **argv) {
    int num_cams, num_points, num_obvs;
    vector<observation> obvs;
    vector<camera> cams;
    vector<Eigen::Vector3d> points;
    std::ifstream infile("problem-93-61203-pre.txt");
    std::string line;
    int line_id = 0;
    int cam_cnt = 0, point_cnt = 0;
    while (std::getline(infile, line, '\n')){
        std::istringstream iss(line);
        if(line_id == 0){
            cout << "0: " << line_id << endl;
            iss >> num_cams >> num_points >> num_obvs;
            obvs.resize(num_obvs);
            cams.resize(num_cams);
            points.resize(num_points);
            line_id++;
        }else if(line_id >= 1 && line_id <= num_obvs){
            observation obv;
            iss >> obv.cam_id >> obv.point_id >> obv.p2d(0) >> obv.p2d(1);
            obvs[line_id-1] = obv;
            line_id++;
        }else if(line_id >= num_obvs+1 && line_id <= num_obvs+num_cams*9){
            camera cam;
            Eigen::Vector3d rv;
            iss >> rv(0);
            std::getline(infile, line, '\n'); iss = std::istringstream(line);
            iss >> rv(1);
            std::getline(infile, line, '\n'); iss = std::istringstream(line);
            iss >> rv(2);
            cam.R = Sophus::SO3d::exp(Eigen::Vector3d(rv));
            std::getline(infile, line, '\n'); iss = std::istringstream(line);
            iss >> cam.t(0);
            std::getline(infile, line, '\n'); iss = std::istringstream(line);
            iss >> cam.t(1);
            std::getline(infile, line, '\n'); iss = std::istringstream(line);
            iss >> cam.t(2);
            std::getline(infile, line, '\n'); iss = std::istringstream(line);
            iss >> cam.f;
            std::getline(infile, line, '\n'); iss = std::istringstream(line);
            iss >> cam.k1;
            std::getline(infile, line, '\n'); iss = std::istringstream(line);
            iss >> cam.k2;
            cams[(line_id-(num_obvs+1))/9] = cam;
            cam_cnt = (line_id-(num_obvs+1))/9;
            line_id += 9;
        }else if(line_id >= num_obvs+num_cams*9+1 && line_id <= num_obvs+num_cams*9+num_points*3){
            Eigen::Vector3d point;
            iss >> point(0);
            std::getline(infile, line, '\n'); iss = std::istringstream(line);
            iss >> point(1);
            std::getline(infile, line, '\n'); iss = std::istringstream(line);
            iss >> point(2);
            points[(line_id-(num_obvs+num_cams*9+1))/3] = point;
            point_cnt = (line_id-(num_obvs+num_cams*9+1))/3;
            line_id += 3;
        }
    }

    cout << "cam: " << cam_cnt << endl;
    cout << "point: " << point_cnt << endl;

    cout << "cams: " << cams.size() << endl;
    cout << "points: " << points.size() << endl;
    cout << "obvs: " << obvs.size() << endl;

    cout << "calling bundle adjustment by g2o" << endl;
    vector<camera> poses_g2o;
    vector<Eigen::Vector3d> positions_g2o;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    bundleAdjustmentG2O(cams, points, obvs, poses_g2o, positions_g2o);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp by g2o cost time: " << time_used.count() << " seconds." << endl;

    ofstream pcfile;
    pcfile.open ("pointcloud.txt");
    for(size_t i = 0; i < points.size(); ++i){
        pcfile << points[i](0) << ", " << points[i](1) << ", " << points[i](2) << endl;
    }
    pcfile.close();
    pcfile.open ("pointcloud_BA.txt");
    for(size_t i = 0; i < positions_g2o.size(); ++i){
        pcfile << positions_g2o[i](0) << ", " << positions_g2o[i](1) << ", " << positions_g2o[i](2) << endl;
    }
    pcfile.close();

    return 0;
}

Point2d pixel2cam(const Point2d &p, const Mat &K) {
    return Point2d
        (
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
        );
}

/// vertex and edges used in g2o ba
// 9個自由度,用camera儲存
class VertexCamera : public g2o::BaseVertex<9, camera> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override {
        // cout << "pose setToOriginImpl" << endl;
        _estimate = camera();
    }

    /// left multiplication on SE3
    virtual void oplusImpl(const double *update) override {
        // cout << "pose oplusImpl" << endl;
        // 擾動模型:求解李代數小量se(3),然後將它轉成李群表示,之後左乘到估計的pose上
        Sophus::SO3d update_R = Sophus::SO3d::exp(Eigen::Vector3d(update[0], update[1], update[2]));
        Eigen::Vector3d update_t = Eigen::Vector3d(update[3], update[4], update[5]);
        _estimate.R = update_R * _estimate.R;
        _estimate.t += update_t;
        _estimate.f += update[6];
        _estimate.k1 += update[7];
        _estimate.k2 += update[8];
    }

    virtual bool read(istream &in) override {}

    virtual bool write(ostream &out) const override {}
};

class VertexLandmark : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override {
        // cout << "landmark setToOriginImpl" << endl;
        _estimate = Eigen::Vector3d::Zero();
    }

    /// left multiplication on SE3
    virtual void oplusImpl(const double *update) override {
        // cout << "landmark oplusImpl" << endl;
        // 擾動模型:求解李代數小量se(3),然後將它轉成李群表示,之後左乘到估計的pose上
        Eigen::Vector3d update_eigen;
        update_eigen << update[0], update[1], update[2];
        _estimate = update_eigen + _estimate;
    }

    virtual bool read(istream &in) override {}

    virtual bool write(ostream &out) const override {}
};

// <函數輸出值(觀測值)的個數,函數輸出值(觀測值)的類型,待估計變量(頂點)的類型>
class EdgeProjection : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexCamera, VertexLandmark> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjection() {}

    virtual void computeError() override {
        // cout << "computeError" << endl;
        const VertexCamera *vc = static_cast<VertexCamera *> (_vertices[0]);
        const VertexLandmark *vl = static_cast<VertexLandmark *> (_vertices[1]);
        camera cam = vc->estimate();
        Eigen::Vector3d p3d = vl->estimate();
        Eigen::Vector2d pos_pixel = cam.proj(p3d);
        _error = _measurement - pos_pixel;
        // cout << "computeError end" << endl;
    }

    virtual bool read(istream &in) override {}

    virtual bool write(ostream &out) const override {}
};

void bundleAdjustmentG2O(
    const vector<camera> &cams,
    const vector<Eigen::Vector3d> &points,
    const vector<observation> &obvs,
    vector<camera> &poses,
    vector<Eigen::Vector3d> &positions) {

    cout << "cams: " << cams.size() << endl;
    cout << "points: " << points.size() << endl;
    cout << "obvs: " << obvs.size() << endl;

    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 3>> BlockSolverType;    // camera is 9, landmark is 3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    /**
     * use g2o::OptimizationAlgorithmGaussNewton will lead to divergence
     * use g2o::OptimizationAlgorithmLevenberg will lead to convergence
     **/
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;         // 图模型
    optimizer.setAlgorithm(solver);     // 设置求解器
    optimizer.setVerbose(true);             // 打开调试输出

    // vertex
    cout << "adding cam vertex" << endl;
    vector<VertexCamera*> vertex_cameras(cams.size());
    for(size_t i = 0; i < cams.size(); ++i){
        VertexCamera *vertex_camera = new VertexCamera(); // camera vertex_camera
        vertex_camera->setId(i);
        vertex_camera->setEstimate(cams[i]);
        optimizer.addVertex(vertex_camera);
        vertex_cameras[i] = vertex_camera;
    }

    cout << "adding point vertex" << endl;
    vector<VertexLandmark*> vertex_landmarks(points.size());
    for(size_t i = 0; i < points.size(); ++i){
        VertexLandmark *vertex_landmark = new VertexLandmark();
        vertex_landmark->setId(cams.size() + i);
        vertex_landmark->setEstimate(points[i]);
        /**
         * this solves:
         * terminate called after throwing an instance of 'std::bad_alloc'
         * what():  std::bad_alloc
         * Aborted (core dumped)
         **/
        vertex_landmark->setMarginalized(true);
        optimizer.addVertex(vertex_landmark);
        vertex_landmarks[i] = vertex_landmark;
    }

    // // camera parameters
    // for(size_t i = 0; i < cams.size(); ++i){
    //     g2o::Parameter* camera = new DistortedCameraParameters(cams[i].f, Eigen::Vector2d::Zero(), cams[i].k1, cams[i].k2);
    //     camera->setId(i);
    //     optimizer.addParameter(camera);
    // }

    // edges
    cout << "adding edge" << endl;
    vector<EdgeProjection*> edges(obvs.size());
    int index = 1;
    for (size_t i = 0; i < obvs.size(); ++i) {
        EdgeProjection *edge = new EdgeProjection();
        // edge->setId(index);
        edge->setVertex(0, vertex_cameras[obvs[i].cam_id]);
        edge->setVertex(1, vertex_landmarks[obvs[i].point_id]);
        edge->setMeasurement(obvs[i].p2d);
        edge->setInformation(Eigen::Matrix2d::Identity());
        // edge->setParameterId(0, 0);
        // 核函数
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        edges[i] = edge;
        index++;
    }

    cout << "optimize..." << endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
    for(VertexCamera* vertex_camera : vertex_cameras){
        poses.push_back(vertex_camera->estimate());
    }
    for(VertexLandmark* vertex_landmark : vertex_landmarks){
        positions.push_back(vertex_landmark->estimate());
    }
}
