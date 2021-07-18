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

typedef struct {
    Eigen::Vector3d rv; //Rodrigues' vector
    // Sophus::SO3d R;
    Eigen::Vector3d t;
    // Sophus::SE3d T;
    double f;
    double k1;
    double k2;
    Sophus::SE3d s = Sophus::SE3d();

    Sophus::SE3d T() const{
        Eigen::AngleAxisd aa(rv.norm(), rv/rv.norm());
        Eigen::Quaterniond q(aa);
        // R = Sophus::SO3d(q);
        return Sophus::SE3d(q, t);
    };

    Eigen::Vector2d proj(const Sophus::SE3d& T, const Eigen::Vector3d& p3d) const{
        /**
         * https://grail.cs.washington.edu/projects/bal/
         * P  =  R * X + t       (conversion from world to camera coordinates)
         * p  = -P / P.z         (perspective division)
         * p' =  f * r(p) * p    (conversion to pixel coordinates)
         * r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4.
         **/
        Eigen::Vector3d cam_p3d = T * p3d;
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
    vector<Sophus::SE3d> &poses,
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
            iss >> cam.rv(0);
            std::getline(infile, line, '\n'); iss = std::istringstream(line);
            iss >> cam.rv(1);
            std::getline(infile, line, '\n'); iss = std::istringstream(line);
            iss >> cam.rv(2);
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
    vector<Sophus::SE3d> poses_g2o;
    vector<Eigen::Vector3d> positions_g2o;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    bundleAdjustmentG2O(cams, points, obvs, poses_g2o, positions_g2o);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp by g2o cost time: " << time_used.count() << " seconds." << endl;
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
// 轉換矩陣有6個自由度,用SE3d儲存
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override {
        // cout << "pose setToOriginImpl" << endl;
        _estimate = Sophus::SE3d();
    }

    /// left multiplication on SE3
    virtual void oplusImpl(const double *update) override {
        // cout << "pose oplusImpl" << endl;
        // 擾動模型:求解李代數小量se(3),然後將它轉成李群表示,之後左乘到估計的pose上
        Eigen::Matrix<double, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }

    virtual bool read(istream &in) override {}

    virtual bool write(ostream &out) const override {}

    void setIntrinsicParameters(const camera& cam) {
        this->cam = cam;
    }

    camera cam;
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

class DistortedCameraParameters : public g2o::Parameter {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 
    double focal_length;
    Eigen::Vector2d principle_point;
    double k1, k2; //radial distortion

    DistortedCameraParameters (double focal_length, const Eigen::Vector2d &principle_point, double k1, double k2) {
        this->focal_length = focal_length;
        this->principle_point = principle_point;
        this->k1 = k1;
        this->k2 = k2;
    }

    Eigen::Vector2d cam_map(const Eigen::Vector3d &trans_xyz) const {
        Eigen::Vector2d proj = g2o::project(trans_xyz);
        double r2 = proj.norm() * proj.norm();
        double r4 = r2 * r2;
        proj(0) *= (1 + k1 * r2 + k2 * r4);
        proj(1) *= (1 + k1 * r2 + k2 * r4);
        Eigen::Vector2d res;
        res[0] = proj[0]*focal_length + principle_point[0];
        res[1] = proj[1]*focal_length + principle_point[1];
        return res;
    }

    virtual bool read(istream &in) override {}

    virtual bool write(ostream &out) const override {}
};

// <函數輸出值(觀測值)的個數,函數輸出值(觀測值)的類型,待估計變量(頂點)的類型>
class EdgeProjection : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPose, VertexLandmark> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjection() {}

    virtual void computeError() override {
        cout << "computeError" << endl;
        const VertexPose *vp = static_cast<VertexPose *> (_vertices[0]);
        const VertexLandmark *vl = static_cast<VertexLandmark *> (_vertices[1]);
        Sophus::SE3d T = vp->estimate();
        Eigen::Vector3d p3d = vl->estimate();
        Eigen::Vector2d pos_pixel = vp->cam.proj(T, p3d);
        _error = _measurement - pos_pixel;
        cout << "computeError end" << endl;
    }

    // virtual void linearizeOplus() override {
    //     const VertexPose *vp = static_cast<VertexPose *> (_vertices[0]);
    //     const VertexLandmark *vl = static_cast<VertexLandmark *> (_vertices[1]);
    //     // 估計出來的旋轉矩陣,即李群SE(3)
    //     camera cam = vp->cam;
    //     Sophus::SE3d T = vp->estimate();
    //     Eigen::Vector3d p3d = vl->estimate();
    //     Eigen::Vector3d pos_cam = T * p3d;
    //     double fx = cam.f;
    //     double fy = cam.f;
    //     double cx = 0;
    //     double cy = 0;
    //     double X = pos_cam[0];
    //     double Y = pos_cam[1];
    //     double Z = pos_cam[2];
    //     double Z2 = Z * Z;
    //     _jacobianOplusXi
    //         << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
    //         0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
    // }

    virtual bool read(istream &in) override {}

    virtual bool write(ostream &out) const override {}
};

void bundleAdjustmentG2O(
    const vector<camera> &cams,
    const vector<Eigen::Vector3d> &points,
    const vector<observation> &obvs,
    vector<Sophus::SE3d> &poses,
    vector<Eigen::Vector3d> &positions) {

    cout << "cams: " << cams.size() << endl;
    cout << "points: " << points.size() << endl;
    cout << "obvs: " << obvs.size() << endl;

    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;    // pose is 6, landmark is 3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;         // 图模型
    optimizer.setAlgorithm(solver);     // 设置求解器
    optimizer.setVerbose(true);             // 打开调试输出

    // vertex
    cout << "adding cam vertex" << endl;
    vector<VertexPose*> vertex_poses(cams.size());
    for(size_t i = 0; i < cams.size(); ++i){
        VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
        vertex_pose->setId(i);
        vertex_pose->setEstimate(cams[i].T());
        vertex_pose->setIntrinsicParameters(cams[i]);
        optimizer.addVertex(vertex_pose);
        vertex_poses[i] = vertex_pose;
    }

    cout << "adding point vertex" << endl;
    vector<VertexLandmark*> vertex_landmarks(points.size());
    for(size_t i = 0; i < points.size(); ++i){
        VertexLandmark *vertex_landmark = new VertexLandmark();
        vertex_landmark->setId(cams.size() + i);
        vertex_landmark->setEstimate(points[i]);
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
        // edge->setVertex(0, dynamic_cast<VertexPose*>(optimizer.vertex(obvs[i].cam_id)));
        // edge->setVertex(1, dynamic_cast<VertexLandmark*>(optimizer.vertex(cams.size() + obvs[i].point_id)));
        edge->setVertex(0, vertex_poses[obvs[i].cam_id]);
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
    optimizer.optimize(1);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
    // for(VertexPose* vertex_pose : vertex_poses){
    //     cout << "pose estimated by g2o =\n" << vertex_pose->estimate().matrix() << endl;
    // }
}
