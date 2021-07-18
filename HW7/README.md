# HW7

## Bundle Adjustment
### 文献阅读

1. 为何说 Bundle Adjustment is slow 是不对的？
2. BA 中有哪些需要注意参数化的地⽅？ Pose 和 Point 各有哪些参数化⽅式？有何优缺点。
3. *本⽂写于 2000 年，但是⽂中提到的很多内容在后⾯⼗⼏年的研究中得到了印证。你能看到哪些
   ⽅向在后续⼯作中有所体现？请举例说明。  

### BAL-dataset

```cpp
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
    vector<Sophus::SE3d> poses_g2o;
    vector<Eigen::Vector3d> positions_g2o;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    bundleAdjustmentG2O(cams, points, obvs, poses_g2o, positions_g2o);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp by g2o cost time: " << time_used.count() << " seconds." << endl;
    return 0;
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
    vector<Sophus::SE3d> &poses,
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
    // for(VertexCamera* vertex_camera : vertex_cameras){
    //     cout << "pose estimated by g2o =\n" << vertex_camera->estimate().matrix() << endl;
    // }
}

```

需要注意的地方有：

1. 相机的参数有9个，使用`camera`数据结构来储存。图优化的顶点`VertexCamera`的底层数据结构即`camera`。

2. 需要对路标点做边缘化，即代码中的`vertex_landmark->setMarginalized(true);`，否则会出现以下错误：

   ```
   terminate called after throwing an instance of 'std::bad_alloc'
   what():  std::bad_alloc
   Aborted (core dumped)
   ```

3. 优化方式使用`g2o::OptimizationAlgorithmGaussNewton`会发散，改用`g2o::OptimizationAlgorithmLevenberg`才会收敛。

## 直接法的 Bundle Adjustment
### 数学模型

$\min\sum\limits_{j=1}^{7}\sum\limits_{i=1}^{N}\sum\limits_{W}\|I(p_i)-I_j(\pi(\bold{K}\bold{T_j}\bold{p_i}))\|_2^2$

1. 如何描述任意⼀点投影在任意⼀图像中形成的 error？

   $\sum\limits_{W}\|I(p_i)-I_j(\pi(\bold{K}\bold{T}_j\bold{p}_i))\|_2^2$

2. 每个 error 关联⼏个优化变量？

   相机内参在代码中是给定的，不需要优化。

   每个$\text{error}_{ji}$关联的变量有变换矩阵$\bold{T}_j$和三维点坐标$\bold{p}_i$。

3. error 关于各变量的雅可⽐是什么？

   使用$\xi$表示变换矩阵$\bold{T}_j$（此处省略下标$j$），$\bold{p}$同样省略下标$i$。

   记$\bold{p}' = (\bold{T}\bold{p})_{1:3} = (\exp(\bold{\xi}^\text{^})\bold{p})_{1:3}$，相机内参$K$包含$f_x,f_y,c_x,c_y$。

   $\frac{\partial \text{e}}{\partial \delta\bold{\xi}} = -\begin{bmatrix}\frac{f_x}{Z'} & 0 & -\frac{f_xX'}{Z'^2} & -\frac{f_xX'Y'}{Z'^2} & f_x + \frac{f_xX'^2}{Z'^2} & -\frac{f_xY'}{Z'} \\ 0 & \frac{f_y}{Z'} & -\frac{f_yY'}{Z'^2} & -f_y - \frac{f_yY'^2}{Z'^2} & \frac{f_yX'Y'}{Z'^2} & \frac
   {f_yX'}{Z'}\end{bmatrix}$
   
   $\frac{\partial \text{e}}{\partial \bold{p}} = -\begin{bmatrix}\frac{f_x}{Z'} & 0 & -\frac{f_xX'}{Z'^2} \\ 0 & \frac{f_y}{Z'} & -\frac{f_yY'}{Z'^2}\end{bmatrix}$

### 实现

1. 能否不要以$[x, y, z]^T$的形式参数化每个点？
2. 取 4x4 的 patch 好吗？取更⼤的 patch 好还是取⼩⼀点的 patch 好？
3. 从本题中，你看到直接法与特征点法在 BA 阶段有何不同？
4. 由于图像的差异，你可能需要鲁棒核函数，例如 Huber。此时 Huber 的阈值如何选取？  

