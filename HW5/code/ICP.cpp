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


using namespace std;

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 7, 1> Vector7d;
typedef vector<Vector7d, Eigen::aligned_allocator<Vector7d>> VecVector7d;

string traj_file = "./compare.txt";
vector<pair<int, int>> dummy_matches;

void DrawTwoTrajectories(vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>>&,
    vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>>&, 
    int iter = -1, vector<pair<int, int>>& matches = dummy_matches);

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

    /**
     * initial solution by RANSAC
     **/


    Sophus::SE3f Tge_total = Sophus::SE3f(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());
    vector<pair<int, double>> matches_e2g, matches_g2e;
    vector<pair<int, int>> matches;
    Eigen::Vector3f center_est, center_gt;
    double dist_thres = numeric_limits<double>::max();

    int iterations = 100;
    double cost = 0, lastCost = numeric_limits<double>::max();
    for (int iter = 0; iter < iterations; iter++) {
        cost = 0;

        matches_e2g.clear();
        matches_g2e.clear();
        matches.clear();
        center_est = Eigen::Vector3f::Zero();
        center_gt = Eigen::Vector3f::Zero();
        vector<double> dists;
        bool double_match = false;
        
        for(size_t i = 0; i < traj_est.size(); ++i){
            double min_dist = numeric_limits<double>::max(), dist;
            int min_j = -1;
            for(size_t j = 0; j < traj_gt.size(); ++j){
                // if(used.find(j) != used.end()) continue;
                dist = (traj_gt[j].translation() - traj_est[i].translation()).norm();
                if(dist < min_dist){
                    min_dist = dist;
                    min_j = j;
                }
            }
            matches_e2g.push_back(make_pair(min_j, min_dist));
        }
        for(size_t j = 0; j < traj_gt.size(); ++j){
            double min_dist = numeric_limits<double>::max(), dist;
            int min_i = -1;
            for(size_t i = 0; i < traj_est.size(); ++i){
                // if(used.find(j) != used.end()) continue;
                dist = (traj_gt[j].translation() - traj_est[i].translation()).norm();
                if(dist < min_dist){
                    min_dist = dist;
                    min_i = i;
                }
            }
            matches_g2e.push_back(make_pair(min_i, min_dist));
        }
        for(size_t eix = 0; eix < matches_e2g.size(); ++eix){
            int gix = matches_e2g[eix].first;
            if(gix != -1 && matches_e2g[eix].second < dist_thres 
                    && (!double_match || matches_g2e[gix].first == eix)){
                cost += pow((traj_gt[gix] * traj_est[eix].inverse()).log().norm(), 2);

                center_est += traj_est[eix].translation();
                center_gt += traj_gt[gix].translation();
                matches.push_back(make_pair(eix, gix));
            }
        }
        cout << "There are " << matches.size() << " matches." << endl;

        // DrawTwoTrajectories(traj_gt, traj_est, iter, matches);

        center_est /= matches.size();
        center_gt /= matches.size();
        cost /= matches.size();

        Eigen::Matrix3f W = Eigen::Matrix3f::Zero();
        for(pair<int, int>& match : matches){
            Eigen::Vector3f est = traj_est[match.first].translation();
            Eigen::Vector3f gt = traj_gt[match.second].translation();

            W += (est - center_est) * (gt - center_gt).transpose();
        }
        JacobiSVD<Matrix3f> svd(W, ComputeFullU | ComputeFullV);
        Matrix3f U = svd.matrixU();
        Matrix3f V = svd.matrixV();
        Vector3f Sing = svd.singularValues();

        Eigen::Matrix3f Rge = U * V.transpose();
        Eigen::Vector3f tge = center_gt - Rge * center_est;
        
        Sophus::SE3f Tge(Rge, tge);
        Tge_total = Tge * Tge_total;

        for(Sophus::SE3f& traj_est_i: traj_est){
            traj_est_i = Tge * traj_est_i;
        }  

        // if (cost >= lastCost) {
        //     // cost increase, update is not good
        //     cout << "iteration " << iter << endl;
        //     cout << "cost: " << cost << ", last cost: " << lastCost << endl;
        //     break;
        // }
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

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

double computeCloudResolution(const pcl::PointCloud<PointXYZ>::ConstPtr &cloud){
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sqr_distances(2);
    pcl::search::KdTree<PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i){
        if (!pcl_isfinite((*cloud)[i].x)){
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
        if (nres == 2){
            res += sqrt(sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0){
        res /= n_points;
    }
    return res;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr iss(const pcl::PointCloud<PointXYZ>::ConstPtr &cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ> ());;
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    
    // Fill in the model cloud
    
    double model_resolution;
    
    // Compute model_resolution
    model_resolution = computeCloudResolution(cloud);
    
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
    
    iss_detector.setSearchMethod (tree);
    iss_detector.setSalientRadius (6 * model_resolution);
    iss_detector.setNonMaxRadius (4 * model_resolution);
    iss_detector.setThreshold21 (0.975);
    iss_detector.setThreshold32 (0.975);
    iss_detector.setMinNeighbors (5);
    iss_detector.setNumberOfThreads (4);
    iss_detector.setInputCloud (model);
    iss_detector.compute (*model_keypoints);

    return model_keypoints;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ()); 
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
    
    // Compute the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud< pcl::Normal>);
    normalEstimation.setRadiusSearch(0.02);
    normalEstimation.compute(*normals);

    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (cloud);
    fpfh.setInputNormals (normals);

    fpfh.setSearchMethod (tree);

    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (0.05);

    // Compute the features
    fpfh.compute (*fpfhs);

    // fpfhs->size () should have the same size as the input cloud->size ()*

    return fpfhs;
}

pcl::CorrespondencesPtr matching(const pcl::FPFHSignature33& src_features,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt_features){

    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    pcl::CorrespondencesPtr src_features(new pcl::Correspondences());
    est.setInputSource(source_features);
    est.setInputTarget(tgt_features);
    est.determineCorrespondences(*correspondences);

    // Duplication rejection Duplicate
    pcl::CorrespondencesPtr correspondences_result_rej_one_to_one(new pcl::Correspondences());
    pcl::registration::CorrespondenceRejectorOneToOne corr_rej_one_to_one;
    corr_rej_one_to_one.setInputCorrespondences(correspondences);
    corr_rej_one_to_one.getCorrespondences(*correspondences_result_rej_one_to_one);

    return correspondences_result_rej_one_to_one;
}

void ransac(){
    // Correspondance rejection RANSAC

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector_sac;
    pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
    rejector_sac.setInputSource(src_keypoints);
    rejector_sac.setInputTarget(tgt_keypoints);
    rejector_sac.setInlierThreshold(2.5); // distance in m, not the squared distance
    rejector_sac.setMaximumIterations(1000000);
    rejector_sac.setRefineModel(false);
    rejector_sac.setInputCorrespondences(correspondences_result_rej_one_to_one);;
    rejector_sac.getCorrespondences(*correspondences_filtered);
    correspondences.swap(correspondences_filtered);
    std::cout << correspondences->size() << " vs. " << correspondences_filtered->size() << std::endl;
    transform = rejector_sac.getBestTransformation();   // Transformation Estimation method 1
}

