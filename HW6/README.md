# HW6

## LK 光流

### 光流文献综述

1. 按此文的分类，光流法可分为哪几类？

   摘自论文原文：

   ```
   We categorize algorithms as either additive or compositional, and as either forwards or inverse.
   ```

   光流法可分为additive或compositional，亦可分为forward或inverse，2*2共4类。

2. 在 compositional 中，为什么有时候需要做原始图像的 wrap？该 wrap 有何物理意义？

   forward compositional:

   $\sum_x [I (W(W(x; \Delta p); p)) - T (x)]^2$

   inverse compositional:

   $\sum_x [T(W(x;\Delta p)) - I(W(x;p))]^2$

3. forward 和 inverse 有何差别？

   forward：对$I$做warp

   inverse：对$T$做warp

### forward-addtive Gauss-Newton 光流的实现

```cpp
void OpticalFlowSingleLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 10;
    bool have_initial = !kp2.empty();

    for (size_t i = 0; i < kp1.size(); i++) {
        auto kp = kp1[i];
        double dx = 0, dy = 0; // dx,dy need to be estimated
        if (have_initial) {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded

        // Gauss-Newton iterations
        /**
         * J^T * J * dx = - J^T * f
         **/
        for (int iter = 0; iter < iterations; iter++) {
            Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();
            cost = 0;

            /**
             * kp.pt.x + dx - (half_patch_size-1) < 0
             * kp.pt.x + dx + half_patch_size >= img1.cols
             **/
            if (kp.pt.x + dx <= half_patch_size || kp.pt.x + dx >= img1.cols - half_patch_size ||
                kp.pt.y + dy <= half_patch_size || kp.pt.y + dy >= img1.rows - half_patch_size) {   // go outside
                succ = false;
                break;
            }

            // compute cost and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {

                    // TODO START YOUR CODE HERE (~8 lines)
                    double error = 0;
                    Eigen::Vector2d J;  // Jacobian
                    if (inverse == false) {
                        // Forward Jacobian
                        J(0) = img2.at<uchar>(kp.pt.y+dy, kp.pt.x+dx+1) - img2.at<uchar>(kp.pt.y+dy, kp.pt.x+dx); //and then divided by 1?
                        J(1) = img2.at<uchar>(kp.pt.y+dy+1, kp.pt.x+dx) - img2.at<uchar>(kp.pt.y+dy, kp.pt.x+dx);
                    } else {
                        // Inverse Jacobian
                        J(0) = img1.at<uchar>(kp.pt.y, kp.pt.x+1) - img1.at<uchar>(kp.pt.y, kp.pt.x); //and then divided by 1?
                        J(1) = img1.at<uchar>(kp.pt.y+1, kp.pt.x) - img1.at<uchar>(kp.pt.y, kp.pt.x);
                    }

                    // compute H, b and set cost;
                    H += J * J.transpose();
                    b += (img1.at<uchar>(kp.pt.y, kp.pt.x) - img2.at<uchar>(kp.pt.y+dy, kp.pt.x+dx)) * J;
                    cost = pow(img1.at<uchar>(kp.pt.y, kp.pt.x) - img2.at<uchar>(kp.pt.y+dy, kp.pt.x+dx), 2);
                    // TODO END YOUR CODE HERE
                }

            // compute update
            // TODO START YOUR CODE HERE (~1 lines)
            Eigen::Vector2d update;
            update = H.ldlt().solve(b);
            // TODO END YOUR CODE HERE

            if (isnan(update[0])) {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;
                break;
            }
            if (iter > 0 && cost > lastCost) {
                cout << "cost increased: " << cost << ", " << lastCost << endl;
                break;
            }

            // update dx, dy
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;
        }

        success.push_back(succ);

        // set kp2
        if (have_initial) {
            kp2[i].pt = kp.pt + Point2f(dx, dy);
        } else {
            KeyPoint tracked = kp;
            tracked.pt += cv::Point2f(dx, dy);
            kp2.push_back(tracked);
        }
    }

    int success_count = 0;
    for(int i = 0; i < success.size(); ++i){
        if(success[i]) success_count++;
    }
    cout << success_count << "/" << success.size() << endl;
}
```

1. 从最小二乘角度来看，每个像素的误差怎么定义？

   为了与论文中使用的符号一致，将$I_1$，$I_2$分别改用$T$，$I$表示。

   $\bold{x}$用$(x_i, y_i)$表示，$(\Delta x_i, \Delta y_i)$用$\bold{p}$表示。

   整体误差：$\min\limits_{\Delta x_i,\Delta y_i} \sum \limits_W \|T(x_i,y_i) - I(x_i + \Delta x_i, y_i + \Delta y_i)\|_2^2 = \min\limits_{\bold{p}} \sum \limits_W \|T(\bold{x}) - I(\bold{x}+\bold{p})\|_2^2$

   每个像素的误差：$\|T(x_i,y_i) - I(x_i + \Delta x_i, y_i + \Delta y_i)\|_2^2 = \|T(\bold{x}) - I(\bold{x}+\bold{p})\|_2^2$

2. 误差相对于自变量的导数如何定义？  

   首先确定$T$及$I$都是有两个自变量及一个应变量的函数。

   $T$不受$\bold{p}$影响，所以$\frac{\partial T(\bold{x})}{\partial \bold{p}} = 0$。

   $I(\bold{x} + \bold{p} + \Delta \bold{p}) = I(\bold{x} + \bold{p}) + J(\bold{x} + \bold{p}) \Delta \bold{p}$，其中$J(\bold{x} + \bold{p})= \nabla I(\bold{x} + \bold{p}) = (\frac {\partial {I(\bold{x + \bold{p}})}}{\partial x_i}, \frac {\partial {I(\bold{x} + \bold{p})}}{\partial y_i})$

   高斯牛顿法一阶近似：$I(\bold{x} + \bold{p} + \Delta \bold{p}) \approx I(\bold{x} + \bold{p}) + J(\bold{x} + \bold{p}) \Delta \bold{p}$。

   $\begin{align}\|T(\bold{x}) - I(\bold{x} + \bold{p} + \Delta \bold{p})\|_2^2 &\approx \|T(\bold{x}) - (I(\bold{x} + \bold{p}) + J(\bold{x} + \bold{p}) \Delta \bold{p})\|_2^2 \\&= T^2 - 2T(I + J\Delta\bold{p}) + (I^2 + 2IJ\Delta\bold{p} + \Delta\bold{p}^TJ^TJ\Delta\bold{p})\end{align}$

   $\frac{\partial \|T(\bold{x}) - I(\bold{x} + \bold{p}+ \Delta\bold{p})\|_2^2}{\partial \Delta\bold{p}} \approx - 2TJ + 2IJ + 2J^TJ\Delta\bold{p}$

   令上式为0得到：$J^TJ\Delta\bold{p} = TJ - IJ = (T - I) J$

### 反向法

forward additive

论文中forward additive方法要最小化的函数：

$\sum \limits_\bold{x}  [I(\bold{W}(\bold{x};\bold{p} + \Delta \bold{p})) - T(\bold{x})]^2 = \sum \limits_\bold{x}  [I(\bold{W}(\bold{x};\bold{p})) + \nabla I \frac{\partial \bold{W}}{\partial \bold{p}}\Delta \bold{p} - T(\bold{x})]^2$

其中$\nabla I = (\frac{\partial I}{\partial x},\frac{\partial I}{\partial y})$

在本题中$\bold{W}(\bold{x}; \bold{p}) = \bold{x} + \bold{p}$，所以上式可以写成$\sum \limits_\bold{x}  [I(\bold{x}+\bold{p}) + \nabla I \Delta \bold{p} - T(\bold{x})]^2$。

inverse additive

将$\nabla I$替换成$\nabla T (\frac{\partial \bold{W}}{\partial \bold{x}})^{-1}$，$\Delta \bold{p}$替换成$-\Delta \bold{p}$。

$\sum \limits_\bold{x}  [T(\bold{x}) + \nabla T (\frac{\partial \bold{W}}{\partial \bold{x}})^{-1}\frac{\partial \bold{W}}{\partial \bold{p}}\Delta \bold{p} - I(\bold{W}(\bold{x};\bold{p}))]^2$

其中$\nabla T = (\frac{\partial T}{\partial x},\frac{\partial T}{\partial y})$

$\bold{p} \leftarrow \bold{p} - \Delta \bold{p}$

仍然是对I更新p？

高斯牛顿法推导：

$\sum \limits_\bold{x}  [T(\bold{x}) + \nabla T (\frac{\partial \bold{W}}{\partial \bold{x}})^{-1}\frac{\partial \bold{W}}{\partial \bold{p}}\Delta \bold{p} - I(\bold{W}(\bold{x};\bold{p}))]^2$

在本题中$\bold{W}(\bold{x}; \bold{p}) = \bold{x} + \bold{p}$，$\frac{\partial \bold{W}}{\partial \bold{x}} = \frac{\partial \bold{W}}{\partial \bold{p}} = I_{2 \times 2}$，故可将上式写成$\sum \limits_\bold{x}  [T(\bold{x}) + \nabla T \Delta \bold{p} - I(\bold{x}+\bold{p})]^2$

$[T(\bold{x}) + \nabla T \Delta \bold{p} - I(\bold{x}+\bold{p})]^2 = (T-I)^2 + (\Delta \bold{p})^T(\nabla T)^T \nabla T \Delta \bold{p} + 2 (T-I) \nabla T\Delta \bold{p}$

$\frac{\partial[T(\bold{x}) + \nabla T \Delta \bold{p} - I(\bold{x}+\bold{p})]^2}{\partial\Delta \bold{p}} = \frac{\partial(T-I)^2 + (\Delta \bold{p})^T(\nabla T)^T \nabla T \Delta \bold{p} + 2 (T-I) \nabla T\Delta \bold{p}}{\partial\Delta \bold{p}} = 2(\nabla T)^T \nabla T \Delta \bold{p} + 2 (T-I) \nabla T$

令上式为0得到：$\nabla T^T \nabla T\Delta \bold{p} = -(T-I) \nabla T$

在代码中，为了实现方便，没有将$\Delta \bold{p}$替换成$-\Delta \bold{p}$，所以$\nabla T^T \nabla T\Delta \bold{p} = (T-I) \nabla T$，更新方式为$\bold{p} \leftarrow \bold{p} + \Delta \bold{p}$。

```cpp
if (inverse == false) {
    // Forward Jacobian
    J(0) = img2.at<uchar>(kp.pt.y+dy, kp.pt.x+dx+1) - img2.at<uchar>(kp.pt.y+dy, kp.pt.x+dx); //and then divided by 1?
    J(1) = img2.at<uchar>(kp.pt.y+dy+1, kp.pt.x+dx) - img2.at<uchar>(kp.pt.y+dy, kp.pt.x+dx);
} else {
    // Inverse Jacobian
    J(0) = img1.at<uchar>(kp.pt.y, kp.pt.x+1) - img1.at<uchar>(kp.pt.y, kp.pt.x); //and then divided by 1?
    J(1) = img1.at<uchar>(kp.pt.y+1, kp.pt.x) - img1.at<uchar>(kp.pt.y, kp.pt.x);
}
```

### 推广至金字塔

```cpp
void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse) {

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<Mat> pyr1, pyr2; // image pyramids
    vector<vector<KeyPoint>> kp1_pyr;
    // TODO START YOUR CODE HERE (~8 lines)
    for (int i = 0; i < pyramids; i++) {
        cv::Mat img1_resized, img2_resized;
        cv::resize(img1, img1_resized, cv::Size(img1.cols * scales[i], img1.rows * scales[i]));
        cv::resize(img2, img2_resized, cv::Size(img2.cols * scales[i], img2.rows * scales[i]));
        pyr1.push_back(img1_resized);
        pyr2.push_back(img2_resized);

        vector<KeyPoint> kp1_resized = kp1;
        for(KeyPoint& kp : kp1_resized){
            kp.pt.x *= scales[i];
            kp.pt.y *= scales[i];
        }
        kp1_pyr.push_back(kp1_resized);
    }
    // TODO END YOUR CODE HERE

    // coarse-to-fine LK tracking in pyramids
    // TODO START YOUR CODE HERE
    vector<KeyPoint> kp2_single;
    vector<bool> success_single;
    for (int i = pyramids-1; i >= 0; i--) {
        // enlarge the keypoints
        for(KeyPoint& kp : kp2_single){
            kp.pt.x /= pyramid_scale;
            kp.pt.y /= pyramid_scale;
        }
        success_single.clear();
        OpticalFlowSingleLevel(pyr1[i], pyr2[i], kp1_pyr[i], kp2_single, success_single, inverse);
        //update "success" vector
        if(success.empty()){
            success = success_single;
        }else{
            int success_count = 0;
            for(int i = 0; i < success.size(); ++i){
                success[i] = success[i] & success_single[i];
            }
        }
    }
    kp2 = kp2_single;

    // TODO END YOUR CODE HERE
    // don't forget to set the results into kp2
}
```



1. 所谓 coarse-to-fine 是指怎样的过程？

   由分辨率最低的图像开始做一次光流法后，得到较粗糙的解后，把解映射到较精细的图像上，再做一次光流法。一步一步直到原始图像为止。

2. 光流法中的金字塔用途和特征点法中的金字塔有何差别？

### 讨论

1. 我们优化两个图像块的灰度之差真的合理吗？哪些时候不够合理？你有解决办法吗？
2. 图像块大小是否有明显差异？取 16x16 和 8x8 的图像块会让结果发生变化吗？
3. 金字塔层数对结果有怎样的影响？缩放倍率呢？

## 直接法

### 单层直接法

$\bold{T}_{\text{cur,ref}} = \frac{1}{N} \sum\limits_{i=1}^N\sum\limits_{W_i} \|I_{\text{ref}}(\pi(\bold{p}_i)) - I_{\text{cur}}(\pi(\bold{T}_{\text{cur,ref}}\bold{p}_i))\|_2^2$

1. 该问题中的误差项是什么？

   单个点所造成的误差为：$I_{\text{ref}}(\pi(\bold{p}_i)) - I_{\text{cur}}(\pi(\bold{T}_{\text{cur,ref}}\bold{p}_i))$

2. 误差相对于自变量的雅可比维度是多少？如何求解？

   误差函数的自变量为$6 \times 1$向量，应变量为$1 \times 1$的纯量，所以Jacobian为$1 \times 6$的向量。

   $\begin{align}J &= -\frac{\partial \bold{I}_2}{\partial \bold{u}}\frac{\partial \bold{u}}{\partial \delta\bold{\xi}} \\&= -\begin{bmatrix}\bold{I}_2(x+1, y)-\bold{I}_2(x, y) & \bold{I}_2(x, y+1)- \bold{I}_2(x, y)\end{bmatrix}\begin{bmatrix}\frac{f_x}{Z} & 0 & -\frac{f_xX}{Z^2} & -\frac{f_xXY}{Z^2} & f_x + \frac{f_xX^2}{Z^2} & -\frac{f_xY}{Z} \\ 0 & \frac{f_y}{Z} & -\frac{f_yY}{Z^2} & -f_y-\frac{f_yY^2}{Z^2} & \frac{f_yXY}{Z^2} & \frac{f_yX}{Z}\end{bmatrix}\end{align}$

3. 窗口可以取多大？是否可以取单个点？  

   继续推导高斯牛顿法：

   $e(\delta T \bigoplus T) \approx e - \frac{\partial I_2}{\partial u}\frac{\partial u }{\partial q}\frac{\partial q}{\partial \xi} \delta \xi$一阶近似

   $e_i^Te_i = e^2 -2e\frac{\partial I_2}{\partial u}\frac{\partial u }{\partial q}\frac{\partial q}{\partial \xi} \delta \xi + (\delta \xi)^T(\frac{\partial I_2}{\partial u}\frac{\partial u }{\partial q}\frac{\partial q}{\partial \xi})^T(\frac{\partial I_2}{\partial u}\frac{\partial u }{\partial q}\frac{\partial q}{\partial \xi} \delta \xi)$

   $\frac{\partial e_i^Te_i}{\partial \delta \xi} = -2e\frac{\partial I_2}{\partial u}\frac{\partial u }{\partial q}\frac{\partial q}{\partial \xi} + 2(\frac{\partial I_2}{\partial u}\frac{\partial u }{\partial q}\frac{\partial q}{\partial \xi})^T(\frac{\partial I_2}{\partial u}\frac{\partial u }{\partial q}\frac{\partial q}{\partial \xi}) \delta \xi = 0$

   $(\frac{\partial I_2}{\partial u}\frac{\partial u }{\partial q}\frac{\partial q}{\partial \xi})^T(\frac{\partial I_2}{\partial u}\frac{\partial u }{\partial q}\frac{\partial q}{\partial \xi}) \delta \xi = e(\frac{\partial I_2}{\partial u}\frac{\partial u }{\partial q}\frac{\partial q}{\partial \xi})$

   $J^TJ\delta \xi = -J^Te$

   ~~一个三维点（对应两个二维点）提供一条约束，因为$\delta \xi$共有六个自由度，所以需要取至少六个点。~~

### 单层直接法

```cpp
void DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3d &T21,
        int img_idx
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 100;

    double cost = 0, lastCost = 0;
    int nGood = 0;  // good projections
    VecVector2d goodProjection;

    for (int iter = 0; iter < iterations; iter++) {
        nGood = 0;
        goodProjection.clear();

        // Define Hessian and bias
        Matrix6d H = Matrix6d::Zero();  // 6x6 Hessian
        Vector6d b = Vector6d::Zero();  // 6x1 bias

        for (size_t i = 0; i < px_ref.size(); i++) {

            // compute the projection in the second image
            // TODO START YOUR CODE HERE
            // 3d point in first camera coordinate
            double Z1 = depth_ref[i];
            double X1 = (px_ref[i][0] - cx)/fx * Z1;
            double Y1 = (px_ref[i][1] - cy)/fy * Z1;
            // 3d point in second camera coordinate
            Eigen::Vector3d P2 = T21 * Eigen::Vector3d(X1, Y1, Z1);
            double X2 = P2[0], Y2 = P2[1], Z2 = P2[2];
            double Z22 = Z2 * Z2;
            // projection in the second image
            float u = fx*P2[0]/P2[2]+cx, v = fy*P2[1]/P2[2]+cy;
            if(u-half_patch_size < 0 || u+half_patch_size-1 >= img2.cols ||
               v-half_patch_size < 0 || v+half_patch_size-1 >= img2.rows){
                break;
            }
            nGood++;
            goodProjection.push_back(Eigen::Vector2d(u, v));

            // and compute error and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {
                    double error = img1.at<uchar>(px_ref[i][1], px_ref[i][0]) - img2.at<uchar>(v+y, u+x);

                    Matrix26d J_pixel_xi;   // pixel to \xi in Lie algebra
                    Eigen::Vector2d J_img_pixel;    // image gradients
                    J_img_pixel(0) = img2.at<uchar>(v+y, u+x+1) - img2.at<uchar>(v+y, u+x);
                    J_img_pixel(1) = img2.at<uchar>(v+y+1, u+x) - img2.at<uchar>(v+y, u+x);

                    // 一個窗口內的所有像素共用深度?
                    J_pixel_xi(0, 0) = fx/Z2;
                    J_pixel_xi(0, 1) = 0;
                    J_pixel_xi(0, 2) = -fx*X2/Z22;
                    J_pixel_xi(0, 3) = -fx*X2*Y2/Z22;
                    J_pixel_xi(0, 4) = fx + fx*X2*X2/Z22;
                    J_pixel_xi(0, 5) = -fx*Y2/Z2;
                    J_pixel_xi(1, 0) = 0;
                    J_pixel_xi(1, 1) = fy/Z2;
                    J_pixel_xi(1, 2) = -fy*Y2/Z22;
                    J_pixel_xi(1, 3) = -fy-fy*Y2*Y2/Z22;
                    J_pixel_xi(1, 4) = fx*X2*Y2/Z22;
                    J_pixel_xi(1, 5) = fy*X2/Z2;

                    // total jacobian
                    Vector6d J = Vector6d::Zero();
                    J = - J_img_pixel.transpose() * J_pixel_xi;

                    H += J * J.transpose();
                    b += -error * J;
                    cost += error * error;
                }
            // END YOUR CODE HERE
        }

        // solve update and put it into estimation
        // TODO START YOUR CODE HERE
        Vector6d update;
        update = H.ldlt().solve(b);
        T21 = Sophus::SE3d::exp(update) * T21;
        // END YOUR CODE HERE

        cost /= nGood;

        if (isnan(update[0])) {
            // sometimes occurred when we have a black or white patch and H is irreversible
            cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost > lastCost) {
            cout << "cost increased: " << cost << ", " << lastCost << endl;
            break;
        }
        lastCost = cost;
        cout << "cost = " << cost << ", good = " << nGood << endl;
    }
    cout << "good projection: " << nGood << endl;
    cout << "T21 = \n" << T21.matrix() << endl;

    // in order to help you debug, we plot the projected pixels here
    cv::Mat img1_show, img2_show;
    cv::cvtColor(img1, img1_show, cv::COLOR_GRAY2BGR);
    cv::cvtColor(img2, img2_show, cv::COLOR_GRAY2BGR);
    for (auto &px: px_ref) {
        cv::rectangle(img1_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(0, 250, 0));
    }
    for (auto &px: goodProjection) {
        cv::rectangle(img2_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(0, 250, 0));
    }
    cv::imshow("reference", img1_show);
    cv::imshow("current", img2_show);
    cv::waitKey();
}
```

### 多层直接法

```cpp
void DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3d &T21
) {

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<cv::Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE
    for (int i = 0; i < pyramids; i++) {
        cv::Mat img1_resized, img2_resized;
        cv::resize(img1, img1_resized, cv::Size(img1.cols * scales[i], img1.rows * scales[i]));
        cv::resize(img2, img2_resized, cv::Size(img2.cols * scales[i], img2.rows * scales[i]));
        pyr1.push_back(img1_resized);
        pyr2.push_back(img2_resized);
    }
    // END YOUR CODE HERE

    double fxG = fx, fyG = fy, cxG = cx, cyG = cy;  // backup the old values
    for (int level = pyramids - 1; level >= 0; level--) {
        VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
        for (auto &px: px_ref) {
            px_ref_pyr.push_back(scales[level] * px);
        }

        // TODO START YOUR CODE HERE
        // scale fx, fy, cx, cy in different pyramid levels
        fx = fxG * scales[level];
        fy = fyG * scales[level];
        cx = cxG * scales[level];
        cy = cyG * scales[level];
        // don't need to scale depth?

        // END YOUR CODE HERE
        DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
    }

}
```



### 延伸讨论

1. 直接法是否可以类似光流，提出 inverse, compositional 的概念？它们有意义吗？
2. 请思考上面算法哪些地方可以缓存或加速？
3. 在上述过程中，我们实际假设了哪两个 patch 不变？
4. 为何可以随机取点？而不用取角点或线上的点？那些不是角点的地方，投影算对了吗？
5. 请总结直接法相对于特征点法的异同与优缺点  

## 使用光流计算视差

