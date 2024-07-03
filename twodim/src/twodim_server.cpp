#include "ros/ros.h"
#include "twodim/Matrix2D.h"
#include <opencv2/opencv.hpp>

bool sobelFilterCallback(twodim::Matrix2D::Request &req, twodim::Matrix2D::Response &res) {
    int n_dim = std::sqrt(req.matrix.size()); // Assuming input is square matrix
    cv::Mat input(n_dim, n_dim, CV_64FC1);
    cv::Mat output(n_dim, n_dim, CV_64FC1);

    ROS_INFO("Input Matrix:");
    for (int i = 0; i < n_dim; ++i) {
        for (int j = 0; j < n_dim; ++j) {
            input.at<double>(i, j) = req.matrix[i * n_dim + j];
            ROS_INFO("%f ", input.at<double>(i, j));
        }
        ROS_INFO("\n");
    }

    // Sobel kernels for horizontal and vertical gradients
    cv::Mat sobelKernelX = (cv::Mat_<double>(3,3) << -1, 0, 1,
                                                    -2, 0, 2,
                                                    -1, 0, 1);

    cv::Mat sobelKernelY = (cv::Mat_<double>(3,3) << -1, -2, -1,
                                                     0,  0,  0,
                                                     1,  2,  1);

    cv::Mat gradientX, gradientY;
    cv::filter2D(input, gradientX, -1, sobelKernelX);
    cv::filter2D(input, gradientY, -1, sobelKernelY);

    // Compute magnitude of gradients
    cv::magnitude(gradientX, gradientY, output);

    for (int i = 0; i < n_dim; ++i) {
        for (int j = 0; j < n_dim; ++j) {
            res.filtered_matrix.push_back(output.at<double>(i, j));
        }
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "twodim_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("apply_sobel_filter", sobelFilterCallback);
    ROS_INFO("Ready to apply Sobel filter.");
    
    ros::spin();

    return 0;
}
