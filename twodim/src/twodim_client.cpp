#include "ros/ros.h"
#include "twodim/Matrix2D.h"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "twodim_client");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<twodim::Matrix2D>("apply_sobel_filter");
    twodim::Matrix2D srv;

    int n_dim;
    std::cout << "Enter the dimension of the matrix: ";
    std::cin >> n_dim;

    std::cout << "Enter values for a " << n_dim << "x" << n_dim << " matrix:" << std::endl;
    for (int i = 0; i < n_dim; ++i) {
        for (int j = 0; j < n_dim; ++j) {
            double value;
            std::cout << "Enter value for element (" << i << ", " << j << "): ";
            std::cin >> value;
            srv.request.matrix.push_back(value);
        }
    }

    if (client.call(srv)) {
        ROS_INFO("Filtered Matrix:");
        cv::Mat image(n_dim, n_dim, CV_64F); 
        for (int i = 0; i < n_dim; ++i) {
            for (int j = 0; j < n_dim; ++j) {
                ROS_INFO("%f ", srv.response.filtered_matrix[i * n_dim + j]);
                image.at<double>(i, j) = srv.response.filtered_matrix[i * n_dim + j];
            }
            ROS_INFO("\n");
        }

        // Resize the image to an enlarged one
        cv::Mat enlargedImage;
        cv::resize(image, enlargedImage, cv::Size(), 100.0, 100.0, cv::INTER_NEAREST);

        // Convert the image to CV_8U for displaying
        cv::Mat scaledImage;
        cv::normalize(enlargedImage, scaledImage, 0, 255, cv::NORM_MINMAX);
        scaledImage.convertTo(scaledImage, CV_8U);

        // Get screen resolution
        int screen_width = cv::getWindowProperty("Filtered Image", cv::WND_PROP_AUTOSIZE);
        int screen_height = cv::getWindowProperty("Filtered Image", cv::WND_PROP_AUTOSIZE);
        
        // Resize the image to full screen
        cv::resizeWindow("Filtered Image", screen_width, screen_height);
        
        // Display the enlarged image
        cv::namedWindow("Filtered Image", cv::WINDOW_NORMAL);
        cv::imshow("Filtered Image", scaledImage);
        cv::waitKey(0);
    } else {
        ROS_ERROR("Failed to call service.");
        return 1;
    }
    
    return 0;
}
