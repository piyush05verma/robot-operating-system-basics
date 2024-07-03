#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <queue>
#include <vector>

struct Node {
    int x, y;
    int g, h;
    Node* parent;

    Node(int _x, int _y, int _g, int _h, Node* _parent = nullptr)
        : x(_x), y(_y), g(_g), h(_h), parent(_parent) {}

    int f() const {
        return g + h;
    }
};

struct NodeComparator {
    bool operator()(Node* a, Node* b) const {
        return a->f() > b->f();
    }
};

int heuristic(int x, int y, int target_x, int target_y) {
    return std::abs(target_x - x) + std::abs(target_y - y); // Manhattan distance heuristic
}

std::vector<Node*> findPath(cv::Mat& binary_map, int start_x, int start_y, int target_x, int target_y) {
    std::priority_queue<Node*, std::vector<Node*>, NodeComparator> open_list;
    std::vector<std::vector<bool>> visited(binary_map.rows, std::vector<bool>(binary_map.cols, false));

    open_list.push(new Node(start_x, start_y, 0, heuristic(start_x, start_y, target_x, target_y)));

    while (!open_list.empty()) {
        Node* current = open_list.top();
        open_list.pop();

        if (current->x == target_x && current->y == target_y) {
            std::vector<Node*> path;
            while (current != nullptr) {
                path.push_back(current);
                current = current->parent;
            }
            return path;
        }

        if (current->x > 0 && binary_map.at<uchar>(current->y, current->x - 1) == 255 && !visited[current->y][current->x - 1]) {
            open_list.push(new Node(current->x - 1, current->y, current->g + 1, heuristic(current->x - 1, current->y, target_x, target_y), current));
            visited[current->y][current->x - 1] = true;
        }
        if (current->x < binary_map.cols - 1 && binary_map.at<uchar>(current->y, current->x + 1) == 255 && !visited[current->y][current->x + 1]) {
            open_list.push(new Node(current->x + 1, current->y, current->g + 1, heuristic(current->x + 1, current->y, target_x, target_y), current));
            visited[current->y][current->x + 1] = true;
        }
        if (current->y > 0 && binary_map.at<uchar>(current->y - 1, current->x) == 255 && !visited[current->y - 1][current->x]) {
            open_list.push(new Node(current->x, current->y - 1, current->g + 1, heuristic(current->x, current->y - 1, target_x, target_y), current));
            visited[current->y - 1][current->x] = true;
        }
        if (current->y < binary_map.rows - 1 && binary_map.at<uchar>(current->y + 1, current->x) == 255 && !visited[current->y + 1][current->x]) {
            open_list.push(new Node(current->x, current->y + 1, current->g + 1, heuristic(current->x, current->y + 1, target_x, target_y), current));
            visited[current->y + 1][current->x] = true;
        }
    }

    return std::vector<Node*>();
}

void drawPath(cv::Mat& map_image, const std::vector<Node*>& path) {
    for (size_t i = 1; i < path.size(); ++i) {
        cv::line(map_image, cv::Point(path[i - 1]->x, path[i - 1]->y), cv::Point(path[i]->x, path[i]->y), cv::Scalar(0, 0, 255), 2);
    }
}

void loadMapFromYAML(const std::string& yaml_file) {
    // Load YAML file
    YAML::Node config = YAML::LoadFile(yaml_file);

    // Extract map image path from YAML
    std::string map_image_path = config["image"].as<std::string>(); // Access the 'image' key from the YAML

    // Load map image
    cv::Mat map_image = cv::imread(map_image_path, cv::IMREAD_COLOR);

    // Check if the map image is loaded successfully
    if (map_image.empty()) {
        std::cerr << "Failed to load map image from path: " << map_image_path << std::endl;
        return;
    }

    // Resize the map image to 20 times its original dimensions
    cv::Mat resized_map;
    cv::resize(map_image, resized_map, cv::Size(map_image.cols * 20, map_image.rows * 20));

    // Create a new matrix to store the binary map
    cv::Mat binary_map(resized_map.rows, resized_map.cols, CV_8UC1);

    // Iterate over each pixel in the resized map
    for (int y = 0; y < resized_map.rows; ++y) {
        for (int x = 0; x < resized_map.cols; ++x) {
            cv::Vec3b intensity = resized_map.at<cv::Vec3b>(y, x);
            // Calculate average of RGB values
            int average_color = (intensity[0] + intensity[1] + intensity[2]) / 3;
            // Determine binary value based on the average color
            if (average_color <= 0.55 * 255) {
                binary_map.at<uchar>(y, x) = 0; // Set to 0 if within 0-55% range
            } else {
                binary_map.at<uchar>(y, x) = 255; // Set to 255 otherwise
            }
        }
    }

    // Print the size of the image
    std::cout << "Image size: " << binary_map.rows << " rows x " << binary_map.cols << " cols" << std::endl;

    // Prompt the user to input start_x, start_y, and target_x
    int start_x, start_y, target_x, target_y;
    std::cout << "Enter start_x: ";
    std::cin >> start_x;
    std::cout << "Enter start_y: ";
    std::cin >> start_y;
    std::cout << "Enter target_x: ";
    std::cin >> target_x;
    std::cout << "Enter target_y: ";
    std::cin >> target_y;

    std::vector<Node*> path = findPath(binary_map, start_x, start_y, target_x, target_y);

    // Draw path
    drawPath(binary_map, path);

    // Display the binary map image with path
    cv::imshow("Binary Map Image with Path", binary_map);
    cv::waitKey(0);
}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "map_image_loader");
    ros::NodeHandle nh;

    // Define path to YAML file
    std::string yaml_file = "/home/piyush/catkin_ws/src/nav/src/map.yaml"; // Provide the path to your YAML file

    // Load map from YAML file
    loadMapFromYAML(yaml_file);

    return 0;
}
