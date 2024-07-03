#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <vector>
#include <bits/stdc++.h>

ros::Publisher path_pub;
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
    int a = x - target_x;
    int b = y - target_y; 
    return (int)(sqrt(x*x + y*y)); // Manhattan distance heuristic
}

std::vector<Node*> findPath(std::vector<std::vector<int>>& binary_map, int start_x, int start_y, int target_x, int target_y) {
    std::priority_queue<Node*, std::vector<Node*>, NodeComparator> open_list;
    std::vector<std::vector<bool>> visited(binary_map.size(), std::vector<bool>(binary_map[0].size(), false));
    std::cout << "hi";
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

            // Publish the path as a ROS topic
            nav_msgs::Path path_msg;
            path_msg.header.frame_id = "map";
            for (auto it = path.rbegin(); it != path.rend(); ++it) {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = (*it)->x;
                pose.pose.position.y = (*it)->y;
                pose.pose.position.z = 0.0;
                path_msg.poses.push_back(pose);
            }
            path_pub.publish(path_msg);

            return path;
        }

        if (current->x > 0 && binary_map[current->y][current->x - 1] == 1 && !visited[current->y][current->x - 1]) {
            open_list.push(new Node(current->x - 1, current->y, current->g + 1, heuristic(current->x - 1, current->y, target_x, target_y), current));
            visited[current->y][current->x - 1] = true;
        }
        if (current->x < binary_map[0].size() - 1 && binary_map[current->y][current->x + 1] == 1 && !visited[current->y][current->x + 1]) {
            open_list.push(new Node(current->x + 1, current->y, current->g + 1, heuristic(current->x + 1, current->y, target_x, target_y), current));
            visited[current->y][current->x + 1] = true;
        }
        if (current->y > 0 && binary_map[current->y - 1][current->x] == 1 && !visited[current->y - 1][current->x]) {
            open_list.push(new Node(current->x, current->y - 1, current->g + 1, heuristic(current->x, current->y - 1, target_x, target_y), current));
            visited[current->y - 1][current->x] = true;
        }
        if (current->y < binary_map.size() - 1 && binary_map[current->y + 1][current->x] == 1 && !visited[current->y + 1][current->x]) {
            open_list.push(new Node(current->x, current->y + 1, current->g + 1, heuristic(current->x, current->y + 1, target_x, target_y), current));
            visited[current->y + 1][current->x] = true;
        }

        if (current->x > 0 && binary_map[current->y - 1][current->x - 1] == 1 && !visited[current->y - 1][current->x - 1]) {
            open_list.push(new Node(current->x - 1, current->y - 1, current->g + 1, heuristic(current->x - 1, current->y - 1, target_x, target_y), current));
            visited[current->y - 1][current->x - 1] = true;
        }
        if (current->x < binary_map[0].size() - 1 && binary_map[current->y + 1][current->x + 1] == 1 && !visited[current->y + 1][current->x + 1]) {
            open_list.push(new Node(current->x + 1, current->y + 1, current->g + 1, heuristic(current->x + 1, current->y + 1, target_x, target_y), current));
            visited[current->y + 1][current->x + 1] = true;
        }
        if (current->y > 0 && binary_map[current->y + 1][current->x - 1] == 1 && !visited[current->y + 1][current->x - 1]) {
            open_list.push(new Node(current->x - 1, current->y + 1, current->g + 1, heuristic(current->x - 1, current->y + 1, target_x, target_y), current));
            visited[current->y + 1][current->x - 1] = true;
        }
        if (current->y < binary_map.size() - 1 && binary_map[current->y - 1][current->x + 1] == 1 && !visited[current->y - 1][current->x + 1]) {
            open_list.push(new Node(current->x + 1, current->y - 1, current->g + 1, heuristic(current->x + 1, current->y - 1, target_x, target_y), current));
            visited[current->y - 1][current->x + 1] = true;
        }
    }

    return std::vector<Node*>();
}

void drawPath(std::vector<std::vector<int>>& map_image, const std::vector<Node*>& path) {
    for (size_t i = 1; i < path.size(); ++i) {
        cv::line(map_image, cv::Point(path[i - 1]->x, path[i - 1]->y), cv::Point(path[i]->x, path[i]->y), cv::Scalar(0, 0, 255), 2);
    }
}

void loadMapFromYAML(struct nav_msgs::OccupancyGrid_<std::allocator<void>> x) {
    std::vector<std::vector<int>> binary_map(x.info.height);
    for (int i=0; i<x.info.height;i++){
        std::vector<int> b(x.info.width);
        for(int j=0;j<x.info.width;j++){
            //std::cout << x.data[i*x.info.width+j]<<j;
            if (x.data[i*x.info.width+j]){
                b[j]=1;
            }
            else{
                b[j]=0;
            }
        }
        //std::cout << "\n";
        binary_map[i]=b;
    }
    std::cout << "hi";
    for(int i=0;i<binary_map.size();i++){
        for(int j=0; j<binary_map[0].size();j++){
            std::cout << binary_map[i][j]<<" ";
        }
        std::cout << "\n";
    }
    std::vector<Node*> path = findPath(binary_map, 0, 0, x.info.height-1, x.info.width-1);
}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "pathfinding");
    ros::NodeHandle nh;

    // Define path to YAML file
    ros::Subscriber path_sub = nh.subscribe("/map",1000,loadMapFromYAML);
    // Advertise path publisher
    path_pub = nh.advertise<nav_msgs::Path>("/path", 1000);

    // Load map f

    // Spin
    ros::spin();

    return 0;
}
