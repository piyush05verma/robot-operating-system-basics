#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <iostream>

using namespace std;
ros::Publisher pub;

int heuristic(pair<int, int> point, pair<int, int> goal) {
    int x = point.first - goal.first;
    int y = point.second - goal.second;
    return (int)(sqrt(x * x + y * y) * 10);
}
vector<pair<int,int>> Stary(vector<vector<int>>& grid){
    int start_x, start_y, target_x, target_y;
    cout << "Enter start_x: ";
    cin >> start_x;
    cout << "Enter start_y: ";
    cin >> start_y;
    cout << "Enter target_x: ";
    cin >> target_x;
    cout << "Enter target_y: ";
    cin >> target_y;

    int rows = grid.size();
    int cols = grid[0].size();

    pair<int,int> start = make_pair(start_x, start_y);
    pair<int,int> goal = make_pair(target_x, target_y);

    int dx[] = {0, 0, 1, -1, 1, 1, -1, -1};
    int dy[] = {1, -1, 0, 0, 1, -1, 1, -1};

    vector<vector<int>> came_from(rows, vector<int>(cols, -1));
    vector<vector<int>> g_score(rows, vector<int>(cols, INT_MAX));

    g_score[start_x][start_y] = 0;

    vector<pair<int,pair<int,int>>> f;
    f.push_back(make_pair(0,start));

    while (!f.empty()) {

        int idx = 0;
        for (int i = 1; i < f.size(); ++i) {
            if (f[i].first < f[idx].first)
                idx = i;
        }

        pair<int,int> current = f[idx].second;
        f.erase(f.begin() + idx);

        int ci = current.first;
        int cj = current.second;

        if (current == goal) {
            vector<pair<int,int>> path;
            while (current != start) {
                path.push_back(current);
                int prev_i = came_from[ci][cj] / cols;
                int prev_j = came_from[ci][cj] % cols;
                ci = prev_i;
                cj = prev_j;
                current = make_pair(ci, cj);
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return path;
        }

        for (int k = 0; k < 8; ++k) {
            int ni = ci + dx[k];
            int nj = cj + dy[k];

            if (ni >= 0 && ni < rows && nj >= 0 && nj < cols && grid[ni][nj] == 0) {
                int new_g_score = g_score[ci][cj] + 10;
                if (k >= 4) // diagonal movement, add extra cost
                    new_g_score += 4;

                if (new_g_score < g_score[ni][nj]) {
                    g_score[ni][nj] = new_g_score;
                    int f_score = new_g_score + heuristic(make_pair(ni, nj), goal);
                    f.push_back(make_pair(f_score, make_pair(ni, nj)));
                    came_from[ni][nj] = ci * cols + cj;
                }
            }
        }
    }
    return {}; // No path found
}


void publisher(vector<pair<int,int>>& dir,int m,int n){
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    for (int i=0;i<dir.size();i++){
       pose.pose.position.x = dir[i].second;
       pose.pose.position.y = dir[i].first;
       pose.pose.position.z = 0;
       path.poses.push_back(pose);
    }
    pub.publish(path);
}

void CallBack(struct nav_msgs::OccupancyGrid_<std::allocator<void>> x){
    vector<vector<int>> a(x.info.height);
    for (int i=0; i<x.info.height;i++){
        vector<int> b(x.info.width);
        for(int j=0; j<x.info.width;j++){
            if (x.data[i*x.info.width+j]){
                b[j] = 1;
            }
            else {
	           b[j] = 0;
            }  
        }
        a[i] = b;
    }
    cout << "X_Limit: 0 to " << a.size() - 1 << endl;
    cout << "Y_limit: 0 to " << a[0].size() - 1 << endl;

    vector<pair<int, int>> path = Stary(a);
    cout << "Check RVIZ " << endl;
    publisher(path,x.info.height,x.info.width);
}

int main(int argc, char**argv){
    ros::init(argc, argv, "map_path");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/map", 1000, CallBack);
    pub = n.advertise<nav_msgs::Path>("/path", 1000);
    ros::spin();
}
    