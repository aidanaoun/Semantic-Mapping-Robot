#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


class GoalPoseCreator : public rclcpp::Node{
    private: 
        nav_msgs::msg::OccupancyGrid map_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
        bool has_map_ = false;
        int loops_completed = 0;
        float map_resolution;   // units are meters
        uint32_t map_cells_x;
        uint32_t map_cells_y;
        std::vector<std::vector<int8_t>> matrix;
         
        void map_sub_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            map_ = *msg;
            map_resolution = map_.info.resolution;
            map_cells_x = map_.info.width;
            map_cells_y = map_.info.height;
            matrix.assign(map_cells_y, std::vector<int8_t>(map_cells_x, 0));
            has_map_ = true;
        }
         

    public: 
        GoalPoseCreator() : rclcpp::Node("goal_pose_creator"){
            auto map_sub_callback_object = [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {map_sub_callback(msg);};
            map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, map_sub_callback_object);
            goal_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
            timer_ = create_wall_timer(std::chrono::milliseconds(100),[this]() {runLoop();});
        }

        
        void runLoop(){
            if (!has_map_) {
                RCLCPP_INFO(get_logger(), "Waiting for map...");
                return;
            }

            row_major_to_matrix(map_.data, matrix);
            RCLCPP_INFO(get_logger(), "y size: %zu x size: %zu", matrix.size(), matrix[0].size());
            loops_completed++;
        }


        void row_major_to_matrix(const std::vector<int8_t>& flattened_entries, std::vector<std::vector<int8_t>>& cell_matrix){
            int k = 0;
            for (int i=0; i < map_cells_y; i++){
                for(int j=0; j < map_cells_x; j++){
                    cell_matrix[i][j] = flattened_entries[k];
                    k++;
                }
            }
        }


};

// NOTE: goal_pose topic coordinates are relative to the chassis 0 point
// NOTE: map x & y are oriented in the same direction as chassis x and y

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPoseCreator>());
    rclcpp::shutdown();
    return 0;
}