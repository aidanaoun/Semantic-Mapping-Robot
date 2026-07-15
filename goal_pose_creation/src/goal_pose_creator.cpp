#include <string>
#include <vector>
#include <array>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/time.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2/utils.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class GoalPoseCreator : public rclcpp::Node{
    public: 
        GoalPoseCreator() : rclcpp::Node("goal_pose_creator"){
            auto map_sub_callback_object = [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {map_sub_callback(msg);};
            map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, map_sub_callback_object);
            goal_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            timer_ = create_wall_timer(std::chrono::milliseconds(1000),[this]() {runLoop();});
        }


    private: // Variables and helper functions
        nav_msgs::msg::OccupancyGrid map_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                        // This buffer holds and processes transforms
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;          // This listener is what puts transforms into buffer
        std::vector<std::vector<int8_t>> matrix;
        bool has_map_ = false;
        bool robot_is_moving = false;
        int loops_completed = 0;

        // Map and grid variables (map units are meters and degrees)
        float map_resolution;   
        uint32_t map_size_x;
        uint32_t map_size_y;
        double grid_origin_x;
        double grid_origin_y;
        double robot_map_x;
        double robot_map_y;
        int robot_grid_x;
        int robot_grid_y;
        double robot_theta;
        double goal_pose_x;
        double goal_pose_y;
        double goal_pose_theta;  


        void map_sub_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            map_ = *msg;
            map_resolution = map_.info.resolution;
            map_size_x = map_.info.width;
            map_size_y = map_.info.height;
            grid_origin_x = map_.info.origin.position.x;
            grid_origin_y = map_.info.origin.position.y;
            matrix.assign(map_size_y, std::vector<int8_t>(map_size_x, 0));
            has_map_ = true;
        }


        void row_major_to_matrix(const std::vector<int8_t>& flattened_entries, std::vector<std::vector<int8_t>>& cell_matrix){
            int k = 0;
            for (int i=0; i < map_size_y; i++){
                for(int j=0; j < map_size_x; j++){
                    cell_matrix[i][j] = flattened_entries[k];
                    k++;
                }
            }
        }


        std::array<double, 2> grid_cell_to_map_coords(int gridX, int gridY){
            double mapX = (map_resolution * gridX) + grid_origin_x;
            double mapY = (map_resolution * gridY) + grid_origin_y;
            return {mapX, mapY};
        }


       std::array<int, 2> map_coords_to_grid_cell(double mapX, double mapY){
            int gridX = std::floor((mapX - grid_origin_x) / map_resolution);
            int gridY = std::floor((mapY - grid_origin_y) / map_resolution); 
            return {gridX, gridY};
        }


        void publish_goal_pose(){
            geometry_msgs::msg::PoseStamped goal_msg;

            goal_msg.header.stamp = get_clock()->now();
            goal_msg.header.frame_id = "map";

            goal_msg.pose.position.x = goal_pose_x;
            goal_msg.pose.position.y = goal_pose_y;
            goal_msg.pose.position.z = 0.0;

            goal_msg.pose.orientation.w = 1.0;

            goal_pose_publisher_->publish(goal_msg);
        }


    private: // Core functions
        void runLoop(){
            if (!has_map_) {
                RCLCPP_INFO(get_logger(), "Waiting for map...");
                return;
            }

            row_major_to_matrix(map_.data, matrix);
            if(!update_robot_pose()){return;}
            
            search_for_goal();
            publish_goal_pose();

            loops_completed++;
        }


        bool update_robot_pose(){
            try{
                auto chassis_transform = tf_buffer_->lookupTransform("map", "chassis", tf2::TimePointZero);
                robot_map_x = chassis_transform.transform.translation.x;
                robot_map_y = chassis_transform.transform.translation.y;
                robot_theta = tf2::getYaw(chassis_transform.transform.rotation) * 180.0 / 3.141592653589793;

                std::array<int, 2> gridCoords =  map_coords_to_grid_cell(robot_map_x, robot_map_y);
                robot_grid_x = gridCoords[0];
                robot_grid_y = gridCoords[1];
            }
            catch (const tf2::TransformException& exception) {
                RCLCPP_WARN(get_logger(), "Could not get robot pose: %s", exception.what());
                return false;
            }
            return true;
        }


        bool search_for_goal(int search_dist = 100){
            int anchor_x = robot_grid_x;
            int anchor_y = robot_grid_y;
            bool foundGoalPose = false;
            int search_count = 1;

            while (!foundGoalPose){
                int x_min = anchor_x - ( search_count * search_dist );
                int x_max = anchor_x + ( search_count * search_dist );
                int y_min = anchor_y - ( search_count * search_dist );
                int y_max = anchor_y + ( search_count * search_dist );
                int x_inner_min = anchor_x - ( (search_count-1) * search_dist );
                int x_inner_max = anchor_x + ( (search_count-1) * search_dist );
                int y_inner_min = anchor_y - ( (search_count-1) * search_dist );
                int y_inner_max = anchor_y + ( (search_count-1) * search_dist );
                double search_size = 0.9;

                check_index_error(x_min, x_max, y_min, y_max);

                int jump_dist = static_cast<int>(search_size / map_resolution);
                for(int x = x_min + jump_dist; x < (x_max - jump_dist); x++){
                    for(int y = y_min + jump_dist; y < (y_max - jump_dist); y++){
                        if((x > x_inner_min && x < x_inner_max) && (y > y_inner_min && y < y_inner_max)){continue;}
                        int sum = sum_matrix_entries(jump_dist, x, y);   
                        if (check_if_goal_found(jump_dist, sum, x, y)){return true;}
                    }
                }

                if(map_fully_explored(x_max, y_max, x_min, y_min)){return false;}
                search_count++;
            }
            return false;
        }


        void check_index_error(int& x_min, int& x_max, int& y_min, int& y_max){
            if (x_min < 0){
                x_min = 0;}
            if (x_max >= map_size_x){
                x_max = map_size_x - 1;}
            if (y_min < 0){
                y_min = 0;}
            if (y_max >= map_size_y){
                y_max = map_size_y - 1;}
        }


        int sum_matrix_entries(int jump_dist, int x, int y){
            int sum = 0;
            for(int i = 0; i < jump_dist; i++){
                for(int j = 0; j < jump_dist; j++){
                    sum += static_cast<int>(matrix[y - j][x - i]);
                }
            }
            return sum;
        }


        bool check_if_goal_found(int jump_dist, int sum, int x, int y){
            if (sum < (jump_dist * jump_dist * 10)){
                auto goal_poses = grid_cell_to_map_coords(x, y);
                goal_pose_x = goal_poses[0];
                goal_pose_y = goal_poses[1];
                return true;
            }
            return false;
        }


        bool map_fully_explored(int x_max, int y_max, int x_min, int y_min){
            if ((x_max == map_size_x-1) && (y_max == map_size_y-1) && (x_min == 0) && (y_min == 0)){
                goal_pose_x = 0.0;
                goal_pose_y = 0.0;
                return true;
            }
            return false;
        }
};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPoseCreator>());
    rclcpp::shutdown();
    return 0;
}