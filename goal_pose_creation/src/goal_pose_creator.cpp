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
    private: 
        nav_msgs::msg::OccupancyGrid map_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                // This buffer is what holds and processes transforms
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  // This listener is what puts transforms into buffer
        std::vector<std::vector<int8_t>> matrix;
        bool has_map_ = false;
        int loops_completed = 0;
        float map_resolution;   // units are meters
        uint32_t map_size_x;
        uint32_t map_size_y;
        double grid_origin_x;
        double grid_origin_y;
        double robot_map_x;
        double robot_map_y;
        int robot_grid_x;
        int robot_grid_y;
        double robot_theta;  // measured in degrees


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
         

    public: 
        GoalPoseCreator() : rclcpp::Node("goal_pose_creator"){
            auto map_sub_callback_object = [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {map_sub_callback(msg);};
            map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, map_sub_callback_object);
            goal_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            timer_ = create_wall_timer(std::chrono::milliseconds(100),[this]() {runLoop();});
        }


    private:
        void runLoop(){
            if (!has_map_) {
                RCLCPP_INFO(get_logger(), "Waiting for map...");
                return;
            }

            row_major_to_matrix(map_.data, matrix);
            if(!update_robot_pose()){return;}

            // Once all coordinates are updated, the searching process can start

            loops_completed++;
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



};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPoseCreator>());
    rclcpp::shutdown();
    return 0;
}