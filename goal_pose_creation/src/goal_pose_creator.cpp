#include <vector>
#include <array>
#include <chrono>
#include <set>
#include <cmath>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/time.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2/utils.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class GoalPoseCreator : public rclcpp::Node{
    public: 
        using ComputePath = nav2_msgs::action::ComputePathToPose;
        using ComputePathGoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using NavigateToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        GoalPoseCreator() : rclcpp::Node("goal_pose_creator"){
            auto map_sub_callback_object = [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {map_sub_callback(msg);};
            auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
            map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos, map_sub_callback_object);
            path_client_ = rclcpp_action::create_client<ComputePath>(this, "compute_path_to_pose");
            navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            timer_ = create_wall_timer(std::chrono::milliseconds(3000),[this]() {runLoop();});
        }


    private: // Variables and helper functions
        nav_msgs::msg::OccupancyGrid map_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp_action::Client<ComputePath>::SharedPtr path_client_;
        rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                        // This buffer holds and processes transforms
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;          // This listener is what puts transforms into buffer
        std::vector<std::vector<int8_t>> matrix;
        std::vector<std::array<double, 2>> previous_goals;
        std::set<std::array<int, 2>> rejected_goal_cells;
        std::array<int, 2> pending_goal_cell = {0, 0};
        bool has_map_ = false;
        bool path_check_pending = false;
        bool navigation_active = false;

        // Map and grid variables (map units are meters and degrees)
        float map_resolution;   
        uint32_t map_size_x;
        uint32_t map_size_y;
        double grid_origin_x;
        double grid_origin_y;
        double grid_origin_yaw;
        double robot_map_x;
        double robot_map_y;
        int robot_grid_x;
        int robot_grid_y;
        double goal_pose_x;
        double goal_pose_y;
        double goal_pose_yaw = 0.0;


        void map_sub_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            size_t expected_size = static_cast<size_t>(msg->info.width) * static_cast<size_t>(msg->info.height);
            if(msg->info.resolution <= 0.0 || msg->info.width < 3 || msg->info.height < 3 || msg->data.size() != expected_size){
                RCLCPP_ERROR(get_logger(), "Received an invalid occupancy grid");
                has_map_ = false;
                return;
            }
            map_ = *msg;
            map_resolution = map_.info.resolution;
            map_size_x = map_.info.width;
            map_size_y = map_.info.height;
            grid_origin_x = map_.info.origin.position.x;
            grid_origin_y = map_.info.origin.position.y;
            grid_origin_yaw = tf2::getYaw(map_.info.origin.orientation);
            matrix.assign(map_size_y, std::vector<int8_t>(map_size_x, 0));
            has_map_ = true;
        }


        void row_major_to_matrix(const std::vector<int8_t>& flattened_entries, std::vector<std::vector<int8_t>>& cell_matrix){
            if(flattened_entries.size() != static_cast<size_t>(map_size_x) * static_cast<size_t>(map_size_y)){
                RCLCPP_ERROR(get_logger(), "Occupancy grid data size does not match its dimensions");
                return;
            }
            size_t k = 0;
            for(size_t i = 0; i < map_size_y; i++){
                for(size_t j = 0; j < map_size_x; j++){
                    cell_matrix[i][j] = flattened_entries[k++];
                }
            }
        }


        std::array<double, 2> grid_cell_to_map_coords(int gridX, int gridY){
            double localX = map_resolution * (gridX + 0.5);
            double localY = map_resolution * (gridY + 0.5);
            double mapX = grid_origin_x + (std::cos(grid_origin_yaw) * localX) - (std::sin(grid_origin_yaw) * localY);
            double mapY = grid_origin_y + (std::sin(grid_origin_yaw) * localX) + (std::cos(grid_origin_yaw) * localY);
            return {mapX, mapY};
        }


       std::array<int, 2> map_coords_to_grid_cell(double mapX, double mapY){
            double dx = mapX - grid_origin_x;
            double dy = mapY - grid_origin_y;
            double localX = (std::cos(grid_origin_yaw) * dx) + (std::sin(grid_origin_yaw) * dy);
            double localY = (-std::sin(grid_origin_yaw) * dx) + (std::cos(grid_origin_yaw) * dy);
            int gridX = std::floor(localX / map_resolution);
            int gridY = std::floor(localY / map_resolution); 
            return {gridX, gridY};
        }


        void set_goal_yaw_from_path(const nav_msgs::msg::Path& path){
            goal_pose_yaw = std::atan2(goal_pose_y - robot_map_y, goal_pose_x - robot_map_x);
            if(path.poses.size() < 2){return;}
            const auto& end = path.poses.back().pose.position;
            constexpr double approach_distance = 0.3;
            for(int i = static_cast<int>(path.poses.size()) - 2; i >= 0; i--){
                const auto& previous = path.poses[static_cast<size_t>(i)].pose.position;
                double dx = end.x - previous.x;
                double dy = end.y - previous.y;
                if((dx * dx) + (dy * dy) >= approach_distance * approach_distance){
                    goal_pose_yaw = std::atan2(dy, dx);
                    return;
                }
            }
            const auto& start = path.poses.front().pose.position;
            double dx = end.x - start.x;
            double dy = end.y - start.y;
            if((dx * dx) + (dy * dy) > 1.0e-8){goal_pose_yaw = std::atan2(dy, dx);}
        }


        void navigate_to_validated_goal(){
            if(navigation_active){return;}
            if(!navigation_client_->wait_for_action_server(std::chrono::seconds(0))){
                RCLCPP_WARN(get_logger(), "Nav2 NavigateToPose server is not available");
                return;
            }

            NavigateToPose::Goal request;
            request.pose.header.stamp = now();
            request.pose.header.frame_id = "map";
            request.pose.pose.position.x = goal_pose_x;
            request.pose.pose.position.y = goal_pose_y;
            request.pose.pose.orientation.z = std::sin(goal_pose_yaw * 0.5);
            request.pose.pose.orientation.w = std::cos(goal_pose_yaw * 0.5);

            auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
            options.goal_response_callback = [this](NavigateToPoseGoalHandle::SharedPtr handle){
                if(!handle){
                    RCLCPP_WARN(get_logger(), "Nav2 rejected the navigation goal");
                    navigation_active = false;
                    return;
                }
                previous_goals.push_back({goal_pose_x, goal_pose_y});
                RCLCPP_INFO(get_logger(), "Navigation goal accepted");
            };
            options.result_callback = [this](const NavigateToPoseGoalHandle::WrappedResult& result){
                navigation_active = false;
                if(result.code == rclcpp_action::ResultCode::SUCCEEDED){
                    rejected_goal_cells.clear();
                    RCLCPP_INFO(get_logger(), "Navigation goal completed; searching for another frontier");
                    return;
                }
                if(result.code == rclcpp_action::ResultCode::ABORTED){rejected_goal_cells.insert(pending_goal_cell);}
                RCLCPP_WARN(get_logger(), "Navigation goal did not succeed: %s", result.result && !result.result->error_msg.empty() ? result.result->error_msg.c_str() : "no detailed Nav2 error");
            };

            navigation_active = true;
            navigation_client_->async_send_goal(request, options);
        }


        void check_goal_with_nav2(){
            if(path_check_pending){return;}
            if(!path_client_->wait_for_action_server(std::chrono::seconds(0))){
                RCLCPP_WARN(get_logger(), "Nav2 planner is not available");
                return;
            }

            ComputePath::Goal request;
            request.goal.header.stamp = now();
            request.goal.header.frame_id = "map";
            request.goal.pose.position.x = goal_pose_x;
            request.goal.pose.position.y = goal_pose_y;
            double initial_goal_yaw = std::atan2(goal_pose_y - robot_map_y, goal_pose_x - robot_map_x);
            request.goal.pose.orientation.z = std::sin(initial_goal_yaw * 0.5);
            request.goal.pose.orientation.w = std::cos(initial_goal_yaw * 0.5);
            request.use_start = false;
            request.planner_id = "GridBased";

            auto options = rclcpp_action::Client<ComputePath>::SendGoalOptions();
            options.goal_response_callback = [this](ComputePathGoalHandle::SharedPtr handle){
                if(!handle){
                    RCLCPP_WARN(get_logger(), "Nav2 planner rejected the path request");
                    path_check_pending = false;
                }
            };
            options.result_callback = [this](const ComputePathGoalHandle::WrappedResult& result){
                path_check_pending = false;
                bool valid_path = result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result && result.result->error_code == ComputePath::Result::NONE && !result.result->path.poses.empty();
                if(valid_path){
                    set_goal_yaw_from_path(result.result->path);
                    RCLCPP_INFO(get_logger(), "Candidate goal is reachable");
                    navigate_to_validated_goal();
                    return;
                }

                bool reject_candidate = result.result && (result.result->error_code == ComputePath::Result::GOAL_OUTSIDE_MAP || result.result->error_code == ComputePath::Result::GOAL_OCCUPIED || result.result->error_code == ComputePath::Result::NO_VALID_PATH || (result.result->error_code == ComputePath::Result::NONE && result.result->path.poses.empty()));
                if(reject_candidate){rejected_goal_cells.insert(pending_goal_cell);}
                RCLCPP_WARN(get_logger(), "Candidate goal failed path validation: %s", result.result ? result.result->error_msg.c_str() : "unknown planner error");
            };

            path_check_pending = true;
            path_client_->async_send_goal(request, options);
        }


    private: // Core functions
        void runLoop(){
            if (!has_map_) {
                RCLCPP_INFO(get_logger(), "Waiting for map...");
                return;
            }

            if(path_check_pending || navigation_active){return;}

            row_major_to_matrix(map_.data, matrix);
            if(!update_robot_pose()){
                return;
            }
            
            if(search_for_goal()){
                check_goal_with_nav2();
            } else{
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No frontier candidate passed validation");
            }
        }


        bool update_robot_pose(){
            try{
                auto chassis_transform = tf_buffer_->lookupTransform("map", "chassis", tf2::TimePointZero);
                robot_map_x = chassis_transform.transform.translation.x;
                robot_map_y = chassis_transform.transform.translation.y;
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
            int search_count = 1;

            while(true){
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
                if(jump_dist < 1){return false;}
                for(int x = x_min; x <= x_max; x++){
                    for(int y = y_min; y <= y_max; y++){
                        if((x > x_inner_min && x < x_inner_max) && (y > y_inner_min && y < y_inner_max)){continue;}

                        int frontiers = count_frontier_cells(jump_dist, x, y);
                        if(check_if_goal_valid(jump_dist, frontiers, x, y)){return true;}
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
            if(x_max >= static_cast<int>(map_size_x)){
                x_max = static_cast<int>(map_size_x) - 1;}
            if (y_min < 0){
                y_min = 0;}
            if(y_max >= static_cast<int>(map_size_y)){
                y_max = static_cast<int>(map_size_y) - 1;}
        }


        int count_frontier_cells(int window_size, int x, int y){
            int frontiers = 0;
            int lower_offset = window_size / 2;
            int upper_offset = window_size - lower_offset - 1;
            for(int offset_y = -lower_offset; offset_y <= upper_offset; offset_y++){
                for(int offset_x = -lower_offset; offset_x <= upper_offset; offset_x++){
                    int cell_x = x + offset_x;
                    int cell_y = y + offset_y;
                    if(cell_x <= 0 || cell_x >= static_cast<int>(map_size_x) - 1 || cell_y <= 0 || cell_y >= static_cast<int>(map_size_y) - 1){continue;}
                    int cell_val = static_cast<int>(matrix[cell_y][cell_x]);
                    if(cell_val < 0 || cell_val > 15){continue;}
                    bool up_is_unknown = matrix[cell_y + 1][cell_x] == -1;
                    bool down_is_unknown = matrix[cell_y - 1][cell_x] == -1;
                    bool left_is_unknown = matrix[cell_y][cell_x - 1] == -1;
                    bool right_is_unknown = matrix[cell_y][cell_x + 1] == -1;
                    if(up_is_unknown || down_is_unknown || left_is_unknown || right_is_unknown){frontiers++;}
                }
            }
            return frontiers;
        }


        bool check_if_goal_valid(int jump_dist, int frontiers, int x, int y){
            if(x < 0 || x >= static_cast<int>(map_size_x) || y < 0 || y >= static_cast<int>(map_size_y)){return false;}
            int candidate_value = static_cast<int>(matrix[y][x]);
            if(candidate_value < 0 || candidate_value > 15){return false;}
            std::array<int, 2> candidate_cell = {x, y};
            if(rejected_goal_cells.find(candidate_cell) != rejected_goal_cells.end()){return false;}

            int min_frontiers = jump_dist * 2;
            if(frontiers < min_frontiers){return false;}
            
            auto goal_coords = grid_cell_to_map_coords(x, y);
            for(const auto& prev_goal : previous_goals){
                double dx = prev_goal[0] - goal_coords[0];
                double dy = prev_goal[1] - goal_coords[1];
                bool goal_too_close = (dx * dx) + (dy * dy) <= 0.8 * 0.8;
                if (goal_too_close){
                    return false;
                }
            }

            if (too_close_to_robot(goal_coords[0], goal_coords[1])){
                return false;
            }
            
            goal_pose_x = goal_coords[0];
            goal_pose_y = goal_coords[1];
            pending_goal_cell = candidate_cell;
            return true;
        }
        

        bool too_close_to_robot(double goal_x, double goal_y){
            double dx = goal_x - robot_map_x;
            double dy = goal_y - robot_map_y;
            return (dx * dx) + (dy * dy) <= 0.8 * 0.8;
        }


        bool map_fully_explored(int x_max, int y_max, int x_min, int y_min){
            return x_max == static_cast<int>(map_size_x) - 1 && y_max == static_cast<int>(map_size_y) - 1 && x_min == 0 && y_min == 0;
        }

};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPoseCreator>());
    rclcpp::shutdown();
    return 0;
}
