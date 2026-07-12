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

    void map_sub_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->map_ = *msg;
        has_map_ = true;
    }
         
    void runLoop(){

        // most coordinates will be with respect to map
        // task one: find robots current position through transform tree and convert it into grid cells
        // run through algorithm detailed in ipad
        //
    }

    public: 
        GoalPoseCreator() : rclcpp::Node("goal_pose_creator"){
            auto map_sub_callback_object = [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                this->map_sub_callback(msg);
            };
            map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, map_sub_callback_object);
            goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100),[this]() {this->runLoop();});
        }






};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPoseCreator>());
    rclcpp::shutdown();
    return 0;
}