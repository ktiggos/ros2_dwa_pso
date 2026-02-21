#include "ros2_dwa_pso/dwa_pso_planner.hpp"

DwaPsoPlanner::DwaPsoPlanner()
: Node("dwa_pso_planner")
{
    std::cout<<"Init"<<std::endl;
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DwaPsoPlanner>());
}