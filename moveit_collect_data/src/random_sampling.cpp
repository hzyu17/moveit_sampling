//
// Created by hongzhe on 4/11/22.
//
#include <moveit_msgs/RobotState.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


#include <moveit_msgs/MoveGroupActionResult.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <sstream>

moveit::planning_interface::MoveGroupInterfacePtr move_group_;
moveit::planning_interface::MoveGroupInterface::PlanPtr current_plan_;
std::string default_planning_pipeline_;
int plan_result = -1;
int execution_result = -1;
bool new_plan_res = false;
bool new_exec_res = false;


void PlanningResultCallback(const moveit_msgs::MoveGroupActionResult& msg){
    std::cout << "========= Planning result.error_code.val" << std::endl << msg.result.error_code.val;
    plan_result = msg.result.error_code.val;
    new_plan_res = true;
}

void ExecutionResultCallback(const moveit_msgs::ExecuteTrajectoryActionResult& msg){
    execution_result = msg.result.error_code.val;
    std::cout << "========= Execution result.error_code.val" << std::endl << msg.result.error_code.val;
    new_exec_res = true;
}

void publish_goal_state(const ros::NodeHandle& n,
                        const ros::Publisher& pub, std::vector<double> positions){
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok())
    {
        moveit_msgs::RobotState msg;
        std::stringstream ss;
        ss << "sending goal state" << count;

        sensor_msgs::JointState joint_goal_state;
        joint_goal_state.name = {"panda_joint1", "panda_joint2", "panda_joint3",
                                 "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};

        joint_goal_state.header.stamp = ros::Time::now();
        joint_goal_state.position = positions;
        joint_goal_state.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        joint_goal_state.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        msg.joint_state = joint_goal_state;

        ROS_INFO("CUSTOM GOAL STATE [%f, %f, %f, %f, %f, %f, %f]",
                 joint_goal_state.position[0],
                 joint_goal_state.position[1],
                 joint_goal_state.position[2],
                 joint_goal_state.position[3],
                 joint_goal_state.position[4],
                 joint_goal_state.position[5],
                 joint_goal_state.position[6]);

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;

        if (count > 0) return;
    }
}


void publish_plan(const ros::NodeHandle& n, const ros::Publisher& pub_plan){
    std_msgs::Empty empty_msg;

    pub_plan.publish(empty_msg);
    ROS_INFO("Sending planning message");

    ros::spinOnce();

}

void publish_execute(const ros::NodeHandle& n, const ros::Publisher& pub_execute){
    std_msgs::Empty empty_msg;

    pub_execute.publish(empty_msg);
    ROS_INFO("Sending execute message");
    ros::spinOnce();

}

void publish_random_valid_goal(const ros::NodeHandle& n, const ros::Publisher& pub_rd_goal){
    std_msgs::Empty empty_msg;

    pub_rd_goal.publish(empty_msg);
    ROS_INFO("Sending random goal message");

    ros::spinOnce();

}

void random_sample_plan_execute(int num_samples){
    ros::NodeHandle n;
    ros::Publisher pub_goal = n.advertise<moveit_msgs::RobotState>("/rviz/moveit/update_custom_goal_state", 1);

    ros::Publisher pub_plan = n.advertise<std_msgs::Empty>("/rviz/moveit/plan", 1);
    ros::Publisher pub_execute = n.advertise<std_msgs::Empty>("/rviz/moveit/execute", 1);
    ros::Publisher pub_random_valid = n.advertise<std_msgs::Empty>("/rviz/moveit/random_valid_goal_state", 1);

    ros::Subscriber sub_planning_res = n.subscribe("/move_group/result", 1, PlanningResultCallback);
    ros::Subscriber sub_execution_res = n.subscribe("/execute_trajectory/result", 1, ExecutionResultCallback);

//    std::vector<double> goal_positions = {0.0, -0.785/5, 0.0, -2.356, 0.0, 1.571, 0.785};
    ros::Rate r(0.5); // 10 hz

    for (int i=0; i<num_samples; i++){
        std::cout << "====== iteration number " << i << " =======" << std::endl;
        publish_random_valid_goal(n, pub_random_valid);
        ros::Duration(3).sleep();
        publish_plan(n, pub_plan);
        for (int count=0; count<10; count++){
            while (!new_plan_res){
                ros::spinOnce();
                r.sleep();
            }
            new_plan_res = false;
            if (plan_result != 1){
                ros::Duration(1).sleep();
                publish_plan(n, pub_plan);
            }
            else break;
        }
        if (plan_result != 1) continue;
        ROS_INFO("Plan result Success");
        plan_result = -50;

        ros::Duration(3).sleep();

        publish_execute(n, pub_execute);

        ros::Duration(20).sleep();
        while (!new_exec_res){
            ros::spinOnce();
            r.sleep();
        }
        new_exec_res = false;

//        if (execution_result != 1){
//            r.sleep();
//            ros::spinOnce();
//        }
//        else break;

        ROS_INFO("Execution result Success");
        execution_result = -1;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_external_communication");
    random_sample_plan_execute(15000);

    return 0;
}