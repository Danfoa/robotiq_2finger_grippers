#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
// Robotiq msgs 
#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperFeedback.h> 
#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperAction.h>
#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperResult.h> 
#include <robotiq_2f_gripper_msgs/CommandRobotiqGripperGoal.h>

typedef robotiq_2f_gripper_msgs::CommandRobotiqGripperGoal     CommandRobotiqGripperGoal;
typedef robotiq_2f_gripper_msgs::CommandRobotiqGripperResult   CommandRobotiqGripperResult;
typedef robotiq_2f_gripper_msgs::CommandRobotiqGripperAction   CommandRobotiqGripperAction;
typedef robotiq_2f_gripper_msgs::CommandRobotiqGripperFeedback CommandRobotiqGripperFeedback;

namespace robotiq_2f_gripper_control{

    class RobotiqActionClient : actionlib::SimpleActionClient<CommandRobotiqGripperAction>{

        private:
        CommandRobotiqGripperGoal goal;

        public:
        RobotiqActionClient(std::string action_name, bool wait_for_server = true)
            : actionlib::SimpleActionClient<CommandRobotiqGripperAction>(action_name, true)
            {
            if( wait_for_server ){
                waitForServer( ros::Duration(10) );
                if( !isServerConnected() )
                    ROS_ERROR( "Robotiq Action Server (%s) seems not to be running", action_name.c_str() );
            }
        }
        
        void close(float speed = 0.1, float force = 220, bool wait  = true){
            goal.position = 0.0;
            goal.speed = speed;
            goal.force = force;
            goal.emergency_release = false;
            if( wait )
                sendGoalAndWait( goal );
            else
                sendGoal( goal );
        }

        void open(float speed = 0.1, float force = 220, bool wait  = true){
            goal.position = 255;
            goal.speed = speed;
            goal.force = force;
            goal.emergency_release = false;
            if( wait )
                sendGoalAndWait( goal );
            else
                sendGoal( goal );
        }

        void goToPosition(float position, float speed = 0.1, float force = 220, bool wait  = true){
            goal.position = position;
            goal.speed = speed;
            goal.force = force;
            goal.emergency_release = false;
            if( wait )
                sendGoalAndWait( goal );
            else
                sendGoal( goal );
        }

    };

}