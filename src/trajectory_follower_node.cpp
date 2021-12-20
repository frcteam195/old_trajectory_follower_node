#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <map>

#include "teb_local_planner/FeedbackMsg.h"
#include "trajectory_follower_node/TrajectoryFollowCue.h"

std::mutex traj_follow_mutex;

ros::NodeHandle* node;
std::map<int64_t, teb_local_planner::TrajectoryPointMsg> active_trajectory;
std::map<int64_t, teb_local_planner::TrajectoryPointMsg>::iterator active_trajectory_pose = active_trajectory.end();

int64_t start_time;

void feedback_msg_callback(const teb_local_planner::FeedbackMsg &msg)
{
    std::lock_guard<std::mutex> guard(traj_follow_mutex);
    start_time = ros::Time::now().toNSec();
    for(std::vector<teb_local_planner::TrajectoryPointMsg>::const_iterator i =
            msg.trajectories[msg.selected_trajectory_idx].trajectory.begin();
        i != msg.trajectories[msg.selected_trajectory_idx].trajectory.end();
        i++)
    {
        active_trajectory[(*i).time_from_start.toNSec()] = (*i);
    }
    active_trajectory_pose = active_trajectory.begin();
}

double interpolate_double(double left, double right, double progress)
{
    return left + ((right - left) * progress);
}

teb_local_planner::TrajectoryPointMsg lerp_trajectory_points
   (teb_local_planner::TrajectoryPointMsg left,
    teb_local_planner::TrajectoryPointMsg right,
    double progress)
{

    // ideally this would do a kinematically feasible lerp
    // but how much value there is in that depends on how
    // heavily discretized the points are -
    // leaving it like this for initial testing
    teb_local_planner::TrajectoryPointMsg output;

    output.velocity.linear.x = interpolate_double(left.velocity.linear.x, right.velocity.linear.x, progress);
    output.velocity.linear.y = interpolate_double(left.velocity.linear.y, right.velocity.linear.y, progress);
    output.velocity.linear.z = interpolate_double(left.velocity.linear.z, right.velocity.linear.z, progress);

    output.velocity.angular.x = interpolate_double(left.velocity.angular.x, right.velocity.angular.x, progress);
    output.velocity.angular.y = interpolate_double(left.velocity.angular.y, right.velocity.angular.y, progress);
    output.velocity.angular.z = interpolate_double(left.velocity.angular.z, right.velocity.angular.z, progress);

    output.acceleration.linear.x = interpolate_double(left.acceleration.linear.x, right.acceleration.linear.x, progress);
    output.acceleration.linear.y = interpolate_double(left.acceleration.linear.y, right.acceleration.linear.y, progress);
    output.acceleration.linear.z = interpolate_double(left.acceleration.linear.z, right.acceleration.linear.z, progress);

    output.acceleration.angular.x = interpolate_double(left.acceleration.angular.x, right.acceleration.angular.x, progress);
    output.acceleration.angular.y = interpolate_double(left.acceleration.angular.y, right.acceleration.angular.y, progress);
    output.acceleration.angular.z = interpolate_double(left.acceleration.angular.z, right.acceleration.angular.z, progress);

    return output;
}

void trajectory_follower_loop()
{
    static ros::Publisher target_traj_publisher =
       node->advertise<trajectory_follower_node::TrajectoryFollowCue>("/active_trajectory", 1);

	ros::Rate rate(100);

	while (ros::ok())
	{
        teb_local_planner::TrajectoryPointMsg active_point;
        teb_local_planner::TrajectoryPointMsg prev_point;
        int64_t current_time = ros::Time::now().toNSec() - start_time;

        bool planning_active = false;

		{
            std::lock_guard<std::mutex> guard(traj_follow_mutex);

            while(active_trajectory_pose != active_trajectory.end() &&
                  current_time > (*active_trajectory_pose).first)
            {
                active_trajectory_pose++;

            }
            if(active_trajectory_pose != active_trajectory.end())
            {
                active_point = (*active_trajectory_pose).second;
                prev_point = (*active_trajectory_pose).second;
                prev_point.time_from_start = ros::Duration(0);

                planning_active = true;

                if(active_trajectory_pose != active_trajectory.begin())
                {
                    prev_point = (*std::prev(active_trajectory_pose)).second;
                }
            }
        }

        trajectory_follower_node::TrajectoryFollowCue output;

        if(planning_active)
        {
            double progress = (current_time - prev_point.time_from_start.toNSec()) /
                              (active_point.time_from_start.toNSec() - prev_point.time_from_start.toNSec());

            teb_local_planner::TrajectoryPointMsg output_point =
               lerp_trajectory_points(prev_point, active_point, progress);


            output.velocity.linear.x = output_point.velocity.linear.x;
            output.velocity.linear.y = output_point.velocity.linear.y;
            output.velocity.linear.z = output_point.velocity.linear.z;

            output.velocity.angular.x = output_point.velocity.angular.x;
            output.velocity.angular.y = output_point.velocity.angular.y;
            output.velocity.angular.z = output_point.velocity.angular.z;

            output.acceleration.linear.x = output_point.acceleration.linear.x;
            output.acceleration.linear.y = output_point.acceleration.linear.y;
            output.acceleration.linear.z = output_point.acceleration.linear.z;

            output.acceleration.angular.x = output_point.acceleration.angular.x;
            output.acceleration.angular.y = output_point.acceleration.angular.y;
            output.acceleration.angular.z = output_point.acceleration.angular.z;

        }
        else
        {
            output.velocity.linear.x = 0.0;
            output.velocity.linear.y = 0.0;
            output.velocity.linear.z = 0.0;

            output.velocity.angular.x = 0.0;
            output.velocity.angular.y = 0.0;
            output.velocity.angular.z = 0.0;

            output.acceleration.linear.x = 0.0;
            output.acceleration.linear.y = 0.0;
            output.acceleration.linear.z = 0.0;

            output.acceleration.angular.x = 0.0;
            output.acceleration.angular.y = 0.0;
            output.acceleration.angular.z = 0.0;
        }

        output.traj_follow_active = planning_active;

        target_traj_publisher.publish(output);
    }
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "trajectory_follower_node");

	ros::NodeHandle n;

	node = &n;

	ros::Subscriber modeOverride = node->subscribe("/teb_feedback", 10, feedback_msg_callback);

	std::thread trajectory_follower_thread(trajectory_follower_loop);

	ros::spin();
	return 0;
}
