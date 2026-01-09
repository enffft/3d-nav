#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

namespace pct_planner
{

class PctPathPlanner : public nav_core::BaseGlobalPlanner
{
public:
  PctPathPlanner() {}

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override
  {
    ros::NodeHandle nh("~/" + name);
    sub_path_ = nh.subscribe("/pct_path", 1, &PctPathPlanner::pathCallback, this);
    ROS_INFO("PctPathPlanner initialized, waiting for /pct_path");
  }

  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override
  {
    plan.clear();
    if (!last_path_.poses.empty())
    {
      for (auto& p : last_path_.poses)
      {
        plan.push_back(p);
      }
      ROS_INFO("PctPathPlanner: Provided path with %zu poses", plan.size());
      return true;
    }
    else
    {
      ROS_WARN("PctPathPlanner: No path received yet!");
      return false;
    }
  }

private:
  void pathCallback(const nav_msgs::Path::ConstPtr& msg)
  {
    last_path_ = *msg;
  }

  ros::Subscriber sub_path_;
  nav_msgs::Path last_path_;
};

}  // namespace pct_planner

// 注册插件
PLUGINLIB_EXPORT_CLASS(pct_planner::PctPathPlanner, nav_core::BaseGlobalPlanner)
