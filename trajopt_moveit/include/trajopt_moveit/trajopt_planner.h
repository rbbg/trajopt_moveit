#ifndef TRAJOPT_MOVEIT_TRAJOPT_PLANNER_H_
#define TRAJOPT_MOVEIT_TRAJOPT_PLANNER_H_

#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>

namespace trajopt_moveit
{

class TrajoptPlanner: public planning_interface::PlanningContext
{
public:
  TrajoptPlanner(const std::string& group, const moveit::core::RobotModelConstPtr &model);
  virtual ~TrajoptPlanner();

  bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req)  const;
  virtual void clear() override;
  virtual bool terminate() override;

  virtual bool solve(planning_interface::MotionPlanResponse &res) override;
  virtual bool solve(planning_interface::MotionPlanDetailedResponse &res) override;

protected:
  const moveit::core::RobotModelConstPtr& robot_model_;
};

}
#endif 