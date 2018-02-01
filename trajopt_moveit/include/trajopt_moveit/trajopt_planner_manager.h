#ifndef TRAJOPT_MOVEIT_TRAJOPT_PLANNER_MANAGER_H_
#define TRAJOPT_MOVEIT_TRAJOPT_PLANNER_MANAGER_H_

#include <moveit/planning_interface/planning_interface.h>
#include <trajopt_moveit/trajopt_planner.h>
#include <ros/node_handle.h>

namespace trajopt_moveit
{

class TrajoptPlannerManager : public planning_interface::PlannerManager
{
public:
  TrajoptPlannerManager();
  virtual ~TrajoptPlannerManager();

  bool initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns) override;

  bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const override;

  std::string getDescription() const override
  {
    return "Trajopt";
  }

  void getPlanningAlgorithms(std::vector<std::string> &algs) const override;

  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr &planning_scene,
      const planning_interface::MotionPlanRequest &req,
      moveit_msgs::MoveItErrorCodes &error_code) const override;

protected:
  ros::NodeHandle nh_;
  boost::shared_ptr<TrajoptPlanner> planner_;
};

} 

#endif /* TRAJOPT_MOVEIT_TRAJOPT_PLANNER_MANAGER_H_ */