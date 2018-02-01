#include <trajopt_moveit/trajopt_planner_manager.h>
#include <class_loader/class_loader.h>

namespace trajopt_moveit
{

TrajoptPlannerManager::TrajoptPlannerManager()
{}

TrajoptPlannerManager::~TrajoptPlannerManager()
{}

bool TrajoptPlannerManager::initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns)
{
  if (!ns.empty())
    nh_ = ros::NodeHandle(ns);

  planner_.reset(new TrajoptPlanner("manipulator", model));
  return true;
}

bool TrajoptPlannerManager::canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
{
  return true;
}

void TrajoptPlannerManager::getPlanningAlgorithms(std::vector<std::string> &algs) const
{
  algs.clear();
  algs.push_back("Default Trajopt");
}

planning_interface::PlanningContextPtr TrajoptPlannerManager::getPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                                                               const planning_interface::MotionPlanRequest &req,
                                                                               moveit_msgs::MoveItErrorCodes &error_code) const
{
  // Setup PlanningScene
  planner_->setPlanningScene(planning_scene);

  // Setup Planner
  planner_->setMotionPlanRequest(req);

  // Return Planner
  return planner_;
}

}
CLASS_LOADER_REGISTER_CLASS(trajopt_moveit::TrajoptPlannerManager, planning_interface::PlannerManager)