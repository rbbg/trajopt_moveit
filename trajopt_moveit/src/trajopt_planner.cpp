#include <trajopt_moveit/trajopt_planner.h>
#include <class_loader/class_loader.h>

namespace trajopt_moveit
{

TrajoptPlanner::TrajoptPlanner(const std::string& group, const moveit::core::RobotModelConstPtr &model): 
    PlanningContext("trajopt", group),
    robot_model_(model)
{
}


TrajoptPlanner::~TrajoptPlanner()
{
}

void TrajoptPlanner::clear(){}

bool TrajoptPlanner::terminate()
{
  return true;
}

bool TrajoptPlanner::solve(planning_interface::MotionPlanResponse &res)
{
  ros::WallTime start_time = ros::WallTime::now();
  planning_interface::MotionPlanDetailedResponse detailed_res;
  bool success = solve(detailed_res);

  res.trajectory_ = detailed_res.trajectory_.back();
  ros::WallDuration wd = ros::WallTime::now() - start_time;
  res.planning_time_ = ros::Duration(wd.sec, wd.nsec).toSec();
  res.error_code_ = detailed_res.error_code_;

  return success;
}

bool TrajoptPlanner::solve(planning_interface::MotionPlanDetailedResponse &res)
{

  /*
  ROS_INFO("traj planning!");
  std::string group_name = "manipulator";

  // Needs to be a parameter
  int numSteps = 10; 
  int numJoints = robot_model_->getVariableCount();

  std::vector<int> joint_ind;
  std::vector<double> goal;


  // Goal constraints need to be parsed using joint name
  for (int i=0; i<numJoints; i++){
    joint_ind.push_back(i);
    goal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
  }

  std::vector<double> init;

  planning_scene_->getCurrentState().copyJointGroupPositions(group_name, init);

  Eigen::VectorXd initialState = util::toVectorXd(init);
  Eigen::VectorXd goalState = util::toVectorXd(goal);

  trajopt::ProblemConstructionInfo pci(planning_scene_);

  trajopt::RobotAndDOFPtr rad(new trajopt::RobotAndDOF(planning_scene_, joint_ind, group_name));
  pci.rad = rad; 

  trajopt::InitInfo initinfo;
  initinfo.type = trajopt::InitInfo::GIVEN_TRAJ;
  initinfo.data = util::toVectorXd(rad->GetDOFValues()).transpose().replicate(numSteps, 1);

  // Linearly interpolate in joint space
  for(int k=0; k<numJoints; k++){
    initinfo.data.col(k) = Eigen::VectorXd::LinSpaced(numSteps, initialState(k), goalState(k));
  }
  // ROS_INFO("Init Traj dimensions: %d x %d", initinfo.data.rows(), initinfo.data.cols());


  pci.basic_info.n_steps = numSteps;
  pci.basic_info.manip = group_name;
  pci.basic_info.start_fixed = true;

  pci.init_info = initinfo;

  boost::shared_ptr<trajopt::JointVelCostInfo> jvci(new trajopt::JointVelCostInfo());
  int jointVelCoeffs;
  nh_.param("joint_vel_coeffs", jointVelCoeffs, 1);
  jvci->coeffs = trajopt::DblVec(numJoints, jointVelCoeffs);
  jvci->name = "jvel0";
  pci.cost_infos.push_back(jvci);


  // boost::shared_ptr<trajopt::ContinuousCollisionCostInfo> cci(new trajopt::ContinuousCollisionCostInfo());
  // cci->first_step = 0;
  // cci->last_step = numSteps-1;
  // cci->coeffs = trajopt::DblVec(numSteps, 20);
  // cci->dist_pen = trajopt::DblVec(numSteps, 0.035);
  // cci->name = "continuous_collision";  
  // pci.cost_infos.push_back(cci);


  // Create optimization constraints for each goal constraint
  // combining position and orientation into pose
  // There are also path and trajectory constraints, not sure of the difference - ignoring for now
  // Also not sure about absolute xyz tolerances on orientation constraints
  


  // for(int c = 0; c < req.goal_constraints.size(); c++){

  //   // Pose constraints
  //   map<std::string, geometry_msgs::Point> positions;
  //   vector<moveit_msgs::PositionConstraint> position_constraints = req.goal_constraints[c].position_constraints;
  //   // This loop is just in case position and orientation constraints aren't matched up in order for each link
  //   for(int p = 0; p < position_constraints.size(); p++){
  //     // Might need to convert to world coordinates?
  //     positions[position_constraints[p].link_name] = position_constraints[p].constraint_region.primitive_poses[0].position;
  //   }
  //   vector<moveit_msgs::OrientationConstraint> orientation_constraints = req.goal_constraints[c].orientation_constraints;
  //   for(int o = 0; o < position_constraints.size(); o++){
  //     // Might need to convert to world frame?
  //     geometry_msgs::Point pos = positions[orientation_constraints[o].link_name];
  //     geometry_msgs::Quaternion quat = orientation_constraints[o].orientation;
  //     // Construct a pose constraint
  //     boost::shared_ptr<trajopt::PoseCntInfo> ppci(new trajopt::PoseCntInfo());
  //     ppci->timestep = numSteps - 1;
  //     ppci->xyz = Eigen::Vector3d(pos.x, pos.y, pos.z);
  //     ppci->wxyz = Eigen::Vector4d(quat.w, quat.x, quat.y, quat.z);
  //     ppci->pos_coeffs = Eigen::Vector3d::Ones(); // TODO: Configure coefficients
  //     ppci->rot_coeffs = Eigen::Vector3d::Ones();
  //     ppci->link = myRobot->GetLink(orientation_constraints[o].link_name);
  //     pci.cnt_infos.push_back(ppci);
  //   }

  //   // Joint constraints -  kind of hacky
  //   sensor_msgs::JointState js;
  //   js.name.clear();
  //   js.position.clear();
  //   vector<moveit_msgs::JointConstraint> joint_constraints = req.goal_constraints[c].joint_constraints;
  //   for(int jc = 0; jc < joint_constraints.size(); jc++){
  //     js.name.push_back(joint_constraints[jc].joint_name);
  //     js.position.push_back(joint_constraints[jc].position);
  //   }
  //   if(!joint_constraints.empty()){
  //     Eigen::VectorXd goalJointConstraints;
  //     goalJointConstraints.resize(joint_constraints.size());
  //     jointStateToArray(planning_scene->getKinematicModel(), js,
  //                       req.group_name, goalJointConstraints);
  //     boost::shared_ptr<trajopt::JointConstraintInfo> jci(new trajopt::JointConstraintInfo());
  //     jci->vals = util::toDblVec(goalJointConstraints);
  //     jci->timestep = numSteps - 1;
  //     jci->name = "joint0";
  //     pci.cnt_infos.push_back(jci);
  //   }
  // }

  boost::shared_ptr<trajopt::JointConstraintInfo> jci;
  jci->vals = goal;
  jci->timestep = numSteps-1;
  jci->name = "endpoint";


  trajopt::JointConstraintInfo jci;
  jci.vals = goal;
  jci.timestep = numSteps-1;
  jci.name = "endpoint";

  boost::shared_ptr<trajopt::TermInfo> jci_term = jci;

  pci.cnt_infos.push_back(jci_term);

  trajopt::TrajOptProbPtr prob = trajopt::ConstructProblem(pci);

  ROS_INFO("start optim");
  trajopt::TrajOptResultPtr result = trajopt::OptimizeProblem(prob, false);
  ROS_INFO("Finished!");

  */








  // init response
  res.description_.resize(1,"");
  res.processing_time_.resize(1);
  res.trajectory_.resize(1);
  res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(robot_model_, group_));
  res.trajectory_.back()->clear();

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setJointGroupPositions(group_,request_.start_state.joint_state.position);

  res.trajectory_.back()->addSuffixWayPoint(robot_state, 0.0);

  // last point and joint names
  std::vector<double> goal_val;
  for (std::size_t i = 0; i < request_.goal_constraints[0].joint_constraints.size(); i++)
  {
    goal_val.push_back(request_.goal_constraints[0].joint_constraints[i].position);
  }

  robot_state.setJointGroupPositions(group_,goal_val);
  res.trajectory_.back()->addSuffixWayPoint(robot_state, 1.0);

  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  return true;
}


bool TrajoptPlanner::canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
{
  return true;
}

}