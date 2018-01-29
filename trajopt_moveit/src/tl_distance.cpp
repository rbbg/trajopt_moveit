#include <ros/ros.h>
#include <algorithm>
// #include <array> 
#include <sstream> 
#include <string>
#include <math.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"

#include "moveit_msgs/RobotTrajectory.h"
#include "moveit_msgs/CollisionObject.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "sco/optimizers.hpp"
#include "trajopt/problem_description.hpp"


using namespace std;

class DynTrajController {

public:
  DynTrajController():
    kp_(1),
    ki_(0),
    kd_(0),
    movingSpeed_(1),
    goalObject_(""),
    execution_(false),
    tracking_(false),
    drift_(false){

    for (unsigned int i = 0; i<6;i++){
      derivator_.push_back(0.0);
      integrator_.push_back(0.0);
      lastSetPoint_.push_back(0.0);
      lastInput_.push_back(0.0);
    }

    urPub_ = nh_.advertise<std_msgs::String>("/ur_driver/URScript", 10);
    markerPub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    debugPub_ = nh_.advertise<std_msgs::Float64>("/tl_dtc/debug", 10);
    debugPub2_ = nh_.advertise<std_msgs::Float64>("/tl_dtc/debug2", 10);
    debugPub3_ = nh_.advertise<std_msgs::Float64>("/tl_dtc/debug3", 10);

    trajSub_ = nh_.subscribe("/tl_dtc/trajectory", 10, &DynTrajController::TrajectoryCB, this);
    ObjSub_ = nh_.subscribe("/tl_dtc/tracking_object", 10, &DynTrajController::TrackObjectCB, this);
    jointSub_ = nh_.subscribe("/joint_states", 10, &DynTrajController::JointStateCB, this);
    collisionSub_ = nh_.subscribe("/collision_object", 10, &DynTrajController::CollisionCB, this);
    
    moveGroup_ = boost::shared_ptr<moveit::planning_interface::MoveGroup>(new moveit::planning_interface::MoveGroup("manipulator"));

    scene_ = planning_scene_monitor::PlanningSceneMonitorPtr(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    scene_->startStateMonitor();
    scene_->startSceneMonitor();
    scene_->startWorldGeometryMonitor();

    planningScene_ = scene_->getPlanningScene();

    executionTimer_ = nh_.createTimer(ros::Duration(1), &DynTrajController::distanceCB, this);
  }

  ~DynTrajController(){
    ros::shutdown();
  }
  

  void distanceCB(const ros::TimerEvent& event){
	  ROS_INFO("DistanceCB");
	  std::vector<std::string> ObjectList = planningScene_->getWorld()->getObjectIds();

	  for (std::size_t i = 0; i<ObjectList.size(); ++i){
		  ROS_INFO("object ID: %s", ObjectList[i].c_str());
	  }





	  /*
	  const string objectID = "ojectTest";
	  planningScene_->getWorld()->getObject(objectID);
	  */






  }
  void JointStateCB(const sensor_msgs::JointState::ConstPtr& jointstate){
    currentPosition_ = jointstate->position;
    currentVelocity_ = jointstate->velocity;
  }

  void CollisionCB(const moveit_msgs::CollisionObject::ConstPtr& jointstate){
    checkPath();

    if (tracking_){
      getTrackingTrajectory();
    } else if (!collisionIndices_.empty() && execution_){
      trajectory_msgs::JointTrajectory trajectory;
      bool success = plan(currentTrajectory_.points.back().positions, &trajectory);
      if (success){
        mergeNewTrajectory(trajectory);
      } else {
        ROS_INFO("replanning failed!");
      }
    }
  }

  void TrackObjectCB(const std_msgs::String::ConstPtr& object){
    goalObject_ = object->data;
    if (goalObject_==""){
      tracking_ = false;
    } else {
      tracking_ = true;
      getTrackingTrajectory();
      startExecution();
      // inverseKinematics();
    }
  }

  void TrajectoryCB(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory){
    tracking_ = false;
    newTrajectory(*trajectory);
    startExecution();
  }

  void startExecution(){
    execution_ = true;
    executionTimer_ = nh_.createTimer(ros::Duration(0.016), &DynTrajController::ExecutionCB, this);
  }
   
  void newTrajectory(trajectory_msgs::JointTrajectory trajectory){
    if (trajectory.points[0].time_from_start.toSec() == 0.0){
      currentTrajectory_ = trajectory;
      virtualTime_ = 0;
      delayBuffer_ = 0;
      lastTime_ = ros::Time::now().toSec();
      // pathTimer_ = nh_.createTimer(ros::Duration(0.1), &DynTrajController::PathValidCB, this);
      updateMarkers();
      // checkPath(); // probably not needed as the planner was working with the same scene
    } else {
      mergeNewTrajectory(trajectory);
    }
  }

  void mergeNewTrajectory(trajectory_msgs::JointTrajectory trajectory){
    
    trajectory_msgs::JointTrajectory temptraj;

    int current_ind = 0;
    while(currentTrajectory_.points[current_ind+1].time_from_start.toSec() < virtualTime_){
      current_ind++;
      temptraj.points.push_back(currentTrajectory_.points[current_ind]);
    }
    
    for (unsigned int i = 0; i < trajectory.points.size(); i++){
      if (trajectory.points[i].time_from_start.toSec() > virtualTime_){
        temptraj.points.push_back(trajectory.points[i]);
      }
    }
    currentTrajectory_ = temptraj;
    delayBuffer_ = 0;
    updateMarkers();
  }
  
  void ExecutionCB(const ros::TimerEvent& event){

    double mspeed = 1;

    for (unsigned i=0; i<collisionIndices_.size(); i++){
      double coltime = currentTrajectory_.points[collisionIndices_[i]].time_from_start.toSec();
      coltime = (virtualTime_ - coltime)*2;
      mspeed += 1/(pow(coltime,1));
    }

    mspeed = min(1.0,max(-1.0,mspeed));    //limiting the speed setpoint
    mspeed = mspeed-movingSpeed_;       // finding difference old and new
    movingSpeed_ += min(mspeed,0.1);    // limit the rate of change for more gradual speedups/downs

    delayBuffer_ += max(0.0,0.7-movingSpeed_);

    double virtual_step = (ros::Time::now().toSec()-lastTime_)*movingSpeed_;
    virtualTime_ = max(0.0,virtualTime_+virtual_step);

    vector<double> setpoint;

    if (virtualTime_ <= 0){
      setpoint = currentTrajectory_.points[0].positions;
      setpoint.insert(setpoint.end(),
        currentTrajectory_.points[0].velocities.begin(),
        currentTrajectory_.points[0].velocities.end());

    } else if (virtualTime_ >= currentTrajectory_.points.back().time_from_start.toSec()){
      setpoint = currentTrajectory_.points.back().positions;
      setpoint.insert(setpoint.end(),
        currentTrajectory_.points.back().velocities.begin(),
        currentTrajectory_.points.back().velocities.end());

        if ((virtualTime_ >= currentTrajectory_.points.back().time_from_start.toSec()+1) && !tracking_){
          execution_ = false;
        }
    } else {
      int i = 0;
      while(currentTrajectory_.points[i+1].time_from_start.toSec() < virtualTime_){
        i++;
      }

      double T = currentTrajectory_.points[i+1].time_from_start.toSec();
      T -= currentTrajectory_.points[i].time_from_start.toSec();
      double t = virtualTime_-currentTrajectory_.points[i].time_from_start.toSec();

      vector<double> p0 = currentTrajectory_.points[i].positions;
      vector<double> v0 = currentTrajectory_.points[i].velocities;
      vector<double> p1 = currentTrajectory_.points[i+1].positions;
      vector<double> v1 = currentTrajectory_.points[i+1].velocities;

      setpoint = interp_cubic(T, t, p0, v0, p1, v1);
    }

    if (execution_ == false){
      zeroSpeed();
      executionTimer_.stop();
      pathTimer_.stop();
      ROS_INFO("Finished Trajectory");
    } else if (delayBuffer_>30){
      execution_ = false;
      executionTimer_.stop();
      pathTimer_.stop();
      trajectory_msgs::JointTrajectory traj;
      plan(currentTrajectory_.points.back().positions, &traj);
      newTrajectory(traj);
    } else {
      trackSetpoint(setpoint);
      lastTime_ = ros::Time::now().toSec();
    } 
  }

  void PathValidCB(const ros::TimerEvent& event){
    checkPath();
  }

  void checkPath(){
    ROS_DEBUG("Check Path");
    if (!currentTrajectory_.points.empty()){
      
      ros::Duration(0.001).sleep();
      moveit_msgs::RobotTrajectory robtraj;
      robtraj.joint_trajectory = currentTrajectory_;

      moveit_msgs::RobotState rs;
      rs.joint_state.position = currentTrajectory_.points[0].positions;

      rs.joint_state.name = planningScene_->getRobotModel()->getVariableNames();
      // rs.joint_state.name = kinematicModel_->getVariableNames();

      std::vector<std::size_t> index;
      std::string nm = "";

      bool valid = planningScene_->isPathValid(rs, robtraj,nm,false,&index);

      collisionIndices_.clear();
      if (valid){
        ROS_DEBUG("Path is valid!");
      } else { 
        ROS_DEBUG("Path is invalid!");
        for (std::size_t i = 0 ; i < index.size() ; ++i) {
          collisionIndices_.push_back(int(index[i]));
        }
      }
    }
  }

  bool plan(vector<double> target, trajectory_msgs::JointTrajectory *joint_trajectory){
    moveit::planning_interface::MoveGroup::Plan plan;

    robot_state::RobotState current_robot_state = planningScene_->getCurrentState();
    moveGroup_->setStartState(current_robot_state);

    // bool targetSet = moveGroup_->setJointValueTarget(currentTrajectory_.points.back().positions);

    bool targetSet = moveGroup_->setJointValueTarget(target);
    bool success = false;
    
    if (targetSet){
      success = moveGroup_->plan(plan);
      if (success){
        *joint_trajectory = plan.trajectory_.joint_trajectory;
      } else { 
        ROS_INFO("PLAN FAIL");
      }
    } else { 
      ROS_INFO("TargetSet FAIL");
    } 
    return success;
  }

  void getTrackingTrajectory(){
    if (goalObject_!=""){
      const Eigen::Affine3d objectTF = planningScene_->getFrameTransform(goalObject_);

      Eigen::Vector3d new_loc, loc_trans;
      loc_trans << -0.1,0.0,0.0;
      new_loc = objectTF * loc_trans;

      geometry_msgs::Pose goalpose;
      goalpose.position.x = new_loc.x();
      goalpose.position.y = new_loc.y();
      goalpose.position.z = new_loc.z();

      Eigen::Quaterniond rot(objectTF.rotation());
      goalpose.orientation.x = rot.x();
      goalpose.orientation.y = rot.y();
      goalpose.orientation.z = rot.z();
      goalpose.orientation.w = rot.w();

      visualization_msgs::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = ros::Time();
      marker.ns = "IK";
      marker.id = 1;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = goalpose;
      marker.scale.x = 0.1;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      markerPub_.publish(marker);

      robot_state::RobotState kinematic_state = planningScene_->getCurrentState();
      const robot_state::JointModelGroup* joint_model_group = planningScene_->getRobotModel()->getJointModelGroup("manipulator");

      bool found_ik = kinematic_state.setFromIK(joint_model_group, goalpose, 10, 0.1);

      if (found_ik){
        vector<double> joint_values;
        kinematic_state.copyJointGroupPositions(joint_model_group, joint_values);

        kinematic_state.setJointGroupPositions(joint_model_group, currentPosition_);
        const Eigen::Affine3d &ee_state = kinematic_state.getGlobalLinkTransform("ee_link");

        vector<double> goal,cur;
        
        for (unsigned int i = 0; i<3;i++) {
          cur.push_back(ee_state.translation()[i]);
          goal.push_back(new_loc[i]);
        }
        
        // bool far = 0.2<vector2Norm(currentPosition_,joint_values);
        bool far = 0.2<vector2Norm(cur,goal);

        bool collisions = !collisionIndices_.empty();

        if (execution_){
          
          kinematic_state.setJointGroupPositions(joint_model_group, currentTrajectory_.points.back().positions);
          const Eigen::Affine3d &ee_state = kinematic_state.getGlobalLinkTransform("ee_link");
          
          for (unsigned int i= 0; i<3;i++) cur[i] = ee_state.translation()[i];

          bool targetchange = 0.002<vector2Norm(cur,goal);

          // bool targetchange = 0.1<vector2Norm(currentTrajectory_.points.back().positions,joint_values);          

          if ((far && collisions)||(far && targetchange)){  //far away, collision or change causes replan
            ROS_INFO("replan");
            drift_ = true;
            double current_virtual_time = virtualTime_;
            
            trajectory_msgs::JointTrajectory joint_trajectory;
            bool good = plan(joint_values,&joint_trajectory);

            if (good){
              for (unsigned int i = 0; i < joint_trajectory.points.size(); i++){
                double newtime = joint_trajectory.points[i].time_from_start.toSec()+current_virtual_time;
                joint_trajectory.points[i].time_from_start = ros::Duration(newtime);
              }
              newTrajectory(joint_trajectory);
              drift_ = false;
              ROS_INFO("replan_done");
            } else {
              ROS_INFO("Replan Cannot find a new plan!");
            }

          } else if (!far && targetchange) { //close and changed goal; quickplan
            ROS_INFO("requickPlan");

            trajectory_msgs::JointTrajectory joint_trajectory;            
            trajectory_msgs::JointTrajectoryPoint jtp;

            jtp.time_from_start = ros::Duration(0.0);
            jtp.positions = currentPosition_;
            jtp.velocities = currentVelocity_;
            
            joint_trajectory.points.push_back(jtp);

            jtp.time_from_start = ros::Duration(0.2);
            jtp.positions = joint_values;
            for (unsigned int i = 0; i<6; i++) jtp.velocities[i] = 0.0;
            joint_trajectory.points.push_back(jtp);

            newTrajectory(joint_trajectory);    
          }
        } else {
          if (far){
            ROS_INFO("nonexec_track_plan");
            
            trajectory_msgs::JointTrajectory joint_trajectory;
            bool good = plan(joint_values,&joint_trajectory);
            if (good){
              newTrajectory(joint_trajectory);
            } else {
              ROS_INFO("Cannot find a plan!");
            }

          } else {
            ROS_INFO("non_exec_quickPlan");
            trajectory_msgs::JointTrajectory joint_trajectory;            
            trajectory_msgs::JointTrajectoryPoint jtp;

            jtp.time_from_start = ros::Duration(0.0);
            jtp.positions = currentPosition_;
            jtp.velocities = currentVelocity_;
            
            joint_trajectory.points.push_back(jtp);

            jtp.time_from_start = ros::Duration(0.2);
            jtp.positions = joint_values;
            for (unsigned int i = 0; i<6; i++) jtp.velocities[i] = 0.0;
            joint_trajectory.points.push_back(jtp);

            newTrajectory(joint_trajectory);             
          }
        }
      } else {
        ROS_INFO("NO IK SOLUTION FOUND!");
      }
    }
  }

  double vector2Norm(vector<double> v1,vector<double> v2){
    double result = 0;
    for (unsigned int i = 0; i < v1.size(); i++){
      result += pow(v1[i]-v2[i],2);
    }
    result = sqrt(result);
    return result;
  }

  void updateMarkers(){
    ROS_DEBUG("updateMarkers");
    
    visualization_msgs::Marker marker;
    std::vector<geometry_msgs::Point> markerPoints;

    robot_state::RobotState kinematic_state = planningScene_->getCurrentState();
    
    const robot_state::JointModelGroup* joint_model_group = planningScene_->getRobotModel()->getJointModelGroup("manipulator");
    // const robot_state::JointModelGroup* joint_model_group = kinematicModel_->getJointModelGroup("manipulator");

    for (unsigned int i=0; i<currentTrajectory_.points.size(); i++) {
      geometry_msgs::Point pt;

      kinematic_state.setJointGroupPositions(joint_model_group, currentTrajectory_.points[i].positions);

      const Eigen::Affine3d &ee_state = kinematic_state.getGlobalLinkTransform("ee_link");

      pt.x = ee_state.translation()[0];
      pt.y = ee_state.translation()[1];
      pt.z = ee_state.translation()[2];

      markerPoints.push_back(pt); 
    }

    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "Path";
    marker.id = 0;
    
    marker.points = markerPoints;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    // marker.scale.z = 0.1;

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    
    markerPub_.publish(marker);
  }

  std::vector<double> interp_cubic(double T,
    double t,
    std::vector<double> p0,
    std::vector<double> v0,
    std::vector<double> p1,
    std::vector<double> v1){

    std::vector<double> setpoint;
    std::vector<double> dot_setpoint;

    double a,b,c,d;

    for (unsigned int cnt = 0; cnt < 6; cnt++) {
      a = p0[cnt];
      b = v0[cnt];
      c = (-3 * p0[cnt] + 3 * p1[cnt] - 2 * T * v0[cnt]
          - T * v1[cnt]) / pow(T, 2);
      d = (2 * p0[cnt] - 2 * p1[cnt] + T * v0[cnt]
          + T * v1[cnt]) / pow(T, 3);

      setpoint.push_back(a + b * t + c * pow(t, 2) + d * pow(t, 3));
      dot_setpoint.push_back(b + 2*c*t + 3*d*pow(t, 2));
    }
    setpoint.insert(setpoint.end(),dot_setpoint.begin(),dot_setpoint.end());
    return setpoint;
  }

  void zeroSpeed(){
    std_msgs::String msg;
    msg.data = "speedj([0,0,0,0,0,0],30,1)";
    urPub_.publish(msg); 
  }

  void trackSetpoint(std::vector<double> setpoint){
    std::ostringstream oss;
    std_msgs::String msg;

    oss << "speedj([";

    double error[6],input[6];

    int lp_weight = 1;
    
    for (unsigned int i = 0; i < 5; i++){
      error[i] = setpoint[i] - currentPosition_[i];

      integrator_[i] += error[i];
      
      input[i] = kp_*error[i];
      input[i] += ki_*integrator_[i];
      input[i] += kd_*(error[i]-derivator_[i]);

      derivator_[i] = error[i];

      
      // input[i] = (lp_weight*lastInput_[i]+input[i])/(lp_weight+1.0);      

      // input[i] = ((setpoint[i] - lastSetPoint_[i])*62.5);
      input[i] = setpoint[i+6];
      input[i] += kp_*error[i];

      // input[i] = max(min(input[i],lastInput_[i]+0.03),lastInput_[i]-0.03);

      // input[i] = (lp_weight*lastInput_[i]+input[i])/(lp_weight+1.0);


      if (drift_){
        input[i] = 0.0;
      }

      lastInput_[i] = input[i];
      input[i] = max(min(input[i],3.3),-3.3);
      oss << input[i] << ",";

    }


    error[5] = setpoint[5] - currentPosition_[5];
    input[5] = kp_*error[5];
    input[5] = max(min(input[5],3.3),-3.3);

    

    std_msgs::Float64 fl;
    fl.data = setpoint[6]; //speed setpoint
    debugPub_.publish(fl);
    fl.data = setpoint[0]; //position setpoint
    debugPub2_.publish(fl);
    fl.data = input[0];    //input setpoint
    debugPub3_.publish(fl);

    
    oss << input[5] << "],30,1)";
    lastSetPoint_ = setpoint;

    msg.data = oss.str();
    urPub_.publish(msg);
  }


protected:
  
  double kp_,ki_,kd_;

  vector<double> derivator_;
  vector<double> integrator_;

  ros::NodeHandle nh_;
  
  double virtualTime_;
  double lastTime_;
  double movingSpeed_;
  double delayBuffer_;

  string goalObject_;

  bool execution_;
  bool tracking_;
  bool drift_;

  std::vector<int> collisionIndices_;

  std::vector<double> currentPosition_;
  std::vector<double> currentVelocity_;

  std::vector<double> setPoint_;
  std::vector<double> lastSetPoint_;
  std::vector<double> lastInput_;

  ros::Timer executionTimer_;
  ros::Timer pathTimer_;
  
  trajectory_msgs::JointTrajectory currentTrajectory_;
  
  ros::Publisher urPub_;
  ros::Publisher markerPub_;
  
  ros::Publisher debugPub_;
  ros::Publisher debugPub2_;
  ros::Publisher debugPub3_;

  ros::Subscriber trajSub_;
  ros::Subscriber jointSub_;
  ros::Subscriber ObjSub_;
  ros::Subscriber collisionSub_;

  // robot_model::RobotModelPtr kinematicModel_;
  
  planning_scene_monitor::PlanningSceneMonitorPtr scene_;
  planning_scene::PlanningScenePtr planningScene_;

  boost::shared_ptr<moveit::planning_interface::MoveGroup> moveGroup_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tl_dtc");

  DynTrajController DTC;

  ros::MultiThreadedSpinner s(4);
  ros::spin(s);

  return 0;
}
