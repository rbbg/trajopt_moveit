#pragma once
#include "typedefs.hpp"
#include <openrave/openrave.h>
#include "macros.h"
namespace trajopt {

class TRAJOPT_API Configuration {
public:

  virtual void SetDOFValues(const DblVec& dofs) = 0;
  virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const = 0;
  virtual DblVec GetDOFValues() = 0;
  virtual int GetDOF() const = 0;
  
//  these are changed from OPENRAVE

  virtual planning_scene_monitor::PlanningSceneMonitorPtr GetEnv() = 0;

//  virtual DblMatrix PositionJacobian(int link_ind, const DblVec& pt) const = 0;
//  virtual DblMatrix RotationJacobian(int link_ind) const = 0;

//  virtual bool DoesAffect(const KinBody::Link& link) = 0;
//  virtual vector<OpenRAVE::KinBodyPtr> GetBodies() = 0;
//  virtual vector<KinBody::LinkPtr> GetAffectedLinks() = 0;

//  virtual void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds) = 0;

//Ends here

  virtual DblVec RandomDOFValues() = 0;
  virtual ~Configuration() {}

//  Commenting the Saver structs for now.
 /*

  struct Saver {
    virtual ~Saver(){}
  };
  typedef boost::shared_ptr<Saver> SaverPtr;
  struct GenericSaver : public Saver {
    DblVec dofvals;
    Configuration* parent;
    GenericSaver(Configuration* _parent) : dofvals(_parent->GetDOFValues()), parent(_parent) {}
    ~GenericSaver() {
      parent->SetDOFValues(dofvals);
    }
  }; // inefficient
  virtual SaverPtr Save() {
    return SaverPtr(new GenericSaver(this));
  }

  */
};


typedef boost::shared_ptr<Configuration> ConfigurationPtr;

/**
Stores an OpenRAVE robot and the active degrees of freedom  
*/
class TRAJOPT_API RobotAndDOF : public Configuration {
public:
  RobotAndDOF(moveit::core::RobotStatePtr _robot, const IntVec& _joint_inds) :
    robot(_robot), joint_inds(_joint_inds){}

  void SetDOFValues(const DblVec& dofs);
  void GetDOFLimits(DblVec& lower, DblVec& upper) const;
  DblVec GetDOFValues();
  int GetDOF() const;

  virtual planning_scene_monitor::PlanningSceneMonitorPtr GetEnv() {return robot->GetEnv();};

  IntVec GetJointIndices() const {return joint_inds;}
//  DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const;
//  DblMatrix RotationJacobian(int link_ind) const;

//  OR::RobotBasePtr GetRobot() const {return boost::dynamic_pointer_cast<RobotBase>(robot);}
//  virtual vector<OpenRAVE::KinBodyPtr> GetBodies();

//  bool DoesAffect(const KinBody::Link& link);
//  std::vector<KinBody::LinkPtr> GetAffectedLinks();
//  void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds);

//  DblVec RandomDOFValues();


/*
  struct RobotSaver : public Saver {
    OpenRAVE::KinBody::KinBodyStateSaver saver;
    RobotSaver(OpenRAVE::KinBodyPtr robot) : saver(robot) {}
  };
  SaverPtr Save() {
    return SaverPtr(new RobotSaver(robot));
  }
  void SetRobotActiveDOFs();
  
  */

private:
  robot_state::RobotStatePtr robot;
  IntVec joint_inds;
//  int affinedofs;
//  OR::Vector rotationaxis;
};

typedef boost::shared_ptr<RobotAndDOF> RobotAndDOFPtr;

}
