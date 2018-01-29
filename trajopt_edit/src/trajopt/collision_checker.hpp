#pragma once
#include "typedefs.hpp"
#include <set>
#include <utility>
#include <iostream>
//#include <openrave/openrave.h>
//#include "configuration_space.hpp"
#include "macros.h"

#include <Eigen/Eigen>
#include <geometric_shapes/shapes.h>
#include <moveit/planning_scene/planning_scene.h>

namespace trajopt {

struct MoveitKinBody {
	Eigen::Affine3d 		tf;
	shapes::ShapeConstPtr 	shape;
	std::string 			name;
	int 					filtermask;
	MoveitKinBody(Eigen::Affine3d _tf, shapes::ShapeConstPtr _shape, string _name, int _filtermask):
			tf(_tf),shape(_shape),name(_name),filtermask(_filtermask){}
};

typedef boost::shared_ptr<MoveitKinBody> MoveitKinBodyPtr;

enum CastCollisionType {
  CCType_None,
  CCType_Time0,
  CCType_Time1,
  CCType_Between
};

struct Collision {
  MoveitKinBodyPtr linkA;
  MoveitKinBodyPtr linkB;
  Eigen::Vector3d ptA, ptB, normalB2A; /* normal points from 2 to 1 */
  Eigen::Vector3d ptB1;
  double distance; /* pt1 = pt2 + normal*dist */
  float weight, time;
  CastCollisionType cctype;
  Collision(const MoveitKinBodyPtr linkA, const MoveitKinBodyPtr linkB, const Eigen::Vector3d& ptA, const Eigen::Vector3d& ptB, const Eigen::Vector3d& normalB2A, double distance, float weight=1, float time=0) :
    linkA(linkA), linkB(linkB), ptA(ptA), ptB(ptB), normalB2A(normalB2A), distance(distance), weight(weight), time(0), cctype(CCType_None) {}
};
//TRAJOPT_API std::ostream& operator<<(std::ostream&, const Collision&);

enum CollisionFilterGroups {
  RobotFilter = 1,
  KinBodyFilter = 2,
};

/** 
Each CollisionChecker object has a copy of the world, so for performance, don't make too many copies  
*/ 
class TRAJOPT_API CollisionChecker {
public:

  /** check everything vs everything else */
//  virtual void AllVsAll(vector<Collision>& collisions)=0;
  /** check link vs everything else */

  virtual void LinkVsAll(const string& link, vector<Collision>& collisions, short filterMask)=0;
  virtual void LinksVsAll(const vector<string>& links, vector<Collision>& collisions, short filterMask)=0;

  /** check robot vs everything else. includes attached bodies */
//  void BodyVsAll(const KinBody& body, vector<Collision>& collisions, short filterMask=-1) {
//    LinksVsAll(body.GetLinks(), collisions, filterMask);
//  }

  /** contacts of distance < (arg) will be returned */
  virtual void SetContactDistance(float distance)  = 0;
  virtual double GetContactDistance() = 0;
  
//  virtual void PlotCollisionGeometry(vector<OpenRAVE::GraphHandlePtr>&) {throw std::runtime_error("not implemented");}

//  virtual void ContinuousCheckTrajectory(const TrajArray& traj, Configuration& rad, vector<Collision>& collisions) {throw std::runtime_error("not implemented");}
  
  /** Find contacts between swept-out shapes of robot links and everything in the environment, as robot goes from startjoints to endjoints */ 
  virtual void CastVsAll(const vector<string>& links, const DblVec& startjoints, const DblVec& endjoints, vector<Collision>& collisions) {throw std::runtime_error("not implemented");}

  /** Finds all self collisions when all joints are set to zero, and ignore collisions between the colliding links */
//  void IgnoreZeroStateSelfCollisions();
//  void IgnoreZeroStateSelfCollisions(OpenRAVE::KinBodyPtr body);

  /** Prevent this pair of links from colliding */
  virtual void ExcludeCollisionPair(const MoveitKinBody& link0, const MoveitKinBody& link1) = 0;
  virtual void IncludeCollisionPair(const MoveitKinBody& link0, const MoveitKinBody& link1) = 0;

  planning_scene::PlanningScenePtr GetEnv() {return m_env;}
  vector<double> GetDofvals() {return m_dofvals;}
  void SetDofvals(vector<double> df) {m_dofvals = df;}

  virtual ~CollisionChecker() {}
  /** Get or create collision checker for this environment */
//  static boost::shared_ptr<CollisionChecker> GetOrCreate(planning_scene::PlanningScenePtr env);

protected:
  CollisionChecker(planning_scene::PlanningScenePtr env) : m_env(env) {
	  for (int i=0;i<6;i++){
		  m_dofvals.push_back(0.0);
	  }
  }

  planning_scene::PlanningScenePtr m_env;
  vector<double> m_dofvals;
};
typedef boost::shared_ptr<CollisionChecker> CollisionCheckerPtr;

CollisionCheckerPtr TRAJOPT_API CreateCollisionChecker(planning_scene::PlanningScenePtr env);

//TRAJOPT_API void PlotCollisions(const std::vector<Collision>& collisions, OR::EnvironmentBase& env, vector<OR::GraphHandlePtr>& handles, double safe_dist);

}

