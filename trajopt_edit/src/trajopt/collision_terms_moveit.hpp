#pragma once
#include "trajopt/common.hpp"
#include "trajopt/collision_checker.hpp"
#include "sco/modeling.hpp"
#include "sco/sco_fwd.hpp"
#include "cache.hxx"

//#include "moveit/collision_detection/collision_common.h"
//#include <moveit/planning_scene/planning_scene.h>

using namespace std;

namespace trajopt {

typedef collision_detection::Contact Contact;
typedef collision_detection::CollisionResult::ContactMap ContactMap;

struct CollisionEvaluator {
  virtual void CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) = 0;

  virtual void CalcDists(const DblVec& x, DblVec& exprs) = 0;
  virtual void CalcCollisions(const DblVec& x, vector<Contact>& collisions) = 0;

  void GetCollisionsCached(const DblVec& x, vector<Contact>&);
  virtual ~CollisionEvaluator() {}
  virtual VarVector GetVars()=0;

  Cache<size_t, vector<Contact>, 3> m_cache;
};
typedef boost::shared_ptr<CollisionEvaluator> CollisionEvaluatorPtr;



struct SingleTimestepCollisionEvaluator : public CollisionEvaluator {
public:
  SingleTimestepCollisionEvaluator(ConfigurationPtr rad, const VarVector& vars, double dist_check);
  /**
  @brief linearize all contact distances in terms of robot dofs

  Do a collision check between robot and environment.
  For each contact generated, return a linearization of the signed distance function
  */
  void CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs);
  /**
   * Same as CalcDistExpressions, but just the distances--not the expressions
   */
  void CalcDists(const DblVec& x, DblVec& dists);
  void CalcCollisions(const DblVec& x, vector<Contact>& collisions);
  VarVector GetVars() {return m_vars;}

  planning_scene::PlanningScenePtr m_env;
  CollisionCheckerPtr m_cc;
  ConfigurationPtr m_rad;
  VarVector m_vars;
  std::vector<std::string> m_links;
  double m_dist_check;
//  vector<OR::KinBody::LinkPtr> m_links;
//  short m_filterMask;
};





class TRAJOPT_API CollisionCost : public Cost {
public:
  /* constructor for single timestep */
  CollisionCost(double dist_pen, double dist_check, double coeff, ConfigurationPtr rad, const VarVector& vars);
  /* constructor for cast cost */
//  CollisionCost(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars0, const VarVector& vars1);

  virtual ConvexObjectivePtr convex(const vector<double>& x, Model* model);
  virtual double value(const vector<double>&);

private:
  CollisionEvaluatorPtr m_calc;
  double m_dist_pen;
  double m_dist_check;
  double m_coeff;
};


}
