#include "trajopt/collision_terms_moveit.hpp"
#include "trajopt/collision_checker.hpp"
//#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "sco/expr_vec_ops.hpp"
#include "sco/expr_ops.hpp"
#include "sco/sco_common.hpp"
#include <boost/foreach.hpp>
#include "utils/eigen_conversions.hpp"
#include "sco/modeling_utils.hpp"
#include "utils/stl_to_string.hpp"
#include "utils/logging.hpp"
#include <boost/functional/hash.hpp>

#include <algorithm>

using namespace sco;
using namespace util;
using namespace std;

namespace trajopt {


void CollisionsToDistances(const vector<Contact>& collisions, const vector<string>& m_links,
    DblVec& dists, double dist_check) {
  // Note: this checking (that the links are in the list we care about) is probably unnecessary
  // since we're using LinksVsAll
	dists.clear();
	dists.reserve(collisions.size());

	BOOST_FOREACH(const Contact& col, collisions) {

		vector<string>::const_iterator itA = find(m_links.begin(), m_links.end(),col.body_name_1);
		vector<string>::const_iterator itB = find(m_links.begin(), m_links.end(),col.body_name_2);

		if (itA != m_links.end() || itB != m_links.end()) {
			dists.push_back(col.depth - dist_check); //subtracting dist_check for the padding
    }
  }
}



void CollisionsToDistanceExpressions(const vector<Contact>& collisions, Configuration& rad,
    const vector<string>& m_links, const VarVector& vars, const DblVec& dofvals, vector<AffExpr>& exprs, bool isTimestep1, double dist_check) {

  exprs.clear();
  exprs.reserve(collisions.size());


  robot_state::RobotState* rs = new robot_state::RobotState(rad.GetEnv()->getCurrentStateNonConst());
  rs->setJointGroupPositions(rad.GetGroupName(), dofvals);
  rs->update(true);

//  rad.SetDOFValues(dofvals); // since we'll be calculating jacobians (commented as it does not work like this atm)

  BOOST_FOREACH(const Contact& col, collisions) {

	AffExpr dist(col.depth - dist_check); //subtract PADDING!

//    Link2Int::const_iterator itA = link2ind.find(col.linkA);

	vector<string>::const_iterator itA = find(m_links.begin(), m_links.end(),col.body_name_1);
	vector<string>::const_iterator itB = find(m_links.begin(), m_links.end(),col.body_name_2);

	Vector3d pt1, pt2;
	//calculate the points on A and B where they collide. Not sure what points are required -exactly-

    if (itA != m_links.end()) {

//      Eigen::MatrixXd jacobian;
      Eigen::Vector3d ref = rad.GetEnv()->getFrameTransform(*itA).inverse()*col.pos;

      const moveit::core::JointModelGroup* jmg = rs->getJointModelGroup(rad.GetGroupName());
      Eigen::MatrixXd jacobian = rs->getJacobian(jmg, jmg->getLinkModel(*itA), ref);

      VectorXd dist_grad = -col.normal.transpose()*jacobian;
      exprInc(dist, varDot(dist_grad, vars));
      exprInc(dist, -dist_grad.dot(toVectorXd(dofvals)));

    }
    if (itB != m_links.end()) {

      Eigen::Vector3d ref = rad.GetEnv()->getFrameTransform(*itB).inverse()*(col.pos+col.normal*col.depth);

      const moveit::core::JointModelGroup* jmg = rs->getJointModelGroup(rad.GetGroupName());
      Eigen::MatrixXd jacobian = rs->getJacobian(jmg, jmg->getLinkModel(*itB), ref);

      VectorXd dist_grad = col.normal.transpose()*jacobian;
      exprInc(dist, varDot(dist_grad, vars));
      exprInc(dist, -dist_grad.dot(toVectorXd(dofvals)));

    }
    if (itA != m_links.end() || itB != m_links.end()) {
      exprs.push_back(dist);
    }
  }
  delete rs;
  LOG_DEBUG("%ld distance expressions\n", exprs.size());
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline size_t hash(const DblVec& x) {
  return boost::hash_range(x.begin(), x.end());
}

void CollisionEvaluator::GetCollisionsCached(const DblVec& x, vector<Contact>& collisions) {
  double key = hash(getDblVec(x, GetVars()));
  vector<Contact>* it = m_cache.get(key);
  if (it != NULL) {
    LOG_DEBUG("using cached collision check\n");
    collisions = *it;
  }
  else {
    LOG_DEBUG("not using cached collision check\n");
    CalcCollisions(x, collisions);
    m_cache.put(key, collisions);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(ConfigurationPtr rad, const VarVector& vars, double dist_check):
  m_env(rad->GetEnv()),
  m_cc(CollisionChecker::GetOrCreate(m_env)),
  m_rad(rad),
  m_vars(vars),
  m_dist_check(dist_check),
  m_links(rad->getAffectedLinks()){}
//
////  vector<KinBody::LinkPtr> links;
//  vector<int> inds;
//  rad->GetAffectedLinks(m_links, true, inds);
//  for (int i=0; i < m_links.size(); ++i) {
//    m_link2ind[m_links[i].get()] = inds[i];
//  }
//  // TODO add argument

void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x, vector<Contact>& collisions) {
  DblVec dofvals = getDblVec(x, m_vars);

  ContactMap contacts;

  robot_state::RobotState rs = m_env->getCurrentStateNonConst();
  rs.setJointGroupPositions(m_rad->GetGroupName(), dofvals);
  LOG_INFO("Collision Checking on:[ %f, %f, %f, %f, %f, %f]",dofvals[0],dofvals[1],dofvals[2],dofvals[3],dofvals[4],dofvals[5]);
  rs.update(true);

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  req.group_name = "manipulator";
  req.contacts = true;
  req.max_contacts = 100;

  m_env->checkCollision(req, res, rs, m_env->getAllowedCollisionMatrix());



  //  m_env->getCollidingPairs(contacts, rs, m_env->getAllowedCollisionMatrix());
  typedef ContactMap::iterator it;
  for (it i = contacts.begin(); i != contacts.end(); i++){
    collisions.push_back(i->second[0]);
  }

//  LOG_INFO("Number of Collisions: %zu", collisions.size());
}


void SingleTimestepCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists) {
  vector<Contact> collisions;
  GetCollisionsCached(x, collisions);
  CollisionsToDistances(collisions, m_links, dists, m_dist_check);
}


void SingleTimestepCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) {
  vector<Contact> collisions;
  GetCollisionsCached(x, collisions);

  DblVec dofvals = getDblVec(x, m_vars);
  CollisionsToDistanceExpressions(collisions, *m_rad, m_links, m_vars, dofvals, exprs, false, m_dist_check);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

CollisionCost::CollisionCost(double dist_pen, double dist_check, double coeff, ConfigurationPtr rad, const VarVector& vars) :
    Cost("collision"),
//    m_calc(new SingleTimestepCollisionEvaluator(rad, vars)),
	m_calc(new SingleTimestepCollisionEvaluator(rad, vars, dist_check)),
	m_dist_pen(dist_pen),
	m_dist_check(dist_check),
	m_coeff(coeff){}

/*
CollisionCost::CollisionCost(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars0, const VarVector& vars1) :
    Cost("cast_collision"),
    m_calc(new CastCollisionEvaluator(rad, vars0, vars1)), m_dist_pen(dist_pen), m_coeff(coeff)
{}

*/


ConvexObjectivePtr CollisionCost::convex(const vector<double>& x, Model* model) {
  ConvexObjectivePtr out(new ConvexObjective(model));
  vector<AffExpr> exprs;
  m_calc->CalcDistExpressions(x, exprs);

  for (int i=0; i < exprs.size(); ++i) {
    AffExpr viol = exprSub(AffExpr(m_dist_pen), exprs[i]);
    out->addHinge(viol, m_coeff);
  }
  return out;
}

double CollisionCost::value(const vector<double>& x) {
  DblVec dists;
  m_calc->CalcDists(x, dists);
  double out = 0;
  for (int i=0; i < dists.size(); ++i) {
    out += pospart(m_dist_pen - dists[i]) * m_coeff;
  }
  return out;
}

}
