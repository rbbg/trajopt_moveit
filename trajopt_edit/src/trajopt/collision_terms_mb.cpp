#include "trajopt/collision_terms_mb.hpp"
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


void CollisionsToDistances(const vector<Collision>& collisions, const vector<string>& m_links,
    DblVec& dists, double dist_check) {
  // Note: this checking (that the links are in the list we care about) is probably unnecessary
  // since we're using LinksVsAll
	dists.clear();
	dists.reserve(collisions.size());

	BOOST_FOREACH(const Collision& col, collisions) {

		vector<string>::const_iterator itA = find(m_links.begin(), m_links.end(),col.linkA->name);
		vector<string>::const_iterator itB = find(m_links.begin(), m_links.end(),col.linkB->name);

		if (itA != m_links.end() || itB != m_links.end()) {
			dists.push_back(col.distance);
    }
  }
}



void CollisionsToDistanceExpressions(const vector<Collision>& collisions, Configuration& rad,
    const vector<string>& m_links, const VarVector& vars, const DblVec& dofvals, vector<AffExpr>& exprs, bool isTimestep1, double dist_check) {

  exprs.clear();
  exprs.reserve(collisions.size());

  robot_state::RobotState* rs = new robot_state::RobotState(rad.GetEnv()->getCurrentStateNonConst());
  rs->setJointGroupPositions(rad.GetGroupName(), dofvals);
  rs->update(true);

//  rad.SetDOFValues(dofvals); // since we'll be calculating jacobians (commented as it does not work like this atm)
  BOOST_FOREACH(const Collision& col, collisions) {

	AffExpr dist(col.distance); //subtract PADDING!

	vector<string>::const_iterator itA = find(m_links.begin(), m_links.end(),col.linkA->name);
	vector<string>::const_iterator itB = find(m_links.begin(), m_links.end(),col.linkB->name);

	Vector3d pt1, pt2;
	//calculate the points on A and B where they collide. Not sure what points are required -exactly-

//	LOG_INFO("starting jacob calcs");

    if (itA != m_links.end()) {

//      Eigen::MatrixXd jacobian;

//      LOG_INFO("Calculating Jacobian for: %s with dofs: %f, %f, %f, %f, %f, %f", col.linkA->name.c_str(), dofvals[0],dofvals[1],dofvals[2],dofvals[3],dofvals[4],dofvals[5]);

      const moveit::core::JointModelGroup* jmg = rs->getJointModelGroup(rad.GetGroupName());

      Eigen::Vector3d ref(0.0, 0.0, 0.0);
//      Eigen::Vector3d ref = col.ptA;

      ref = rad.GetEnv()->getFrameTransform(col.linkA->name).inverse()*ref;

      Eigen::MatrixXd jacobian = rs->getJacobian(jmg, jmg->getLinkModel(col.linkA->name), ref); // ref was col.ptA

      std::cout << jacobian << "\n";

      jacobian = jacobian.transpose();

      Eigen::MatrixXd tjac = jacobian.block<3,6>(0,0); //convert to translation jacobian

      VectorXd dist_grad = col.normalB2A.transpose()*tjac;
      exprInc(dist, varDot(dist_grad, vars));
      exprInc(dist, -dist_grad.dot(toVectorXd(dofvals)));

    }
    if (itB != m_links.end()) {

//      LOG_INFO("Calculating Jacobian for: %s with dofs: %f, %f, %f, %f, %f, %f", col.linkB->name.c_str(), dofvals[0],dofvals[1],dofvals[2],dofvals[3],dofvals[4],dofvals[5]);

      const moveit::core::JointModelGroup* jmg = rs->getJointModelGroup(rad.GetGroupName());

//      Eigen::Vector3d ref = (isTimestep1 && (col.cctype == CCType_Between)) ? col.ptB1 : col.ptB;
      Eigen::Vector3d ref(0.0, 0.0, 0.0);

      ref = rad.GetEnv()->getFrameTransform(col.linkB->name).inverse()*ref;

//      LOG_INFO("B: %f %f %f",col.ptB[0],col.ptB[1],col.ptB[2]);
//      LOG_INFO("ref: %f %f %f",ref[0],ref[1],ref[2]);

      Eigen::MatrixXd jacobian = rs->getJacobian(jmg, jmg->getLinkModel(col.linkB->name), ref);

      jacobian = jacobian.transpose();

//      std::cout << jacobian << '\n';

      Eigen::MatrixXd tjac = jacobian.block<3,6>(0,0); //convert to just translation jacobian

//      std::cout << tjac << '\n';

      VectorXd dist_grad2 = -col.normalB2A.transpose()*jacobian;
      VectorXd dist_grad = -col.normalB2A.transpose()*tjac;

//      std::cout << "dist_grad: " << dist_grad << "\n";

      exprInc(dist, varDot(dist_grad, vars));
      exprInc(dist, -dist_grad.dot(toVectorXd(dofvals)));

    }
    if (itA != m_links.end() || itB != m_links.end()) {
      exprs.push_back(dist);
    }
  }
  delete rs;
}


void CollisionsToDistanceExpressions(const vector<Collision>& collisions, Configuration& rad,
	    const vector<string>& m_links, const VarVector& vars0, const VarVector& vars1,
		const DblVec& vals0, const DblVec& vals1, vector<AffExpr>& exprs, double dist_check) {

	vector<AffExpr> exprs0, exprs1;
	CollisionsToDistanceExpressions(collisions, rad, m_links, vars0, vals0, exprs0, false, dist_check);
	CollisionsToDistanceExpressions(collisions, rad, m_links, vars1, vals1, exprs1, true, dist_check);

	exprs.resize(exprs0.size());
	for (int i = 0; i < exprs0.size(); ++i){
		exprScale(exprs0[i], (1-collisions[i].time));
		exprScale(exprs1[i], collisions[i].time);
		exprs[i] = AffExpr(0);
		exprInc(exprs[i], exprs0[i]);
		exprInc(exprs[i], exprs1[i]);
		cleanupAff(exprs[i]);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline size_t hash(const DblVec& x) {
  return boost::hash_range(x.begin(), x.end());
}

void CollisionEvaluator::GetCollisionsCached(const DblVec& x, vector<Collision>& collisions) {
//  for (int i =0; i<x.size();i=i+6){
//	  LOG_INFO("getCol %f,%f,%f,%f,%f,%f", x[i],x[i+1],x[i+2],x[i+3],x[i+4],x[i+5]);
//  }
  double key = hash(getDblVec(x, GetVars()));
  vector<Collision>* it = m_cache.get(key);
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
//  m_cc(boost::dynamic_pointer_cast<CollisionChecker>(CreateCollisionChecker(m_env))),
  m_rad(rad),
  m_vars(vars),
  m_dist_check(dist_check),
  m_filterMask(-1),
  m_links(rad->getAffectedLinks()){}
//
////  vector<KinBody::LinkPtr> links;
//  vector<int> inds;
//  rad->GetAffectedLinks(m_links, true, inds);
//  for (int i=0; i < m_links.size(); ++i) {
//    m_link2ind[m_links[i].get()] = inds[i];
//  }
//  // TODO add argument

void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x, vector<Collision>& collisions) {
  DblVec dofvals = getDblVec(x, m_vars);
  m_rad->GetCC()->SetDofvals(dofvals);
  m_rad->GetCC()->LinksVsAll(m_links, collisions, m_filterMask);
}


void SingleTimestepCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  CollisionsToDistances(collisions, m_links, dists, m_dist_check);
}


void SingleTimestepCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);

//  LOG_INFO("SINGLE numCollisions: %zu", collisions.size());
//
//  for (int i = 0; i < collisions.size(); i++){
//	  LOG_INFO("Cast col between: %s, %s",collisions[i].linkA->name.c_str(),collisions[i].linkB->name.c_str());
//  }

  DblVec dofvals = getDblVec(x, m_vars);
  CollisionsToDistanceExpressions(collisions, *m_rad, m_links, m_vars, dofvals, exprs, false, m_dist_check);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

CastCollisionEvaluator::CastCollisionEvaluator(ConfigurationPtr rad, const VarVector& vars0, const VarVector& vars1, double dist_check):
  m_env(rad->GetEnv()),
//  m_cc(boost::dynamic_pointer_cast<CollisionChecker>(CreateCollisionChecker(m_env))),
  m_rad(rad),
  m_vars0(vars0),
  m_vars1(vars1),
  m_dist_check(dist_check),
  m_filterMask(-1),
  m_links(rad->getAffectedLinks()){}
//
////  vector<KinBody::LinkPtr> links;
//  vector<int> inds;
//  rad->GetAffectedLinks(m_links, true, inds);
//  for (int i=0; i < m_links.size(); ++i) {
//    m_link2ind[m_links[i].get()] = inds[i];
//  }
//  // TODO add argument

void CastCollisionEvaluator::CalcCollisions(const DblVec& x, vector<Collision>& collisions) {
  DblVec dofvals0 = getDblVec(x, m_vars0);
  DblVec dofvals1 = getDblVec(x, m_vars1);

  m_rad->GetCC()->SetDofvals(dofvals0);
  m_rad->GetCC()->CastVsAll(m_links, dofvals0, dofvals1, collisions);
}


void CastCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);

//  LOG_INFO("Calc Dists: numCollisions: %zu", collisions.size());
//  for (int i = 0; i < collisions.size(); i++){
//	  LOG_INFO("Cast col between: %s, %s",collisions[i].linkA->name.c_str(),collisions[i].linkB->name.c_str());
//  }
//  for (int i = 0; i < collisions.size(); i++){
//	  LOG_INFO("Cast col between: %s, %s",collisions[i].linkA->name.c_str(),collisions[i].linkB->name.c_str());
//  }

  CollisionsToDistances(collisions, m_links, dists, m_dist_check);
}


void CastCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);

//  LOG_INFO("CAST numCollisions: %zu", collisions.size());
//  for (int i = 0; i < collisions.size(); i++){
//	  LOG_INFO("Cast col between: %s, %s",collisions[i].linkA->name.c_str(),collisions[i].linkB->name.c_str());
//  }

  DblVec dofvals0 = getDblVec(x, m_vars0);
  DblVec dofvals1 = getDblVec(x, m_vars1);
  CollisionsToDistanceExpressions(collisions, *m_rad, m_links, m_vars0, m_vars1, dofvals0, dofvals1, exprs, m_dist_check);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////


CollisionCost::CollisionCost(double dist_pen, double dist_check, double coeff, ConfigurationPtr rad, const VarVector& vars) :
    Cost("collision"),
	m_calc(new SingleTimestepCollisionEvaluator(rad, vars, dist_check)),
	m_dist_pen(dist_pen),
	m_dist_check(dist_check),
	m_coeff(coeff){}


CollisionCost::CollisionCost(double dist_pen, double dist_check, double coeff, ConfigurationPtr rad, const VarVector& vars0, const VarVector& vars1) :
    Cost("cast_collision"),
    m_calc(new CastCollisionEvaluator(rad, vars0, vars1, dist_check)),
	m_dist_pen(dist_pen),
	m_dist_check(dist_check),
	m_coeff(coeff){}

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//ALMOST COPIED

CollisionConstraint::CollisionConstraint(double dist_pen, double dist_check, double coeff, ConfigurationPtr rad, const VarVector& vars) :
	m_calc(new SingleTimestepCollisionEvaluator(rad, vars, dist_check)),
	m_dist_pen(dist_pen),
	m_dist_check(dist_check),
	m_coeff(coeff)
	{
	name_ = "collision";
}


CollisionConstraint::CollisionConstraint(double dist_pen, double dist_check, double coeff, ConfigurationPtr rad, const VarVector& vars0, const VarVector& vars1) :
    m_calc(new CastCollisionEvaluator(rad, vars0, vars1, dist_check)),
	m_dist_pen(dist_pen),
	m_dist_check(dist_check),
	m_coeff(coeff)
	{
	name_ = "collision";
}

ConvexConstraintsPtr CollisionConstraint::convex(const vector<double>& x, Model* model) {
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  vector<AffExpr> exprs;

  m_calc->CalcDistExpressions(x, exprs);

  for (int i=0; i < exprs.size(); ++i) {
    AffExpr viol = exprSub(AffExpr(m_dist_pen), exprs[i]);
    out->addIneqCnt(exprMult(viol, m_coeff));
  }
  return out;
}

DblVec CollisionConstraint::value(const vector<double>& x) {
  DblVec dists;
  m_calc->CalcDists(x, dists);
  DblVec out(dists.size());
  for (int i=0; i < dists.size(); ++i) {
    out[i] = pospart(m_dist_pen - dists[i]) * m_coeff;
  }
  return out;
}

}
