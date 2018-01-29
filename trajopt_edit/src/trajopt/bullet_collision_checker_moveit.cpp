#include "trajopt/collision_checker.hpp"
#include "trajopt/configuration_space.hpp"
#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>

//#include <openrave-core.h>
#include "utils/eigen_conversions.hpp"
#include <boost/foreach.hpp>
#include <vector>
#include <iostream>
#include <LinearMath/btConvexHull.h>
#include <utils/stl_to_string.hpp>
#include "utils/logging.hpp"
//#include "openrave_userdata_utils.hpp"

#include <geometric_shapes/shapes.h>
#include <boost/make_shared.hpp>

using namespace util;
using namespace std;
using namespace trajopt;
//using namespace OpenRAVE;

namespace {

#define METERS 
// there's some scale-dependent parameters. By convention I'll put METERS to mark it
const float MARGIN = 0;

#if 1
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
ostream &operator<<(ostream &stream, const btVector3& v) {
  stream << v.x() << " " << v.y() << " " << v.z();
  return stream;
}
ostream &operator<<(ostream &stream, const btQuaternion& v) {
  stream << v.w() << " " << v.x() << " " << v.y() << " " << v.z();
  return stream;
}
ostream &operator<<(ostream &stream, const btTransform& v) {
  stream << v.getOrigin() << " " << v.getRotation();
  return stream;
}
#pragma GCC diagnostic pop
#endif


class CollisionObjectWrapper : public btCollisionObject {
public:
  CollisionObjectWrapper(MoveitKinBodyPtr body) : m_body(body), m_index(-1) {
	  //setUserPointer(body);
  }
  vector<boost::shared_ptr<void> > m_data;
  MoveitKinBodyPtr m_body;
  int m_index; // index into collision matrix

  template<class T>
  void manage(T* t) { // manage memory of this object
    m_data.push_back(boost::shared_ptr<T>(t));
  }

  template<class T>
  void manage(boost::shared_ptr<T> t) {
    m_data.push_back(t);
  }
};


typedef CollisionObjectWrapper COW;
typedef boost::shared_ptr<CollisionObjectWrapper> COWPtr;



inline const MoveitKinBodyPtr getLink(const btCollisionObject* o) {
  return static_cast<const CollisionObjectWrapper*>(o)->m_body;
}

extern void nearCallback(btBroadphasePair& collisionPair,
    btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo);



//btVector3 toBt(const OR::Vector& v){
//  return btVector3(v[0], v[1], v[2]);
//}
//OR::Vector toOR(const btVector3& v) {
//  return OR::Vector(v.x(), v.y(), v.z());
//}
//btQuaternion toBtQuat(const OR::Vector& q) {
//  return btQuaternion(q[1], q[2], q[3], q[0]);
//}
//btTransform toBt(const OR::Transform& t){
//  return btTransform(toBtQuat(t.rot), toBt(t.trans));
//}

btTransform toBt(const Eigen::Affine3d& t){
	btVector3 tran(t.translation().x(),t.translation().y(),t.translation().z());
	Eigen::Quaterniond quat(t.rotation());
	btQuaternion rot(quat.x(),quat.y(),quat.z(),quat.w());
  return btTransform(rot, tran);
}

Eigen::Vector3d toEigen(const btVector3& v) {
	return Eigen::Vector3d(v.x(),v.y(),v.z());
}

//bool isIdentity(const OpenRAVE::Transform& T) {
//  float e = 1e-6;
//  return
//      fabs(T.trans.x) < e &&
//      fabs(T.trans.y) < e &&
//      fabs(T.trans.z) < e &&
//      fabs(T.rot[0]-1) < e &&
//      fabs(T.rot[1]) < e &&
//      fabs(T.rot[2]) < e &&
//      fabs(T.rot[3]) < e;
//}


void GetAverageSupport(const btConvexShape* shape, const btVector3& localNormal, float& outsupport, btVector3& outpt) {
  btVector3 ptSum(0,0,0);
  float ptCount = 0;
  float maxSupport=-1000;
  const float EPSILON = 1e-3;
  const btPolyhedralConvexShape* pshape = dynamic_cast<const btPolyhedralConvexShape*>(shape);
  if (pshape) {
    int nPts = pshape->getNumVertices();

    for (int i=0; i < nPts; ++i) {
      btVector3 pt;
      pshape->getVertex(i, pt);
//      cout << "pt: " << pt << endl;
      float sup  = pt.dot(localNormal);
      if (sup > maxSupport + EPSILON) {
        ptCount=1;
        ptSum = pt;
        maxSupport = sup;
      }
      else if (sup < maxSupport - EPSILON) {
      }
      else {
        ptCount += 1;
        ptSum += pt;
      }
    }
    outsupport = maxSupport;
    outpt = ptSum / ptCount;
  }
  else  {
    outpt = shape->localGetSupportingVertexWithoutMargin(localNormal);
    outsupport = localNormal.dot(outpt);
  }
}


btCollisionShape* createShapePrimitive(shapes::ShapeConstPtr geom, bool useTrimesh, CollisionObjectWrapper* cow) {

  btCollisionShape* subshape=0;

/*
#if OPENRAVE_VERSION_MINOR <= 8
    #define GT_Box KinBody::Link::GEOMPROPERTIES::GeomBox 
    #define GT_Sphere KinBody::Link::GEOMPROPERTIES::GeomSphere 
    #define GT_Cylinder KinBody::Link::GEOMPROPERTIES::GeomCylinder 
    #define GT_TriMesh KinBody::Link::GEOMPROPERTIES::GeomTrimesh 
    #define TriMesh KinBody::Link::TRIMESH
#endif
*/

  switch (geom->type) {
  case 4: { //box
	boost::shared_ptr<const shapes::Box> boxptr = boost::static_pointer_cast<const shapes::Box>(geom);
	LOG_DEBUG("box shape %f %f %f",boxptr->size[0],boxptr->size[1],boxptr->size[2]);
    subshape = new btBoxShape(btVector3(boxptr->size[0],boxptr->size[1],boxptr->size[2]));
    break;
  }
  case 1: { //sphere
	boost::shared_ptr<const shapes::Sphere> sphereptr = boost::static_pointer_cast<const shapes::Sphere>(geom);
    subshape = new btSphereShape(sphereptr->radius);
    break;
  }
  case 2: //cylinder
    // cylinder axis aligned to Y
  {
	boost::shared_ptr<const shapes::Cylinder> cylptr = boost::static_pointer_cast<const shapes::Cylinder>(geom);
    double r = cylptr->radius;
    double h = cylptr->length/2;
    subshape = new btCylinderShapeZ(btVector3(r, r, h));
    break;
  }

  case 6: {

	boost::shared_ptr<const shapes::Mesh> meshptr = boost::static_pointer_cast<const shapes::Mesh>(geom);

//	const OpenRAVE::TriMesh &mesh = geom->GetCollisionMesh();
//  assert(mesh.indices.size() >= 3);

	boost::shared_ptr<btTriangleMesh> ptrimesh(new btTriangleMesh());

//    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
//      ptrimesh->addTriangle(toBt(mesh.vertices[mesh.indices[i]]), toBt(mesh.vertices[mesh.indices[i + 1]]),
//              toBt(mesh.vertices[mesh.indices[i + 2]]));
//    }

	float max_x = -1000, min_x = 1000;
	float max_y = -1000, min_y = 1000;
	float max_z = -1000, min_z = 1000;

//	meshptr->print();

//	LOG_INFO("vertices:");
//	for (int i = 0; i < 15; i++){
//		LOG_INFO("meshptr vertex %u: %f", i, meshptr->vertices[i]);
//	}

    for (int i = 0; i < meshptr->triangle_count; i += 1) {

//    	int ind0,ind1,ind2;
//
    	int ind0 = meshptr->triangles[3*i];
    	int ind1 = meshptr->triangles[3*i+1];
    	int ind2 = meshptr->triangles[3*i+2];

		btVector3 vert0(meshptr->vertices[3*ind0+0],meshptr->vertices[3*ind0+1],meshptr->vertices[3*ind0+2]);
		btVector3 vert1(meshptr->vertices[3*ind1+0],meshptr->vertices[3*ind1+1],meshptr->vertices[3*ind1+2]);
		btVector3 vert2(meshptr->vertices[3*ind2+0],meshptr->vertices[3*ind2+1],meshptr->vertices[3*ind2+2]);

//    	btVector3 tri1(meshptr->vertices[meshptr->triangles[3*i]],meshptr->vertices[meshptr->triangles[3*i]+1],meshptr->vertices[meshptr->triangles[3*i]+2]);
//    	btVector3 tri2(meshptr->vertices[meshptr->triangles[3*i+1]],meshptr->vertices[meshptr->triangles[3*i+1]+1],meshptr->vertices[meshptr->triangles[3*i+1]+2]);
//    	btVector3 tri3(meshptr->vertices[meshptr->triangles[3*i+2]],meshptr->vertices[meshptr->triangles[3*i+2]+1],meshptr->vertices[meshptr->triangles[3*i+2]+2]);

    	max_x = max(max_x, max(vert0.x(), max(vert1.x(),vert2.x())));
    	max_y = max(max_y, max(vert0.y(), max(vert1.y(),vert2.y())));
    	max_z = max(max_z, max(vert0.z(), max(vert1.z(),vert2.z())));
//
    	min_x = min(min_x, min(vert0.x(), min(vert1.x(),vert2.x())));
    	min_y = min(min_y, min(vert0.y(), min(vert1.y(),vert2.y())));
    	min_z = min(min_z, min(vert0.z(), min(vert1.z(),vert2.z())));


//
//    	if(true){
//    		LOG_INFO("large z?! i = %u",i);
//    		LOG_INFO("indices: %u, %u, %u", ind0, ind1, ind2);
//    		LOG_INFO("xyz index1: %f, %f, %f", vert0.x(), vert0.y(), vert0.z());
//    		LOG_INFO("xyz index1: %f, %f, %f", meshptr->vertices[3*ind0+0], meshptr->vertices[3*ind0+1], meshptr->vertices[3*ind0+2]);
//    	}


    	ptrimesh->addTriangle(vert0,vert1,vert2);
    }

//	LOG_INFO("min: %f,%f,%f",min_x,min_y,min_z);
//	LOG_INFO("max: %f,%f,%f",max_x,max_y,max_z);


    if (useTrimesh) {
      subshape = new btBvhTriangleMeshShape(ptrimesh.get(), true);
      cow->manage(ptrimesh);
    }
    else { // CONVEX HULL
      btConvexTriangleMeshShape convexTrimesh(ptrimesh.get());
      convexTrimesh.setMargin(MARGIN); // margin: hull padding
      //Create a hull shape to approximate Trimesh

      bool useShapeHull;

      btShapeHull shapeHull(&convexTrimesh);
      if (meshptr->vertex_count >= 50) {
        bool success = shapeHull.buildHull(-666); // note: margin argument not used
        if (!success) LOG_WARN("shapehull convex hull failed! falling back to original vertices");
        useShapeHull = success;
      }
      else {
        useShapeHull = false;
      }

      btConvexHullShape *convexShape = new btConvexHullShape();
      subshape = convexShape;
      if (useShapeHull) {
        for (int i = 0; i < shapeHull.numVertices(); ++i)
          convexShape->addPoint(shapeHull.getVertexPointer()[i]);
        break;
      }
      else {
        for (int i = 0; i < meshptr->vertex_count; ++i)
          convexShape->addPoint(btVector3(meshptr->vertices[3*i],meshptr->vertices[3*i+1],meshptr->vertices[3*i+2]));
        break;
      }
    }     
  }
  default:
    assert(0 && "unrecognized collision shape type");
    break;
  }
  return subshape;
}


COW* CollisionObjectFromBody(MoveitKinBodyPtr body, bool useTrimesh) {
  LOG_DEBUG("creating bt collision object from from %s",body->name.c_str());

  COW* cow = new CollisionObjectWrapper(body);

  btCollisionShape* shape = createShapePrimitive(body->shape, useTrimesh, cow);
  shape->setMargin(MARGIN);

  cow->manage(shape);
  cow->setCollisionShape(shape);
  cow->setWorldTransform(toBt(body->tf));

  return cow;
}


/*

btCollisionObject* CollisionObjectFromBody2(MoveitKinBodyPtr body, bool useTrimesh) {
  LOG_INFO("creating bt collision object from from %s",body->name.c_str());

  COW* cow = new CollisionObjectWrapper(body.get());

  btCollisionObject* btcol;

  btCollisionShape* shape = createShapePrimitive(body->shape, useTrimesh, cow);
  shape->setMargin(MARGIN);
//  cow->manage(shape);

  btcol->setCollisionShape(shape);
  btcol->setWorldTransform(toBt(body->tf));
//  btcol->setUserPointer(body.get());

  return btcol;
}

*/




class BulletCollisionChecker : public CollisionChecker {
  btCollisionWorld* m_world;
  btBroadphaseInterface* m_broadphase;
  btCollisionDispatcher* m_dispatcher;
  btCollisionConfiguration* m_coll_config;

//  typedef map<const OR::KinBody::Link*, CollisionObjectWrapper*> Link2Cow;

  //typedef map<const MoveitKinBody*, CollisionObjectWrapper*> Link2Cow;

  typedef map<const std::string, CollisionObjectWrapper*> Link2Cow;
  Link2Cow m_link2cow;

  double m_contactDistance;
  vector<MoveitKinBodyPtr> m_prevbodies;

  typedef std::pair<const MoveitKinBody*, const MoveitKinBody*> LinkPair;

  set< LinkPair > m_excludedPairs;

  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> m_allowedCollisionMatrix;
  collision_detection::AllowedCollisionMatrix m_acm;

public:
  BulletCollisionChecker(planning_scene::PlanningScenePtr env);
  ~BulletCollisionChecker();

  ///////// public interface /////////
  virtual void SetContactDistance(float distance);
  virtual double GetContactDistance() {return m_contactDistance;}
//  virtual void PlotCollisionGeometry(vector<OpenRAVE::GraphHandlePtr>& handles);

  virtual void ExcludeCollisionPair(const MoveitKinBody& link0, const MoveitKinBody& link1) {
    m_excludedPairs.insert(LinkPair(&link0, &link1));
    COW *cow0 = GetCow(link0.name), *cow1 = GetCow(link1.name);
    if (cow0 && cow1) m_allowedCollisionMatrix(cow0->m_index, cow1->m_index) = 0;
  }
  virtual void IncludeCollisionPair(const MoveitKinBody& link0, const MoveitKinBody& link1) {
    m_excludedPairs.erase(LinkPair(&link0, &link1));
    COW *cow0 = GetCow(link0.name), *cow1 = GetCow(link1.name);
    if (cow0 && cow1) m_allowedCollisionMatrix(cow0->m_index, cow1->m_index) = 1;
  }

  // collision checking
//  virtual void AllVsAll(vector<Collision>& collisions);

  virtual void LinksVsAll(const vector<string>& links, vector<Collision>& collisions, short filterMask);
  virtual void LinkVsAll(const string& link, vector<Collision>& collisions, short filterMask);

//  virtual void ContinuousCheckTrajectory(const TrajArray& traj, Configuration& rad, vector<Collision>&);
  virtual void CastVsAll(const vector<string>& links, const DblVec& startjoints, const DblVec& endjoints, vector<Collision>& collisions);
  ////
  ///////

  CollisionObjectWrapper* GetCow(std::string link) {
    Link2Cow::iterator it = m_link2cow.find(link);
    return (it == m_link2cow.end()) ? 0 : it->second;
  }


  void SetCow(std::string link, COW* cow) {m_link2cow[link] = cow;}

  void LinkVsAll_NoUpdate(const string& link, vector<Collision>& collisions, short filterMask);

  void UpdateBulletFromMoveit();

  void getBodies(vector<MoveitKinBodyPtr>& bodies);

  void AddKinBody(const MoveitKinBodyPtr& body);
  void RemoveKinBody(const MoveitKinBodyPtr& body);
  void AddAndRemoveBodies(const vector<MoveitKinBodyPtr>& curVec, const vector<MoveitKinBodyPtr>& prevVec, vector<MoveitKinBodyPtr>& addedBodies);
  bool CanCollide(const CollisionObjectWrapper* cow0, const CollisionObjectWrapper* cow1) {


	collision_detection::AllowedCollision::Type type;

//	return true;

	bool retval = !m_acm.getAllowedCollision(cow0->m_body->name,cow1->m_body->name, type);

//	LOG_INFO("cancollide for: %s and %s: %d", cow0->m_body->name.c_str(), cow1->m_body->name.c_str(), retval);

	return retval;
//    return m_allowedCollisionMatrix(cow0->m_index, cow1->m_index);
  }
  void SetLinkIndices();
  void UpdateAllowedCollisionMatrix();
  void CheckShapeCast(btCollisionShape* shape, const btTransform& tf0, const btTransform& tf1,
      CollisionObjectWrapper* cow, btCollisionWorld* world, vector<Collision>& collisions);
};


struct CollisionCollector : public btCollisionWorld::ContactResultCallback {
  std::vector<Collision>& 			m_collisions;
  const CollisionObjectWrapper* 	m_cow;
  BulletCollisionChecker* 			m_cc;

  CollisionCollector(vector<Collision>& collisions, CollisionObjectWrapper* cow, BulletCollisionChecker* cc) :
    m_collisions(collisions), m_cow(cow), m_cc(cc) {}

  virtual btScalar addSingleResult(btManifoldPoint& cp,
      const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,
      const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1) {

    if (cp.m_distance1 > m_cc->GetContactDistance()) return 0;

    MoveitKinBodyPtr linkA = getLink(colObj0Wrap->getCollisionObject());
    MoveitKinBodyPtr linkB = getLink(colObj1Wrap->getCollisionObject());

    m_collisions.push_back(Collision(linkA, linkB, toEigen(cp.m_positionWorldOnA), toEigen(cp.m_positionWorldOnB),
        toEigen(cp.m_normalWorldOnB), cp.m_distance1));

    LOG_DEBUG("CollisionCollector: adding collision %s-%s (%.4f)", linkA->name.c_str(), linkB->name.c_str(), cp.m_distance1);
    return 1;
  }

  bool needsCollision(btBroadphaseProxy* proxy0) const {
    return (proxy0->m_collisionFilterGroup & m_collisionFilterMask)
        && (m_collisionFilterGroup & proxy0->m_collisionFilterMask)
        && m_cc->CanCollide(m_cow, static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject));
  }
};


// only used for AllVsAll
void nearCallback(btBroadphasePair& collisionPair,
    btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo) {
  BulletCollisionChecker* cc = static_cast<BulletCollisionChecker*>(dispatcher.m_userData);
  if ( cc->CanCollide(static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy0->m_clientObject),
                      static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy1->m_clientObject)))
    dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
}


BulletCollisionChecker::BulletCollisionChecker(planning_scene::PlanningScenePtr env) :
  CollisionChecker(env) {
  m_coll_config = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_coll_config);
  m_broadphase = new btDbvtBroadphase();
  m_world = new btCollisionWorld(m_dispatcher, m_broadphase, m_coll_config);
  m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,
      m_coll_config->getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));
  m_dispatcher->setNearCallback(&nearCallback);
  m_dispatcher->m_userData = this;
  SetContactDistance(.05);

  vector<double> dofs;
  m_env->getCurrentState().copyJointGroupPositions("manipulator",dofs);
  m_dofvals = dofs;
  m_acm = m_env->getAllowedCollisionMatrixNonConst();

  LOG_INFO("starting update now");
  UpdateBulletFromMoveit();
}


BulletCollisionChecker::~BulletCollisionChecker() {
  delete m_world;
  delete m_broadphase;
  delete m_dispatcher;
  delete m_coll_config;

  for (Link2Cow::iterator it = m_link2cow.begin(); it != m_link2cow.end(); it++){
	  delete it->second;
  }
}


void BulletCollisionChecker::SetContactDistance(float dist) {
  LOG_DEBUG("setting contact distance to %.2f", dist);
  m_contactDistance = dist;
  SHAPE_EXPANSION = btVector3(1,1,1)*dist;
  gContactBreakingThreshold = 2.001*dist; // wtf. when I set it to 2.0 there are no contacts with distance > 0
  btCollisionObjectArray& objs = m_world->getCollisionObjectArray();
  for (int i=0; i < objs.size(); ++i) {
    objs[i]->setContactProcessingThreshold(dist);
  }
  btCollisionDispatcher* dispatcher = static_cast<btCollisionDispatcher*>(m_world->getDispatcher());
  dispatcher->setDispatcherFlags(dispatcher->getDispatcherFlags() & ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);
}

void BulletCollisionChecker::LinksVsAll(const vector<string>& links, vector<Collision>& collisions, short filterMask) {
//  AllVsAll(collisions);
//  return;

//  LOG_INFO("before link2cow (%zu):",m_link2cow.size());

//  for (Link2Cow::iterator it = m_link2cow.begin(); it != m_link2cow.end(); it++){
//	  LOG_INFO("cow for: %s",it->first.c_str());
//	  LOG_INFO("cow addr: %p", (void *)it->second);
//	  LOG_INFO("cow body name: %s",it->second->m_body->name.c_str());
//  }

  UpdateBulletFromMoveit();

//  LOG_INFO("after  link2cow summary (%zu):",m_link2cow.size());
//  for (Link2Cow::iterator it = m_link2cow.begin(); it != m_link2cow.end(); it++){
//	  LOG_INFO("cow for: %s",it->first.c_str());
//	  LOG_INFO("cow body name: %s",it->second->m_body->name.c_str());
//  }

  m_world->updateAabbs();
  
//  LOG_INFO("just updated m_world Aabbs");

  for (int i=0; i < links.size(); ++i) {
    LinkVsAll_NoUpdate(links[i], collisions, filterMask);
  }
}


void BulletCollisionChecker::LinkVsAll(const string& link, vector<Collision>& collisions, short filterMask) {
  UpdateBulletFromMoveit();
  LinkVsAll_NoUpdate(link, collisions, filterMask);
}

void BulletCollisionChecker::LinkVsAll_NoUpdate(const string& link, vector<Collision>& collisions, short filterMask) {

  CollisionObjectWrapper* cow = GetCow(link);
//  LOG_INFO("doing testing now...: %s",link.c_str());
//  LOG_INFO("DOFS: %f, %f, %f, %f, %f, %f", m_dofvals[0],m_dofvals[1],m_dofvals[2],m_dofvals[3],m_dofvals[4],m_dofvals[5]);
//
//  LOG_INFO("link2cow summary (%zu):",m_link2cow.size());
//  for (Link2Cow::iterator it = m_link2cow.begin(); it != m_link2cow.end(); it++){
//	  LOG_INFO("cow for: %s",it->first.c_str());
//	  LOG_INFO("cow addr: %p", (void *)it->second);
//	  LOG_INFO("cow body name: %s",it->second->m_body->name.c_str());
//  }

  CollisionCollector cc(collisions, cow, this);
  cc.m_collisionFilterMask = filterMask;
  m_world->contactTest(cow, cc);

//  LOG_INFO("AfterTESTlink2cow summary (%zu):",m_link2cow.size());
//  for (Link2Cow::iterator it = m_link2cow.begin(); it != m_link2cow.end(); it++){
//	  LOG_INFO("cow for: %s",it->first.c_str());
//	  LOG_INFO("cow addr: %p", (void *)it->second);
//	  LOG_INFO("cow body name: %s",it->second->m_body->name.c_str());
//  }
}



/*
struct KinBodyCollisionData;
typedef boost::shared_ptr<KinBodyCollisionData> CDPtr;
struct KinBodyCollisionData : public OpenRAVE::UserData {
  OpenRAVE::KinBodyWeakPtr body;
  std::vector<KinBody::Link*> links;
  std::vector<COWPtr> cows;
  KinBodyCollisionData(OR::KinBodyPtr _body) : body(_body) {}
};
*/


void BulletCollisionChecker::AddKinBody(const MoveitKinBodyPtr& body) {

//  CDPtr cd(new KinBodyCollisionData(body));

  bool useTrimesh = false; //making this false enables convex hull for nonconvex meshes.

  COW* new_cow = CollisionObjectFromBody(body, useTrimesh);

//  btCollisionObject* new_cow = CollisionObjectFromBody2(body, useTrimesh);

  if (new_cow){
      SetCow(body->name, new_cow);
      m_world->addCollisionObject(new_cow, body->filtermask);

      new_cow->setContactProcessingThreshold(m_contactDistance);
      LOG_DEBUG("added collision object for link %s", body->name.c_str());
//      cd->links.push_back(link.get());  //this is for logging/plotting?
//      cd->cows.push_back(new_cow);
  } else {
	  LOG_WARN("ignoring link %s", body->name.c_str());
  }
}


void BulletCollisionChecker::RemoveKinBody(const MoveitKinBodyPtr& body) {
//  LOG_INFO("removing %s", body->name.c_str());

  CollisionObjectWrapper* cow = GetCow(body->name);
  if (cow) {
    m_world->removeCollisionObject(cow);
    delete cow; //deleting the pointer.
    m_link2cow.erase(body->name);
  }
}

//template <typename T>
//void SetDifferences(const vector<T>& A, const vector<T>& B, vector<T>& AMinusB, vector<T>& BMinusA) {
//  set<T> Aset, Bset;
//  AMinusB.clear();
//  BMinusA.clear();
//  BOOST_FOREACH(const T& a, A) {
//    Aset.insert(a);
//  }
//  BOOST_FOREACH(const T& b, B) {
//    Bset.insert(b);
//  }
//  BOOST_FOREACH(const T& a, A) {
//    if (Bset.count(a) == 0) AMinusB.push_back(a);
//  }
//  BOOST_FOREACH(const T& b, B) {
//    if (Aset.count(b) == 0) BMinusA.push_back(b);
//  }
//}

void SetDifferences(const vector<MoveitKinBodyPtr>& A, const vector<MoveitKinBodyPtr>& B, vector<MoveitKinBodyPtr>& AMinusB, vector<MoveitKinBodyPtr>& BMinusA) {
  vector<string> Aset, Bset;
  AMinusB.clear();
  BMinusA.clear();

  BOOST_FOREACH(const MoveitKinBodyPtr& a, A) {
    Aset.push_back(a->name);
  }
  BOOST_FOREACH(const MoveitKinBodyPtr& b, B) {
    Bset.push_back(b->name);
  }
  BOOST_FOREACH(const MoveitKinBodyPtr& a, A) {
    if (find(Bset.begin(), Bset.end(), a->name) == Bset.end()){
    	AMinusB.push_back(a);
    }
  }
  BOOST_FOREACH(const MoveitKinBodyPtr& b, B) {
    if (find(Aset.begin(), Aset.end(), b->name) == Aset.end()){
    	BMinusA.push_back(b);
    }
  }
}



void BulletCollisionChecker::AddAndRemoveBodies(const vector<MoveitKinBodyPtr>& curVec, const vector<MoveitKinBodyPtr>& prevVec, vector<MoveitKinBodyPtr>& toAdd) {
  vector<MoveitKinBodyPtr> toRemove;

  SetDifferences(curVec, prevVec, toAdd, toRemove);

//  LOG_INFO("add size %zu",toAdd.size());
//  LOG_INFO("remove size %zu",toRemove.size());

  BOOST_FOREACH(const MoveitKinBodyPtr& body, toRemove) {
    RemoveKinBody(body);
  }

  BOOST_FOREACH(const MoveitKinBodyPtr& body, toAdd) {
    AddKinBody(body);
  }

  SetLinkIndices();
}

void BulletCollisionChecker::SetLinkIndices() {
  btCollisionObjectArray& objs = m_world->getCollisionObjectArray();

  for (int i=0; i < objs.size(); ++i) {
	btCollisionObject* btobj = objs[i];
    CollisionObjectWrapper* cow = static_cast<CollisionObjectWrapper*>(objs[i]);
    cow->m_index = i;
  }
  m_allowedCollisionMatrix.resize(objs.size(), objs.size());
  m_allowedCollisionMatrix.setOnes();
}



void BulletCollisionChecker::UpdateAllowedCollisionMatrix() {
  BOOST_FOREACH(const LinkPair& pair, m_excludedPairs) {
    const MoveitKinBody* linkA = pair.first;
    const MoveitKinBody* linkB = pair.second;
    const CollisionObjectWrapper* cowA = GetCow(linkA->name);
    const CollisionObjectWrapper* cowB = GetCow(linkB->name);
    if (cowA != NULL && cowB != NULL) {
      m_allowedCollisionMatrix(cowA->m_index, cowB->m_index) = 0;
      m_allowedCollisionMatrix(cowB->m_index, cowA->m_index) = 0;
    }
  }
}

void BulletCollisionChecker::getBodies(vector<MoveitKinBodyPtr>& bodies){
	// add bodies

	//collision bodies:
	vector<string> obj_ids = m_env->getWorld()->getObjectIds();

	for (size_t i = 0; i<obj_ids.size();i++){
		collision_detection::World::ObjectConstPtr obj = m_env->getWorld()->getObject(obj_ids[i]);
		if (obj->shapes_.size()>0){
			MoveitKinBodyPtr mkb(new MoveitKinBody(obj->shape_poses_[0],obj->shapes_[0], obj->id_, 2));
			bodies.push_back(mkb);
		}
	}
	//robot links:

	const vector<string> all = m_env->getRobotModel()->getLinkModelNames();

	robot_state::RobotState rs = m_env->getCurrentStateNonConst();
	rs.setJointGroupPositions("manipulator",m_dofvals);

	int filtermask = 2;
	for (size_t i; i< all.size(); i++){

		if (rs.getJointModelGroup("manipulator")->hasLinkModel(all[i])){
			filtermask = 1;
		}

		const moveit::core::LinkModel* lm = rs.getLinkModel(all[i]);
		if (lm->getShapes().size()>0){

			Eigen::Affine3d lmtf = m_env->getFrameTransform(all[i]);
			shapes::ShapeConstPtr shapeptr = lm->getShapes()[0];

			MoveitKinBodyPtr mkb(new MoveitKinBody(lmtf,shapeptr,all[i], filtermask));
			bodies.push_back(mkb);
		}
	}
}


void BulletCollisionChecker::UpdateBulletFromMoveit() {
	vector<MoveitKinBodyPtr> bodies, addedBodies, toAdd, toRemove;

	getBodies(bodies);

	SetDifferences(bodies, m_prevbodies, toAdd, toRemove);

//	LOG_INFO("add size %zu",toAdd.size());
//	LOG_INFO("remove size %zu",toRemove.size());

	BOOST_FOREACH(const MoveitKinBodyPtr& body, toRemove) {
	  RemoveKinBody(body);
	}

	BOOST_FOREACH(const MoveitKinBodyPtr& body, toAdd) {
	  AddKinBody(body);
	}

	m_prevbodies = bodies;

	SetLinkIndices();
	m_world->updateAabbs();

    btCollisionObjectArray& objs = m_world->getCollisionObjectArray();

	for (int i=0; i < objs.size(); ++i) {

	  btCollisionObject* test = objs[i];
	  CollisionObjectWrapper* cow = static_cast<CollisionObjectWrapper*>(objs[i]);
	  cow->setWorldTransform(toBt(cow->m_body->tf));

//	  LOG_INFO("cow addr: %p", cow);
//	  LOG_INFO("%s",cow->m_body->name.c_str());

	  btVector3 aabbMin,aabbMax,origin;
	  cow->getCollisionShape()->getAabb(cow->getWorldTransform(),aabbMin,aabbMax);
	  origin = cow->getWorldTransform().getOrigin();

//	  LOG_INFO("tf: %f,%f,%f",origin.x(),origin.y(),origin.z());
//	  LOG_INFO("min: %f,%f,%f",aabbMin.x(),aabbMin.y(),aabbMin.z());
//	  LOG_INFO("max: %f,%f,%f",aabbMax.x(),aabbMax.y(),aabbMax.z());

	}

//	LOG_INFO("link2cowsize %zu",m_link2cow.size());

//	LOG_INFO("Finished Update from Moveit");
}



////////// Continuous collisions ////////////////////////


namespace {

vector<btTransform> rightMultiplyAll(const vector<btTransform>& xs, const btTransform& y) {
  vector<btTransform> out(xs.size());
  for (int i=0; i < xs.size(); ++i) out[i] = xs[i]*y;
  return out;
}
}

/*

void ContinuousCheckShape(btCollisionShape* shape, const vector<btTransform>& transforms,
    KinBody::Link* link, btCollisionWorld* world, vector<Collision>& collisions) {
  if (btConvexShape* convex = dynamic_cast<btConvexShape*>(shape)) {
    for (int i=0; i < transforms.size()-1; ++i) {
      btCollisionWorld::ClosestConvexResultCallback ccc(btVector3(NAN, NAN, NAN), btVector3(NAN, NAN, NAN));
      ccc.m_collisionFilterMask = KinBodyFilter;
      world->convexSweepTest(convex, transforms[i], transforms[i+1], ccc, 0);
      if (ccc.hasHit()) {
        collisions.push_back(Collision(link, getLink(ccc.m_hitCollisionObject),
            toEigen(ccc.m_hitPointWorld), toEigen(ccc.m_hitPointWorld), toEigen(ccc.m_hitNormalWorld), 0, 1, i+ccc.m_closestHitFraction));
      }
    }
  }
  else if (btCompoundShape* compound = dynamic_cast<btCompoundShape*>(shape)) {
    for (int i = 0; i < compound->getNumChildShapes(); ++i) {
      ContinuousCheckShape(compound->getChildShape(i), rightMultiplyAll(transforms, compound->getChildTransform(i)),  link, world, collisions);
    }
  }
  else {
    throw std::runtime_error("I can only continuous collision check convex shapes and compound shapes made of convex shapes");
  }

}

*/

/*
void BulletCollisionChecker::ContinuousCheckTrajectory(const TrajArray& traj, Configuration& rad, vector<Collision>& collisions) {
  UpdateBulletFromRave();
  m_world->updateAabbs();

  // first calculate transforms of all the relevant links at each step
  vector<KinBody::LinkPtr> links;
  vector<int> link_inds;
  rad.GetAffectedLinks(links, true, link_inds);


  // don't need to remove them anymore because now I only check collisions
  // against KinBodyFilter stuff
  // remove them, because we can't check moving stuff against each other
  vector<CollisionObjectWrapper*> cows;
  BOOST_FOREACH(KinBody::LinkPtr& link, links) {
    CollisionObjectWrapper* cow = GetCow(link.get());
    assert(cow != NULL);
    cows.push_back(cow);
#if 0
    m_world->removeCollisionObject(cow);
#endif
  }


  typedef vector<btTransform> TransformVec;
  vector<TransformVec> link2transforms(links.size(), TransformVec(traj.rows()));
  Configuration::SaverPtr save = rad.Save();

  for (int iStep=0; iStep < traj.rows(); ++iStep) {
    rad.SetDOFValues(toDblVec(traj.row(iStep)));
    for (int iLink = 0; iLink < links.size(); ++iLink) {
      link2transforms[iLink][iStep] = toBt(links[iLink]->GetTransform());
    }
  }

  for (int iLink = 0; iLink < links.size(); ++iLink) {
    ContinuousCheckShape(cows[iLink]->getCollisionShape(), link2transforms[iLink], links[iLink].get(), m_world, collisions);
  }

#if 0
  // add them back
  BOOST_FOREACH(CollisionObjectWrapper* cow, cows) {
    m_world->addCollisionObject(cow);
  }
#endif
}

#if 0
class CompoundHullShape : public btConvexShape {
  std::vector<btConvexHullShape*> m_children;
  btVector3   localGetSupportingVertex(const btVector3& vec)const {
    btVector3 sv = m_children[0]->localGetSupportingVertex(vec);
    float support = sv.dot(vec);
    for (int i=1; i < m_children.size(); ++i) {
      btVector3 newsv = m_children[i]->localGetSupportingVertex(vec);
      float newsupport = vec.dot(newsv);
      if (newsupport > support) {
        support = newsupport;
        sv = newsv;
      }
    }
  }
#if 0
  void project(const btTransform& trans, const btVector3& dir, btScalar& min, btScalar& max) const {
    m_children[0]->project(trans, dir, min, max);
    for (int i=1; i < m_children.size(); ++i) {
      btScalar newmin, newmax;
      m_children[i]->project(trans, dir, newmin, newmax);
      btSetMin(min, newmin);
      btSetMax(max, newmax);
    }
  }
#endif

  //notice that the vectors should be unit length
  void    batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const {
    throw std::runtime_error("not implemented");
  }

  ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
  void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const {
    m_children[0]->getAabb(t, aabbMin, aabbMax);
    for (int i=1; i < m_children.size(); ++i) {
      btVector3 newmin, newmax;
      m_children[i]->getAabb(t, newmin, newmax);
      aabbMin.setMin(newmin);
      aabbMax.setMax(newmax);
    }
  }

  virtual void getAabbSlow(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const {
    throw std::runtime_error("shouldn't happen");
  }

  virtual void    setLocalScaling(const btVector3& scaling) {}
  virtual const btVector3& getLocalScaling() const {return btVector3(1,1,1);}

  virtual void    setMargin(btScalar margin) {}
  virtual btScalar    getMargin() const {return 0;}

  virtual int     getNumPreferredPenetrationDirections() const {return 0;}
  virtual void    getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const=0;


};
#endif
*/



struct CastHullShape : public btConvexShape {
public:
  btConvexShape* m_shape;
  btTransform m_t01, m_t10; // T_0_1 = T_w_0^-1 * T_w_1
  CastHullShape(btConvexShape* shape, const btTransform& t01) : m_shape(shape), m_t01(t01) {
    m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;

  }
  btVector3   localGetSupportingVertex(const btVector3& vec)const {
    btVector3 sv0 = m_shape->localGetSupportingVertex(vec);
    btVector3 sv1 = m_t01*m_shape->localGetSupportingVertex(vec*m_t01.getBasis());
    return (vec.dot(sv0) > vec.dot(sv1)) ? sv0 : sv1;
  }
#if 0
  void project(const btTransform& trans, const btVector3& dir, btScalar& min, btScalar& max) const {
    m_children[0]->project(trans, dir, min, max);
    for (int i=1; i < m_children.size(); ++i) {
      btScalar newmin, newmax;
      m_children[i]->project(trans, dir, newmin, newmax);
      btSetMin(min, newmin);
      btSetMax(max, newmax);
    }
  }
#endif

  //notice that the vectors should be unit length
  void    batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const {
    throw std::runtime_error("not implemented");
  }

  ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
  void getAabb(const btTransform& t_w0,btVector3& aabbMin,btVector3& aabbMax) const {
    m_shape->getAabb(t_w0, aabbMin, aabbMax);
    btVector3 min1, max1;
    m_shape->getAabb(t_w0*m_t01, min1, max1 );
    aabbMin.setMin(min1);
    aabbMax.setMax(max1);
  }

  virtual void getAabbSlow(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const {
    throw std::runtime_error("shouldn't happen");
  }

  virtual void    setLocalScaling(const btVector3& scaling) {}
  virtual const btVector3& getLocalScaling() const {
    static btVector3 out(1,1,1);
    return out;
  }

  virtual void    setMargin(btScalar margin) {}
  virtual btScalar    getMargin() const {return 0;}

  virtual int     getNumPreferredPenetrationDirections() const {return 0;}
  virtual void    getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const {throw std::runtime_error("not implemented");}


  virtual void calculateLocalInertia(btScalar, btVector3&) const {throw std::runtime_error("not implemented");}
  virtual const char* getName() const {return "CastHull";}
  virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& v) const {return localGetSupportingVertex(v);}

  void calculateContactTime(Collision& col) {    
    // float support0 = localGetSupportingVertex(col.)
  }

};


struct CastCollisionCollector : public CollisionCollector {
  CastCollisionCollector(vector<Collision>& collisions, CollisionObjectWrapper* cow, BulletCollisionChecker* cc) :
    CollisionCollector(collisions, cow, cc) {}  
  virtual btScalar addSingleResult(btManifoldPoint& cp,
      const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,
      const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1);
};


btScalar CastCollisionCollector::addSingleResult(btManifoldPoint& cp,
    const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,
    const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1) {      
      float retval = CollisionCollector::addSingleResult(cp, colObj0Wrap,partId0,index0, colObj1Wrap,partId1,index1); // call base class func
      if (retval == 1) { // if contact was added
        bool castShapeIsFirst =  (colObj0Wrap->getCollisionObject() == m_cow);
        btVector3 normalWorldFromCast = -(castShapeIsFirst ? 1 : -1) * cp.m_normalWorldOnB;
        const CastHullShape* shape = dynamic_cast<const CastHullShape*>((castShapeIsFirst ? colObj0Wrap : colObj1Wrap)->getCollisionObject()->getCollisionShape());
        assert(!!shape);
        btTransform tfWorld0 = m_cow->getWorldTransform();
        btTransform tfWorld1 = m_cow->getWorldTransform() * shape->m_t01;
        btVector3 normalLocal0 = normalWorldFromCast * tfWorld0.getBasis();
        btVector3 normalLocal1 = normalWorldFromCast * tfWorld1.getBasis();

        Collision& col = m_collisions.back();
        const float SUPPORT_FUNC_TOLERANCE = .01 METERS;

//        cout << normalWorldFromCast << endl;

        if (castShapeIsFirst) {
          swap(col.ptA, col.ptB);
          swap(col.linkA, col.linkB);
          col.normalB2A *= -1;
        }

#if 0
        btVector3 ptWorld0 = tfWorld0*shape->m_shape->localGetSupportingVertex(normalLocal0);
        btVector3 ptWorld1 = tfWorld1*shape->m_shape->localGetSupportingVertex(normalLocal1);
#else
        btVector3 ptLocal0;
        float localsup0;
        GetAverageSupport(shape->m_shape, normalLocal0, localsup0, ptLocal0);
        btVector3 ptWorld0 = tfWorld0 * ptLocal0;
        btVector3 ptLocal1;
        float localsup1;
        GetAverageSupport(shape->m_shape, normalLocal1, localsup1, ptLocal1);
        btVector3 ptWorld1 = tfWorld1 * ptLocal1;
#endif
        float sup0 = normalWorldFromCast.dot(ptWorld0);
        float sup1 = normalWorldFromCast.dot(ptWorld1);


        // TODO: this section is potentially problematic. think hard about the math
        if (sup0 - sup1 > SUPPORT_FUNC_TOLERANCE) {
          col.time = 0;
          col.cctype = CCType_Time0;
        }
        else if (sup1 - sup0 > SUPPORT_FUNC_TOLERANCE) {
          col.time = 1;
          col.cctype = CCType_Time1;
        }
        else {
          const btVector3& ptOnCast = castShapeIsFirst ? cp.m_positionWorldOnA : cp.m_positionWorldOnB;
          float l0c = (ptOnCast - ptWorld0).length(), 
                l1c = (ptOnCast - ptWorld1).length();

          col.ptB = toEigen(ptWorld0);
          col.ptB1 = toEigen(ptWorld1);
          col.cctype = CCType_Between;

          const float LENGTH_TOLERANCE = .001 METERS;

          if ( l0c + l1c < LENGTH_TOLERANCE) {

            col.time = .5;
          }
          else {
            col.time = l0c/(l0c + l1c); 
          }

        }
          
      }
      return retval;          
}


void BulletCollisionChecker::CheckShapeCast(btCollisionShape* shape, const btTransform& tf0, const btTransform& tf1,
    CollisionObjectWrapper* cow, btCollisionWorld* world, vector<Collision>& collisions) {

  if (btConvexShape* convex = dynamic_cast<btConvexShape*>(shape)) {

    CastHullShape* shape = new CastHullShape(convex, tf0.inverseTimes(tf1));
    CollisionObjectWrapper* obj = new CollisionObjectWrapper(cow->m_body);
    obj->setCollisionShape(shape);
    obj->setWorldTransform(tf0);
    obj->m_index = cow->m_index;
    CastCollisionCollector cc(collisions, obj, this);
    cc.m_collisionFilterMask = KinBodyFilter;
    // cc.m_collisionFilterGroup = cow->m_collisionFilterGroup;
    world->contactTest(obj, cc);
    delete obj;
    delete shape;
  }
  else if (btCompoundShape* compound = dynamic_cast<btCompoundShape*>(shape)) {
    for (int i = 0; i < compound->getNumChildShapes(); ++i) {
      CheckShapeCast(compound->getChildShape(i), tf0*compound->getChildTransform(i), tf1*compound->getChildTransform(i), cow, world, collisions);
    }
  }
  else {
    throw std::runtime_error("I can only continuous collision check convex shapes and compound shapes made of convex shapes");
  }

}



void BulletCollisionChecker::CastVsAll(const std::vector<std::string>& links,
    const DblVec& startjoints, const DblVec& endjoints, vector<Collision>& collisions) {

	robot_state::RobotState rs = m_env->getCurrentStateNonConst();
	rs.setJointGroupPositions("manipulator",startjoints);

	int nlinks = links.size(); // links are strings now, handle with link2cow thing
	vector<btTransform> tbefore(nlinks), tafter(nlinks);

	for (int i=0; i < nlinks; ++i) {
		tbefore[i] = toBt(rs.getFrameTransform(links[i]));
	}
    rs.setJointGroupPositions("manipulator",endjoints);

	for (int i=0; i < nlinks; ++i) {
		tafter[i] = toBt(rs.getFrameTransform(links[i]));
	}

  UpdateBulletFromMoveit();
  m_world->updateAabbs();

  for (int i=0; i < nlinks; ++i) {
    assert(m_link2cow[links[i]] != NULL);
    CollisionObjectWrapper* cow = m_link2cow[links[i]];
    CheckShapeCast(cow->getCollisionShape(), tbefore[i], tafter[i], cow, m_world, collisions);
  }
  LOG_DEBUG("CastVsAll checked %li links and found %li collisions", links.size(), collisions.size());
}

}


namespace trajopt {

CollisionCheckerPtr CreateCollisionChecker(planning_scene::PlanningScenePtr env) {
  CollisionCheckerPtr checker(new BulletCollisionChecker(env));
  return checker;
}
}
