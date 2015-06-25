#ifndef DRIVER_H
#define DRIVER_H

#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletDynamicsCommon.h"
#include <iostream>
#include <unistd.h>
#include <string>
#include <vector>
#include <mpi.h>
#include "tools.h"
#include "rnn.h"

#ifdef GRAPHICS
#include "GL_ShapeDrawer.h"
#define STEPSIZE 5
#endif

class Driver
{
 public:  
  Driver(condition_t *cond, double* weights, int num_weights, double* output, int num_outputs,
	 std::default_random_engine &gen, int num_args);
  void exitPhysics();
  int run();
  double calc_world(int elem, int id, double param);
  double calc_rigid(int elem, int id, double param);
  double calc_joint(int elem, int id, double param);
  double calc_brain(int elem, int id, double param);
  double calc_random(int elem, int id, double param);
  double calc(int id);
  
  static bool myContactProcessedCallback(btManifoldPoint& cp, void* body0, void* body1);
  
  btDynamicsWorld*                          m_dynamicsWorld;
  btAlignedObjectArray<btCollisionShape*>   m_collisionShapes;
  btBroadphaseInterface*                    m_broadphase;
  btCollisionDispatcher*                    m_dispatcher;
  btConstraintSolver*                       m_solver;
  btDefaultCollisionConfiguration*          m_collisionConfiguration;
  //btAlignedObjectArray<struct touch*>     m_touches;
  btAlignedObjectArray<int *>               m_idents;

  std::default_random_engine m_gen;

  condition_t *m_cond;
  double *m_output, m_sprint_progress, m_eval_progress = 0.0;
  int m_num_outputs, m_step_counter = -1, m_num_args;
  std::vector<btCollisionShape*> m_shapes;
  std::vector<btRigidBody*> m_rbodys;
  std::vector<btTypedConstraint*> m_joints;
  std::vector<RNN *> m_brains;
  std::vector<std::vector<double>> m_stores;
  ~Driver();
  
#ifdef GRAPHICS
  bool      m_paused = 0;
  btClock   m_clock;
  float     m_cameraDistance = 10.0;
  int       m_debugMode = 0;
  float     m_ele = 10.f;
  float     m_azi = 180.f;
  btVector3 m_cameraPosition = btVector3(0.f, 0.f, 0.f);
  btVector3 m_cameraTargetPosition = btVector3(0.f, 0.f, 0.f);
  int       m_mouseOldX = 0;
  int       m_mouseOldY = 0;
  int       m_mouseButtons = 0;
  int       m_modifierKeys = 0;
  float     m_scaleBottom = 0.5;
  float     m_scaleFactor = 2.f;
  btVector3 m_cameraUp = btVector3(0, 1, 0);
  int       m_forwardAxis = 2;
  float     m_zoomStepSize = 0.4;
  int       m_glutScreenWidth = 800;
  int       m_glutScreenHeight = 600;
  float     m_frustumZNear = 1.f;
  float     m_frustumZFar = 10000.f;
  int       m_ortho = 0;
  bool      m_stepping = true;
  bool      m_singleStep = false;
  bool      m_idle = false;
  int       m_lastKey = 0;
  GL_ShapeDrawer* m_shapeDrawer;
  bool      m_enableshadows = false;
  btVector3 m_sundirection = btVector3(1000, -2000, 1000);
  btScalar  m_defaultContactProcessingThreshold = BT_LARGE_FLOAT;
  void setOrthographicProjection();
  void resetPerspectiveProjection();
  bool setTexturing(bool enable) { return(m_shapeDrawer->enableTexture(enable)); }
  bool setShadows(bool enable) { bool p = m_enableshadows; m_enableshadows = enable; return(p); }
  bool getTexturing() const { return m_shapeDrawer->hasTextureEnabled(); }
  bool getShadows() const { return m_enableshadows; }
  int  getDebugMode() { return m_debugMode; }
  void setDebugMode(int mode);
  void draw_line(double x1, double y1, double z1, double x2, double y2, double z2);
  void setAzi(float azi) { m_azi = azi; }
  void setEle(float ele) { m_ele = ele; }
  void setCameraUp(const btVector3& camUp) { m_cameraUp = camUp; }
  void setCameraForwardAxis(int axis) { m_forwardAxis = axis; }
  void toggleIdle() { m_idle = !m_idle; };
  virtual void updateCamera();
  btVector3 getCameraPosition() { return m_cameraPosition; }
  btVector3 getCameraTargetPosition() { return m_cameraTargetPosition; }
  btScalar  getDeltaTimeMicroseconds() { btScalar dt = (btScalar) m_clock.getTimeMicroseconds(); m_clock.reset(); return dt; }
  void setFrustumZPlanes(float zNear, float zFar) { m_frustumZNear = zNear; m_frustumZFar = zFar; }
  float getCameraDistance() { return m_cameraDistance; };
  void setCameraDistance(float dist) {m_cameraDistance  = dist; };
  btVector3 getRayTo(int x,int y);
  btRigidBody* localCreateRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape);
  void keyboardCallback(unsigned char key, int x, int y);
  void mouseCallback(int button, int state, int x, int y);
  void displayCallback();
  void setup_gl(int argc, char** argv);
  void myinit();
  void swapBuffers();
  void updateModifierKeys();
  void stepLeft() { m_azi -= STEPSIZE; if (m_azi < 0) m_azi += 360; updateCamera(); };
  void stepRight() { m_azi += STEPSIZE; if (m_azi >= 360) m_azi -= 360; updateCamera(); };
  void stepFront() { m_ele += STEPSIZE; if (m_ele >= 360) m_ele -= 360; updateCamera(); };
  void stepBack() { m_ele -= STEPSIZE; if (m_ele < 0) m_ele += 360; updateCamera(); };
  void zoomIn() {  m_cameraDistance -= btScalar(m_zoomStepSize); updateCamera(); };
  void zoomOut() { m_cameraDistance += btScalar(m_zoomStepSize); updateCamera(); };
  bool isIdle() const { return m_idle; }
  void setIdle(bool idle) { m_idle = idle; }
  #endif
};


#endif
