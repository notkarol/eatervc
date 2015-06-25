#include "driver.h"

static Driver* driver;

Driver::Driver(condition_t *cond, double* weights, int num_weights, double* output,
	       int num_outputs, std::default_random_engine &gen, int num_args)
{
  driver = this;
  m_cond = cond;
  m_gen = gen;
  m_output = output;
  m_num_outputs = num_outputs;
  m_sprint_progress = weights[0];
  m_num_args = num_args;

  // Initialize World
  gContactProcessedCallback = myContactProcessedCallback;
  m_collisionConfiguration = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_broadphase = new btAxisSweep3 (m_cond->world.aabb_min, m_cond->world.aabb_max);
  m_solver = new btSequentialImpulseConstraintSolver;  
  m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, 
						m_solver, m_collisionConfiguration);

  // Initialize Rigid Bodies
  for (unsigned int i = 0; i < m_cond->rbodys.size(); ++i)
    {
      switch (m_cond->rbodys[i].shape)
	{
	case SHAPE_BOX:
	  m_shapes.push_back(new btBoxShape(m_cond->rbodys[i].dim)); break;
	case SHAPE_CYLINDER:
	  m_shapes.push_back(new btCylinderShape(m_cond->rbodys[i].dim)); break;
	case SHAPE_CAPSULE:
	  m_shapes.push_back(new btCapsuleShape(m_cond->rbodys[i].dim.getY(),
						m_cond->rbodys[i].dim.getX())); break;
	case SHAPE_SPHERE:
	  m_shapes.push_back(new btSphereShape(m_cond->rbodys[i].dim.getX())); break;
	case SHAPE_CONE:
	  m_shapes.push_back(new btConeShape(m_cond->rbodys[i].dim.getX(),
					     m_cond->rbodys[i].dim.getY())); break;
	default: fprintf(stderr, "UNKNOWN SHAPE for rbody %i\n", i); exit(-1);
	}
      
      btTransform offset_trans;
      offset_trans.setIdentity();
      offset_trans.setOrigin(m_cond->rbodys[i].pos);
      offset_trans.getBasis().setEulerZYX(m_cond->rbodys[i].ori.z(),
					  m_cond->rbodys[i].ori.y(),
					  m_cond->rbodys[i].ori.x());
      btDefaultMotionState* myMotionState = new btDefaultMotionState(offset_trans);
      btVector3 localInertia(0,0,0);
      if (m_cond->rbodys[i].mass != 0.f) // isDynamic
	m_shapes.back()->calculateLocalInertia(m_cond->rbodys[i].mass, localInertia);

      btRigidBody::btRigidBodyConstructionInfo rbInfo(m_cond->rbodys[i].mass, myMotionState,
						      m_shapes.back(), localInertia);
      rbInfo.m_additionalDamping = true;
      m_rbodys.push_back(new btRigidBody(rbInfo));
      m_rbodys.back()->setFriction(m_cond->rbodys[i].friction_sliding);
      m_rbodys.back()->setRollingFriction(m_cond->rbodys[i].friction_rolling);
      m_dynamicsWorld->addRigidBody(m_rbodys.back());
    }

  // Connect rigid bodies with constraints
  for (unsigned int i = 0; i < m_cond->joints.size(); ++i)
    {//((btHingeConstraint *) input_objects[i])->getHingeAngle();
      switch (m_cond->joints[i].jtype)
	{
	case CONSTRAINT_HINGE:
	  m_joints.push_back
	    (new btHingeConstraint
	     (*(m_rbodys[m_cond->joints[i].rbody_id_0]),
	      *(m_rbodys[m_cond->joints[i].rbody_id_1]),
	      PointWorldToLocal(m_rbodys[m_cond->joints[i].rbody_id_0], m_cond->joints[i].pos),
	      PointWorldToLocal(m_rbodys[m_cond->joints[i].rbody_id_1], m_cond->joints[i].pos),
	      AxisWorldToLocal(m_rbodys[m_cond->joints[i].rbody_id_0], m_cond->joints[i].ori),
	      AxisWorldToLocal(m_rbodys[m_cond->joints[i].rbody_id_1], m_cond->joints[i].ori)));
	  ((btHingeConstraint *) m_joints.back())->setLimit(m_cond->joints[i].ang_min,
							    m_cond->joints[i].ang_max);
	  break;
	case CONSTRAINT_POINT:
	  m_joints.push_back
	    (new btPoint2PointConstraint
	     (*(m_rbodys[m_cond->joints[i].rbody_id_0]),
	      *(m_rbodys[m_cond->joints[i].rbody_id_1]),
	      PointWorldToLocal(m_rbodys[m_cond->joints[i].rbody_id_0], m_cond->joints[i].pos),
	      PointWorldToLocal(m_rbodys[m_cond->joints[i].rbody_id_1], m_cond->joints[i].pos)));
	  break;
	case CONSTRAINT_GENERIC:
	  break;
	}
      m_dynamicsWorld->addConstraint(m_joints.back(), true);
    }

  // Create brain. Eventauilly scale up
  m_brains.push_back(new RNN(m_cond->brains[0].num_i, m_cond->brains[0].num_h,
			     m_cond->brains[0].num_o, &(weights[1]),
			     m_cond->brains[0].recurrent));
  // Initialize stores
  for (int i = 0; i < m_num_outputs; ++i)
    {
      std::vector<double> vec;
      m_stores.push_back(vec);
    }
}


double Driver::calc_world(int elem, int id, double param)
{
  switch (elem)
    {
    case 0: return m_cond->world.id;
    case 1: return m_cond->world.eval_steps;
    case 2: return m_cond->world.eval_dt;
    default: fprintf(stderr, "UNKNOWN ELEM IN WORLD\n"); return -1.0;
    }
}

double Driver::calc_rigid(int elem, int id, double param)
{
  btScalar ax, ay, az;
  switch (elem)
    {
    case 0: return m_cond->rbodys[id].id;
    case 1: return m_cond->rbodys[id].dim.x();
    case 2: return m_cond->rbodys[id].dim.y();
    case 3: return m_cond->rbodys[id].dim.z();
    case 4: return m_rbodys[id]->getWorldTransform().getOrigin().x();
    case 5: return m_rbodys[id]->getWorldTransform().getOrigin().y();
    case 6: return m_rbodys[id]->getWorldTransform().getOrigin().z();
    case 7: m_rbodys[id]->getWorldTransform().getBasis().getEulerZYX(az, ay, ax); return ax;
    case 8: m_rbodys[id]->getWorldTransform().getBasis().getEulerZYX(az, ay, ax); return ay;
    case 9: m_rbodys[id]->getWorldTransform().getBasis().getEulerZYX(az, ay, ax); return az;
    case 10: return m_cond->rbodys[id].shape;
    case 11: return m_cond->rbodys[id].mass;
    case 12: return m_cond->rbodys[id].friction_sliding;
    case 13: return m_cond->rbodys[id].friction_rolling;
    default: fprintf(stderr, "UNKNOWN ELEM IN RIGID\n"); return -1.0;
    }
}

double Driver::calc_joint(int elem, int id, double param)
{
  switch (elem)
    {
    case 0: 
      return (((btHingeConstraint*) m_joints[id])->getHingeAngle() - m_cond->joints[id].offset) / 
	(std::abs(m_cond->joints[id].ang_min - m_cond->joints[id].ang_max) * 0.5);
    default: fprintf(stderr, "UNKNOWN ELEM IN JOINT\n"); return -1.0;
    }
}

double Driver::calc_brain(int elem, int id, double param)
{
  int neuron = (int) (param + 0.5);
  switch (elem)
    {
    case 1: return m_brains[id]->node_i[neuron];
    case 2: return m_brains[id]->node_h[neuron];
    case 3: return m_brains[id]->node_o[neuron];
    case 4: return std::abs(m_brains[id]->node_o[neuron] - m_cond->goal);
    default: fprintf(stderr, "UNKNOWN ELEM IN BRAINT\n"); return -1.0;
    }
}

double Driver::calc_random(int elem, int id, double param)
{
  return next_uniform(-1, 1, m_gen);
}

  
double Driver::calc(int action_id)
{
  action_t *a = &m_cond->actions[action_id];
  
  if (a->operation == 0) return a->parameter;
  else if (a->operation == 1)
    {
      if (a->src_type_0 == FIXED)
	return a->parameter;
      else if (a->src_type_0 == WORLD) return calc_world(a->src_elem_0, a->src_id_0, a->parameter);
      else if (a->src_type_0 == RIGID) return calc_rigid(a->src_elem_0, a->src_id_0, a->parameter);
      else if (a->src_type_0 == JOINT) return calc_joint(a->src_elem_0, a->src_id_0, a->parameter);
      else if (a->src_type_0 == BRAIN) return calc_brain(a->src_elem_0, a->src_id_0, a->parameter);
      else if (a->src_type_0 == RANDOM)return calc_random(a->src_elem_0, a->src_id_0, a->parameter);
      else if (a->src_type_0 == SUSSEX)return calc_sussex(m_cond->sussexs[a->src_id_0], m_dynamicsWorld);
    }
  else if (a->operation > 1) // weighted average of two sensors
    {
      double v0 = 0.0, v1 = 0.0;
      if      (a->src_type_0 == FIXED)  v0 = a->src_id_0 == 0 ? a->src_elem_0 : ((double) a->src_elem_0) / a->src_id_0;
      else if (a->src_type_0 == WORLD)  v0 = calc_world(a->src_elem_0, a->src_id_0, a->parameter);
      else if (a->src_type_0 == RIGID)  v0 = calc_rigid(a->src_elem_0, a->src_id_0, a->parameter);
      else if (a->src_type_0 == JOINT)  v0 = calc_joint(a->src_elem_0, a->src_id_0, a->parameter);
      else if (a->src_type_0 == BRAIN)  v0 = calc_brain(a->src_elem_0, a->src_id_0, a->parameter);
      else if (a->src_type_0 == RANDOM) v0 = calc_random(a->src_elem_0, a->src_id_0, a->parameter);
      else if (a->src_type_0 == SUSSEX) v0 = calc_sussex(m_cond->sussexs[a->src_id_0], m_dynamicsWorld);
      if      (a->src_type_1 == FIXED)  v1 = a->src_id_1 == 0 ? a->src_elem_1 : ((double) a->src_elem_1) / a->src_id_1;
      else if (a->src_type_1 == WORLD)  v1 = calc_world(a->src_elem_1, a->src_id_1, a->parameter);
      else if (a->src_type_1 == RIGID)  v1 = calc_rigid(a->src_elem_1, a->src_id_1, a->parameter);
      else if (a->src_type_1 == JOINT)  v1 = calc_joint(a->src_elem_1, a->src_id_1, a->parameter);
      else if (a->src_type_1 == BRAIN)  v1 = calc_brain(a->src_elem_1, a->src_id_1, a->parameter);
      else if (a->src_type_1 == RANDOM) v1 = calc_random(a->src_elem_1, a->src_id_1, a->parameter);
      else if (a->src_type_1 == SUSSEX) v1 = calc_sussex(m_cond->sussexs[a->src_id_1], m_dynamicsWorld);
      if      (a->operation == 2)
	return a->parameter * v0 + (1 - a->parameter) * v1;
      else if (a->operation == 3)
	{
	  // Progress is the increasing intensity of use of the second varibable
	  double progress = (m_sprint_progress - a->sprint_beg) / (a->sprint_end - a->sprint_beg);
	  return progress * v1 + (1 - progress) * v0;
	}
      else if (a->operation == 4)
	{
	  double progress = (m_eval_progress - a->eval_beg) / (a->eval_end - a->eval_beg);
	  return progress * v1 + (1 - progress) * v0;
	}
      else if (a->operation == 5)
	{
	  double sprint_p = (m_sprint_progress - a->sprint_beg) / (a->sprint_end - a->sprint_beg);
	  double eval_p = (m_eval_progress - a->eval_beg) / (a->eval_end - a->eval_beg);
	  return eval_p < (1.0 - sprint_p) ? v0 : v1;
	}
      else if (a->operation == 6)
	{
	  double sprint_p = (m_sprint_progress - a->sprint_beg) / (a->sprint_end - a->sprint_beg);
	  double eval_p = (m_eval_progress - a->eval_beg) / (a->eval_end - a->eval_beg);
	  double v1_frac = erf((eval_p + sprint_p * 2 - 1) * 4 - 2) * 0.5 + 0.5;
	  return v0 * (1.0 - v1_frac) + v1 * v1_frac;
	}
    }
  return 0.0;
}

int Driver::run()
{
  m_eval_progress = ((double) ++m_step_counter) / m_cond->world.eval_steps;
  //m_touches.resize(0);

  int pos;
  for (unsigned int i = 0; i < m_cond->actions.size(); ++i)
    {
      // Only perform the actions we're in the time range of
      if (m_cond->actions[i].eval_beg > m_eval_progress ||
	  m_cond->actions[i].eval_end < m_eval_progress || 
	  m_cond->actions[i].sprint_beg > m_sprint_progress ||
	  m_cond->actions[i].sprint_end < m_sprint_progress)
	continue;
      pos = m_cond->actions[i].position;
      if (m_cond->actions[i].atype == ACTION_UPDATE_NEURON) // need to do pos mod num_i
	{
	  m_brains[0]->node_i[pos] = calc(i);
	}
      else if (m_cond->actions[i].atype == ACTION_UPDATE_BRAIN)
	{
#ifdef GRAPHICS
	  //m_brains[pos]->print();
#endif
	  m_brains[pos]->update();
	}
      else if (m_cond->actions[i].atype == ACTION_MOTOR)
	{
	  ActuateJointAngular((btHingeConstraint *) m_joints[pos], calc(i)
			      * 90 + m_cond->joints[pos].offset * 180 / M_PI,
			      m_cond->world.eval_dt, m_cond->joints[pos].force);
	}
      else if (m_cond->actions[i].atype == ACTION_STORE)
	{
	  m_stores[pos].push_back(calc(i));
	}
      else fprintf(stderr, "UNKNOWN ACTION TYPE: %i\n", m_cond->actions[i].atype);
    }
  // If arguments are given then we're doing a diagnostic run
  if (m_num_args > 1)
    {
      printf("%.6f %.6f", 
	     m_rbodys[m_rbodys.size() - 1]->getWorldTransform().getOrigin().x(),
	     m_rbodys[m_rbodys.size() - 1]->getWorldTransform().getOrigin().z());
      m_brains[0]->print();
    }

  // Exit if we're done
  if (m_step_counter >= m_cond->world.eval_steps)
    {
      //m_brains[0]->print();
      unsigned int i, j;
      double sum;
      for (i = 0; i < m_stores.size(); ++i)
	{
	  sum = 0;
	  for (j = 0; j < m_stores[i].size(); ++j)
	    sum += m_stores[i][j];
	  m_output[i] = sum / m_stores[i].size(); // average
	}
      return 1;
    }
  m_dynamicsWorld->stepSimulation(m_cond->world.eval_dt);
  return 0;
}

bool Driver::myContactProcessedCallback(btManifoldPoint& cp, void* body0, void* body1)
{
  /*
  struct touch *t = (struct touch*) malloc(sizeof(struct touch));
  t->obj1 = *static_cast<int*>(static_cast<btCollisionObject*>(body0)->getUserPointer());
  t->obj2 = *static_cast<int*>(static_cast<btCollisionObject*>(body1)->getUserPointer());
  t->point = cp.m_positionWorldOnB;
  driver->m_touches.push_back(t);
  */
  return false;
}

Driver::~Driver()
{
  for (int i = m_dynamicsWorld->getNumCollisionObjects() - 1;  i>= 0; i--)
    {
      btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
      btRigidBody* body = btRigidBody::upcast(obj);
      if (body && body->getMotionState())
	delete body->getMotionState();
      m_dynamicsWorld->removeCollisionObject(obj);
      delete obj;
    }
  for (int j = 0; j < m_collisionShapes.size(); j++)
    delete m_collisionShapes[j];
  
  //  for (int j = 0; j < m_touches.size(); j++) delete m_touches[j];
  for (int j = m_idents.size() - 1; j >= 0; --j) delete m_idents[j];
  if (m_dynamicsWorld) delete m_dynamicsWorld;
  if (m_solver) delete m_solver;
  if (m_broadphase) delete m_broadphase;
  if (m_dispatcher) delete m_dispatcher;
  if (m_collisionConfiguration) delete m_collisionConfiguration;
}  


///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
#ifdef GRAPHICS

static void keyboardStatic(unsigned char key, int x, int y)
{
  driver->keyboardCallback(key, x, y);
}
static void mouseStatic(int button, int state, int x, int y)
{
  driver->mouseCallback(button, state, x, y);
}
static void displayStatic()
{
  driver->displayCallback();
}

void Driver::setup_gl(int argc, char** argv)
{
  m_shapeDrawer = new GL_ShapeDrawer();
  m_shapeDrawer->enableTexture(true);
  //m_ele = 0.0; // camera up down
  //m_azi = 0.0; // camera left right

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(m_glutScreenWidth, m_glutScreenHeight);
  glutCreateWindow("off to work we go");
  //glutSetOption (GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

  myinit();

  glutIdleFunc(displayStatic);
  glutKeyboardFunc(keyboardStatic);
  glutMouseFunc(mouseStatic);
  glutDisplayFunc(displayStatic);
  displayStatic();
}


void Driver::updateCamera()
{

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  btScalar rele = m_ele * btScalar(0.01745329251994329547);// rads per deg 
  btScalar razi = m_azi * btScalar(0.01745329251994329547);// radds per deg
  btQuaternion rot(m_cameraUp,razi);
  btVector3 eyePos(0,0,0);
  eyePos[m_forwardAxis] = -m_cameraDistance;
  btVector3 forward(eyePos[0],eyePos[1],eyePos[2]);
  if (forward.length2() < SIMD_EPSILON)
    forward.setValue(1.f,0.f,0.f);
  btVector3 right = m_cameraUp.cross(forward);
  btQuaternion roll(right,-rele);

  eyePos = btMatrix3x3(rot) * btMatrix3x3(roll) * eyePos;

  m_cameraPosition[0] = eyePos.getX();
  m_cameraPosition[1] = eyePos.getY();
  m_cameraPosition[2] = eyePos.getZ();
  m_cameraPosition += m_cameraTargetPosition;

  if (m_glutScreenWidth == 0 && m_glutScreenHeight == 0)
    return;

  btScalar aspect;
  btVector3 extents;

  aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;
  extents.setValue(aspect * 1.0f, 1.0f,0);

  if (m_ortho)
    {
      // reset matrix
      glLoadIdentity();
      extents *= m_cameraDistance;
      btVector3 lower = m_cameraTargetPosition - extents;
      btVector3 upper = m_cameraTargetPosition + extents;
      //gluOrtho2D(lower.x, upper.x, lower.y, upper.y);
      glOrtho(lower.getX(), upper.getX(), lower.getY(), upper.getY(),-1000,1000);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      //glTranslatef(100,210,0);
    } else
    {
      //              glFrustum (-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);
      glFrustum (-aspect * m_frustumZNear, aspect * m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2],
		m_cameraTargetPosition[0], m_cameraTargetPosition[1], m_cameraTargetPosition[2],
		m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());
    }
}

void Driver::keyboardCallback(unsigned char key, int x, int y)
{
  (void)x;
  (void)y;
  
  m_lastKey = 0;
  
  switch (key)
    {
    case 8:
      {
	int numObj = m_dynamicsWorld->getNumCollisionObjects();
	if (numObj)
	  {
	    btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[numObj-1];

	    m_dynamicsWorld->removeCollisionObject(obj);
	    btRigidBody* body = btRigidBody::upcast(obj);
	    if (body && body->getMotionState())
	      {
		delete body->getMotionState();
	      }
	    delete obj;
	  }
	break;
      }
    case 'q' :
#ifdef BT_USE_FREEGLUT
      //return from glutMainLoop(), detect memory leaks etc.
      glutLeaveMainLoop();
#else
      exit(0);
#endif
      break;

    case 'l' : stepLeft(); break;
    case 'r' : stepRight(); break;
    case 'f' : stepFront(); break;
    case 'b' : stepBack(); break;
    case 'z' : zoomIn(); break;
    case 'x' : zoomOut(); break;
    case 'i' : toggleIdle(); break;
    case 'g' : m_enableshadows=!m_enableshadows;break;
    case 'u' : m_shapeDrawer->enableTexture(!m_shapeDrawer->enableTexture(false));break;
    case 'w':
      if (m_debugMode & btIDebugDraw::DBG_DrawWireframe)
	m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawWireframe);
      else
	m_debugMode |= btIDebugDraw::DBG_DrawWireframe;
      break;
    case '=':
      {
	int maxSerializeBufferSize = 1024*1024*5;
	btDefaultSerializer*    serializer = new btDefaultSerializer(maxSerializeBufferSize);
	//serializer->setSerializationFlags(BT_SERIALIZE_NO_DUPLICATE_ASSERT);
	m_dynamicsWorld->serialize(serializer);
	FILE* f2 = fopen("testFile.bullet","wb");
	fwrite(serializer->getBufferPointer(),serializer->getCurrentBufferSize(),1,f2);
	fclose(f2);
	delete serializer;
	break;
      }
    case 'm':
      if (m_debugMode & btIDebugDraw::DBG_EnableSatComparison)
	m_debugMode = m_debugMode & (~btIDebugDraw::DBG_EnableSatComparison);
      else
	m_debugMode |= btIDebugDraw::DBG_EnableSatComparison;
      break;

    case 'n':
      if (m_debugMode & btIDebugDraw::DBG_DisableBulletLCP)
	m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DisableBulletLCP);
      else
	m_debugMode |= btIDebugDraw::DBG_DisableBulletLCP;
      break;
    case 'N':
      if (m_debugMode & btIDebugDraw::DBG_DrawNormals)
	m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawNormals);
      else
	m_debugMode |= btIDebugDraw::DBG_DrawNormals;
      break;

    case 't' :
      if (m_debugMode & btIDebugDraw::DBG_DrawText)
	m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawText);
      else
	m_debugMode |= btIDebugDraw::DBG_DrawText;
      break;
    case 'y':
      if (m_debugMode & btIDebugDraw::DBG_DrawFeaturesText)
	m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawFeaturesText);
      else
	m_debugMode |= btIDebugDraw::DBG_DrawFeaturesText;
      break;
    case 'a':
      if (m_debugMode & btIDebugDraw::DBG_DrawAabb)
	m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawAabb);
      else
	m_debugMode |= btIDebugDraw::DBG_DrawAabb;
      break;
    case 'c' :
      if (m_debugMode & btIDebugDraw::DBG_DrawContactPoints)
	m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawContactPoints);
      else
	m_debugMode |= btIDebugDraw::DBG_DrawContactPoints;
      break;
    case 'C' :
      if (m_debugMode & btIDebugDraw::DBG_DrawConstraints)
	m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawConstraints);
      else
	m_debugMode |= btIDebugDraw::DBG_DrawConstraints;
      break;
    case 'L' :
      if (m_debugMode & btIDebugDraw::DBG_DrawConstraintLimits)
	m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawConstraintLimits);
      else
	m_debugMode |= btIDebugDraw::DBG_DrawConstraintLimits;
      break;
    case 'd' :
      if (m_debugMode & btIDebugDraw::DBG_NoDeactivation)
	m_debugMode = m_debugMode & (~btIDebugDraw::DBG_NoDeactivation);
      else
	m_debugMode |= btIDebugDraw::DBG_NoDeactivation;
      if (m_debugMode & btIDebugDraw::DBG_NoDeactivation)
	{
	  gDisableDeactivation = true;
	} else
	{
	  gDisableDeactivation = false;
	}
      break;
    case 'o' :
      {
	m_ortho = !m_ortho;//m_stepping = !m_stepping;
	break;
      }
    case 'p':
      {
	m_paused = !m_paused;
      }
    default:
      //std::cout << "unused key : " << key << std::endl;
      break;
    }

  if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(m_debugMode);
}

void Driver::setDebugMode(int mode)
{
  m_debugMode = mode;
  if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(mode);
}

btVector3 Driver::getRayTo(int x,int y)
{
  if (m_ortho)
    {

      btScalar aspect;
      btVector3 extents;
      aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;
      extents.setValue(aspect * 1.0f, 1.0f,0);

      extents *= m_cameraDistance;
      btVector3 lower = m_cameraTargetPosition - extents;
      btVector3 upper = m_cameraTargetPosition + extents;

      btScalar u = x / btScalar(m_glutScreenWidth);
      btScalar v = (m_glutScreenHeight - y) / btScalar(m_glutScreenHeight);

      btVector3       p(0,0,0);
      p.setValue((1.0f - u) * lower.getX() + u * upper.getX(),(1.0f - v) * lower.getY() + v * upper.getY(),m_cameraTargetPosition.getZ());
      return p;
    }

  float top = 1.f;
  float bottom = -1.f;
  float nearPlane = 1.f;
  float tanFov = (top-bottom)*0.5f / nearPlane;
  float fov = btScalar(2.0) * btAtan(tanFov);

  btVector3       rayFrom = getCameraPosition();
  btVector3 rayForward = (getCameraTargetPosition()-getCameraPosition());
  rayForward.normalize();
  float farPlane = 10000.f;
  rayForward*= farPlane;

  btVector3 rightOffset;
  btVector3 vertical = m_cameraUp;

  btVector3 hor;
  hor = rayForward.cross(vertical);
  hor.normalize();
  vertical = hor.cross(rayForward);
  vertical.normalize();

  float tanfov = tanf(0.5f*fov);


  hor *= 2.f * farPlane * tanfov;
  vertical *= 2.f * farPlane * tanfov;

  btScalar aspect;

  aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;

  hor*=aspect;


  btVector3 rayToCenter = rayFrom + rayForward;
  btVector3 dHor = hor * 1.f/float(m_glutScreenWidth);
  btVector3 dVert = vertical * 1.f/float(m_glutScreenHeight);


  btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
  rayTo += btScalar(x) * dHor;
  rayTo -= btScalar(y) * dVert;
  return rayTo;
}

void Driver::mouseCallback(int button, int state, int x, int y)
{
  // Lols what's a mouse?
}

void Driver::displayCallback()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clears last state's drawing
  
  if (m_paused && run())
    glutLeaveMainLoop(); 

  myinit();
  updateCamera();
  if (m_dynamicsWorld)
    {
      glDisable(GL_CULL_FACE);
      
      btScalar m[16];
      btMatrix3x3 rot; rot.setIdentity();
      const int numObjects = m_dynamicsWorld->getNumCollisionObjects();
      for(int i = 0; i < numObjects; i++)
	{
	  btCollisionObject*colObj=m_dynamicsWorld->getCollisionObjectArray()[i];
	  btRigidBody*body=btRigidBody::upcast(colObj);
	  if(body&&body->getMotionState())
	    {
	      btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
	      myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
	      rot=myMotionState->m_graphicsWorldTrans.getBasis();
	    }
	  else
	    {
	      colObj->getWorldTransform().getOpenGLMatrix(m);
	      rot=colObj->getWorldTransform().getBasis();
	    }
	  
	  btVector3 wireColor(0.0f,0.0f,1.0f);
	  switch (colObj->getActivationState())
	    {
	    case 1: wireColor = btVector3 (1.0f, 0.0f, 0.0f); break;
	    case 2: wireColor = btVector3 (0.0f, 1.0f, 0.0f); break;
	    case 3: wireColor = btVector3 (0.0f, 0.0f, 1.0f); break;
	    default: wireColor = btVector3 (0.0f, 0.0f, 0.0f);
	    }
	  if (i == 0)
	    wireColor = btVector3 (0.05f, 0.05f, 0.05f);
	  if (i > 7)
	    wireColor = btVector3 (0.0f, 1.0f, 0.0f);
	  
	  btVector3 aabbMin(0,0,0),aabbMax(0,0,0);
	  aabbMin-=btVector3(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
	  aabbMax+=btVector3(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
	  
	  m_shapeDrawer->drawOpenGL(m,colObj->getCollisionShape(),wireColor,getDebugMode(),aabbMin,aabbMax);
	}      
      
      glDisable(GL_LIGHTING);
      glColor3f(0, 0, 0);
      glDisable(GL_LIGHTING);
    }
  updateCamera();
  glFlush();
  glutSwapBuffers();
}

btRigidBody* Driver::localCreateRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape)
{
  btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

  //rigidbody is dynamic if and only if mass is non zero, otherwise static
  bool isDynamic = (mass != 0.f);

  btVector3 localInertia(0,0,0);
  if (isDynamic)
    shape->calculateLocalInertia(mass,localInertia);

  //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects              

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
  btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

  btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

  btRigidBody* body = new btRigidBody(cInfo);
  body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
  btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);
  body->setWorldTransform(startTransform);
#endif

  m_dynamicsWorld->addRigidBody(body);

  return body;
}

//See http://www.lighthouse3d.com/opengl/glut/index.php?bmpfontortho
void Driver::setOrthographicProjection()
{
  // switch to projection mode
  glMatrixMode(GL_PROJECTION);

  // save previous matrix which contains the
  //settings for the perspective projection
  glPushMatrix();
  // reset matrix
  glLoadIdentity();
  // set a 2D orthographic projection
  gluOrtho2D(0, m_glutScreenWidth, 0, m_glutScreenHeight);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // invert the y axis, down is positive
  glScalef(1, -1, 1);
  // mover the origin from the bottom left corner
  // to the upper left corner
  glTranslatef(btScalar(0), btScalar(-m_glutScreenHeight), btScalar(0));
}

void Driver::resetPerspectiveProjection()
{

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  updateCamera();
}

void Driver::myinit(void)
{
  GLfloat light_ambient[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0) };
  GLfloat light_diffuse[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0) };
  GLfloat light_specular[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0 )};
  /* light_position is NOT default value */
  GLfloat light_position0[] = { btScalar(1.0), btScalar(10.0), btScalar(1.0), btScalar(0.0 )};
  GLfloat light_position1[] = { btScalar(-1.0), btScalar(-10.0), btScalar(-1.0), btScalar(0.0) };

  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position0);

  glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
  
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  
  glClearColor(btScalar(0.01),btScalar(0.01),btScalar(0.01), btScalar(0.0));
}

#endif
