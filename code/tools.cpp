#include "tools.h"
#include "assert.h"

int get_pos_in_list(int* list, int len, int id)
{
  for (int i = 0; i < len; ++i)
    if (list[i] == id)
      return i;
  return -1;
}


double next_powerlaw(double x0, double alpha, std::default_random_engine &gen)
{
  std::uniform_real_distribution<double> distribution(0.0, 1.0); // min max
  return x0 * pow(1.0 - distribution(gen), -1 / (alpha - 1));
}

double next_uniform(double start, double end, std::default_random_engine &gen)
{
  std::uniform_real_distribution<double> distribution(start, end); // min max
  return distribution(gen);
}

double next_normal(double mean, double stdev, std::default_random_engine &gen)
{
  std::normal_distribution<double> distribution(mean, stdev);  // mean std
  return distribution(gen);
}


void update_variations(condition_t &cond)
{
  int pos;
  for (unsigned int i = 0; i < cond.variators.size(); ++i)
    {
      pos = cond.variators[i].obj_id - 1;
      if (cond.variators[i].obj_type == 2) // rbody
	{
	  switch (cond.variators[i].obj_elem)
	    {
	    case 2: cond.rbodys[pos].dim.setX(cond.variators[i].value); break;
	    case 4: cond.rbodys[pos].pos.setX(cond.variators[i].value); break;
	    case 6: cond.rbodys[pos].pos.setZ(cond.variators[i].value); break;
	    default: fprintf(stderr, "UNKNOWN CASE IN RBODY"); break;
	    }
	}
    }
}

void fetch_variator(std::vector<variator_t> &variators, int condition_id,
		    sqlite3 *db, sqlite3_stmt *stmt)
{
  char sql[255];
  sprintf(sql, "SELECT vari_id, obj_type, obj_elem, obj_id, value FROM condvari LEFT JOIN variator ON vari_id = id WHERE cond_id = %i;", condition_id);
  variator_t variator;
  int rc = sqlite3_prepare(db, sql, -1, &stmt, 0);
  if (rc != SQLITE_OK)
    {
      fprintf(stderr, "SQL error: %d : %s\n",  rc, sqlite3_errmsg(db));
      exit(rc);
    }
  do{
    rc = sqlite3_step(stmt);
    switch (rc) {
    case SQLITE_DONE: break;
    case SQLITE_ROW:
      variator.id = sqlite3_column_int(stmt, 0);
      variator.obj_type = sqlite3_column_int(stmt, 1);
      variator.obj_elem = sqlite3_column_int(stmt, 2);
      variator.obj_id = sqlite3_column_int(stmt, 3);
      variator.value = sqlite3_column_double(stmt, 4);
      variators.push_back(variator);      
      break;
    default: fprintf(stderr, "Error: %d : %s\n",  rc, sqlite3_errmsg(db)); break;
    }
  } while (rc == SQLITE_ROW);
  sqlite3_finalize(stmt);
}

void fetch_world(world_t &world, sqlite3 *db, sqlite3_stmt *stmt)
{
  char sql[255];
  sprintf(sql, "SELECT id, aabb_min_x, aabb_min_y, aabb_min_z, aabb_max_x, aabb_max_y, aabb_max_z,  eval_steps, eval_dt FROM world;");
  int rc = sqlite3_prepare(db, sql, -1, &stmt, 0);
  if (rc != SQLITE_OK)
    {
      fprintf(stderr, "SQL error: %d : %s\n",  rc, sqlite3_errmsg(db));
      exit(rc);
    }
  do{
    rc = sqlite3_step(stmt);
    switch (rc) {
    case SQLITE_DONE: break;
    case SQLITE_ROW:
      world.id = sqlite3_column_int(stmt, 0);
      world.aabb_min = btVector3(sqlite3_column_double(stmt, 1),
				 sqlite3_column_double(stmt, 2),
				 sqlite3_column_double(stmt, 3));
      world.aabb_max = btVector3(sqlite3_column_double(stmt, 4),
				 sqlite3_column_double(stmt, 5),
				 sqlite3_column_double(stmt, 6));
      world.eval_steps = sqlite3_column_int(stmt, 7);
      world.eval_dt = sqlite3_column_double(stmt, 8);
      break;
    default: fprintf(stderr, "Error: %d : %s\n",  rc, sqlite3_errmsg(db)); break;
    }
  } while (rc == SQLITE_ROW);
  sqlite3_finalize(stmt);
}

void fetch_rbody(std::vector<rbody_t> &rbodys, sqlite3 *db, sqlite3_stmt *stmt)
{
  char sql[255];
  sprintf(sql, "SELECT * FROM rbody;");
  rbody_t rbody;
  int rc = sqlite3_prepare(db, sql, -1, &stmt, 0);
  if (rc != SQLITE_OK)
    {
      fprintf(stderr, "SQL error: %d : %s\n",  rc, sqlite3_errmsg(db));
      exit(rc);
    }
  do{
    rc = sqlite3_step(stmt);
    switch (rc) {
    case SQLITE_DONE: break;
    case SQLITE_ROW:
      rbody.id = sqlite3_column_int(stmt, 0);
      rbody.dim = btVector3(sqlite3_column_double(stmt, 1),
			    sqlite3_column_double(stmt, 2),
			    sqlite3_column_double(stmt, 3));
      rbody.pos = btVector3(sqlite3_column_double(stmt, 4),
			    sqlite3_column_double(stmt, 5),
			    sqlite3_column_double(stmt, 6));
      rbody.ori = btVector3(sqlite3_column_double(stmt, 7),
			    sqlite3_column_double(stmt, 8),
			    sqlite3_column_double(stmt, 9));
      rbody.shape = sqlite3_column_int(stmt, 10);
      rbody.mass = sqlite3_column_double(stmt, 11);
      rbody.friction_sliding = sqlite3_column_double(stmt, 12);
      rbody.friction_rolling = sqlite3_column_double(stmt, 13);
      rbodys.push_back(rbody);
      break;
    default: fprintf(stderr, "Error: %d : %s\n",  rc, sqlite3_errmsg(db)); break;
    }
  } while (rc == SQLITE_ROW);
  sqlite3_finalize(stmt);
}

void fetch_joint(std::vector<joint_t> &joints, sqlite3 *db, sqlite3_stmt *stmt)
{
  char sql[255];
  sprintf(sql, "SELECT * FROM joint;");
  joint_t joint;
  int rc = sqlite3_prepare(db, sql, -1, &stmt, 0);
  if (rc != SQLITE_OK)
    {
      fprintf(stderr, "SQL error: %d : %s\n",  rc, sqlite3_errmsg(db));
      exit(rc);
    }
  do{
    rc = sqlite3_step(stmt);
    switch (rc) {
    case SQLITE_DONE: break;
    case SQLITE_ROW:
      joint.id = sqlite3_column_int(stmt, 0);
      joint.rbody_id_0 = sqlite3_column_int(stmt, 1);
      joint.rbody_id_1 = sqlite3_column_int(stmt, 2);
      joint.pos = btVector3(sqlite3_column_double(stmt, 3),
			    sqlite3_column_double(stmt, 4),
			    sqlite3_column_double(stmt, 5));
      joint.ori = btVector3(sqlite3_column_double(stmt, 6),
			    sqlite3_column_double(stmt, 7),
			    sqlite3_column_double(stmt, 8));
      joint.jtype = sqlite3_column_int(stmt, 9);
      joint.ang_min = sqlite3_column_double(stmt, 10);
      joint.ang_max = sqlite3_column_double(stmt, 11);
      joint.offset = sqlite3_column_double(stmt, 12);
      joint.force = sqlite3_column_double(stmt, 13);
      joints.push_back(joint);
      break;
    default: fprintf(stderr, "Error: %d : %s\n",  rc, sqlite3_errmsg(db)); break;
    }
  } while (rc == SQLITE_ROW);
  sqlite3_finalize(stmt);
}

void fetch_brain(std::vector<brain_t> &brains, sqlite3 *db, sqlite3_stmt *stmt)
{
  char sql[255];
  sprintf(sql, "SELECT * FROM brain;");
  brain_t brain;
  int rc = sqlite3_prepare(db, sql, -1, &stmt, 0);
  if (rc != SQLITE_OK)
    {
      fprintf(stderr, "SQL error: %d : %s\n",  rc, sqlite3_errmsg(db));
      exit(rc);
    }
  do{
    rc = sqlite3_step(stmt);
    switch (rc) {
    case SQLITE_DONE: break;
    case SQLITE_ROW:
      brain.id = sqlite3_column_int(stmt, 0);
      brain.num_i = sqlite3_column_int(stmt, 1);
      brain.num_h = sqlite3_column_int(stmt, 2);
      brain.num_o = sqlite3_column_int(stmt, 3);
      brain.recurrent = sqlite3_column_int(stmt, 4);
      brain.connectivity = sqlite3_column_double(stmt, 5);
      brain.num_s = (brain.num_i + (brain.recurrent ? brain.num_h : 0) + brain.num_o) * brain.num_h;
      brains.push_back(brain);
      break;
    default: fprintf(stderr, "Error: %d : %s\n",  rc, sqlite3_errmsg(db)); break;
    }
  } while (rc == SQLITE_ROW);
  sqlite3_finalize(stmt);
}


void fetch_sussex(std::vector<sussex_t> &sussexs, sqlite3 *db, sqlite3_stmt *stmt)
{
  char sql[255];
  sprintf(sql, "SELECT * FROM sussex;");
  sussex_t sussex;
  int rc = sqlite3_prepare(db, sql, -1, &stmt, 0);
  if (rc != SQLITE_OK)
    {
      fprintf(stderr, "SQL error: %d : %s\n",  rc, sqlite3_errmsg(db));
      exit(rc);
    }
  do{
    rc = sqlite3_step(stmt);
    switch (rc) {
    case SQLITE_DONE: break;
    case SQLITE_ROW:
      sussex.id = sqlite3_column_int(stmt, 0);
      sussex.src = btVector3(sqlite3_column_double(stmt, 1),
			     sqlite3_column_double(stmt, 2),
			     sqlite3_column_double(stmt, 3));
      sussex.radius = sqlite3_column_double(stmt, 4);
      sussex.azi = sqlite3_column_double(stmt, 5);
      sussex.zen = sqlite3_column_double(stmt, 6);
      sussex.rays_x = sqlite3_column_int(stmt, 7);
      sussex.rays_y = sqlite3_column_int(stmt, 8);
      sussex.spread = sqlite3_column_double(stmt, 9);
      sussex.stype = sqlite3_column_int(stmt, 10);
      sussexs.push_back(sussex);
      break;
    default: fprintf(stderr, "Error: %d : %s\n",  rc, sqlite3_errmsg(db)); break;
    }
  } while (rc == SQLITE_ROW);
  sqlite3_finalize(stmt);
}


void fetch_action(std::vector<action_t> &actions, sqlite3 *db, sqlite3_stmt *stmt)
{
  char sql[255];
  sprintf(sql, "SELECT * FROM action;");
  action_t action;
  int rc = sqlite3_prepare(db, sql, -1, &stmt, 0);
  if (rc != SQLITE_OK)
    {
      fprintf(stderr, "SQL error: %d : %s\n",  rc, sqlite3_errmsg(db));
      exit(rc);
    }
  do{
    rc = sqlite3_step(stmt);
    switch (rc) {
    case SQLITE_DONE: break;
    case SQLITE_ROW:
      action.id = sqlite3_column_int(stmt, 0);
      action.atype      = sqlite3_column_int(stmt, 1);
      action.sprint_beg = sqlite3_column_double(stmt, 2);
      action.sprint_end = sqlite3_column_double(stmt, 3);
      action.eval_beg   = sqlite3_column_double(stmt, 4);
      action.eval_end   = sqlite3_column_double(stmt, 5);
      action.operation  = sqlite3_column_int(stmt, 6);
      action.position   = sqlite3_column_int(stmt, 7);
      action.parameter  = sqlite3_column_double(stmt, 8);
      action.src_type_0 = sqlite3_column_int(stmt, 9);
      action.src_elem_0 = sqlite3_column_int(stmt, 10);
      action.src_id_0   = sqlite3_column_int(stmt, 11);
      action.src_type_1 = sqlite3_column_int(stmt, 12);
      action.src_elem_1 = sqlite3_column_int(stmt, 13);
      action.src_id_1   = sqlite3_column_int(stmt, 14);
      actions.push_back(action);
      break;
    default: fprintf(stderr, "Error: %d : %s\n",  rc, sqlite3_errmsg(db)); break;
    }
  } while (rc == SQLITE_ROW);
  sqlite3_finalize(stmt);
}


void fetch_condition(condition_t &condition, int condition_id)
{
  // Connect to the database, exit on failure
  sqlite3 *db;
  sqlite3_stmt *stmt;
  int rc = sqlite3_open("phe.db", &db);
  if (rc != SQLITE_OK) {
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
    exit(rc);
  }

  char sql[255];
  sprintf(sql, "SELECT goal, num_interintra FROM condition WHERE id = %i;", condition_id);
  rc = sqlite3_prepare(db, sql, -1, &stmt, 0);
  if (rc != SQLITE_OK)
    {
      fprintf(stderr, "SQL error: %d : %s\n",  rc, sqlite3_errmsg(db));
      exit(rc);
    }
  do{
    rc = sqlite3_step(stmt);
    switch (rc) {
    case SQLITE_DONE: break;
    case SQLITE_ROW:
      condition.id = condition_id;
      condition.goal = sqlite3_column_double(stmt, 0);
      condition.num_interintra = sqlite3_column_int(stmt, 1);
      break;
    default: fprintf(stderr, "Error: %d : %s\n",  rc, sqlite3_errmsg(db)); break;
    }
  } while (rc == SQLITE_ROW);
  sqlite3_finalize(stmt);

  fetch_world(condition.world, db, stmt);
  fetch_rbody(condition.rbodys, db, stmt);
  fetch_joint(condition.joints, db, stmt);
  fetch_brain(condition.brains, db, stmt);
  fetch_sussex(condition.sussexs, db, stmt);
  fetch_action(condition.actions, db, stmt);
  fetch_variator(condition.variators, condition_id, db, stmt);
  update_variations(condition);
  sqlite3_close(db);
}

double calc_sussex(const sussex_t &s, btDynamicsWorld *world)
{
  if (s.rays_x == 0 || s.rays_y == 0) return 0.0;
  int i, j, pm_i = 1, pm_j = 1, count = 1;
  double x, y, z;
  double color = 0.0;
  btVector3 dst;
  double distance = s.radius;
  // Draw the first ray
  z = s.src.z() - s.radius * sin(s.zen) * cos(s.azi);
  y = s.src.y() + s.radius * cos(s.zen);
  x = s.src.x() + s.radius * sin(s.zen) * sin(s.azi);
  dst = btVector3(x, y, z);
  
  btCollisionWorld::ClosestRayResultCallback RayCallback(s.src, dst);
  world->rayTest(s.src, dst, RayCallback);

  if (RayCallback.hasHit())
    {      
      //int id = *((int *) RayCallback.m_collisionObject->getUserPointer());
      x = RayCallback.m_hitPointWorld.getX();
      y = RayCallback.m_hitPointWorld.getY();
      z = RayCallback.m_hitPointWorld.getZ();
      distance = sqrt(pow(s.src.x() - x, 2) + pow(s.src.y() - y, 2) + pow(s.src.z() - z, 2));
      color = 0.2;
#ifdef GRAPHICS
      draw_line(s.src, dst, btVector3(1.0, 1.0, 1.0));
#endif
    }
#ifdef GRAPHICS
  else
    {
      draw_line(s.src, dst, btVector3(0.5, 0.5, 0.5));
    }
#endif
  for (i = 0; i < s.rays_x; ++i)
    for (j = 0; j < s.rays_y; ++j)
      for (pm_i = -1; pm_i <= 1; pm_i += 2)
	for (pm_j = -1; pm_j <= 1; pm_j += 1) 
	  {
	    if (i == 0 && j == 0) continue;
	    ++count;
	    z = s.src.z() - s.radius * sin(s.zen + j * s.spread * pm_j) *
	      cos(s.azi + i * s.spread * pm_i);
	    y = s.src.y() + s.radius * cos(s.zen + j * s.spread * pm_j);
	    x = s.src.x() + s.radius * sin(s.zen + j * s.spread * pm_j) *
	      sin(s.azi + i * s.spread * pm_i);
	    dst.setX(x);
	    dst.setY(y);
	    dst.setZ(z);
	    btCollisionWorld::ClosestRayResultCallback RayCallback(s.src, dst);
	    world->rayTest(s.src, dst, RayCallback);  
	    if (RayCallback.hasHit())
	      {
		x = RayCallback.m_hitPointWorld.getX();
		y = RayCallback.m_hitPointWorld.getY();
		z = RayCallback.m_hitPointWorld.getZ();
		distance += sqrt(pow(s.src.x() - x, 2) + pow(s.src.y() - y, 2) + pow(s.src.z() - z, 2));
#ifdef GRAPHICS
		draw_line(s.src, dst, btVector3(0.0, 1.0, 1.0));
#endif
	      }
	    else
	      {
		distance += s.radius;
		color += 0.05;
#ifdef GRAPHICS
		draw_line(s.src, dst, btVector3(1.0, 1.0, 1.0));
#endif
	      }
	  }
  color = (s.stype ? color / count : distance / s.radius / count) * 2 - 1.0;
  return color;
}

btVector3 PointWorldToLocal(btRigidBody* body, btVector3 point)
{
  return body->getCenterOfMassTransform().inverse() * point;
};

btVector3 AxisWorldToLocal(btRigidBody* body, btVector3 point)
{
  btTransform comt = body->getCenterOfMassTransform().inverse();
  comt.setOrigin(btVector3(0,0,0));
  return comt * point;
};

void ActuateJoint(btHingeConstraint* joint, double degrees, double timestep, double amplitude)
{
  double target = degrees * M_PI / 180.0;
  joint->enableMotor(true);
  joint->setMaxMotorImpulse(amplitude);
  joint->setMotorTarget(target, timestep);
};

void ActuateJointAngular(btHingeConstraint* joint, double degrees, double timestep, double amplitude)
{
  double target = degrees * M_PI / 180.0;
  double diff = target - joint->getHingeAngle();
  joint->setMotorTarget(target, timestep);
  joint->enableAngularMotor(true, diff, amplitude);
};

int* AddNewID(btAlignedObjectArray<int *>* identities, int id)
 {
   int* init = (int *) malloc(sizeof(int));
   *init = id;
   identities->push_back(init);
   return (*identities)[*init];
 }

#ifdef GRAPHICS
void draw_line(btVector3 src, btVector3 dst, btVector3 color)
{
  glLineWidth(1.0);
  glColor3f(color.x(), color.y(), color.z());
  glBegin(GL_LINES);
  glVertex3f(src.x(), src.y(), src.z());
  glVertex3f(dst.x(), dst.y(), dst.z());
  glEnd();
}
#endif
