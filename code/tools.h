#ifndef TOOLS_H
#define TOOLS_H

#define DEG2RAD 0.01745329251
#define BUFFLEN 1024

#include "btBulletDynamicsCommon.h"
#include <sqlite3.h>
#include <string>
#include <math.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <unistd.h>
#include <random>
#ifdef GRAPHICS
#include <GL/gl.h>
#include <GL/glut.h>
#endif

const int FIXED = 0;
const int WORLD = 1;
const int RIGID = 2;
const int JOINT = 3;
const int BRAIN = 4;
const int RANDOM = 5;
const int SUSSEX = 6;
const int CALC = 7;

const int SHAPE_BOX = 1;
const int SHAPE_CYLINDER = 2;
const int SHAPE_CAPSULE = 3;
const int SHAPE_SPHERE = 4;
const int SHAPE_CONE = 5;

const int CONSTRAINT_HINGE = 1;
const int CONSTRAINT_POINT = 2;
const int CONSTRAINT_GENERIC = 3;

const int ACTION_UPDATE_NEURON = 1;
const int ACTION_UPDATE_BRAIN = 2;
const int ACTION_MOTOR = 3;
const int ACTION_STORE = 4;

typedef struct variator_t
{
  int id, obj_type, obj_elem, obj_id;
  double value;
} variator_t;

typedef struct world_t
{
  int id, eval_steps;
  double eval_dt;
  btVector3 aabb_min, aabb_max;
} world_t;

typedef struct rbody_t
{
  int id, shape;
  btVector3 dim, pos, ori;
  double mass, friction_sliding, friction_rolling;
} rbody_t;

typedef struct joint_t
{
  int id, rbody_id_0, rbody_id_1, jtype;
  btVector3 pos, ori;
  double ang_min, ang_max, offset, force;
} joint_t;

typedef struct brain_t
{
  int id, num_i, num_h, num_o, recurrent, connectivity, num_s;
} brain_t;


typedef struct sussex_t
{
  int id, rays_x, rays_y, stype;
  btVector3 src;
  double radius, azi, zen, spread;
} sussex_t;

typedef struct action_t
{
  int id, atype, position, operation, 
    src_type_0, src_elem_0, src_id_0, 
    src_type_1, src_elem_1, src_id_1;
  double sprint_beg, sprint_end, eval_beg, eval_end, parameter;
} action_t;

typedef struct condition_t
{
  int id;
  double goal;
  int num_interintra;
  world_t world;
  std::vector<variator_t> variators;
  std::vector<rbody_t> rbodys;
  std::vector<joint_t> joints;
  std::vector<brain_t> brains;
  std::vector<action_t> actions;
  std::vector<sussex_t> sussexs;
} condition_t;

double next_powerlaw(double x0, double alpha, std::default_random_engine &gen);
double next_uniform(double start, double end, std::default_random_engine &gen);
double next_normal(double mean, double stdev, std::default_random_engine &gen);
  
void update_variations(condition_t &cond);
void fetch_variator(std::vector<variator_t> &variators, int condition_id,
		    sqlite3 *db, sqlite3_stmt *stmt);
void fetch_world(world_t &world, sqlite3 *db, sqlite3_stmt *stmt);
void fetch_rbody(std::vector<rbody_t> &rbodys, sqlite3 *db, sqlite3_stmt *stmt);
void fetch_joint(std::vector<joint_t> &joints, sqlite3 *db, sqlite3_stmt *stmt);
void fetch_brain(std::vector<brain_t> &brains, sqlite3 *db, sqlite3_stmt *stmt);
void fetch_action(std::vector<action_t> &actions, sqlite3 *db, sqlite3_stmt *stmt);
void fetch_sussex(std::vector<sussex_t> &sussexs, sqlite3 *db, sqlite3_stmt *stmt);
void fetch_condition(condition_t &condition, int condition_id);  
int get_pos_in_list(int* list, int len, int id);


double calc_sussex(const sussex_t &s, btDynamicsWorld *world);
btVector3 PointWorldToLocal(btRigidBody* body, btVector3 point);
btVector3 AxisWorldToLocal(btRigidBody* body, btVector3 point);
void ActuateJoint(btHingeConstraint* joint, double degrees, double timestep, double amplitude);
void ActuateJointAngular(btHingeConstraint* joint, double degrees, double timestep, double amplitude);
int* AddNewID(btAlignedObjectArray<int *>* identities, int id);

#ifdef GRAPHICS
void draw_line(btVector3 src, btVector3 dst, btVector3 color);
#endif

#endif
