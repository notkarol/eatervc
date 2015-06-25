#ifndef RNN_H
#define RNN_H

#define DEG2RAD 0.01745329251

#include <math.h>
#include <vector>
#include <cstdlib>
#include <stdio.h>
#include "tools.h"

class RNN
{
 public:  
  int recurrent;
  int num_s, num_i, num_h, num_o;
  double *node_i, *node_h, *node_o;  
  double **synapse_i, **synapse_h, **synapse_o;

  RNN(int input, int hidden, int output, double* synapses, bool rec);
  ~RNN();
  static double flatten(double val);
  void update();
  void print();
};

#endif
