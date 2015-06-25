#include "rnn.h"

void RNN::print()
{
  for (int i = 0; i < num_i; ++i)
    printf(" %6.3f", node_i[i]);
  for (int i = 0; i < num_h; ++i)
    printf(" %6.3f", node_h[i]);
  for (int i = 0; i < num_o; ++i)
    printf(" %6.3f", node_o[i]);
  printf("\n");
}

double RNN::flatten(double val)
{
  //return 0.63661977236 * atan(6.28318530718 * pow(val, 3));
  double a = (std::abs(val) - 0.2) / 0.8;
  int sign = val >= 0 ? 1 : -1;
  if (a <= 0.0) return 0.0;
  if (a >= 1.0) return sign;
  return a * sign;
}
  
// using feed forward to update output before hidden
void RNN::update()
{
  int i, j;
  double temp;

  for (i = 0; i < num_h; ++i)
    {
      temp = 0;
      for (j = 0; j < num_i; ++j)
	{
	  temp += node_i[j] * synapse_i[j][i];
	}
      if (recurrent)
	for (j = 0; j < num_h; ++j)
	  temp += node_h[j] * synapse_h[i][j];
      node_h[i] = std::erf(temp);
    }
  for (i = 0; i < num_o; ++i)
    {
      temp = 0;
      for (j = 0; j < num_h; ++j)
	temp += node_h[j] * synapse_o[i][j];
      node_o[i] = ((i == num_o - 1) ? std::erf(temp) : flatten(temp));
    }
}

RNN::RNN (int input, int hidden, int output, double* synapses, bool rec)
{
  num_i = input;
  num_h = hidden;
  num_o = output;
  num_s = rec ? num_h * (num_h + num_i + num_o): num_h * (num_i + num_o);
  recurrent = rec;
  
  node_i = (double *) std::malloc(num_i * sizeof(double));
  node_o = (double *) std::malloc(num_o * sizeof(double));
  node_h = (double *) std::malloc(num_h * sizeof(double));
  for (int i = 0; i < num_i; ++i) node_i[i] = 0.0;
  for (int i = 0; i < num_h; ++i) node_h[i] = 0.0;
  for (int i = 0; i < num_o; ++i) node_o[i] = 0.0;
  
  synapse_i = (double **) std::malloc(num_i * sizeof(double *));
  if (recurrent) synapse_h = (double **) std::malloc(num_h * sizeof(double *));
  synapse_o = (double **) std::malloc(num_o * sizeof(double *));

  int pos = 0, i, j;
  for (i = 0; i < num_i; ++i)
    {
      synapse_i[i] = (double *) std::malloc(num_h * sizeof(double));
      for (j = 0; j < num_h; ++j)
	synapse_i[i][j] = synapses[pos++];
    }

  if (recurrent)
    for (i = 0; i < num_h; ++i)
      {
	synapse_h[i] = (double *) std::malloc(num_h * sizeof(double));
	for (j = 0; j < num_h; ++j)
	  synapse_h[i][j] = synapses[pos++];
      }

  for (i = 0; i < num_o; ++i)
    {
      synapse_o[i] = (double *) std::malloc(num_h * sizeof(double));
      for (j = 0; j < num_h; ++j)
	synapse_o[i][j] = synapses[pos++];
    }
}

RNN::~RNN()
{
  for (int i = 0; i < num_i; ++i) free(synapse_i[i]);
  for (int i = 0; i < num_h; ++i) free(synapse_h[i]);
  for (int i = 0; i < num_o; ++i) free(synapse_o[i]);
  free(synapse_i);
  free(synapse_h);
  free(synapse_o);
  free(node_i);
  free(node_h);
  free(node_o);
}
