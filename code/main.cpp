#include "driver.h"

int main(int argc, char** argv)
{
  // Initialize the MPI library:
  MPI::Init(argc, argv);
  int rank = MPI::COMM_WORLD.Get_rank();
  MPI_Status status;
  
  // Receive our condition ID. Initialize with command line if possible, and MPI if we're a slave
  int condition_id = (argc >= 2 ? atoi(argv[1]) : 0); 
  condition_t condition;
  if (rank) MPI_Recv(&condition_id, 1, MPI_INT, 0, rank, MPI_COMM_WORLD, &status);
  fprintf(stderr, "Evaluator %i using condition %i\n", rank,  condition_id);
  fetch_condition(condition, condition_id); 
  
  // Figure out the size of and create send and receive buffers
  int num_recv_vars = 1, num_send_vars = 0;
  for (unsigned int i = 0; i < condition.actions.size(); ++i)
    if (condition.actions[i].atype == ACTION_STORE)
      ++num_send_vars;
  for (unsigned int i = 0; i < condition.brains.size(); ++i)
    num_recv_vars += condition.brains[i].num_s;
  double *recv_buffer = (double *) malloc(num_recv_vars * sizeof(double));
  double *send_buffer = (double *) malloc(num_send_vars * sizeof(double));
  if (argc > 2) for (int i = 2; i < argc; ++i) recv_buffer[i - 2] = atof(argv[i]);

  // Initialize RNG
  std::random_device rdev{};
  std::default_random_engine gen {rdev()};

  // Run the simulation
#ifdef GRAPHICS
  if (rank) MPI_Recv(recv_buffer, num_recv_vars, MPI_DOUBLE, 0, rank, MPI_COMM_WORLD, &status);
  Driver driver = Driver(&condition, recv_buffer, num_recv_vars, send_buffer, num_send_vars, gen, argc);
  driver.setup_gl(argc, argv);
  glutMainLoop();
#else
  Driver *driver;
  while (true)
    {
      if (rank) MPI_Recv(recv_buffer, num_recv_vars, MPI_DOUBLE, 0, rank, MPI_COMM_WORLD, &status);
      if (recv_buffer[0] == -1337.0) break;

      // Calculate seed 
      double sum = 0;
      for (int i = 0; i <  num_recv_vars; ++i)
	sum += std::abs(recv_buffer[i]);
      gen.seed((int) condition_id + (int) (sum * 10000.));

      // Run experiment
      driver = new Driver(&condition, recv_buffer, num_recv_vars, send_buffer, num_send_vars, gen, argc);
      while (driver->run() == 0) {}
      delete driver;
      if (argc > 1) break;
      
      if (rank) MPI_Send(send_buffer, num_send_vars, MPI_DOUBLE, 0, rank, MPI_COMM_WORLD);
    }
#endif
  fprintf(stderr, "EXITING EVALUATOR %i\n", rank);
  MPI::Finalize();
  return 0;
}
