#ifndef MPC_H
#define MPC_H

#include <vector>
// for file


using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(std::vector<double> state, std::vector<double> ptsx, std::vector<double>  ptsy);
};


// Set the timestep length and duration
// const int N = 20;
const double dt = 0.05;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 0.324;
const int N = 40;                 
const double ref_v = 0.3;

const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t delta_start = v_start + N;
const size_t a_start = delta_start + N - 1;



#endif /* MPC_H */
