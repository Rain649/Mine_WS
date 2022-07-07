#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

double Lfr = 2.88;

// =========================================
// FG_eval class definition implementation.
// =========================================
class FG_eval
{
public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;

  double _dt, _ref_cte, _ref_etheta, _ref_vel;
  double _w_cte, _w_etheta, _w_vel, _w_angle, _w_accel, _w_angle_d, _w_accel_d;
  int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _cte_start, _epsi_start, _delta_start, _a_start;

  AD<double> cost_cte, cost_etheta, cost_vel, cost_actuators, cost_gaps;
  // Constructor
  FG_eval(Eigen::VectorXd coeffs)
  {
    this->coeffs = coeffs;

    // Set default value
    _dt = 0.1; // in sec
    _ref_cte = 0;
    _ref_etheta = 0;
    _ref_vel = 3; // m/s
    _w_cte = 100;
    _w_etheta = 100;
    _w_vel = 1;
    _w_angle = 100;
    _w_accel = 50;
    _w_angle_d = 0;
    _w_accel_d = 0;

    _mpc_steps = 50;

    _x_start = 0;
    _y_start = _x_start + _mpc_steps;
    _theta_start = _y_start + _mpc_steps;

    // 1. cte, cross track error (distance between vehicle's position and reference trajectory)
    // 2. epsi (orientation diff between vehicle and ref trajectory)
    // 3. diff bw v and ref_v

    _v_start = _theta_start + _mpc_steps;
    _cte_start = _v_start + _mpc_steps;
    _epsi_start = _cte_start + _mpc_steps;

    _delta_start = _epsi_start + _mpc_steps;
    _a_start = _delta_start + _mpc_steps - 1;
  }

  // Load parameters for constraints
  void LoadParams(const std::map<string, double> &params)
  {
    _dt = params.find("DT") != params.end() ? params.at("DT") : _dt;
    _mpc_steps = params.find("STEPS") != params.end() ? params.at("STEPS") : _mpc_steps;
    _ref_cte = params.find("REF_CTE") != params.end() ? params.at("REF_CTE") : _ref_cte;
    _ref_etheta = params.find("REF_ETHETA") != params.end() ? params.at("REF_ETHETA") : _ref_etheta;
    _ref_vel = params.find("REF_V") != params.end() ? params.at("REF_V") : _ref_vel;

    _w_cte = params.find("W_CTE") != params.end() ? params.at("W_CTE") : _w_cte;
    _w_etheta = params.find("W_EPSI") != params.end() ? params.at("W_EPSI") : _w_etheta;
    _w_vel = params.find("W_V") != params.end() ? params.at("W_V") : _w_vel;
    _w_angle = params.find("W_ANGLE") != params.end() ? params.at("W_ANGLE") : _w_angle;
    _w_accel = params.find("W_A") != params.end() ? params.at("W_A") : _w_accel;
    _w_angle_d = params.find("W_DANGLE") != params.end() ? params.at("W_DANGLE") : _w_angle_d;
    _w_accel_d = params.find("W_DA") != params.end() ? params.at("W_DA") : _w_accel_d;

    _x_start = 0;
    _y_start = _x_start + _mpc_steps;
    _theta_start = _y_start + _mpc_steps;
    _v_start = _theta_start + _mpc_steps;
    _cte_start = _v_start + _mpc_steps;
    _epsi_start = _cte_start + _mpc_steps;
    _delta_start = _epsi_start + _mpc_steps;
    _a_start = _delta_start + _mpc_steps - 1;

    // cout << "\n!! FG_eval Obj parameters updated !! " << _mpc_steps << endl;
  }

  // MPC implementation (cost func & constraints)
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  // fg: function that evaluates the objective and constraints using the syntax
  void operator()(ADvector &fg, const ADvector &vars)
  {
    // fg[0] for cost function
    fg[0] = 0;
    cost_cte = 0;    // cross deviation error
    cost_etheta = 0; // heading error
    cost_vel = 0;    // speed error
    cost_actuators = 0;
    cost_gaps = 0;

    for (int i = 0; i < _mpc_steps; ++i)
    {
      cost_cte += _w_cte * CppAD::pow(vars[_cte_start + i] - _ref_cte, 2);
      cost_etheta += _w_etheta * CppAD::pow(vars[_epsi_start + i] - _ref_etheta, 2);

      // cost_vel += (_w_vel * CppAD::pow(vars[_v_start + i] - _ref_vel, 2));
    }
    fg[0] = cost_cte + cost_etheta + cost_vel;

    cout << "-----------------------------------------------" << endl;
    cout << "cost_cte, etheta, velocity: " << cost_cte << ", " << cost_etheta << ", " << cost_vel << endl;

    // Minimize the use of actuators.
    for (int i = 0; i < _mpc_steps - 1; ++i)
    {
      cost_actuators += _w_angle * CppAD::pow(vars[_delta_start + i], 2);
      cost_actuators += _w_accel * CppAD::pow(vars[_a_start + i], 2);
      // cout << "delta ：" << vars[_delta_start + i] << "; ars[_a_start + i] : " << vars[_a_start + i] << endl;
    }
    fg[0] += cost_actuators;
    // cout << "cost of actuators: " << cost_actuators << endl;

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < _mpc_steps - 2; ++i)
    {
      cost_gaps += _w_angle_d * CppAD::pow(vars[_delta_start + i + 1] - vars[_delta_start + i], 2);
      cost_gaps += _w_accel_d * CppAD::pow(vars[_a_start + i + 1] - vars[_a_start + i], 2);
    }
    fg[0] += cost_gaps;
    // cout << "cost of gap: " << cost_gaps << endl;

    // fg[x] for constraints
    // Initial constraints
    fg[1 + _x_start] = vars[_x_start];
    fg[1 + _y_start] = vars[_y_start];
    fg[1 + _theta_start] = vars[_theta_start];
    fg[1 + _v_start] = vars[_v_start];
    fg[1 + _cte_start] = vars[_cte_start];
    fg[1 + _epsi_start] = vars[_epsi_start];

    // Add system dynamic model constraint
    for (int i = 0; i < _mpc_steps - 1; ++i)
    {
      // The state at time t+1 .
      AD<double> x1 = vars[_x_start + i + 1];
      AD<double> y1 = vars[_y_start + i + 1];
      AD<double> theta1 = vars[_theta_start + i + 1];
      AD<double> v1 = vars[_v_start + i + 1];
      AD<double> cte1 = vars[_cte_start + i + 1];
      AD<double> etheta1 = vars[_epsi_start + i + 1];

      // The state at time t.
      AD<double> x0 = vars[_x_start + i];
      AD<double> y0 = vars[_y_start + i];
      AD<double> theta0 = vars[_theta_start + i];
      AD<double> v0 = vars[_v_start + i];
      AD<double> cte0 = vars[_cte_start + i];
      AD<double> etheta0 = vars[_epsi_start + i];

      // Only consider the actuation at time t.
      // AD<double> angle0 = vars[_delta_start + i];
      AD<double> w0 = vars[_delta_start + i];
      AD<double> a0 = vars[_a_start + i];

      // AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> f0 = 0.0;
      for (auto i = 0; i < coeffs.size(); ++i)
      {
        f0 += coeffs[i] * CppAD::pow(x0, i);
      }

      // AD<double> trj_grad0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));
      AD<double> trj_grad0 = 0.0;
      for (auto i = 1; i < coeffs.size(); ++i)
      {
        trj_grad0 += i * coeffs[i] * CppAD::pow(x0, i - 1); // f'(x0)
      }
      trj_grad0 = CppAD::atan(trj_grad0);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // This is also CppAD can compute derivatives and pass
      // these to the solver.
      // TODO: Setup the rest of the model constraints
      fg[2 + _x_start + i] = x1 - (x0 + v0 * CppAD::cos(theta0) * _dt);
      fg[2 + _y_start + i] = y1 - (y0 + v0 * CppAD::sin(theta0) * _dt);
      fg[2 + _theta_start + i] = theta1 - (theta0 + v0 * w0 / Lfr * _dt);
      fg[2 + _v_start + i] = v1 - (v0 + a0 * _dt);
      fg[2 + _cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(etheta0) * _dt));
      fg[2 + _epsi_start + i] = etheta1 - ((theta0 - trj_grad0) + v0 * w0 / Lfr * _dt);
    }
  }
};

// ====================================
// MPC class definition implementation.
// ====================================
MPC::MPC()
{
  // Set default value
  _mpc_steps = 20;
  _max_angle = 35 * M_PI / 180; // Maximal ANGLE radian (~30 deg)
  _max_throttle = 1.0;          // Maximal throttle accel
  _bound_value = 1.0e3;         // Bound value for other variables
  _ref_vel = 10 / 3.6;
  _min_vel = 1.0;

  _x_start = 0;
  _y_start = _x_start + _mpc_steps;
  _theta_start = _y_start + _mpc_steps;
  _v_start = _theta_start + _mpc_steps;
  _cte_start = _v_start + _mpc_steps;
  _epsi_start = _cte_start + _mpc_steps;
  _delta_start = _epsi_start + _mpc_steps;
  _a_start = _delta_start + _mpc_steps - 1;
}

void MPC::LoadParams(const std::map<string, double> &params)
{
  _params = params;
  // Init parameters for MPC object
  _mpc_steps = _params.find("STEPS") != _params.end() ? _params.at("STEPS") : _mpc_steps;
  _max_angle = _params.find("ANGLE") != _params.end() ? _params.at("ANGLE") : _max_angle;
  _max_throttle = _params.find("MAXTHR") != _params.end() ? _params.at("MAXTHR") : _max_throttle;
  _bound_value = _params.find("BOUND") != _params.end() ? _params.at("BOUND") : _bound_value;
  _ref_vel = _params.find("REF_V") != _params.end() ? _params.at("REF_V") : _ref_vel;
  _min_vel = params.find("MIN_V") != params.end() ? params.at("MIN_V") : _min_vel;

  _x_start = 0;
  _y_start = _x_start + _mpc_steps;
  _theta_start = _y_start + _mpc_steps;
  _v_start = _theta_start + _mpc_steps;
  _cte_start = _v_start + _mpc_steps;
  _epsi_start = _cte_start + _mpc_steps;
  _delta_start = _epsi_start + _mpc_steps;
  _a_start = _delta_start + _mpc_steps - 1;

  // cout << "\n!! MPC Obj parameters updated !! " << endl;
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  const double x = state[0];
  const double y = state[1];
  const double theta = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double etheta = state[5];

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  // 4 * 10 + 2 * 9
  size_t n_vars = _mpc_steps * 6 + (_mpc_steps - 1) * 2;

  // Set the number of constraints
  size_t n_constraints = _mpc_steps * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; ++i)
  {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[_x_start] = x;
  vars[_y_start] = y;
  vars[_theta_start] = theta;
  vars[_v_start] = v;
  vars[_cte_start] = cte;
  vars[_epsi_start] = etheta;

  // Set lower and upper limits for variables.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < _delta_start; ++i)
  {
    vars_lowerbound[i] = -_bound_value;
    vars_upperbound[i] = _bound_value;
  }

  // Set lower and upper limits for velocity.
  for (int i = _v_start; i < _cte_start; ++i)
  {
    // vars_lowerbound[i] = _min_vel;
    vars_lowerbound[i] = _ref_vel;
    vars_upperbound[i] = _ref_vel;
  }

  // The upper and lower limits of ANGLE are set to -25 and 25
  // degrees (values in radians).
  for (int i = _delta_start; i < _a_start; ++i)
  {
    vars_lowerbound[i] = -_max_angle;
    vars_upperbound[i] = _max_angle;
  }
  // Acceleration/decceleration upper and lower limits
  for (auto i = _a_start; i < static_cast<int>(n_vars); ++i)
  {
    vars_lowerbound[i] = -_max_throttle;
    vars_upperbound[i] = _max_throttle;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; ++i)
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[_x_start] = x;
  constraints_lowerbound[_y_start] = y;
  constraints_lowerbound[_theta_start] = theta;
  constraints_lowerbound[_v_start] = v;
  constraints_lowerbound[_cte_start] = cte;
  constraints_lowerbound[_epsi_start] = etheta;
  constraints_upperbound[_x_start] = x;
  constraints_upperbound[_y_start] = y;
  constraints_upperbound[_theta_start] = theta;
  constraints_upperbound[_v_start] = v;
  constraints_upperbound[_cte_start] = cte;
  constraints_upperbound[_epsi_start] = etheta;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);
  fg_eval.LoadParams(_params);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.2\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound, constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "------------ Total Cost(solution): " << cost << "------------" << std::endl;
  cout << "-----------------------------------------------" << endl;

  this->mpc_x.clear();
  this->mpc_y.clear();
  for (int i = 0; i < _mpc_steps; ++i)
  {
    this->mpc_x.push_back(solution.x[_x_start + i]);
    this->mpc_y.push_back(solution.x[_y_start + i]);
  }
  vector<double> result;
  result.push_back(solution.x[_delta_start]);
  result.push_back(solution.x[_a_start]);
  result.push_back(solution.x[_v_start]);

  // for (auto i = 0; i < solution.x.size() / _mpc_steps+1; ++i)
  // {
  //   for (auto j = 0; j < _mpc_steps; ++j)
  //   {
  //     printf("%f ,", solution.x[i * result.size() + j]);
  //   }
  //   printf("\n");
  // }
  // printf("\n");

  return result;
}
