The state takes account of the current x y and psi values which are 0 in car coordinate system as 
is assumed that the vehicle is traveling in straight line at any given state.
The update occurs when solve is called. A prediction of x y psi v cte and epsi is calculated then 
using the difference between their values and the value given by the path planning system is compared.
These values are used to calculated the cost. 

x y v psi to be predicted values
 x0 = (px + v * cos(psi)*0.1);
        double y0 = (py + v * sin(psi)*0.1);
        double v0 = (v + aprev * 0.1);

psi += v*-delta*0.1/2.67;sideltprev = vars[6];
way points are translated into vehicle coordinate from map coordinates with predicted x and y and current steering.
for(int i = 0; i < ptsx.size(); i++)
{     double x = ptsx.at(i) - x0;
     double y = ptsy.at(i) - y0;
     ptsxxd(i) = x * cos(-psi) - y * sin(-psi);
     ptsyxd(i) = x * sin(-psi) + y * cos(-psi);


}

a third order polynomial is fitted with the translated waypoints 
         auto coeffs = polyfit(ptsxxd, ptsyxd, 3);
error is then calculated from the derivate of the polynomial
        double cte = polyeval(coeffs, 0) - 0;
        double epsi = - atan(coeffs[1]);
        Eigen::VectorXd state(6);

        state << 0, 0, 0, v0, cte, epsi;
 std::vector<double> x_vals = {0};
  std::vector<double> y_vals = {0};

 std::vector<double> psi_vals = {psi};
  std::vector<double> a_vals = {};

the values are sent to the solve function
        auto vars = mpc.Solve(state, coeffs);


The update step line 96 down from MCP.cpp estimate predicted y and psi value via a third order polynomial this is the
 AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] *
 x0 * x0 * x0;
      AD<double> psides0 =  CppAD::atan(coeffs[1] + 2.0 * coeffs[2] * x0 + 3.0 *
 coeffs[3] * x0 * x0);

        fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0)*dt);
        fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0)*dt);
        fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
        fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
        fg[1 + cte_start + t] = cte1 - ((f0 - y0)+(v0 * CppAD::sin(epsi0)*dt));
        fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0)+v0 * delta0/Lf * dt);
 used to give further accuracy through additional error information provided.







 
The cost along with the actuation constrations are then
fed to Ipopt to predict the actuation values of psi and acceleration.

The actuators used for the model is steering and throttle. Both being a value between -1 and 1

changed time steps to 10 to reduce amount of processing and dt to 0.1. 0.1 seems fine enough at first but gave a lot of choppy oscolations. I then changed to 0.05 dt and this has garnered a huge improvement in smoothness. This should give smooth actuations without sacificing too much computation time.

The way points are transformed from map coordinates to vehicle coordinates before polynomial fit.

To account for the latency the intial state is the predicted state of the next state. X and Y values are changes with psi and everything else remaining the same. CTE and EPSI is recalculated to be used for prediction.
