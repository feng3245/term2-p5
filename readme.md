The state takes account of the current x y and psi values which are 0 in car coordinate system as 
is assumed that the vehicle is traveling in straight line at any given state.
The update occurs when solve is called. A prediction of x y psi v cte and epsi is calculated then 
using the difference between their values and the value given by the path planning system is compared.
These values are used to calculated the cost. 
The update step line 96 down from MCP.cpp estimate predicted y and psi value via a third order polynomial this is the
 used to give further accuracy through additional error information provided. 
The cost along with the actuation constrations are then
fed to Ipopt to predict the actuation values of psi and acceleration.

The actuators used for the model is steering and throttle. Both being a value between -1 and 1

time step 25 and dt 0.05 seemed reasonable. The processing of the times steps are rather fast and the amount of JSON messages passed back isn't overwhelmingly huge as I saw the display of JSON output on the console.

The way points are transformed from map coordinates to vehicle coordinates before polynomial fit.

To account for the latency the intial state is the predicted state of the next state. X and Y values are changes with psi and everything else remaining the same. CTE and EPSI is recalculated to be used for prediction.
