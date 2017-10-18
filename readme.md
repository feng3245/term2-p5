The state takes account of the current x y and psi values which are 0 in car coordinate system as 
is assumed that the vehicle is traveling in straight line at any given state.
The update occurs when solve is called. A prediction of x y psi v cte and epsi is calculated then 
using the difference between their values and the value given by the path planning system is compared.
These values are used to calculated the cost. The cost along with the actuation constrations are then
fed to Ipopt to predict the actuation values of psi and acceleration.

time step 25 and dt 0.05 seemed reasonable. Although I believe dt of 0.1 would be better given the latency is 100ms.

The way points are transformed from map coordinates to vehicle coordinates before polynomial fit.

Since the vehicle starts with new initial after every latency period it have a much better chance at producing accurate accuation values.
