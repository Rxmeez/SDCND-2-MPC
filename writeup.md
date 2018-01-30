# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## The Model
- Student describes their model in detail. This includes the state, actuators and update equations.

The model used was a kinematic model. The model includes the vehicles position (x, y), vehicles angle (psi), velocity(v), cross-track error (cte) and psi error (epsi). With Actuators output being throttle (a) and steering angle(delta).

The model took the state of the vehicle and actuator from the previous timestep to predict the current timestep.

This was calcuated by using the Update Equations below:
```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psi_des[t] + v[t] * delta[t] / Lf * dt
```

## Timestep Length and Elapsed Duration (N & dt)
- Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

Various values were tried by adjusting the Timestep Length (N) or Elasped Duration (dt). Using large N values, N = 20, caused low computation speed that caused the car to be unstable, and if the value is too small, N = 3, it was unable to plan ahead causing it to be unstable. dt was choosen to be 0.1, as it matched the 100ms delay the actuators provided and small enough for quick computation with adequate planning ahead. The values were also suggested by Udacity's Q&A N = 10, dt = 0.1, which worked well.

## Polynomial Fitting and MPC Preprocessing
- A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

Waypoints are converted to vehicle coordinates, where firstly each waypoint is subtracted by the vehicle position(px, py) and then using a 2d vector transformation to get it in vehicle cordinates, this results in px, py and psi to equal to 0 (center). Later a third degree polynomial line will fit the transformed waypoints as it should cover any curves in the road that might occur.



## Model Predictive Control with Latency
- The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

This delay meant it caused area with turns/curve to be unstable at high speed as the decisions were being late. So we have to take this latency into account when constraining the controls to the previous values for the duration of the latency, so it can predict a more accurate current position and action to be taken by the actuators.
