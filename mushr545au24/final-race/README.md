[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/QNPJhiQ3)
# final
### 1. What controller type did you choose to use for path following with multiple waypoints (pid, pp, mpc)? Why did you choose this controller type?
We choose to use mpc controller for path following with multiple waypoints. Our mpc performs very well in obstacle avoidance compared to pp and pid. Since mpc is more intelligent than simple controllers since it looks ahead into the future, it can avoid local minima, sudden braking, sharp turns and prevent collisions. It is more adaptive and flexible than an open loop planner which looks ahead in the future but then executes the whole trajectory. MPC can consider and handle constraints like speed limit, steer angle limit, road boundary, range to other cars and lanes which cannot easily be handled by PID.

### 2. What roadmap parameters did you use for planning (num_vertices, connection_radius, curvature)? Why did you choose these parameters?
We use the roadmap parameters as follows.
```
num_vertices:=2000
connection_radius:=20
curvature:=1.05
```
These parameters performed best in our tests. When num_vertices is 2000, compared to 1000, it causes the path to be subdivided into more segments, each with a corresponding discrete point. This improves the accuracy of the path because each vertex describes the changes in the path more accurately and better captures the details of the path. And compared to 5000, it takes less time to costruct roadmap, making the planning process more efficient. The curvature is 1.05, compared to 1.0, it will turn a little tighter, smoothing out the turning process as well as making the turning range not have to be as wide. And compared to larger curvature, it has smoother turns, with no sharp turns that can lead to an unstable ride.

### 3. What was the most difficult part of making your code work with multiple waypoints?
The most difficult part of making our code work with multiple waypoints is that with our original code and parameters, the car undergoes some small left-right shaking while running, but because the planned route is closer to walls, pillars or other obstacles, sometimes it hits these obstacles during the shaking process. We tried to solve this problem by adding expansion coefficients to the car model and the obstacle to keep it as far away from the obstacle as possible, but it did not work very well. Then we added waypoints so that the car has more data during runnning to guarantee its stability. And also, we edited the code in mpc.py to minimize the lag in running the car that could be caused by for being too slow in its computation.

# Final Race Video & Bagfile
### A video that demonstrates your system (Final Race)
https://drive.google.com/file/d/1nriv4NEzBN0NSo6vs7JDYozCU6G83DVU/view

### A bag file (include all topics) that demonstrates your system (Final Race)
https://drive.google.com/file/d/120oUubGSQPO3WXU9-UainaQE1UmtBp-6/view?usp=drive_link
