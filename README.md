------------------------------------------------------------------------------------------------------------------------------------
# Steering-Behavior-Movement-Algorithms

The Steering Behavior Movement Algorithms were implemented using C# and Unity. These algorithms were inspired by the psuedo code in the book "AI For Game Third Edition" by Ian Millington. Steering Behavior Algorithms build-off of Kinematic Movement Algorithms but also incorporate acceleration. The implemented behaviors are Seek, Flee, Arrive, Align, Wander, Velocity Matching, Pursue, Evade, Separation, Obstacle Avoidance, and Collision Avoidance. 

-------------------------------------------------------------------------------------------------------------------------------
## *Seek*

Seek takes both an agent's and its target's position and orientation to calculate the needed velocity for the agent. Once the velocity is calculated the agent moves by multiplying the velocity by the agent's max-speed. Best used for when the agent is chasing a target as it will never actually reach its goal, just continue to seek it. If used to go to a stationary point it will cause the agent to wiggle and overshoot an exact point in the world. 

------------------------------------------------------------------------------------------------------------------------------------
## *Flee*

Flee is very similar to Seek however we change one line of code. For this we calculate the direction by doing `this.transform.position - target.transform.position` instead of `target.transform.position - this.transform.position`. 

-------------------------------------------------------------------------------------------------------------------------------
## *Arrive*

Similar to Kinematic Arrive but incorporates Steering Behavior Elements
    - Uses two radii(One for arrival and one for slowing down)
         - radius 1: The Radius of Satisfaction
         - radius 2: Much larger than the Radius of Satisfaction, agent slows down when it passes this radius(algo calculates an appropiate speed); Aka the "SlowRadius"
        
Calculates the distance just same as in the Kinematic Arrive algorithm BUT we combine that with the desired speed for a target velocity. Looks at current velocity of the agent and computes the needed acceleration based on how fast it will reach the target given its current velocity

Intution: If agent has a high velocity → Target velocity will be small and applies newly calculated velocity in the opposite direction to slow it down & vice-versa

-------------------------------------------------------------------------------------------------------------------------------
## *Algin*

Aligns the agent with the target, this how the agent rotates. The other behaviors do not incorporate rotation. 

-------------------------------------------------------------------------------------------------------------------------------
## *Velocity Matching*

Used to make the agent mimic its target's motion. Works best when combined with other behaviors.

-------------------------------------------------------------------------------------------------------------------------------
## *Separation*

Prevents agents from getting to close to one another. Uses a threshold to determine if an agent is too close or not. Will move with respect to a nearby agent.

-------------------------------------------------------------------------------------------------------------------------------
## *Collision Avoidance*

Calculates if agents will collide or not and makes a change in velocity depending on that result. Targets are assumed to be spherical.

-------------------------------------------------------------------------------------------------------------------------------
## *Obstacle Avoidance*

Thea agent casts 1 or 1+ rays out in the direction its moving in, if a ray collides with an obstacle the agent moves to a target that will make it move in a manner such that it will avoid the obstacle. The rays are not infinite, only extend a short distance to detect nearby obstacles
-------------------------------------------------------------------------------------------------------------------------------
# **Delegated Steering Behaviors**
-------------------------------------------------------------------------------------------------------------------------------
## *Pursue*

Tries to predict where the target will be at some time in the future and moves toward that point. This alogirthm calculates the distance between agent and target and calculates the time it will to reach that location, at maximum speed.

Delegates to Seek

-------------------------------------------------------------------------------------------------------------------------------
## *Evade*

Tries to predict where the target will be at some time in the future and moves away from that point. This alogirthm calculates the distance between agent and target and calculates the time it will to get away from that location, at maximum speed.

Delegates to Flee

-------------------------------------------------------------------------------------------------------------------------------
## *Wander*

The agent only moves forward but will move to a target that is randomly placed in front of it, this target changes location over time. WanderRate controls how much we choose a new location to wander to. WanderOrientation is the newly calculated orientation. WanderOffset is used to set how far away the WanderRadius will be at. WanderRadius is an imaginary circle where the agent can select a new location in.

Delegates to Seek
