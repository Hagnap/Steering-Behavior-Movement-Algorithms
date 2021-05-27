using UnityEngine;

public class SteeringBehaviorMovement : MonoBehaviour
{
	#region Variables
	private Quaternion rotation;
	private float distance;
	private float calculatedSpeed;
	private float calculatedOrientation;
	private float calculatedRotation;
	private float initialAngle;
	private Vector3 direction;
	private Vector3 velocity;
	private Vector3 targetVelocity;
	private Vector3 targetPosition;

	private AgentController agent;
	private Rigidbody rb;
    #endregion

    #region Unity Methods
    // Start is called before the first frame update
    void Start()
    {
		agent = this.GetComponent<AgentController>();
		rb = this.GetComponent<Rigidbody>();

		initialAngle += Random.Range(0.0f, Mathf.PI);
		agent.WanderAngle = initialAngle;
		rb.velocity = new Vector3(Mathf.Cos(initialAngle) * agent.TurnSpeed, 0f, Mathf.Sin(initialAngle) * agent.TurnSpeed);
	}

    // Update is called once per frame
    void Update()
    {
		// Debug.Log("Orientation: " + (float)Mathf.Atan2(-rb.velocity.x, rb.velocity.z));
	}
	#endregion

	#region Steering Behavior Methods
	public void runSteering()
	{
		// Update position & Orientation
		transform.position += rb.velocity * Time.deltaTime;
		transform.forward += rb.angularVelocity * Time.deltaTime;

		// Update Velocity and Rotation
		rb.velocity *= Time.deltaTime;
		rb.angularVelocity *= Time.deltaTime;

		// Check for speeding and clip
		if (rb.velocity.magnitude > agent.MaxSpeed)
		{
			rb.velocity.Normalize();
			rb.velocity *= agent.MaxSpeed;
		}
	}

	/// <summary>
	/// Similar to the Kinematic Seek Algo, just incorporates acceleration and does NOT alter the orientation.
	/// Acceleration is constantly at its max, doesn't change even if it's at its target destination.
	/// </summary>
	/// <param name="target">What the agent wants to go towards</param>
	public void runSteeringBehaviorSeek(GameObject target)
	{
		// Gets the direction & distance to the target 
		direction = target.transform.position - this.transform.position;
		distance = direction.magnitude;

		//velocity = velocity.normalized;
		direction.Normalize();

		// Sets the velocity, at full speed
		direction *= agent.MaxAcceleration * agent.SeekForce;

		Debug.DrawLine(agent.transform.position, target.transform.position);

		// Moves the agent
		rb.velocity = direction;
		rb.angularVelocity = Vector3.zero;
	}

	/// <summary>
	/// Similar to Kinematic Flee but incorporates same concpets as Steering Behavior Seek
	/// </summary>
	/// <param name="target">What the agent moves away from</param>
	public void runSteeringBehaviorFlee(GameObject target)
	{
		// Gets the direction & distance to the target 
		direction = this.transform.position - target.transform.position;
		distance = direction.magnitude;

		//velocity = velocity.normalized;
		direction.Normalize();

		// Sets the velocity, at full speed
		direction *= agent.MaxAcceleration;

		Debug.DrawLine(agent.transform.position, target.transform.position);

		// Moves the agent
		rb.velocity = direction;
		rb.angularVelocity = Vector3.zero;
	}

	/// <summary>
	/// Similar to Kinematic Arrive but incorporates Steering Behavior Elements
	///     - Uses two radii(One for arrival and one for slowing down)
	///         - radius 1: The Radius of Satisfaction
	///         - radius 2: Much larger than the Radius of Satisfaction, agent slows down when it passes this radius(algo calculates an appropiate speed)
	///                     Aka the "SlowRadius"
	///
	/// Calculates the distance just same as in the Kinematic Arrive algorithm BUT we combine that with the desired speed for a target velocity
	/// Looks at current velocity of the agent and computes the needed acceleration based on how fast it will reach the target given its current velocity
	/// 
	/// Intution: 
	///		If agent has a high velocity → Target velocity will be small and applies newly calculated velocity in the opposite direction to slow it down & vice-versa
	/// </summary>
	/// <param name="target">What the agent wants to go towards</param>
	public void runSteeringBehaviorArrive(GameObject target)
	{
		// Gets the direction to the target
		direction = target.transform.position - this.transform.position;
		distance = direction.magnitude;
		//Debug.Log("Distance: " + distance);

		// If the agent has reached the radiusOfSatisfaction → Stop
		if (distance < agent.RadiusOfSatisfaction)
		{
			///Debug.Log("Stopped");
			return;
		}

		// If agent hasn't reached the SlowRadius → Move at max speed; Otherwise, Reached the SlowDownRadius → Calculate a scaled speed
		if (distance > agent.RadiusOfSlowingDown)
		{
			// Updates targetSpeed to max speed
			calculatedSpeed = agent.MaxSpeed;
			///Debug.Log("Fast");
		}
		else
		{
			///Debug.Log("Slowing Down");

			// Updates targetSpeed to a calculated value → targetSpeed = maxSpeed * distance / slowDownRadius
			calculatedSpeed = agent.MaxSpeed * direction.magnitude / agent.RadiusOfSlowingDown;
		}

		//Debug.Log("Calculated Speed " + calculatedSpeed);

		// targetVelocity combines speed & direction
		targetVelocity = direction;
		targetVelocity.Normalize();
		targetVelocity *= calculatedSpeed;

		// Acceleration tries to get the target velocity
		velocity = targetVelocity - rb.velocity;
		velocity /= agent.TimeToTarget; // Controls the agent's deceleration

		// Checks if the agent is moving 'too fast'
		if (velocity.magnitude > agent.MaxAcceleration)
		{
			velocity.Normalize();
			velocity *= agent.MaxAcceleration;
		}

		Debug.DrawLine(agent.transform.position, target.transform.position);

		// Updates the agent's velocity
		rb.velocity = velocity;
		rb.angularVelocity = Vector3.zero;
	}

	/// <summary>
	/// Aligns the agent with the target.
	/// 
	/// Works but is not implemented properly...
	/// </summary>
	/// <param name="target">What we want the agent to face towards.</param>
	public void runSteeringBehaviorAlign(GameObject target)
	{
		/*
		float rot = 0f;
		Vector3 location = Vector3.one;
		location.x = Mathf.Sin(Mathf.Deg2Rad * target.transform.position.x);
		location.z = Mathf.Cos(Mathf.Deg2Rad * target.transform.position.z);
		calculatedOrientation = location.magnitude;
		calculatedRotation  = calculatedOrientation - agent.Orientation;

		calculatedRotation = runMapToRange(calculatedRotation);
		float size = Mathf.Abs(calculatedRotation);

		if(size < agent.RadiusOfSatisfaction)
		{
			return;
		}

		if(size > agent.RadiusOfSlowingDown)
		{
			rot = agent.MaxRotation;	
		}
		else
		{
			rot = agent.MaxRotation * size / agent.RadiusOfSlowingDown;
		}

		rot *= calculatedRotation / size;

		rb.angularVelocity = new Vector3(0f, rot, 0f) - this.transform.eulerAngles;
		*/

		// Face the direction we want the agent to move towards
		Debug.DrawLine(this.transform.position, target.transform.position);
		rotation = Quaternion.LookRotation(target.transform.position - agent.transform.position);

		direction = target.transform.position - agent.transform.position;
		distance = direction.magnitude;

		// Distance impacts turn speed
		if(distance < agent.RadiusOfSatisfaction * 2.0f) // If close enough, turn half speed
		{
			this.transform.rotation = Quaternion.Lerp(this.transform.rotation, rotation, (agent.TurnSpeed * 0.5f) * Time.deltaTime);
			///Debug.Log("Slow Turn " + agent.TurnSpeed * 0.5f);
		}
		if(distance > agent.RadiusOfSlowingDown) // If far away, turn at normal speed
		{
			this.transform.rotation = Quaternion.Lerp(this.transform.rotation, rotation, (agent.TurnSpeed * 1.0f) * Time.deltaTime);
			///Debug.Log("Fast Turn " + agent.TurnSpeed * 1.0f);
		}
		else // if not too close but not far away, turn at 75% speed
		{
			this.transform.rotation = Quaternion.Lerp(this.transform.rotation, rotation, (agent.TurnSpeed * 0.75f) * Time.deltaTime);
			///Debug.Log("Medium Turn " + agent.TurnSpeed * 0.75f);
		}

	}

	/// <summary>
	/// Used to make the agent mimic its target's motion. Works best when combined with other behaviors.
	/// </summary>
	/// <param name="target">The object we want the agent to match its velocity with.</param>
	public void runSteeringBehaviorVelocityMatch(GameObject target)
	{

		direction = (target.transform.position - agent.transform.position);
		direction /= agent.TimeToTarget;
		distance = direction.magnitude;

		if (distance > agent.MaxAcceleration)
		{
			direction.Normalize();
			direction *= agent.MaxAcceleration;

			rb.velocity = direction;
			rb.angularVelocity = Vector3.zero;
		}
	}

	/// <summary>
	/// Prevents agents from getting to close to one another. Uses a threshold to determine if an agent is too close or not. Will
	/// move with respect to a nearby agent.
	/// </summary>
	public void runSteeringBehaviorSeparation()
	{
		float strength = 0f;

		// Loop through each agent
		foreach (AgentController neighbor in agent.Agents)
		{
			// Gets the distance & direction to the target
			direction = this.transform.position - neighbor.transform.position;
			distance = direction.magnitude;

			// Check distance to the target
			if (distance < agent.Threshold)
			{
				// Calculate strength of repulsion via Inverse Square Law
				strength = Mathf.Min(agent.Decay / (distance * distance), agent.MaxAcceleration);

				// Add the acceleration
				direction.Normalize();
				rb.velocity += (strength * direction);
			}
		}
	}

	/// <summary>
	/// Calculates if agents will collide or not and makes a change in velocity depending on that result. Targets are assumed to be spherical.
	/// </summary>
	public void runSteeringBehaviorCollisionAvoidance()
	{
		float shortestTime = Mathf.Infinity;

		GameObject firstTarget = null;
		float firstMinSeparation = 0f;
		float firstDistance = 0f;
		Vector3 relativePos;
		Vector3 firstRelativePosition = Vector3.zero;
		Vector3 firstRelativeVelocity = Vector3.zero;

		// Loop through each target (target → Nearby/Local agent)
		foreach(AgentController target in agent.Agents)
		{
			// Calculate time to collision
			Vector3 relativePosition = this.transform.position - target.transform.position;
			Vector3 relativeVelocity = this.GetComponent<Rigidbody>().velocity - target.GetComponent<Rigidbody>().velocity;
			float relativeSpeed = relativeVelocity.magnitude;

			float timeToCollision = Vector3.Dot(relativePosition, relativeVelocity) / (relativeSpeed * relativeSpeed);

			// Check for a potential collision
			distance = relativePosition.magnitude;
			float minSeparation = distance - relativeSpeed * timeToCollision;
			Debug.Log(this + " minSeparation: " + minSeparation);

			
			if(minSeparation < agent.RadiusOfAvoidance)
			{
				// Check if its the shortest
				if (timeToCollision > 0 && timeToCollision < shortestTime)
				{
					shortestTime = timeToCollision;
					firstTarget = target.gameObject;
					firstMinSeparation = minSeparation;
					firstDistance = distance;
					firstRelativePosition = relativePosition;
					firstRelativeVelocity = relativeVelocity;
				}
			}

			else
			{
				Debug.Log("Not avoiding..");

				//rb.velocity = Vector3.zero;
				//rb.angularVelocity = Vector3.zero;

				continue;
			}
			
		}

		
		// Calculate the steering
		// If no target → Exit function
		if(firstTarget == null)
		{
			//Debug.Log(firstTarget);
			rb.velocity = Vector3.zero;
			rb.angularVelocity = Vector3.zero;
			return;
		}
		else
		{
			// Otherwise, going to hit exatcly or already colliding
			if (firstMinSeparation <= 0 || firstDistance < agent.RadiusOfAvoidance)
			{
				relativePos = this.transform.position - firstTarget.transform.position;
				Debug.DrawLine(this.transform.position, relativePos);

				relativePos.Normalize();

				rb.velocity = relativePos * agent.MoveAwaySpeed;
				rb.angularVelocity = Vector3.zero;
			}
			else // Otherwise calculate the future relative position
			{
				relativePos = firstRelativePosition + firstRelativeVelocity * shortestTime;
				Debug.DrawLine(this.transform.position, relativePos);

				relativePos.Normalize();

				rb.velocity = relativePos * agent.MoveAwaySpeed;
				rb.angularVelocity = Vector3.zero;
			}
		}
		
	}

	/// <summary>
	/// Used for avoiding non-spherical objects. Uses raycasts to detect obstacles and will move in the opposite direction if 
	/// an obstacle is detected via a raycast. 
	/// </summary>
	public void runSteeringBehaviorObstacleAvoidance()
	{
		int layerMask = 8;

		// Inverts the bitmask -- To ignore every layer but layer 8
		layerMask = ~layerMask;

		// Ray at the center of the body
		Vector3 ray1 = agent.RaycastSpot.transform.forward;
		ray1.Normalize();
		ray1 *=  agent.LookAheadDistance;

		// Ray at the center of the right eye
		Vector3 ray2 = agent.RightEyeLocation.transform.forward;
		ray2.Normalize();
		ray2 *= agent.LookAheadDistance * agent.EyeSightScalar;

		// Ray at the center of the left eye
		Vector3 ray3 = agent.LeftEyeLocation.transform.forward;
		ray3.Normalize();
		ray3 *= agent.LookAheadDistance * agent.EyeSightScalar;

		RaycastHit hit;

		// Draws the Rays
		Debug.DrawRay(agent.RaycastSpot.transform.position, ray1, Color.red);
		//Debug.DrawRay(agent.RightEyeLocation.transform.position + new Vector3(0f, 0.425f, 0f), ray2, Color.red);
		Debug.DrawRay(agent.RightEyeLocation.transform.position, ray2, Color.red);
		//Debug.DrawRay(agent.LeftEyeLocation.transform.position + new Vector3(0f, 0.425f, 0f), ray3, Color.red);
		Debug.DrawRay(agent.LeftEyeLocation.transform.position, ray3, Color.red);

		// Finds the obstacle
		// Obstacle detected straight ahead
		if (Physics.Raycast(agent.RaycastSpot.transform.position, ray1, out hit, agent.AvoidDistance, layerMask))
		{
			if (hit.transform.gameObject.layer == 8)
			{
				Debug.Log("Obstacle detected ahead. Avoiding...");

				// Goes back and to the side (side movement is weighted more)
				///formationManager.SetAnchorOffset(new Vector3(-anchorOffset, 0f, -anchorOffset));
				agent.TempTarget.transform.position = 
					new Vector3(agent.transform.position.x - (agent.AvoidanceOffset * 1.5f), 0f, agent.transform.position.z - agent.AvoidanceOffset);
				///move = false;
				agent.RunMovementBehavior = false;
				///goToAnchor = true;
				agent.Avoid = true;
			}
		}

		// Obstacle detected to the right
		if (Physics.Raycast(agent.RightEyeLocation.transform.position, ray2, out hit, agent.AvoidDistance, layerMask))
		{
			if (hit.transform.gameObject.layer == 8)
			{
				Debug.Log("Obstacle detected to the right. Avoiding...");

				// Goes back and to the side (side movement is weighted more)
				///formationManager.SetAnchorOffset(new Vector3(-anchorOffset, 0f, -anchorOffset));
				agent.TempTarget.transform.position =
					new Vector3(agent.transform.position.x - agent.AvoidanceOffset, 0f, agent.transform.position.z);
				///move = false;
				agent.RunMovementBehavior = false;
				///goToAnchor = true;
				agent.Avoid = true;
			}
		}

		// Obstacle detected to the left
		if (Physics.Raycast(agent.LeftEyeLocation.transform.position, ray3, out hit, agent.AvoidDistance, layerMask))
		{
			if (hit.transform.gameObject.layer == 8)
			{
				Debug.Log("Obstacle detected to the left. Avoiding...");

				// Goes back and to the side (side movement is weighted more)
				///formationManager.SetAnchorOffset(new Vector3(-anchorOffset, 0f, -anchorOffset));
				agent.TempTarget.transform.position =
					new Vector3(agent.transform.position.x + agent.AvoidanceOffset, 0f, agent.transform.position.z);
				///move = false;
				agent.RunMovementBehavior = false;
				///goToAnchor = true;
				agent.Avoid = true;
			}
		}
	}

	/*
	 * DELEGATED STEERING BEHAVIORS
	 */
	/// <summary>
	/// The agent will move in a forward direction to its target. 
	/// WanderRate controls how much we choose a new location to wander to.
	/// WanderOrientation is the newly calculated orientation.
	/// WanderOffset is used to set how far away the WanderRadius will be at.
	/// WanderRadius is an imaginary circle where the agent can select a new location in.
	/// </summary>
	public void runSteeringBehaviorWander()
	{
		/*
		float orientation = rb.rotation.eulerAngles.y * Mathf.Deg2Rad;

		//
		agent.WanderOrientation = generateRandomBinomial() * agent.WanderRate;

		agent.Target.transform.eulerAngles = new Vector3(agent.Target.transform.eulerAngles.x, agent.Target.transform.eulerAngles.y + agent.WanderOrientation, agent.Target.transform.eulerAngles.z);

		//
		calculatedOrientation = agent.WanderOrientation + orientation;

		//
		agent.WanderOffset = Random.Range(-1.5f, 1.5f);

		//
		targetPosition = this.transform.position + (2f * transform.forward) + (new Vector3(Mathf.Cos(-orientation), 0, Mathf.Sin(-orientation)) * agent.WanderOffset);

		//
		targetPosition = targetPosition + (transform.forward) - (2f * transform.right) + (new Vector3(Mathf.Cos(-calculatedOrientation), 0, Mathf.Sin(-calculatedOrientation)) * agent.WanderRadius);

		Debug.DrawLine(transform.position, targetPosition);

		agent.Target.transform.position = targetPosition;

		runSteeringBehaviorSeek(agent.Target);
		*/

		agent.WanderAngle += Random.Range(-agent.WanderAngleDisplacement, agent.WanderAngleDisplacement);

		Vector3 displacementForce = new Vector3(Mathf.Cos(agent.WanderAngle * agent.WanderStrength), 0f, Mathf.Sin(agent.WanderAngle * agent.WanderStrength));


		agent.Target.transform.position = (rb.velocity + displacementForce).normalized * (agent.TurnSpeed / 2.5f);

		Debug.DrawLine(agent.transform.position, agent.Target.transform.position);

		runSteeringBehaviorSeek(agent.Target);
	}

	/// <summary>
	/// Given a suitable time to predict, predict the future position of the target given its velocity. Once a prediction is made, pass the prediction into Seek(). 
	/// The higher the MaxPredictionTime value is, the less accurate the predicted location is.
	/// 
	/// Assumptions:
	///		- Target will follow its path for some time
	///		- Target will deviate from its path at some point (cant predict for too long)
	///		
	/// Delegates to Seek.
	/// </summary>
	/// <param name="target">What we want the agent to chase</param>
	public void runSteeringBehaviorPurse(GameObject target)
	{
		// Sets the maximum prediction time
		float speed;
		float predictionTime;

		// Gets the direction to the target
		direction = target.transform.position - this.transform.position;
		distance = direction.magnitude;
		Debug.Log("Towards Direction: " + distance);

		// Calculate agent speed
		speed = rb.velocity.magnitude;

		// Check if the speed gives a reasonable prediction time
		if (speed <= distance / agent.MaxPredictionTime)
		{
			predictionTime = agent.MaxPredictionTime;
		}
		// Otherwise, calculate the prediction time
		else
		{
			predictionTime = distance / speed;
		}

		// Put the target together (uses the empty object `target` in the Unity Scene, just reusing an object)
		agent.Target.transform.position = target.transform.position;
		agent.Target.transform.position += target.GetComponent<Rigidbody>().velocity * predictionTime;

		Debug.DrawLine(this.transform.position, agent.Target.transform.position);

		runSteeringBehaviorSeek(agent.Target);
	}

	/// <summary>
	/// The opposite action for Pursue. Essentially the same as Purse but moves away from the target.
	/// 
	/// Delegates to Flee.
	/// </summary>
	/// <param name="target">What we want the agent to get away from</param>
	public void runSteeringBehaviorEvade(GameObject target)
	{
		// Sets the maximum prediction time
		float speed;
		float predictionTime;

		// Gets the direction to the target
		direction = target.transform.position - this.transform.position;
		distance = direction.magnitude;
		Debug.Log("Towards Direction: " + distance);

		// Calculate agent speed
		speed = rb.velocity.magnitude;

		// Check if the speed gives a reasonable prediction time
		if (speed <= distance / agent.MaxPredictionTime)
		{
			predictionTime = agent.MaxPredictionTime;
		}
		// Otherwise, calculate the prediction time
		else
		{
			predictionTime = distance / speed;
		}

		// Put the target together (uses the empty object `target` in the Unity Scene, just reusing an object)
		agent.Target.transform.position = target.transform.position;
		agent.Target.transform.position += target.GetComponent<Rigidbody>().velocity * predictionTime;

		Debug.DrawLine(this.transform.position, agent.Target.transform.position);

		runSteeringBehaviorFlee(agent.Target);
	}
	#endregion

	#region Custom Methods
	private float runMapToRange(float rotation)
	{
		rotation %= 360.0f;

		if(rotation < 0.0f)
		{
			rotation += 360.0f;
		}
		else
		{
			rotation -= 360.0f;
		}

		return rotation;
	}

	float generateRandomBinomial()
	{
		return (Random.value - Random.value);
	}
	#endregion
}
