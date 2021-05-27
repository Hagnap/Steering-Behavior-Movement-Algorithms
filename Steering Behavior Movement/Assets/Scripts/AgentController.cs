using System.Collections;
using System.Collections.Generic;
using System.Linq;
using TMPro;
using UnityEngine;

public class AgentController : MonoBehaviour
{
	#region Variables
	[SerializeField] private List<AgentController> agents;

	[SerializeField] private GameObject rightEye;
	[SerializeField] private GameObject leftEye;
	[SerializeField] private GameObject raycastSpot;

	[SerializeField] private float threshold;
	[SerializeField] private float decay;
	[SerializeField] private float maxSpeed;
	[SerializeField] private float maxAcceleration;
	[SerializeField] private float moveAwaySpeed;
	[SerializeField] private float turnSpeed;
	[SerializeField] private float timeToTarget;
	[SerializeField] private float radiusOfSatisfaction;
	[SerializeField] private float radiusOfSlowingDown;
	[SerializeField] private float radiusOfAvoidance;
	[SerializeField] private float maxPredictionTime;
	[SerializeField] private float maxRotation;
	[SerializeField] private float wanderTurnSpeed;
	[SerializeField] private float wanderAngleDisplacement;
	[SerializeField] private float wanderStrength;
	[SerializeField] private float wanderAngle;
	[SerializeField] private float lookAheadDistance;
	[SerializeField] private float avoidDistance;
	[SerializeField] private float avoidanceOffset;
	[Range(0.0f, 1.0f)]
	[SerializeField] private float eyeSightScalar;

	[Range(0.0f, 1.0f)]
	[SerializeField] private float seekForce;

	[Tooltip("An empty gameobject")]
	[SerializeField] private GameObject target;
	[Tooltip("A rolling sphere")]
	[SerializeField] private GameObject movingTarget;
	private GameObject tempTarget;

	

	private bool runMovementBehavior;
	private bool avoid;

	private float trueMaxSpeed;
	private float orientation = 0f; // Total rotation in the world about the y-axis (vertical axis)
	private float rotation = 0f; // How much rotation each frame
	
	private Vector3 unitVector = Vector3.one;
	private Vector3 velocity = new Vector3();

	private SteeringBehaviorMovement steering;
	# endregion
	
	# region Properties
	public bool RunMovementBehavior
	{
		get { return this.runMovementBehavior; }
		set { this.runMovementBehavior = value; }
	}
	public bool Avoid
	{
		get { return this.avoid; }
		set { this.avoid = value; }
	}
	public List<AgentController> Agents
	{
		get { return this.agents; }
		set { this.agents = value; }
	}
	public float Threshold
	{
		get { return this.threshold; }
		set { this.threshold = value; }
	}
	public float Decay
	{
		get { return this.decay; }
		set { this.decay = value; }
	}
	public float Orientation
	{
		get { return this.orientation; }
		set { this.orientation = value; }
	}

	public float MaxSpeed
	{
		get { return this.maxSpeed; }
		set { this.maxSpeed = value; }
	}
	public float MaxRotation
	{
		get { return this.maxRotation; }
		set { this.maxRotation = value; }
	}

	public float TurnSpeed
	{
		get { return this.turnSpeed; }
		set { this.turnSpeed = value; }
	}

	public float WanderTurnSpeed
	{
		get { return this.wanderTurnSpeed; }
		set { this.wanderTurnSpeed = value; }
	}

	public float TimeToTarget
	{
		get { return this.timeToTarget; }
		set { this.timeToTarget = value; }
	}

	public float RadiusOfSatisfaction
	{
		get { return this.radiusOfSatisfaction; }
		set { this.radiusOfSatisfaction = value; }
	}
	public float RadiusOfSlowingDown
	{
		get { return this.radiusOfSlowingDown; }
		set { this.radiusOfSlowingDown = value; }
	}
	public float RadiusOfAvoidance
	{
		get { return this.radiusOfAvoidance; }
		set { this.radiusOfAvoidance = value; }
	}
	public float MaxPredictionTime
	{
		get { return this.maxPredictionTime; }
		set { this.maxPredictionTime = value; }
	}
	public float WanderAngleDisplacement
	{
		get { return this.wanderAngleDisplacement; }
		set { this.wanderAngleDisplacement = value; }
	}
	public float WanderStrength
	{
		get { return this.wanderStrength; }
		set { this.wanderStrength = value; }
	}
	public float WanderAngle
	{
		get { return this.wanderAngle; }
		set { this.wanderAngle = value; }
	}
	public float MaxAcceleration
	{
		get { return this.maxAcceleration; }
		set { this.maxAcceleration = value; }
	}
	public float MoveAwaySpeed
	{
		get { return this.moveAwaySpeed; }
		set { this.moveAwaySpeed = value; }
	}
	public float SeekForce
	{
		get { return this.seekForce; }
		set { this.seekForce = value; }
	}
	public float LookAheadDistance
	{
		get { return this.lookAheadDistance; }
		set { this.lookAheadDistance = value; }
	}
	public float AvoidDistance
	{
		get { return this.avoidDistance; }
		set { this.avoidDistance = value; }
	}
	public float EyeSightScalar
	{
		get { return this.eyeSightScalar; }
		set { this.eyeSightScalar = value; }
	}

	public float AvoidanceOffset
	{
		get { return this.avoidanceOffset; }
		set { this.avoidanceOffset = value; }
	}

	public GameObject MovingTarget
	{
		get { return this.movingTarget; }
		set { this.movingTarget = value; }
	}

	public GameObject Target
	{
		get { return this.target; }
		set { this.target = value; }
	}
	public GameObject TempTarget
	{
		get { return this.tempTarget; }
		set { this.tempTarget = value; }
	}

	public GameObject RightEyeLocation
	{
		get { return this.rightEye; }
	}

	public GameObject LeftEyeLocation
	{
		get { return this.leftEye; }
	}
	public GameObject RaycastSpot
	{
		get { return this.raycastSpot; }
	}
	# endregion
	
	# region Unity Methods
	// Start is called before the first frame update
    void Start()
    {
		//kinematic = this.GetComponent<KinematicMovement>();
		steering = this.GetComponent<SteeringBehaviorMovement>();
		tempTarget = new GameObject();

		if (seekForce == 0.0f)
			seekForce = 1f;

		foreach(AgentController a in Object.FindObjectsOfType<AgentController>())
		{
			if(a != this)
			{
				this.agents.Add(a);
			}
		}

		this.runMovementBehavior = true;
		this.avoid = false;
	}

    // Update is called at a fixed rate
    void FixedUpdate()
    {
		if(runMovementBehavior)
		{

			//steering.runSteeringBehaviorSeek(movingTarget);
			//steering.runSteeringBehaviorFlee(movingTarget);
			//steering.runSteeringBehaviorArrive(target);
			steering.runSteeringBehaviorPurse(movingTarget);
			//steering.runSteeringBehaviorEvade(movingTarget);
			//steering.runSteeringBehaviorWander();
			//steering.runSteeringBehaviorAlign(movingTarget);
			//steering.runSteeringBehaviorVelocityMatch(movingTarget);
			//steering.runSteeringBehaviorSeparation();

			//steering.runSteeringBehaviorCollisionAvoidance();
			//steering.runSteeringBehaviorObstacleAvoidance();
		}

		if(avoid)
		{
			Debug.Log("** AVOIDING **");
			steering.runSteeringBehaviorArrive(tempTarget);

			if(Vector3.Distance(this.transform.position, tempTarget.transform.position) < radiusOfSatisfaction)
			{
				runMovementBehavior = true;
				avoid = false;
			}
		}


		steering.runSteering();

		updateUnitVector();
	}
	#endregion

	#region Custom Methods
	public void updateUnitVector()
	{
		///Debug.Log("Unit Vector");
		///Debug.Log("X: " + Mathf.Sin(Mathf.Deg2Rad * this.transform.position.x));
		///Debug.Log("Z: " + Mathf.Cos(Mathf.Deg2Rad * this.transform.position.z));

		// Right Handed Movement
		unitVector.x = Mathf.Sin(Mathf.Deg2Rad * this.transform.position.x);
		unitVector.z = Mathf.Cos(Mathf.Deg2Rad * this.transform.position.z);
		
		// Left Handed Movement
		//unitVector.x = -Mathf.Sin(Mathf.Deg2Rad * this.transform.position.x);
		//unitVector.z = Mathf.Cos(Mathf.Deg2Rad * this.transform.position.z);

		orientation = unitVector.magnitude;
		///Debug.Log("Orientation: " + orientation);
	}

	public float generateRandomBinomial()
	{
		return Random.value - Random.value;
	}

	// Used with flocking
	public void runSpeedReser()
	{
		maxSpeed = trueMaxSpeed;
	}
	# endregion
	
	# region IEnumerators
	
	# endregion
}
