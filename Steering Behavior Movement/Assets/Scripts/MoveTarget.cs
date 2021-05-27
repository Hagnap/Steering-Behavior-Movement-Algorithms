using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveTarget : MonoBehaviour
{
	#region Variables
	[SerializeField] private float speed;
	private float orientation;
	private Vector3 unitVector;

	private Rigidbody rb;
	# endregion
	
	# region Properties
	public float Speed
	{
		get { return this.speed; }
		set { this.speed = value; }
	}

	public float Orientation
	{ 
		get { return this.orientation; }
		set { this.orientation = value; }
	}
	# endregion
	
	# region Unity Methods
	// Start is called before the first frame update
    void Start()
    {
		rb = this.GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
		runMoveTarget();

	}
	#endregion

	#region Custom Methods
	public void runMoveTarget()
	{
		rb.velocity = Vector3.forward * speed;
	}
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
		Debug.Log(orientation);

		///Debug.Log("Orientation: " + orientation);
	}
	#endregion
}
