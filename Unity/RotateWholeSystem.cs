using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

/// MouseLook rotates the transform based on the mouse delta.
/// Minimum and Maximum values can be used to constrain the possible rotation

/// To make an FPS style character:
/// - Create a capsule.
/// - Add the MouseLook script to the capsule.
///   -> Set the mouse look to use LookX. (You want to only turn character but not tilt it)
/// - Add FPSInputController script to the capsule
///   -> A CharacterMotor and a CharacterController component will be automatically added.

/// - Create a camera. Make the camera a child of the capsule. Reset it's transform.
/// - Add a MouseLook script to the camera.
///   -> Set the mouse look to use LookY. (You want the camera to tilt up and down like a head. The character already turns.)
//[AddComponentMenu("Camera-Control/Mouse Look")]
public class RotateWholeSystem : MonoBehaviour {
	public GameObject r_palm;//, LeapRig;
	public float sensitivityX = 15F;
	public float minimumX = -360F;
	public float maximumX = 360F;
	float speed = 0.01f;
	float rot = 0F;

	//CharacterController controller;

	void Update ()
	{
		OVRInput.Update();
		//MC_raycast mc = GameObject.Find("Main Camera").GetComponent<MC_raycast>();


		//controller = GetComponent<CharacterController>();
		//controller.center = new Vector3(LeapRig.transform.position.x, 0.931f, LeapRig.transform.position.z);

		if (OVRInput.Get(OVRInput.Button.Two))
		{
			rot = 0.05f;
		}


		else if (Input.GetButton("Fire3"))
		{
			rot = -0.05f;
		}
		else
			rot = 0f;

		float rotationX = transform.localEulerAngles.y + rot * sensitivityX;
		transform.localEulerAngles = new Vector3(0, rotationX, 0);

		//distortion for sampled orientation
		/*
		if(r_palm.activeInHierarchy)
		{
			transform.rotation = Quaternion.Lerp(mc.from, mc.to, Time.time * speed);
		}
		*/

	}

	void Start ()
	{
		// Make the rigid body not change rotation
		if (GetComponent<Rigidbody>())
		GetComponent<Rigidbody>().freezeRotation = true;


	}
}
