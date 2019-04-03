using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class hand_col : MonoBehaviour {

	public GameObject cam;
	// Use this for initialization
	void Start () {

	}

	// Update is called once per frame
	void Update () {
		//float x = cam.transform.position.x;
		//float z = cam.transform.position.z;

		transform.position = new Vector3(cam.transform.position.x, cam.transform.position.y-0.6f, cam.transform.position.z);
		transform.eulerAngles = new Vector3(0f, cam.transform.eulerAngles.y, 0f);

	}

	void OnTriggerEnter(Collider col)
	{
		Debug.Log("Collision Detected");
		/*
		if (col.tag == "hand")
		{
			Debug.Log("connect button");
		}*/
	}


}
