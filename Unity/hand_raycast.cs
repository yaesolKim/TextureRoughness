using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class hand_raycast : MonoBehaviour {
	public Transform target_wall_hand;
	private RaycastHit hit;
	public GameObject r_palm, proxy_base;

	public float pose_x, pose_y, pose_z;
	// Use this for initialization
	void Start () {

	}

	// Update is called once per frame
	void Update () {

		if(r_palm.activeInHierarchy && Physics.Raycast(transform.position, transform.forward, out hit, 20.0f))
		{
			Debug.DrawRay(transform.position, transform.forward * 20.0f, Color.red);
			target_wall_hand = hit.transform;
			Debug.DrawRay(hit.point, hit.normal, Color.green); //wall surface normal

			Vector3 pose = hit.point - proxy_base.transform.position;

			pose_x = pose.x;
			pose_y = pose.y;
			pose_z = pose.z;

			Debug.Log("POSE:" + pose_x + ", " + pose_y + ", " + pose_z);

		}
	}
}
