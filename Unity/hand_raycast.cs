using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class hand_raycast : MonoBehaviour {
	public Transform target_wall, sphere_test;
	private RaycastHit hit;
	public GameObject proxy_base;

	public float pose_x, pose_y, pose_z;
	// Use this for initialization
	void Start () {

	}

	// Update is called once per frame
	void Update () {

		if(Physics.Raycast(transform.position, transform.forward, out hit, 20.0f))
		{
			Debug.DrawRay(transform.position, transform.forward * 20.0f, Color.red);
			target_wall = hit.transform;
			Debug.DrawRay(hit.point, hit.normal, Color.green); //wall surface normal

			Vector3 relPose = target_wall.transform.position - transform.position;
			float d = Mathf.Sqrt(relPose.x*relPose.x + relPose.z*relPose.z);

			float xxx = ((relPose.x/d)*target_wall.transform.lossyScale.x)/2;
			float zzz = ((relPose.z/d)*target_wall.transform.lossyScale.x)/2;


			//predicted colision point -> sphere_test position
			float p_col_x = target_wall.transform.position.x - xxx;
			float p_col_z = target_wall.transform.position.z - zzz;

			sphere_test.transform.position = new Vector3(p_col_x, transform.position.y, p_col_z);

			Vector3 pose = hit.point - proxy_base.transform.position;

			pose_x = pose.x;
			pose_y = pose.y;
			pose_z = pose.z;


			float x1 = Mathf.Sin(proxy_base.transform.eulerAngles.y*Mathf.Deg2Rad);
			float y1 = Mathf.Cos(proxy_base.transform.eulerAngles.y*Mathf.Deg2Rad);
			//float x2 = relPose.x;
			//float y2 = relPose.z;

			float x2 = -relPose.z;
			float y2 = -relPose.x;

			//Debug.Log("x2, y2:" + x2 + ", " + y2);


			float d1 = Mathf.Sqrt(Mathf.Pow(x1,2) + Mathf.Pow(y1,2));
			float d2 = Mathf.Sqrt(Mathf.Pow(x2,2) + Mathf.Pow(y2,2));

			//Vector3 from = new Vector3(x1, y1, 0);
			//Vector3 to = new Vector3(x2, y2, 0);
			//Vector3 v = to - from;


			//float degree13 = Mathf.Atan2(v.y, v.x) * Mathf.Rad2Deg;
			//Debug.Log("degeree:" + degree13);

			float degree13 = (180 / Mathf.PI) *Mathf.Asin((x1*y2 - y1*x2)/(d1*d2));

			Debug.Log("degeree:" + degree13);

		}
	}
}
