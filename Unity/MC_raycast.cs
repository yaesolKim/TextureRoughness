//attatch this script to the main camera
//using raycast, caculate orientation of the target wall
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System; //Math

public class MC_raycast : MonoBehaviour {
	public Transform target_wall, sphere_test;
	private RaycastHit hit;
	public GameObject r_palm, proxy_base, WholeSystem;
	private float[] ori_s = { 315.0f, 330.0f, 345.0f, 0.0f, 15.0f, 30.0f, 45.0f }; //sampled orientations
	public float ori_c, ori, p_x, p_y, p_z, wall_ori;
	private float degree;
	private float old_degree = -1.0f;
	public float speed = 0.001F;


	public Quaternion from, to;

	// Use this for initialization
	void Start () {
	}

	// Update is called once per frame
	void Update () {

		if(r_palm.activeInHierarchy && Physics.Raycast(transform.position, transform.forward, out hit, 20.0f))
		{
			Debug.DrawRay(transform.position, transform.forward * 20.0f, Color.red);
			target_wall = hit.transform;

			Vector3 relPose = target_wall.transform.position - r_palm.transform.position;
			float degree = target_wall.transform.eulerAngles.y - proxy_base.transform.eulerAngles.y;

			if(degree < 0)
			{
				degree += 360;
			}

			if(old_degree != degree)
			{
				//find nearest orientation
				float min = 15;
				for (int i = 0; i < ori_s.Length; i++)
				{
					if (Math.Abs(ori_s[i] - degree) < min)
					{
						min = Math.Abs(ori_s[i] - degree);
						ori_c = ori_s[i];
					}
				}
				if (Math.Abs(360.0f - degree) < min)
				{
					ori_c = 0;
				}

				from = WholeSystem.transform.rotation;
				to = Quaternion.Euler(WholeSystem.transform.eulerAngles + new Vector3(0, degree-ori_c, 0));
			}

			//////////////// collide point calculattion
			//wall_ori = target_wall.transform.eulerAngles.y;

			//float d = Mathf.Sqrt(relPose.x*relPose.x + relPose.z*relPose.z);

			//float costh = relPose.x/d;
			//float sinth = relPose.z/d;

			//float theta = 90 - wall_ori;
			//float cos = Mathf.Cos(theta*Mathf.Deg2Rad)*costh + Mathf.Sin(theta*Mathf.Deg2Rad)*sinth;

			////predicted colision point -> sphere_test position
			//float p_col_x = target_wall.transform.position.x - d*cos*Mathf.Sin(wall_ori*Mathf.Deg2Rad);
			//float p_col_z = target_wall.transform.position.z - d*cos*Mathf.Cos(wall_ori*Mathf.Deg2Rad);

			//sphere_test.transform.position = new Vector3(p_col_x, r_palm.transform.position.y, p_col_z);
			//////////////// end of collision point calculattion


			if(target_wall.gameObject.tag == "static_wall")
			{
				//Debug.Log("static haptic");
				wall_ori = target_wall.transform.eulerAngles.y;

				float d = Mathf.Sqrt(relPose.x*relPose.x + relPose.z*relPose.z);

				float costh = relPose.x/d;
				float sinth = relPose.z/d;

				float theta = 90 - wall_ori;
				float cos = Mathf.Cos(theta*Mathf.Deg2Rad)*costh + Mathf.Sin(theta*Mathf.Deg2Rad)*sinth;

				//predicted colision point -> sphere_test position
				float p_col_x = target_wall.transform.position.x - d*cos*Mathf.Sin(wall_ori*Mathf.Deg2Rad);
				float p_col_z = target_wall.transform.position.z - d*cos*Mathf.Cos(wall_ori*Mathf.Deg2Rad);
				sphere_test.transform.position = new Vector3(p_col_x, r_palm.transform.position.y, p_col_z);
				ori = ori_c;


				WholeSystem.transform.rotation = Quaternion.Lerp(from, to, Time.time * speed);
			}
			else if(target_wall.gameObject.tag == "dynamic_wall")
			{
				//Debug.Log("dynamice haptic");
				wall_ori = target_wall.transform.eulerAngles.y;

				float d = Mathf.Sqrt(relPose.x*relPose.x + relPose.z*relPose.z);

				float costh = relPose.x/d;
				float sinth = relPose.z/d;

				float theta = 90 - wall_ori;
				float cos = Mathf.Cos(theta*Mathf.Deg2Rad)*costh + Mathf.Sin(theta*Mathf.Deg2Rad)*sinth;

				//predicted colision point -> sphere_test position
				float p_col_x = target_wall.transform.position.x - d*cos*Mathf.Sin(wall_ori*Mathf.Deg2Rad);
				float p_col_z = target_wall.transform.position.z - d*cos*Mathf.Cos(wall_ori*Mathf.Deg2Rad);
				sphere_test.transform.position = new Vector3(p_col_x, r_palm.transform.position.y, p_col_z);
				ori = ori_c*(-1);

				WholeSystem.transform.rotation = Quaternion.Lerp(from, to, Time.time * speed);
			}

			//hit point position relative to the robot base coordinate
			//p_x = -1000*(sphere_test.transform.position.z - proxy_base.transform.position.z);
			//p_y = 1000*(sphere_test.transform.position.x - proxy_base.transform.position.x);
			//p_z = 1000*(sphere_test.transform.position.y - proxy_base.transform.position.y);

			p_x = -1000*(sphere_test.transform.localPosition.x);
			p_y = 1000*(sphere_test.transform.localPosition.y);
			p_z = 1000*(sphere_test.transform.localPosition.z);

			Debug.Log("POSE:" + p_x + ", " + p_y + ", " + p_z);
			

			//p_z = 1000*(sphere_test.transform.position.y - 0.915f);

		}
	}
}
