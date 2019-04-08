//attatch this script to the main camera
//caculate orientation of the target wall using raycast

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class MC_raycast : MonoBehaviour {
	public Transform target_wall, proxy_wall;
	public GameObject r_palm, proxy_base, WholeSystem;
	public float ori_c, ori, p_x, p_y, p_z, roughness;

	private RaycastHit hit, hit2;
	private float[] ori_s = { 315.0f, 330.0f, 345.0f, 0.0f, 15.0f, 30.0f, 45.0f }; //sampled orientations

	public float degree, aa, a;
	private float old_degree = -1.0f;
	public float speed = 0.001F;

	public bool touchable = false;

	void Start () {
	}

	// Update is called once per frame
	void Update () {
		if(r_palm.activeInHierarchy && Physics.Raycast(transform.position, transform.forward, out hit, 20.0f))
		{
			Debug.DrawRay(transform.position, transform.forward * 20.0f, Color.red);
			target_wall = hit.transform; //select target wall by raycasting

			Vector3 relPose = target_wall.transform.position - r_palm.transform.position;
			float degree = target_wall.transform.eulerAngles.y - proxy_base.transform.eulerAngles.y;
			aa = degree;

			if(aa < -180)
			{
				aa += 360;
			}

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

			}

			if(target_wall.gameObject.tag == "static_wall")
			{
				string objname = target_wall.gameObject.name;
				roughness = System.Convert.ToSingle(objname);

				float wall_ori = target_wall.transform.eulerAngles.y;

				float d = Mathf.Sqrt(relPose.x*relPose.x + relPose.z*relPose.z);

				float costh = relPose.x/d;
				float sinth = relPose.z/d;

				float theta = 90 - wall_ori;
				float cos = Mathf.Cos(theta*Mathf.Deg2Rad)*costh + Mathf.Sin(theta*Mathf.Deg2Rad)*sinth;

				//predicted colision point -> sphere_test position
				float p_col_x = target_wall.transform.position.x - d*cos*Mathf.Sin(wall_ori*Mathf.Deg2Rad);
				float p_col_z = target_wall.transform.position.z - d*cos*Mathf.Cos(wall_ori*Mathf.Deg2Rad);
				proxy_wall.transform.position = new Vector3(p_col_x, r_palm.transform.position.y, p_col_z);

				//predicted colision point -> sphere_test orientation
				Vector3 direction_hit2 = proxy_wall.transform.position - r_palm.transform.position;
				Physics.Raycast(r_palm.transform.position, direction_hit2, out hit2, 20.0f);
				Debug.DrawRay(proxy_wall.transform.position, hit2.normal, Color.blue);

				Vector3 from = new Vector3(0, 0, -1); //proxy_base
				Vector3 to = hit2.normal; //normal

				a = Vector3.Angle(from, to);

				Vector3 p = Vector3.Cross(from, to);
				Vector3 q = new Vector3(0, 1, 0);

				if(Vector3.Dot(p, q)<0)
				{
				  a  = a*(-1) + 360;
				}

				proxy_wall.transform.eulerAngles = new Vector3(-90, a+90, 180);

				touchable = true;

			}
			/*
			else if(target_wall.gameObject.tag == "dynamic_wall")
			{
				string objname = target_wall.gameObject.name;
				Debug.Log("wall:" + objname);
				roughness = System.Convert.ToSingle(objname);

				float wall_ori = target_wall.transform.eulerAngles.y;

				float d = Mathf.Sqrt(relPose.x*relPose.x + relPose.z*relPose.z);

				float costh = relPose.x/d;
				float sinth = relPose.z/d;

				float theta = 90 - wall_ori;
				float cos = Mathf.Cos(theta*Mathf.Deg2Rad)*costh + Mathf.Sin(theta*Mathf.Deg2Rad)*sinth;

				//predicted colision point -> sphere_test position
				float p_col_x = target_wall.transform.position.x - d*cos*Mathf.Sin(wall_ori*Mathf.Deg2Rad);
				float p_col_z = target_wall.transform.position.z - d*cos*Mathf.Cos(wall_ori*Mathf.Deg2Rad);
				proxy_wall.transform.position = new Vector3(p_col_x, r_palm.transform.position.y, p_col_z);

				ori = aa + 500;
				touchable = true;
				//WholeSystem.transform.rotation = Quaternion.Lerp(from, to, Time.time * speed);
			}*/

			else if(target_wall.gameObject.tag == "static_sphere")
			{
				string objname = target_wall.gameObject.name;
				roughness = System.Convert.ToSingle(objname);

				float wall_ori = target_wall.transform.eulerAngles.y;
				float d = Mathf.Sqrt(relPose.x*relPose.x + relPose.z*relPose.z);

				float xxx = (relPose.x/d)*(target_wall.transform.lossyScale.x/2);
				float zzz = (relPose.z/d)*(target_wall.transform.lossyScale.z/2);

				//set proxy wall configuration
				//position
				float p_col_x = target_wall.transform.position.x - xxx;
				float p_col_z = target_wall.transform.position.z - zzz;
				proxy_wall.transform.position = new Vector3(p_col_x, r_palm.transform.position.y, p_col_z);
				//orientation
				Vector3 direction_hit2 = proxy_wall.transform.position - r_palm.transform.position;
				Physics.Raycast(r_palm.transform.position, direction_hit2, out hit2, 20.0f);
				Debug.DrawRay(proxy_wall.transform.position, hit2.normal, Color.green);

				Vector3 from = new Vector3(0, 0, -1); //proxy_base
				Vector3 to = hit2.normal; //normal

				a = Vector3.Angle(from, to);

				Vector3 p = Vector3.Cross(from, to);
				Vector3 q = new Vector3(0, 1, 0);

				if(Vector3.Dot(p, q)<0)
				{
					a  = a*(-1) + 360;
				}

				proxy_wall.transform.eulerAngles = new Vector3(-90, a+90, 180);
				touchable = true;
			}

			else
			{
				touchable = false;
			}

			p_x = -1000*(proxy_wall.transform.localPosition.x);
			p_y = 1000*proxy_wall.transform.localPosition.y;
			p_z = 1000*(proxy_wall.transform.localPosition.z + 0.067f); //iiwa base height

			ori =  proxy_wall.transform.rotation.eulerAngles.y - proxy_base.transform.rotation.eulerAngles.y;
		}

		else
		{
		touchable = false;
		}
	}
}
