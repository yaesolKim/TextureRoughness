//calculate transformation between world, robot, unity
//attach this script to the WholeSystem
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class initial_calib : MonoBehaviour {

  public GameObject[] cube = new GameObject[4];
  public Vector3[] calib_positions = new Vector3[4];

  public GameObject r_hand, r_index_finger, Walls, GS, Leap_BG;
  public GameObject Panel, iiwa, tool_vr, LeapRig, WholeSystem, proxy_base;

  private bool press = false;
  public bool initialized = false;
  private int point = 0;
  private Vector3 command_position;
  private Vector3 translation;
  private float rotation;

  // Use this for initialization
  void Start () {
    //Vector3 home_positions = new Vector3(672.42F, 0.0F, 486.39F);
    //Vector3 home_orientations = new Vector3(0F, 90.0F, 0.0F);
    Walls.SetActive(false);
    GS.SetActive(false);

    ////for debuging, use 5 line below///////////////////////////
/*
    calib_positions[0] = new Vector3(-0.421F, 1.795F, -0.881F);
    calib_positions[1] = new Vector3(-0.2913342F, 1.785286F, -0.7079377F);
    calib_positions[2] = new Vector3(-0.4441156F, 1.458507F, -0.8914591F);
    calib_positions[3] = new Vector3(-0.3103042F, 1.439708F, -0.7044851F);
    point = 4;
  */
    /////////////////////////////////////////////////////////////
  }

  // Update is called once per frame
  void Update () {

    //for input position of right hand, return its position on robot coordinate
    OVRInput.Update();

    if(!initialized)
    {
      if (OVRInput.Get(OVRInput.Button.Two))
      {
        if (point<4 && !press)
        {
          Debug.Log("y pressed");
          press = true;

          calib_positions[point] = r_index_finger.transform.position;
          cube[point].transform.position = calib_positions[point];
          cube[point].SetActive(true);
          point++;
        }

      }

      else
      {
        if (point == 4)
        {
          GS.SetActive(true);
          //approximate input values to four points make one plane.
          float x = (calib_positions[0].x + calib_positions[2].x) / 2;
          calib_positions[0].x = x;
          calib_positions[2].x = x;

          x = (calib_positions[1].x + calib_positions[3].x) / 2;
          calib_positions[1].x = x;
          calib_positions[3].x = x;

          float y = (calib_positions[0].y + calib_positions[1].y) / 2;
          calib_positions[0].y = y;
          calib_positions[1].y = y;

          y = (calib_positions[2].y + calib_positions[3].y) / 2;
          calib_positions[2].y = y;
          calib_positions[3].y = y;

          float z = (calib_positions[0].z + calib_positions[2].z) / 2;
          calib_positions[0].z = z;
          calib_positions[2].z = z;

          z = (calib_positions[1].z + calib_positions[3].z) / 2;
          calib_positions[1].z = z;
          calib_positions[3].z = z;

          cube[0].transform.position = calib_positions[0];
          cube[1].transform.position = calib_positions[1];
          cube[2].transform.position = calib_positions[2];
          cube[3].transform.position = calib_positions[3];
          //finish approximation

          Vector3 tool_leap = (calib_positions[0] + calib_positions[1] + calib_positions[2] + calib_positions[3]) / 4;

          float diffx = calib_positions[1].x-calib_positions[0].x;
          float angle = 0;

          if(diffx < 0) {
            angle = (Mathf.Atan2(Mathf.Abs(calib_positions[1].z-calib_positions[0].z), Mathf.Abs(calib_positions[1].x-calib_positions[0].x)))* Mathf.Rad2Deg-180;
          }
          else {
            angle = -1*(Mathf.Atan2(Mathf.Abs(calib_positions[1].z-calib_positions[0].z), Mathf.Abs(calib_positions[1].x-calib_positions[0].x)))* Mathf.Rad2Deg;
          }

          //Locate wall panel in VR
          Panel.transform.Rotate(0, angle, 0);
          Panel.transform.position = tool_leap;//midpoint;

          //locate IIWA in VR
          iiwa.transform.Rotate(0,  0, angle);
          iiwa.transform.position += new Vector3(tool_leap.x-tool_vr.transform.position.x, 0F, tool_leap.z-tool_vr.transform.position.z);
          //iiwa.transform.position += new Vector3(midpoint.x-e_e.transform.position.x, 0F, midpoint.z-e_e.transform.position.z);
          //iiwa.transform.position.y = 0.753F;

          //adjust panel position and leap rig position
          float adjust = tool_vr.transform.position.y - tool_leap.y;
          //float adjust = e_e.transform.position.y - midpoint.y;//- 0.085F;
          Panel.transform.position += new Vector3(0F, adjust, 0F);
          LeapRig.transform.position += new Vector3(0F, adjust, 0F);

          proxy_base.transform.position = iiwa.transform.position;
          //proxy_base.transform.position.y = 0.753F;
          proxy_base.transform.rotation = iiwa.transform.rotation;


          //make the proxy base orientation parallel to one wall
          WholeSystem.transform.Rotate(0, -angle, 0);
          WholeSystem.transform.position -= new Vector3(iiwa.transform.position.x, 0F, iiwa.transform.position.z);

          Debug.Log("After all, iiwa ori: " + iiwa.transform.eulerAngles);
          Debug.Log("proxy base ori: " + proxy_base.transform.eulerAngles);

          //after calibration is done, change the scene
          initialized = true;
          Leap_BG.SetActive(false);
          Walls.SetActive(true);

          //cam.clearFlags = CameraClearFlags.Skybox;

          cube[0].SetActive(false);
          cube[1].SetActive(false);
          cube[2].SetActive(false);
          cube[3].SetActive(false);
          //Panel.SetActive(false);
          //iiwa.SetActive(false);

        }
        if (press)
        {
          Debug.Log("y not pressed");
          press = false;
        }
      }
    }

    else { //initialized
      GameObject.Find("NW_manager").GetComponent<client_haptic>().enabled = true;
      GameObject.Find("Main Camera").GetComponent<MC_raycast>().enabled = true;
      GameObject.Find("WholeSystem").GetComponent<RotateWholeSystem>().enabled = true;
    }

  }

}
