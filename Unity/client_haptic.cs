// Socket communication client. it sends message to ROS.
//attach this script to the NW-manager
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using System.Linq;
using System.Net;
using System.Net.Sockets;

public class client_haptic : MonoBehaviour {
  public GameObject cam, r_hand;//, proxy_base;

  // comm: Get ip and port number from unity UI
  public string IP;
  public int Port;

  // comm: Declare variables
  EndPoint ipep_send, ipep_receive;
  Socket client;
  Material nw_material;
  byte[] welcome_message = Encoding.Default.GetBytes("Hello! I'm Unity, client.");
  byte[] SendM = new byte[20];
  byte[] bytes = new byte[1024];

  private float ori, pos_x, pos_y, pos_z, rough;
  private byte[] byte_o, byte_x, byte_y, byte_z, byte_r;

  void Start () {
    Debug.Log("Clinet Start!!!!!!!!!!!!!!!!!!!");
    initial_calib calib = GameObject.Find("WholeSystem").GetComponent<initial_calib>();

    nw_material = GetComponent<Renderer>().material;
    nw_material.color = Color.black;

    // comm: Create a UDP socket.
    ipep_send = new IPEndPoint(IPAddress.Parse(IP), Port);
    ipep_receive = new IPEndPoint(IPAddress.None, 0);
    client = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);

    if(calib.initialized) {
      calib.enabled = false; //deactive the calibration script
      Debug.Log("Deactivate initial_calib.");
    }
  }

  // fimxed time step : 0.01
  void FixedUpdate () {
    //transform.position = new Vector3(cam.transform.position.x, cam.transform.position.y-0.6f, cam.transform.position.z);
    //transform.eulerAngles = new Vector3(0f, cam.transform.eulerAngles.y, 0f);

    MC_raycast mc_raycast = GameObject.Find("Main Camera").GetComponent<MC_raycast>();
    //touch mode: send data to ROS when NW connected and virtual hand is shown
    if((nw_material.color==Color.red) && mc_raycast.touchable) {
      OculusControlEnable(false);
      //Debug.Log("TOUCH MODE");

      ori = mc_raycast.ori;
      pos_x = mc_raycast.p_x;
      pos_y = mc_raycast.p_y;
      pos_z = mc_raycast.p_z;
      rough = mc_raycast.roughness;

      Sending(ori, pos_x, pos_y, pos_z, rough);
    }

    //move mode: moce whole system using oculus touch controller
    else if((nw_material.color==Color.red) && !r_hand.activeInHierarchy) {
      //Debug.Log("oculus touch controller");
      OculusControlEnable(true);
    }

    else {
      //Debug.Log("oculus touch controller");
      OculusControlEnable(true);
    }

  }

  // comm: function for sending message to server
  void Sending(float o, float x, float y, float z, float r) { ///need to be checked
    byte_o = BitConverter.GetBytes(o);
    byte_x = BitConverter.GetBytes(x);
    byte_y = BitConverter.GetBytes(y);
    byte_z = BitConverter.GetBytes(z);
    byte_r = BitConverter.GetBytes(r); ///need to be checked

    Buffer.BlockCopy(byte_o, 0, SendM, 0, 4);
    Buffer.BlockCopy(byte_x, 0, SendM, 4, 4);
    Buffer.BlockCopy(byte_y, 0, SendM, 8, 4);
    Buffer.BlockCopy(byte_z, 0, SendM, 12, 4);
    Buffer.BlockCopy(byte_r, 0, SendM, 16, 4); ///need to be checked

    client.SendTo(SendM, ipep_send);// Send the data through the socket.
    //Debug.Log("send:" + o + ", " + x+", " + y+", " + z+", " + r);
  }

  //TODO: add rotation control button (y, x button on controller)
  void OculusControlEnable(bool control) {
    FPSInputController FPSIC = GameObject.Find("WholeSystem").GetComponent<FPSInputController>();
    CharacterMotor CM = GameObject.Find("WholeSystem").GetComponent<CharacterMotor>();
    CharacterController controller = GameObject.Find("WholeSystem").GetComponent<CharacterController>();

    if(control) {
      FPSIC.enabled = true;
      CM.enabled = true;
      controller.enabled = true;
    }

    else {
      FPSIC.enabled = false;
      CM.enabled = false;
      controller.enabled = false;
    }
  }

  void OnTriggerEnter(Collider col) { //network connection button
    if(col.tag == "hand") {
      //stop nw connection
      if (nw_material.color == Color.red) {
        client.Shutdown(SocketShutdown.Both); //Release the socket.
        client.Close();
        nw_material.color = Color.black; //change the color of nw button
      }
      //start nw connection
      else if(nw_material.color == Color.black) {
        client.SendTo(welcome_message, ipep_send);
        int bytesRec = client.ReceiveFrom(bytes, ref ipep_receive);
        Debug.Log("Message from the server: " + Encoding.ASCII.GetString(bytes,0,bytesRec));
        nw_material.color = Color.red; //change the color of nw button
      }
    }
  } //end of OnTriggerEnter


}
