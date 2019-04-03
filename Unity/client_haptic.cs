// Socket communication client. it sends message to ROS.
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
  public GameObject cam, r_hand, proxy_base;
  public string IP;
  public int Port;

  IPEndPoint ipep;
  Socket client;
  Material nw_material;

  byte[] SendHello = Encoding.Default.GetBytes("Hello! I'm Unity, client.");
  byte[] SendM = new byte[16];

  private float ori, pos_x, pos_y, pos_z;
  private byte[] byte_o, byte_x, byte_y, byte_z;

  byte[] bytes = new byte[1024];

  // Use this for initialization
  void Start () {
    Debug.Log("Clinet Start!!!!!!!!!!!!!!!!!!!");
    initial_calib calib = GameObject.Find("WholeSystem").GetComponent<initial_calib>();

    nw_material = GetComponent<Renderer>().material;
    nw_material.color = Color.black;
    // Create a TCP/IP  socket.
    ipep = new IPEndPoint(IPAddress.Parse(IP), Port);
    client = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

    if(calib.initialized) {
      calib.enabled = false; //deactive the calibration script
      Debug.Log("Deactivate initial_calib!!!!!!!!!!!!!!!!!!!");
    }
  }

  // fimxed time step : 0.01
  void FixedUpdate () {
    //transform.position = new Vector3(cam.transform.position.x, cam.transform.position.y-0.6f, cam.transform.position.z);
    //transform.eulerAngles = new Vector3(0f, cam.transform.eulerAngles.y, 0f);

    //touch mode: send data to ROS when NW connected and virtual hand is shown
    if((nw_material.color==Color.red) && r_hand.activeInHierarchy) {
      OculusControlEnable(false);

      MC_raycast mc_raycast = GameObject.Find("Main Camera").GetComponent<MC_raycast>();

      ori = mc_raycast.ori;
      pos_x = mc_raycast.p_x;
      pos_y = mc_raycast.p_y;
      pos_z = mc_raycast.p_z;

      Sending(ori, pos_x, pos_y, pos_z);
    }

    //move mode: moce whole system using oculus touch controller
    else if((nw_material.color==Color.red) && !r_hand.activeInHierarchy) {
      //Debug.Log("oculus touch controller");
      OculusControlEnable(true);
    }

    else {
      OculusControlEnable(true);
    }

  }

//TODO: add message data - texture
  void Sending(float o, float x, float y, float z) {
    byte_o = BitConverter.GetBytes(o);
    byte_x = BitConverter.GetBytes(x);
    byte_y = BitConverter.GetBytes(y);
    byte_z = BitConverter.GetBytes(z);

    Buffer.BlockCopy(byte_o, 0, SendM, 0, 4);
    Buffer.BlockCopy(byte_x, 0, SendM, 4, 4);
    Buffer.BlockCopy(byte_y, 0, SendM, 8, 4);
    Buffer.BlockCopy(byte_z, 0, SendM, 12, 4);

    client.Send(SendM); // Send the data through the socket.

    Debug.Log("send:" + o +" " + x);
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
        client.Connect(ipep); //Connect socket.
        client.Send(SendHello);
        // Receive the response from the remote device.
        int bytesRec = client.Receive(bytes);
        Debug.Log("Message from the server: " + Encoding.ASCII.GetString(bytes,0,bytesRec));
        nw_material.color = Color.red; //change the color of nw button
      }
    }
  } //end of OnTriggerEnter


}
