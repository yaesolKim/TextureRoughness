//model = a, b, c, d;
//degree = 0, 1, 2, 3, infinite(4)
//velocity = 10, 30, 50, 70, 90	  

package hapticRendering; 

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import additionalFunction.Point;
import additionalFunction.ReadControlPoints;
import additionalFunction.SplineLinker;
import additionalFunction.ReadPCLfile;
import additionalFunction.SamplingPoints;
import additionalFunction.QuadTree;
import additionalFunction.Calibration; 

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.OrientationReferenceSystem;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

import java.io.*;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import java.lang.*;
//import java.math.*;
import Jama.Matrix;

public class record extends RoboticsAPIApplication {
   private Controller cabinet;
   private LBR lbr;
   private Tool tool;
   ObjectFrame tcp_tool;
   
   private CartesianImpedanceControlMode redundCIC = new CartesianImpedanceControlMode();
   
   public ArrayList<Frame> controlPoints;
   public ArrayList<Frame> estimatedNormals;
   public Frame pointP, pointP1, pointQ1, pointQ2, pointQ3, pointQ4, pointQ5, pointX;
   private ArrayList<Frame> planningPath;
 
   public void initialize() {
      cabinet = getController("KUKA_Sunrise_Cabinet_1");
      lbr = (LBR) getDevice(cabinet, "LBR_iiwa_7_R800_1");
      tool = getApplicationData().createFromTemplate("Tool");
      tcp_tool = tool.getFrame("TCP");
      tool.attachTo(lbr.getFlange());      
      
      // conditions
      redundCIC.parametrize(CartDOF.TRANSL).setStiffness(0).setDamping(0.1);
      redundCIC.parametrize(CartDOF.ROT).setStiffness(3).setDamping(0.1);
      redundCIC.setNullSpaceStiffness(10).setNullSpaceDamping(0.3);
      redundCIC.setReferenceSystem(World.Current.getRootFrame());  
   }
   
   public void record_frames() {
	   boolean end_flag = true;	   
	   while (end_flag) {
		   IMotionContainer positionHoldContainer = lbr.moveAsync((new PositionHold(redundCIC, -1, null)));
		   int ret = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Save this position?", "P", "Q1", "Q2", "Q3", "Q4", "Q5", "DONE");
		   positionHoldContainer.cancel();
		   
		   switch (ret) {
		   case 0:
			   pointP = new Frame();
			   pointP = lbr.getCurrentCartesianPosition(tcp_tool);
			   pointP1 = new Frame();
			   pointP1 = lbr.getCurrentCartesianPosition(tcp_tool);
			   
			   pointP.setX(pointP.getX()-20);
			   
			   getLogger().info("get point P1: [ "+pointP1 +"]");
			   //pointP1.setX(pointP1.getX()+2);
			   getLogger().info("save point P1 Frame: [ "+pointP1 +"]");
			   break;
			   
		   case 1:
			   pointQ1 = new Frame();
			   pointQ1 = lbr.getCurrentCartesianPosition(tcp_tool);
			   //pointQ1.setX(pointQ1.getX()+2);
			   getLogger().info("save point Q1 Frame: [ "+pointQ1 +"]");
			   break;
			   
		   case 2:
			   pointQ2 = new Frame();
			   pointQ2 = lbr.getCurrentCartesianPosition(tcp_tool);
			   //pointQ2.setX(pointQ2.getX()+2);
			   getLogger().info("save point Q2 Frame: [ "+pointQ2 +"]");
			   break;
			   
		   case 3:
			   pointQ3 = new Frame();
			   pointQ3 = lbr.getCurrentCartesianPosition(tcp_tool);
			   //pointQ3.setX(pointQ3.getX()+2);
			   getLogger().info("save point Q3 Frame: [ "+pointQ3 +"]");
			   break;
			   
		   case 4:
			   pointQ4 = new Frame();
			   pointQ4 = lbr.getCurrentCartesianPosition(tcp_tool);
			   //pointQ4.setX(pointQ4.getX()+2);
			   getLogger().info("save point Q4 Frame: [ "+pointQ4 +"]");
			   break;
			   
		   case 5:
			   pointQ5 = new Frame();
			   pointQ5 = lbr.getCurrentCartesianPosition(tcp_tool);
			   //pointQ4.setX(pointQ4.getX()+2);
			   getLogger().info("save point Q4 Frame: [ "+pointQ5 +"]");
			   break;
			   
		   case 6:
				 end_flag = false;	// end on black button
				 break;			 
			 }	 
		 }
	   
	   pointX = new Frame();
	   pointX = lbr.getCurrentCartesianPosition(tcp_tool);
	   pointX.setX(pointX.getX()-20);
	   tcp_tool.move(lin(pointX).setCartVelocity(50)); // mm/s
	   
   }// end of recording frames
	  
   public void scanning(char model, int degree, int velocity, CartesianImpedanceControlMode ctrl) {//, CartesianImpedanceControlMode ctrl2) {
	   DataRecorder rec = new DataRecorder("d", 10, TimeUnit.SECONDS, 100);
	   String filename = Character.toString(model) + "_" + String.valueOf(degree) + "_" + String.valueOf(velocity);
	   rec.setFileName(filename);
	   
	   getLogger().info("make file: " + filename + ".txt");
	   rec.addCurrentCartesianPositionXYZ(tcp_tool, getApplicationData().getFrame("/BASE"));
	   rec.addExternalJointTorque(lbr);
	   rec.addCartesianForce(tcp_tool, getApplicationData().getFrame("/BASE"));
	   rec.addCartesianTorque(tcp_tool, getApplicationData().getFrame("/BASE"));
	   rec.enable();
	   
	   tcp_tool.move(ptp(pointP).setJointVelocityRel(0.3)); // mm/s
	   ThreadUtil.milliSleep(2000);
	   
	   tcp_tool.move(lin(pointP1).setCartVelocity(30).setMode(ctrl)); // mm/s
	   ThreadUtil.milliSleep(2000);
	   
	   rec.startRecording();/////////////////////////////////////////////
	   switch(degree){
	   case 0:
		   tcp_tool.move(lin(pointQ1).setCartVelocity(velocity).setMode(ctrl)); // mm/s
		   break;
	   case 1:
		   tcp_tool.move(lin(pointQ2).setCartVelocity(velocity).setMode(ctrl)); // mm/s
		   break;
	   case 2:
		   tcp_tool.move(lin(pointQ3).setCartVelocity(velocity).setMode(ctrl)); // mm/s
		   break;
	   case 3:
		   tcp_tool.move(lin(pointQ4).setCartVelocity(velocity).setMode(ctrl)); // mm/s
		   break;
	   case 4:
		   tcp_tool.move(lin(pointQ5).setCartVelocity(velocity).setMode(ctrl)); // mm/s
		   break;
	   default:
			break;			
	   }
	   ThreadUtil.milliSleep(2000);
	   rec.stopRecording();////////////////////////////////////////////////
	   
	   pointX = new Frame();
	   pointX = lbr.getCurrentCartesianPosition(tcp_tool);
	   pointX.setX(pointX.getX()-20);
	   tcp_tool.move(lin(pointX).setCartVelocity(50)); // mm/s
	   
	   ThreadUtil.milliSleep(5000);		
   }
         
   public void run(){	   
	   CartesianImpedanceControlMode carImp = new CartesianImpedanceControlMode();
	   //compliant only in the x direction
	   carImp.parametrize(CartDOF.X,CartDOF.Y).setStiffness(5000.0); //max, Nm
	   carImp.parametrize(CartDOF.Z).setStiffness(30.0);
//	   carImp.parametrize(CartDOF.Z).setAdditionalControlForce(10.0);
	   carImp.parametrize(CartDOF.Z).setAdditionalControlForce(0.5);
	   carImp.parametrize(CartDOF.ROT).setStiffness(300.0);	//max
	   carImp.parametrize(CartDOF.ALL).setDamping(1.0); //max
	   //move to home position
	   JointPosition home1 = new JointPosition(0, 0.523599, 0, -1.5708, 0, -0.523599, 0);
	   lbr.move(ptp(home1).setJointVelocityRel(0.5));
	   
	   // save the start and end point by manually
	   getLogger().info("Recording frames for model A");
	   record_frames();	   
 
	   getLogger().info("Scanning for model A");
	   for (int j = 0; j<5; j++){
		   for (int i = 0; i<5; i++) {
			   //scanning(char model, int degree, int velocity, CartesianImpedanceControlMode ctrl
			   scanning('a', i, j*20+10, carImp);//, carImp2);			   
		   }
	   }
	   
	   getLogger().info("Recording frames for model B");
	   record_frames();
	   
	   getLogger().info("Scanning for model B");
	   for (int j = 0; j<5; j++){
		   for (int i = 0; i<5; i++) {
			   scanning('b', i, j*20+10, carImp);//, carImp2);			   
		   }
	   }
	   
	   getLogger().info("Recording frames for model C");
	   record_frames();
	   
	   getLogger().info("Scanning for model C");
	   for (int j = 0; j<5; j++){
		   for (int i = 0; i<5; i++) {
			   scanning('c', i, j*20+10, carImp);//, carImp2);			   
		   }
	   }
	   
	   getLogger().info("Recording frames for model D");
	   record_frames();
	   
	   getLogger().info("Scanning for model D");
	   for (int j = 0; j<5; j++){
		   for (int i = 0; i<5; i++) {
			   
			   scanning('d', i, j*20+10, carImp);//, carImp2);			   
		   }
	   }
	   /////////////////////////////////////////*/
	   
   }

   /**
    * Auto-generated method stub. Do not modify the contents of this method.
    */
   public static void main(String[] args) throws IOException {
	   TextureRecording_sunrise app = new TextureRecording_sunrise();
	   app.runApplication();
   }
}