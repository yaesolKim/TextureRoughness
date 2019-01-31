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

public class TextureRecording_sunrise extends RoboticsAPIApplication {
   private Controller cabinet;
   private LBR lbr;
   private Tool tool;
   ObjectFrame tcp_tool;
   
   //private ObjectFrame pen;
/*
   private DataRecorder rec1 = new DataRecorder("d1", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec2 = new DataRecorder("d2", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec3 = new DataRecorder("d3", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec4 = new DataRecorder("d4", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec5 = new DataRecorder("d5", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec6 = new DataRecorder("d6", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec7 = new DataRecorder("d7", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec8 = new DataRecorder("d8", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec9 = new DataRecorder("d9", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec10 = new DataRecorder("d10", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec11 = new DataRecorder("d11", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec12 = new DataRecorder("d12", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec13 = new DataRecorder("d13", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec14 = new DataRecorder("d14", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec15 = new DataRecorder("d15", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec16 = new DataRecorder("d16", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec17 = new DataRecorder("d17", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec18 = new DataRecorder("d18", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec19 = new DataRecorder("d19", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec20 = new DataRecorder("d20", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec21 = new DataRecorder("d21", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec22 = new DataRecorder("d22", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec23 = new DataRecorder("d23", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec24 = new DataRecorder("d24", 10, TimeUnit.SECONDS, 100);
   private DataRecorder rec25 = new DataRecorder("d25", 10, TimeUnit.SECONDS, 100);
   */
   private CartesianImpedanceControlMode redundCIC = new CartesianImpedanceControlMode();
   
   public ArrayList<Frame> controlPoints;
   public ArrayList<Frame> estimatedNormals;
   public Frame pointP, pointQ1, pointQ2, pointQ3, pointQ4;
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
   
   public void recording() {
	   boolean end_flag = true;
	   
	   // record
	   while (end_flag) {
		   IMotionContainer positionHoldContainer = lbr.moveAsync((new PositionHold(redundCIC, -1, null)));
		   int ret = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Save this position?", "P", "Q1", "Q2", "Q3", "Q4", "DONE");
		   positionHoldContainer.cancel();
		   
		   switch (ret) {
		   case 0:
			   pointP = new Frame();
			   pointP = lbr.getCurrentCartesianPosition(tcp_tool);
			   getLogger().info("get point P: [ "+pointP +"]");
			   pointP.setZ(pointP.getX()+20);
			   getLogger().info("save point P1 Frame: [ "+pointP +"]");
			   break;
			   
		   case 1:
			   pointQ1 = new Frame();
			   pointQ1 = lbr.getCurrentCartesianPosition(tcp_tool);
			   getLogger().info("get point Q: [ "+pointQ1 +"]");
			   pointQ1.setZ(pointQ1.getX()+20);
			   getLogger().info("save point Q1 Frame: [ "+pointQ1 +"]");
			   break;
			   
		   case 2:
			   pointQ2 = new Frame();
			   pointQ2 = lbr.getCurrentCartesianPosition(tcp_tool);
			   getLogger().info("get point Q: [ "+pointQ2 +"]");
			   pointQ2.setZ(pointQ2.getX()+20);
			   getLogger().info("save point Q2 Frame: [ "+pointQ2 +"]");
			   break;
			   
		   case 3:
			   pointQ3 = new Frame();
			   pointQ3 = lbr.getCurrentCartesianPosition(tcp_tool);
			   getLogger().info("get point Q: [ "+pointQ3 +"]");
			   pointQ3.setZ(pointQ3.getX()+20);
			   getLogger().info("save point Q3 Frame: [ "+pointQ3 +"]");
			   break;
			   
		   case 4:
			   pointQ4 = new Frame();
			   pointQ4 = lbr.getCurrentCartesianPosition(tcp_tool);
			   getLogger().info("get point Q: [ "+pointQ4 +"]");
			   pointQ4.setZ(pointQ4.getX()+20);
			   getLogger().info("save point Q4 Frame: [ "+pointQ4 +"]");
			   break;
			   
		   case 5:
				 end_flag = false;	// end on black button
				 break;			 
			 }	 
		 }// end of record
	   
   }
   
   
   public void scanning(char model, int degree, int velocity, CartesianImpedanceControlMode ctrl) {
	   DataRecorder rec = new DataRecorder("d", 10, TimeUnit.SECONDS, 100);
	   String filename = Character.toString(model) + "_" + String.valueOf(degree) + "_" + String.valueOf(velocity);
	   rec.setFileName(filename);
	   
	   rec.addCurrentCartesianPositionXYZ(tcp_tool, getApplicationData().getFrame("/BASE"));
	   rec.addExternalJointTorque(lbr);
	   rec.addCartesianForce(tcp_tool, getApplicationData().getFrame("/BASE"));
	   rec.addCartesianTorque(tcp_tool, getApplicationData().getFrame("/BASE"));
	   rec.enable();
	   
	   tcp_tool.move(lin(getApplicationData().getFrame("/HTR/start")).setCartVelocity(90).setMode(ctrl)); // mm/s
	   ThreadUtil.milliSleep(2000);
	   tcp_tool.move(lin(getApplicationData().getFrame("/HTR/pointP")).setCartVelocity(30).setMode(ctrl)); // mm/s
	   ThreadUtil.milliSleep(2000);
	   
	   rec.startRecording();
	   
	   switch(degree){
	   case 0:
		   tcp_tool.move(lin(getApplicationData().getFrame("/HTR/pointQ1")).setCartVelocity(velocity).setMode(ctrl)); // mm/s
		   break;
	   case 1:
		   tcp_tool.move(lin(getApplicationData().getFrame("/HTR/pointQ2")).setCartVelocity(velocity).setMode(ctrl)); // mm/s
		   break;
	   case 2:
		   tcp_tool.move(lin(getApplicationData().getFrame("/HTR/pointQ3")).setCartVelocity(velocity).setMode(ctrl)); // mm/s
		   break;
	   case 3:
		   tcp_tool.move(lin(getApplicationData().getFrame("/HTR/pointQ4")).setCartVelocity(velocity).setMode(ctrl)); // mm/s
		   break;
	   case 4:
		   tcp_tool.move(lin(getApplicationData().getFrame("/HTR/pointQ5")).setCartVelocity(velocity).setMode(ctrl)); // mm/s
		   break;
		   
	   default:
			break;			
	   }
	   
	   ThreadUtil.milliSleep(2000);
	   rec.stopRecording();

	   tcp_tool.move(lin(getApplicationData().getFrame("/HTR/end")).setCartVelocity(90).setMode(ctrl)); // mm/s
	   ThreadUtil.milliSleep(5000);
		
   }
         
   public void run(){
	   CartesianImpedanceControlMode carImp = new CartesianImpedanceControlMode();
	   //compliant only in the x direction
	   carImp.parametrize(CartDOF.X,CartDOF.Y).setStiffness(5000.0); //max, Nm
	   carImp.parametrize(CartDOF.Z).setStiffness(30.0);
	   carImp.parametrize(CartDOF.ROT).setStiffness(300.0);	//max
	   carImp.parametrize(CartDOF.X).setAdditionalControlForce(10.0);
	   carImp.parametrize(CartDOF.ALL).setDamping(1.0); //max	
	   
	   //move to home position
	   JointPosition home1 = new JointPosition(0, 0.523599, 0, -1.5708, 0, -0.523599, 0);
	   lbr.move(ptp(home1).setJointVelocityRel(0.5));
	   
	   // save the start and end point by manually
	   getLogger().info("Recording frames starting, press button when positioned");
	   recording(); //save start & end frames

		
		//lbr.move(ptp(home1).setJointVelocityRel(0.4));
		//ThreadUtil.milliSleep(2000);
		//getLogger().info("move to P[ "+pointP +"]");
		//tcp_tool.move(lin(pointP).setCartVelocity(30)); // mm/s

	   
	   
		//model = a, b, c, d;
		//degree = 0, 1, 2, 3, infinite(4)
		//velocity = 10, 30, 50, 70, 90
		//scanning(char model, int degree, int velocity, carImp)
		
	   for (int i = 0; i<5; i++) {
		   for (int j = 0; j<5; j++) {
			   scanning('a', i, j*20+10, carImp);			   
		   }
	   }
	   
	   for (int i = 0; i<5; i++) {
		   for (int j = 0; j<5; j++) {
			   scanning('b', i, j*20+10, carImp);			   
		   }
	   }
	   
	   for (int i = 0; i<5; i++) {
		   for (int j = 0; j<5; j++) {
			   scanning('c', i, j*20+10, carImp);			   
		   }
	   }
	   
	   for (int i = 0; i<5; i++) {
		   for (int j = 0; j<5; j++) {
			   scanning('d', i, j*20+10, carImp);			   
		   }
	   }
	   
	   /*
	   rec1.setFileName("wood_3.0_10.log");
	   rec1.addCurrentCartesianPositionXYZ(tcp_tool, getApplicationData().getFrame("/BASE"));
	   rec1.addExternalJointTorque(lbr);
	   rec1.addCartesianForce(tcp_tool, getApplicationData().getFrame("/BASE"));
	   rec1.addCartesianTorque(tcp_tool, getApplicationData().getFrame("/BASE"));
	   
	   
	    //record 1
	    rec1.enable();
	    tcp_tool.move(lin(getApplicationData().getFrame("/HTR/start")).setCartVelocity(90).setMode(carImp)); // mm/s
	    ThreadUtil.milliSleep(2000);
	    tcp_tool.move(lin(getApplicationData().getFrame("/HTR/pointP")).setCartVelocity(30).setMode(carImp)); // mm/s
		ThreadUtil.milliSleep(2000);		
		
		rec1.startRecording();		
		tcp_tool.move(lin(getApplicationData().getFrame("/HTR/pointQ")).setCartVelocity(10).setMode(carImp)); // mm/s
		ThreadUtil.milliSleep(2000);
		rec1.stopRecording();
		
		tcp_tool.move(lin(getApplicationData().getFrame("/HTR/end")).setCartVelocity(90).setMode(carImp)); // mm/s
		ThreadUtil.milliSleep(5000);
		
		
		rec2.setFileName("wood_3.0_30.log");
		rec2.addCurrentCartesianPositionXYZ(tcp_tool, getApplicationData().getFrame("/BASE"));
		rec2.addExternalJointTorque(lbr);
		rec2.addCartesianForce(tcp_tool, getApplicationData().getFrame("/BASE"));
		rec2.addCartesianTorque(tcp_tool, getApplicationData().getFrame("/BASE"));
		rec2.enable();
		
		tcp_tool.move(lin(getApplicationData().getFrame("/HTR/start")).setCartVelocity(90).setMode(carImp)); // mm/s
	    ThreadUtil.milliSleep(2000);
	    tcp_tool.move(lin(getApplicationData().getFrame("/HTR/pointP")).setCartVelocity(30).setMode(carImp)); // mm/s
		ThreadUtil.milliSleep(2000);
		rec2.startRecording();		
		tcp_tool.move(lin(getApplicationData().getFrame("/HTR/pointQ")).setCartVelocity(30).setMode(carImp)); // mm/s
		ThreadUtil.milliSleep(2000);
		rec2.stopRecording();
		
		tcp_tool.move(lin(getApplicationData().getFrame("/HTR/end")).setCartVelocity(90).setMode(carImp)); // mm/s
		ThreadUtil.milliSleep(5000);
		
		//record 3
		rec3.setFileName("wood_3.0_50.log");
		rec3.addCurrentCartesianPositionXYZ(tcp_tool, getApplicationData().getFrame("/BASE"));
		rec3.addExternalJointTorque(lbr);
		rec3.addCartesianForce(tcp_tool, getApplicationData().getFrame("/BASE"));
		rec3.addCartesianTorque(tcp_tool, getApplicationData().getFrame("/BASE"));
		rec3.enable();
		
		tcp_tool.move(lin(getApplicationData().getFrame("/HTR/start")).setCartVelocity(90).setMode(carImp)); // mm/s
	    ThreadUtil.milliSleep(2000);
	    tcp_tool.move(lin(getApplicationData().getFrame("/HTR/pointP")).setCartVelocity(30).setMode(carImp)); // mm/s
		ThreadUtil.milliSleep(2000);
		rec3.startRecording();		
		tcp_tool.move(lin(getApplicationData().getFrame("/HTR/pointQ")).setCartVelocity(50).setMode(carImp)); // mm/s
		ThreadUtil.milliSleep(2000);
		rec3.stopRecording();
		
		tcp_tool.move(lin(getApplicationData().getFrame("/HTR/end")).setCartVelocity(90).setMode(carImp)); // mm/s
		ThreadUtil.milliSleep(5000);
		
		//record 4
		rec4.setFileName("wood_3.0_70.log");
		rec4.addCurrentCartesianPositionXYZ(tcp_tool, getApplicationData().getFrame("/BASE"));
		rec4.addExternalJointTorque(lbr);
		rec4.addCartesianForce(tcp_tool, getApplicationData().getFrame("/BASE"));
		rec4.addCartesianTorque(tcp_tool, getApplicationData().getFrame("/BASE"));
		rec4.enable();
		
		tcp_tool.move(lin(getApplicationData().getFrame("/HTR/start")).setCartVelocity(90).setMode(carImp)); // mm/s
	    ThreadUtil.milliSleep(2000);
	    tcp_tool.move(lin(getApplicationData().getFrame("/HTR/pointP")).setCartVelocity(30).setMode(carImp)); // mm/s
		ThreadUtil.milliSleep(2000);
		rec4.startRecording();		
		tcp_tool.move(lin(getApplicationData().getFrame("/HTR/pointQ")).setCartVelocity(70).setMode(carImp)); // mm/s
		ThreadUtil.milliSleep(2000);
		rec4.stopRecording();
		
		tcp_tool.move(lin(getApplicationData().getFrame("/HTR/end")).setCartVelocity(90).setMode(carImp)); // mm/s
		ThreadUtil.milliSleep(5000);
		*/
		
		/*		
		rec.startRecording();
		//getLogger().info("move to Q[ "+pointQ +"]");
		//tcp_tool.move(lin(pointQ).setCartVelocity(30));
		tcp_tool.move(lin(getApplicationData().getFrame("/HTR/pointQ")).setCartVelocity(30).setMode(carImp)); // mm/s
		getLogger().info("End");
		
		ThreadUtil.milliSleep(2000);
		
		lbr.move(ptp(home1).setJointVelocityRel(0.4));
		*/		
   }

   /**
    * Auto-generated method stub. Do not modify the contents of this method.
    */
   public static void main(String[] args) throws IOException {
	   TextureRecording_sunrise app = new TextureRecording_sunrise();
	   app.runApplication();
   }
}