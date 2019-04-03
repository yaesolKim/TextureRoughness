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

public class scan extends RoboticsAPIApplication {
   private Controller cabinet;
   private LBR lbr;
   private Tool tool;
   ObjectFrame tcp_tool;
   
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
   }

   /**
    * Auto-generated method stub. Do not modify the contents of this method.
    */
   public static void main(String[] args) throws IOException {
	   TextureRecording_sunrise app = new TextureRecording_sunrise();
	   app.runApplication();
   }
}
