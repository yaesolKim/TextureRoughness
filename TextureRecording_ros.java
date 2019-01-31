package hapticRendering;

// ROS imports
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import geometry_msgs.PoseStamped;
import iiwa_msgs.CartesianQuantity;
import iiwa_msgs.ConfigureSmartServoRequest;
import iiwa_msgs.ConfigureSmartServoResponse;
import iiwa_msgs.JointQuantity;
import iiwa_msgs.SmartServoMode;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.net.URI;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;


import org.ros.exception.ServiceException;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.time.NtpTimeProvider;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.World;
//import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.ServoMotion;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;

//frame
import com.kuka.roboticsAPI.geometricModel.Frame;
//sleep
import com.kuka.common.ThreadUtil;

public class TextureRecording_ros extends RoboticsAPIApplication {
 private Controller cabinet;
 private LBR robot;
 private Tool tool;
 private SmartServo motion;
 private Lock configureSmartServoLock = new ReentrantLock();
 private ObjectFrame tcp_tool;
 private CartesianImpedanceControlMode redundCIC = new CartesianImpedanceControlMode();
 
 private boolean initSuccessful = false;
 private boolean debug = false;

 public Frame pointP;
 public Frame pointQ;
 
 private iiwaMessageGenen helper; //< Helper class to generate iiwa_msgs from current robot state.
 private iiwaPub publisher; //< IIWARos Publisher.
 private iiwaSub subscriber; //< IIWARos Subscriber.
 private iiwaConfig configuration; //< Configuration via parameters and services.
 
 // ROS Configuration and Node execution objects. Two different configurations are needed
 // for the Publisher and the Subscriber.
 private NodeConfiguration nodeConfPublisher;
 private NodeConfiguration nodeConfSubscriber;
 private NodeConfiguration nodeConfConfiguration;
 private NodeMainExecutor nodeMainExecutor;

 // configurable toolbars
 private List<IUserKeyBar> generalKeyBars = new ArrayList<IUserKeyBar>();
 private List<IUserKey> generalKeys = new ArrayList<IUserKey>();
 private List<IUserKeyListener> generalKeyLists = new ArrayList<IUserKeyListener>();

 private JointPosition home1;
 
 public static class UnsupportedControlModeException extends RuntimeException {
   private static final long serialVersionUID = 1L;
   public UnsupportedControlModeException() { super(); }
   public UnsupportedControlModeException(String message) { super(message); }
   public UnsupportedControlModeException(String message, Throwable cause) { super(message, cause); }
   public UnsupportedControlModeException(Throwable cause) { super(cause); }
 }//end of the UnsupportedControlModeException

 public IMotionControlMode buildMotionControlMode(iiwa_msgs.SmartServoMode params) {
   IMotionControlMode cm;

   switch (params.getMode()) {
   case iiwa_msgs.SmartServoMode.CARTESIAN_IMPEDANCE: {
     CartesianImpedanceControlMode ccm = new CartesianImpedanceControlMode();

     CartesianQuantity stiffness = params.getCartesianStiffness().getStiffness();
     if (stiffness.getX() >= 0)
       ccm.parametrize(CartDOF.X).setStiffness(stiffness.getX());
     if (stiffness.getY() >= 0)
       ccm.parametrize(CartDOF.Y).setStiffness(stiffness.getY());
     if (stiffness.getZ() >= 0)
       ccm.parametrize(CartDOF.Z).setStiffness(stiffness.getZ());
     if (stiffness.getA() >= 0)
       ccm.parametrize(CartDOF.A).setStiffness(stiffness.getA());
     if (stiffness.getB() >= 0)
       ccm.parametrize(CartDOF.B).setStiffness(stiffness.getB());
     if (stiffness.getC() >= 0)
       ccm.parametrize(CartDOF.C).setStiffness(stiffness.getC());

     CartesianQuantity damping = params.getCartesianDamping().getDamping();
     if (damping.getX() > 0)
       ccm.parametrize(CartDOF.X).setDamping(damping.getX());
     if (damping.getY() > 0)
       ccm.parametrize(CartDOF.Y).setDamping(damping.getY());
     if (damping.getZ() > 0)
       ccm.parametrize(CartDOF.Z).setDamping(damping.getZ());
     if (damping.getA() > 0)
       ccm.parametrize(CartDOF.A).setDamping(damping.getA());
     if (damping.getB() > 0)
       ccm.parametrize(CartDOF.B).setDamping(damping.getB());
     if (damping.getC() > 0)
       ccm.parametrize(CartDOF.C).setDamping(damping.getC());

     // TODO: add stiffness along axis
     if (params.getNullspaceStiffness() >= 0)
       ccm.setNullSpaceStiffness(params.getNullspaceStiffness());
     if (params.getNullspaceDamping() > 0)
       ccm.setNullSpaceDamping(params.getNullspaceDamping());

     cm = ccm;
     break;
   }

   case iiwa_msgs.SmartServoMode.JOINT_IMPEDANCE: {
     JointImpedanceControlMode jcm = new JointImpedanceControlMode(7);

     JointQuantity stiffness = params.getJointStiffness().getStiffness();
     jcm.setStiffness(helper.jointQuantityToVector(stiffness));

     JointQuantity damping = params.getJointDamping().getDamping();
     if (damping.getA1() > 0 && damping.getA2() > 0 && damping.getA3() > 0 && damping.getA4() > 0
         && damping.getA5() > 0 && damping.getA6() > 0 && damping.getA7() > 0)
       jcm.setDamping(helper.jointQuantityToVector(damping));

     cm = jcm;
     break;
   }

   default: {
     throw new UnsupportedControlModeException();  // this should just not happen
   }
   }

   return cm;
 } //end of the buildMotionControlMode

 public SmartServo configureSmartServoMotion(iiwa_msgs.SmartServoMode ssm) {
   SmartServo mot = new SmartServo(robot.getCurrentJointPosition());
   mot.setMinimumTrajectoryExecutionTime(8e-3);
   mot.setJointVelocityRel(configuration.getDefaultRelativeJointSpeed());
   mot.setTimeoutAfterGoalReach(300);

   configureSmartServoMotion(ssm, mot);
   return mot;
 }

 public void configureSmartServoMotion(iiwa_msgs.SmartServoMode ssm, SmartServo mot) {
   if (mot == null)
     return; // TODO: exception?

   if (ssm.getRelativeVelocity() > 0)
	   //mot.setJointVelocityRel(1.0);
     mot.setJointVelocityRel(ssm.getRelativeVelocity());
   mot.setMode(buildMotionControlMode(ssm));
 }

 public boolean isSameControlMode(IMotionControlMode kukacm, SmartServoMode roscm) {
   String roscmname = null;
   switch (roscm.getMode()) {
   case SmartServoMode.CARTESIAN_IMPEDANCE:
     roscmname = "CartesianImpedanceControlMode";
     break;
   case SmartServoMode.JOINT_IMPEDANCE:
     roscmname = "JointImpedanceControlMode";
     break;
   }
   String kukacmname = kukacm.getClass().getSimpleName();

   return roscmname.equals(kukacmname);
 }

 public void initialize() {
   cabinet = getController("KUKA_Sunrise_Cabinet_1");
   //lbr = (LBR) getDevice(cabinet, "LBR_iiwa_7_R800_1");
   robot = getContext().getDeviceFromType(LBR.class);
   tool = getApplicationData().createFromTemplate("Tool");
   tcp_tool = tool.getFrame("TCP");
   tool.attachTo(robot.getFlange());

   //front
   home1 = new JointPosition(0, 0.523599, 0, -1.5708, 0, -0.523599, 0);
   robot.move(new PTP(home1).setJointVelocityRel(1));
   getLogger().info("Move to the home1");
   
   helper = new iiwaMessageGenen();
   configuration = new iiwaConfig();
   publisher = new iiwaPub(robot, iiwaConfig.getRobotName());
   subscriber = new iiwaSub(robot, iiwaConfig.getRobotName());

   redundCIC.parametrize(CartDOF.TRANSL).setStiffness(0).setDamping(0.1);
   redundCIC.parametrize(CartDOF.ROT).setStiffness(3).setDamping(0.1);
   redundCIC.setNullSpaceStiffness(10).setNullSpaceDamping(0.3);
   redundCIC.setReferenceSystem(World.Current.getRootFrame());
   
   //robot.getSensorForExternalTorque().getSensorData();
   /////////////////////////////////////////////////////////////////////
   // SmartServo configuration service callback
   subscriber.setConfigureSmartServoCallback(new ServiceResponseBuilder<iiwa_msgs.ConfigureSmartServoRequest, iiwa_msgs.ConfigureSmartServoResponse>() {
	   @Override
     public void build(ConfigureSmartServoRequest req,
         ConfigureSmartServoResponse resp) throws ServiceException {
       // we can change the parameters if it is the same type of control strategy
       // otherwise we have to stop the motion, replace it and start it again
       try {
         if (motion.getMode() != null && isSameControlMode(motion.getMode(), req.getMode())) {
           motion.getRuntime().changeControlModeSettings(buildMotionControlMode(req.getMode()));
         } else {
           configureSmartServoLock.lock();

           SmartServo oldmotion = motion;
           ServoMotion.validateForImpedanceMode(robot);
           motion = configureSmartServoMotion(req.getMode());
           robot.moveAsync(motion);
           oldmotion.getRuntime().stopMotion();

           configureSmartServoLock.unlock();
         }
       } catch (Exception e) {
         resp.setSuccess(false);
         if (e.getMessage() != null) {
           StringWriter sw = new StringWriter();
           PrintWriter pw = new PrintWriter(sw);
           e.printStackTrace(pw);
           resp.setError(e.getClass().getName() + ": " + e.getMessage() + ", " + sw.toString());
         } else {
           resp.setError("because I hate you :)");
         }
         return;
       }
       resp.setSuccess(true);
     }
   });

       try {
         // Set the configuration parameters of the ROS nodes to create.
         URI uri = new URI(iiwaConfig.getMasterURI());

         nodeConfConfiguration = NodeConfiguration.newPublic(iiwaConfig.getRobotIp());
         nodeConfConfiguration.setTimeProvider(iiwaConfig.getTimeProvider());
         nodeConfConfiguration.setNodeName(iiwaConfig.getRobotName() + "/iiwa_configuration");
         nodeConfConfiguration.setMasterUri(uri);

         // Configuration for the Publisher.
         nodeConfPublisher = NodeConfiguration.newPublic(iiwaConfig.getRobotIp());
         nodeConfPublisher.setTimeProvider(iiwaConfig.getTimeProvider());
         nodeConfPublisher.setNodeName(iiwaConfig.getRobotName() + "/iiwa_publisher");
         nodeConfPublisher.setMasterUri(uri);

         // Configuration for the Subscriber.
         nodeConfSubscriber = NodeConfiguration.newPublic(iiwaConfig.getRobotIp());
         nodeConfSubscriber.setTimeProvider(iiwaConfig.getTimeProvider());
         nodeConfSubscriber.setNodeName(iiwaConfig.getRobotName() + "/iiwa_subscriber");
         nodeConfSubscriber.setMasterUri(uri);

         // Publisher and Subscriber nodes are executed. Their onStart method is called here.
         nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
         nodeMainExecutor.execute(publisher, nodeConfPublisher);
         nodeMainExecutor.execute(subscriber, nodeConfSubscriber);
         nodeMainExecutor.execute(configuration, nodeConfConfiguration);

       }
       catch (Exception e) {
         if (debug) getLogger().info("Node Configuration failed.");
         getLogger().info(e.toString());
       }

   if (debug)
     getLogger().info("ROS Nodes initialized.");

   initSuccessful = true;
 }

 public void run() {
	 getLogger().info("Run");
	 //robot.move(new PTP(new JointPosition(0, 0.523599, 0, -1.5708, 0, -0.523599, 0 )).setJointVelocityRel(0.02));
	  
	 robot.move(ptp(home1).setJointVelocityRel(5.0));
	 // save the start and end point by manually
	 getLogger().info("Recording frames starting, press button when positioned");
	 boolean end_flag = true;
	 
	 // record
	 /*
	 while (end_flag) {
		 IMotionContainer positionHoldContainer = robot.moveAsync((new PositionHold(redundCIC, -1, null)));
		 int ret = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Save this position?", "Save point P", "Save point Q", "DONE");
		 positionHoldContainer.cancel();
					
		 switch (ret) {
		 case 0:
			 pointP = new Frame();
			 pointP = robot.getCurrentCartesianPosition(tcp_tool);
			 try {
				publisher.publishCurrentState(robot, motion);
			} catch (InterruptedException e) {
				getLogger().info("point P publish error. " + e.toString());
			}
			 getLogger().info("point P Frame Saved [ "+pointP +"]");
			 getLogger().info("P: x = " + robot.getCurrentCartesianPosition(robot.getFlange()).getX() + ", y = " + robot.getCurrentCartesianPosition(robot.getFlange()).getY() + ", z = " + robot.getCurrentCartesianPosition(robot.getFlange()).getZ());
			 break;			 
		 case 1:
			 pointQ = new Frame();
			 pointQ = robot.getCurrentCartesianPosition(tcp_tool);
			 try {
				publisher.publishCurrentState(robot, motion);
			} catch (InterruptedException e) {
				getLogger().info("point Q publish error. " + e.toString());
			}
			 getLogger().info("point Q Frame Saved [ "+pointQ +"]");
			 getLogger().info("Q: x = " + robot.getCurrentCartesianPosition(robot.getFlange()).getX() + ", y = " + robot.getCurrentCartesianPosition(robot.getFlange()).getY() + ", z = " + robot.getCurrentCartesianPosition(robot.getFlange()).getZ());
			 break;			 
		 case 2:
			 end_flag = false;	// end on black button
			 break;			 
		 }	 
	 }	// end of record
	 */
	 
	//impedance control
	   CartesianImpedanceControlMode carImp = new CartesianImpedanceControlMode();
	   //compliant only in the x direction
	   carImp.parametrize(CartDOF.X,CartDOF.Y).setStiffness(5000.0);	//max, Nm
	   carImp.parametrize(CartDOF.Z).setStiffness(30.0);
	   carImp.parametrize(CartDOF.ROT).setStiffness(300.0);	//max
	   carImp.parametrize(CartDOF.ALL).setDamping(1.0);		//max	
	   
	 robot.move(new PTP(home1).setJointVelocityRel(0.1).setMode(carImp));
	 
	 //repeat record
	 /*
	 getLogger().info("P:" + pointP.getX());
	 //pointP.setX(pointP.getX() + 10);
	 getLogger().info("move to P:" + pointP.getX());
	 getLogger().info("Q:" + pointQ.getX());
	 //pointQ.setX(pointQ.getX() + 10);
	 getLogger().info("move to Q:" + pointQ.getX());
	 
	 tcp_tool.move(lin(pointP).setCartVelocity(30).setMode(carImp)); // mm/s
	 ThreadUtil.milliSleep(2000);
	 tcp_tool.move(lin(pointQ).setCartVelocity(30).setMode(carImp));
	 ThreadUtil.milliSleep(2000);
	 */
	 //end of repeat record
	 
   if (!initSuccessful) {
     throw new RuntimeException("Could not init the RoboticApplication successfully");
   }
   getLogger().info("using time provider: " + iiwaConfig.getTimeProvider().getClass().getSimpleName());

   try {
	   getLogger().info("Waiting for ROS Master to connect... ");
	   configuration.waitForInitialization();
	   getLogger().info("ROS Master is connected!");
   } catch (InterruptedException e1) {
	   e1.printStackTrace();
	   return;
   }

   motion = new SmartServo(robot.getCurrentJointPosition());
   motion.setMinimumTrajectoryExecutionTime(8e-3);
   motion.setJointVelocityRel(configuration.getDefaultRelativeJointSpeed());
   getLogger().info("JointVel: " + configuration.getDefaultRelativeJointSpeed());
   motion.setTimeoutAfterGoalReach(300);
   motion.setMode(carImp); //impedance based control

   // configurable toolbars to publish events on topics
   configuration.setupToolbars(getApplicationUI(), publisher, generalKeys, generalKeyLists, generalKeyBars);

   String toolFromConfig = configuration.getToolName();
   if (toolFromConfig != "") {
     getLogger().info("attaching tool " + toolFromConfig);
     tool = (Tool)getApplicationData().createFromTemplate(toolFromConfig);
     tool.attachTo(robot.getFlange());
   } else {
     getLogger().info("no tool attached");
   }
	
   
   
   
   // publish joint state?
   publisher.setPublishJointStates(configuration.getPublishJointStates());
   //robot.moveAsync(motion);
   robot.move(motion);

   // The run loop
   getLogger().info("Starting the ROS Command loop...");
   
   try {
	   getLogger().info("start try-catch block");
	   while(true) {
		   if (iiwaConfig.getTimeProvider() instanceof org.ros.time.NtpTimeProvider) {
        		((NtpTimeProvider) iiwaConfig.getTimeProvider()).updateTime();
            }
		   
		   publisher.publishCurrentState(robot, motion);
		   
		   if (subscriber.currentCommandType != null) {
			   configureSmartServoLock.lock(); // the service could stop the motion and restart it
			   
			   switch (subscriber.currentCommandType) {
			   case CARTESIAN_POSE: {
				   //getLogger().info("Cartesian_pose");
				   PoseStamped commandPosition = subscriber.getCartesianPose(); // TODO: check that frame_id is consistent
				   Transformation tr = helper.rosPoseToKukaTransformation(commandPosition.getPose());

				   /*
				   //get force from flange
				   ForceSensorData ft_data = robot.getExternalForceTorque(robot.getFlange());
				   Vector force = ft_data.getForce();
				   double forceInZ = force.getZ();
				   getLogger().info("force in z: " + forceInZ);
				   */
				   
				   motion.getRuntime().setDestination(home1);
				   //motion.getRuntime().setDestination(pointP);


	              	
			   } break;
			   
			   case JOINT_POSITION: {
				   getLogger().info("joint_pose");
				   iiwa_msgs.JointPosition commandPosition = subscriber.getJointPosition();
				   JointPosition jp = helper.rosJointPositionToKuka(commandPosition);
				   
				   if (robot.isReadyToMove())
					   motion.getRuntime().setDestination(jp);   
			   } break;
			   
			   default:
				   throw new UnsupportedControlModeException();
   
			   }
			   configureSmartServoLock.unlock();   
		   }//end of if   
	   }//end of while
        
      } catch (Exception ex) {
        getLogger().info("ROS loop aborted. " + ex.toString());
      }
      finally {
        // The ROS nodes are killed.
        if (nodeMainExecutor != null) {
          nodeMainExecutor.shutdownNodeMain(publisher);
          nodeMainExecutor.shutdownNodeMain(subscriber);
          nodeMainExecutor.shutdownNodeMain(configuration);
        }
        motion.getRuntime().stopMotion();
        if (debug)getLogger().info("ROS Node terminated.");
      }
      getLogger().info("ROS loop has ended. Application terminated.");
    }


 @Override
 public void dispose() {
   // The ROS nodes are killed.
   if (nodeMainExecutor != null && publisher != null && subscriber != null) {
     nodeMainExecutor.shutdownNodeMain(publisher);
     nodeMainExecutor.shutdownNodeMain(subscriber);
     nodeMainExecutor.shutdownNodeMain(configuration);
     getLogger().info("ROS nodes have been terminated by Garbage Collection.");
   }
   super.dispose();
 }


}
