package hapticRendering;

//ROS imports
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
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
import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
//import com.kuka.roboticsAPI.motionModel.IServoRuntime; //Iservo!!!
import com.kuka.roboticsAPI.motionModel.ISmartServoLINRuntime;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.ServoMotion;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.SplineOrientationType;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.motionModel.SmartServoLIN;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;


import com.kuka.roboticsAPI.geometricModel.Frame; //frame
import com.kuka.common.StatisticTimer;
import com.kuka.common.ThreadUtil; //sleep
import com.kuka.common.StatisticTimer.OneTimeStep;

public class TexturedHwall_tool extends RoboticsAPIApplication {
	private Controller		cabinet;
	private LBR 			robot;
	
	private Tool 			tool, tool_lin;

	private SmartServo 		motion;
	private SmartServoLIN 	motion_lin;
	
	private ISmartServoRuntime	SSR;
    private ISmartServoLINRuntime SSLR;

	private Lock 			configureSmartServoLock = new ReentrantLock();
	private double 			initial_velocity = 0.05;
	private double			force_threshold = -4.0; //-2: cannot contact collision
	private double			rel_vel; 
	private boolean 		initSuccessful = false;
	private boolean 		debug = false;
	private boolean 		contact = false;
	private ForceSensorData force_data;
	private Vector 			force_vec;
	private double 			forceInZ;
	
	private Frame 			aFrame;
	private StatisticTimer	timing;
    private final double[]	translationOfTool = { 0, 0, 140 };
    private static final int milliSleepToEmulateComputationalEffort = 30;
	
	private iiwaMessageGenen helper; //< Helper class to generate iiwa_msgs from current robot state.
	private iiwaPub publisher; //< IIWARos Publisher.
	private iiwaSub subscriber; //< IIWARos Subscriber.
	private iiwaConfig configuration; //< Configuration via parameters and services.
	
	// ROS Configuration and Node execution objects. Two different configurations are needed for the Publisher and the Subscriber.
	private NodeConfiguration nodeConfPublisher;
	private NodeConfiguration nodeConfSubscriber;
	private NodeConfiguration nodeConfConfiguration;
	private NodeMainExecutor nodeMainExecutor;
	
	// configurable toolbars
	private List<IUserKeyBar> generalKeyBars = new ArrayList<IUserKeyBar>();
	private List<IUserKey> generalKeys = new ArrayList<IUserKey>();
	private List<IUserKeyListener> generalKeyLists = new ArrayList<IUserKeyListener>();
	
	public static class UnsupportedControlModeException extends RuntimeException {
		private static final long serialVersionUID = 1L;
		public UnsupportedControlModeException() { super(); }
		public UnsupportedControlModeException(String message) { super(message); }
		public UnsupportedControlModeException(String message, Throwable cause) { super(message, cause); }
		public UnsupportedControlModeException(Throwable cause) { super(cause); }
	} //end of the UnsupportedControlModeException

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
		robot = getContext().getDeviceFromType(LBR.class);
		
		helper = new iiwaMessageGenen();
		configuration = new iiwaConfig();
		publisher = new iiwaPub(robot, iiwaConfig.getRobotName());
		subscriber = new iiwaSub(robot, iiwaConfig.getRobotName());
				
		tool = getApplicationData().createFromTemplate("tool");
		tool.attachTo(robot.getFlange()); // Attach tool to the robot
		
		tool_lin = getApplicationData().createFromTemplate("tool_lin");
		XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(translationOfTool[0], translationOfTool[1], translationOfTool[2]);
        ObjectFrame aTransformation = tool_lin.addChildFrame("toolFrame" + "(TCP)", trans);
        tool_lin.setDefaultMotionFrame(aTransformation);
		tool_lin.attachTo(robot.getFlange()); // Attach tool to the robot
		
		getLogger().info("move tool with initial vel: " + initial_velocity );  
		tool.move(new PTP(new JointPosition(0, 0.523599, 0, -1.5708, 0, -0.523599, 1.5708)).setJointVelocityRel(initial_velocity));
		
		///////////////////////////// SmartServo configuration service callback
		subscriber.setConfigureSmartServoCallback(new ServiceResponseBuilder<iiwa_msgs.ConfigureSmartServoRequest, iiwa_msgs.ConfigureSmartServoResponse>() {
			@Override
			public void build(ConfigureSmartServoRequest req, ConfigureSmartServoResponse resp) throws ServiceException {
				try {
					if (motion.getMode() != null && isSameControlMode(motion.getMode(), req.getMode())) { //change the parameters if it is the same type of control strategy
						motion.getRuntime().changeControlModeSettings(buildMotionControlMode(req.getMode()));	
					}
					else { // otherwise, stop the motion, replace it and start it again
			    		configureSmartServoLock.lock();
			    		SmartServo oldmotion = motion;
			    		ServoMotion.validateForImpedanceMode(robot);
			    		motion = configureSmartServoMotion(req.getMode());
			    		tool.getDefaultMotionFrame().moveAsync(motion);
			    		SSR = motion.getRuntime();		        
			    		oldmotion.getRuntime().stopMotion();		
			    		configureSmartServoLock.unlock();	
					}	
				}
				catch (Exception e) {
					resp.setSuccess(false);
		    	
					if (e.getMessage() != null) {
						StringWriter sw = new StringWriter();
			    		PrintWriter pw = new PrintWriter(sw);
			    		e.printStackTrace(pw);
			    		resp.setError(e.getClass().getName() + ": " + e.getMessage() + ", " + sw.toString());
			    	}
					else {
			    		resp.setError("UNKNOWN ERROR");
			    	}
					return;
				}
				resp.setSuccess(true);	
			}	
		}); ///////////////////////// End of SmartServo configuration service callback
		
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
			if (debug) {
				getLogger().info("Node Configuration failed.");
			}
			getLogger().info(e.toString());
		}
		
		if (debug) {
			getLogger().info("ROS Nodes initialized.");
		}
		initSuccessful = true;
		
	}
	
	public void run() {
		getLogger().info("-----RUN-----");		
		if (!initSuccessful) {
			throw new RuntimeException("Could not init the RoboticApplication successfully");
		}		
		getLogger().info("using time provider: " + iiwaConfig.getTimeProvider().getClass().getSimpleName());
		
		try {
			getLogger().info("Waiting for ROS Master to connect... ");
			configuration.waitForInitialization();
			getLogger().info("ROS Master is connected!");
		}
		catch (InterruptedException e1) {
			e1.printStackTrace();
			return; 
		}
		
		// Create a new smart servo motion
		motion = new SmartServo(robot.getCurrentJointPosition()); //1-1
		motion.setMinimumTrajectoryExecutionTime(8e-3);
		motion.setJointVelocityRel(configuration.getDefaultRelativeJointSpeed());
		motion.setTimeoutAfterGoalReach(300);        
		
		// configurable toolbars to publish events on topics
		configuration.setupToolbars(getApplicationUI(), publisher, generalKeys, generalKeyLists, generalKeyBars);
		
		String toolFromConfig = configuration.getToolName();		
		if (toolFromConfig != "") {
		  getLogger().info("attaching tool: " + toolFromConfig);
		  tool = (Tool)getApplicationData().createFromTemplate(toolFromConfig);
		  tool.attachTo(robot.getFlange());
		}
		else {
		  getLogger().info("no tool attached");
		}
		
		ThreadUtil.milliSleep(3000); 
		
		getLogger().info("default frame: " + tool.getDefaultMotionFrame().getName());
		tool.getDefaultMotionFrame().moveAsync(motion); //1-2		
		SSR = motion.getRuntime(); //1-3
		
		Frame goalFrame = SSR.getCurrentCartesianPosition(tool.getDefaultMotionFrame());
		
		goalFrame.setX(812.0);
		goalFrame.setY(0.0);
        goalFrame.setZ(487.0);
        
        goalFrame.setAlphaRad(1.5708);
        goalFrame.setBetaRad(0);
        goalFrame.setGammaRad(1.5708);
	
		publisher.setPublishJointStates(configuration.getPublishJointStates());
		
		SSR.setDestination(goalFrame); //1-4
		getLogger().info("Set destination");

		ThreadUtil.milliSleep(2000);
				
		// The run loop
		getLogger().info("Starting the ROS Command loop...");		
		try {
			getLogger().info("start try-catch block");
			while (true) {
				if (iiwaConfig.getTimeProvider() instanceof org.ros.time.NtpTimeProvider) {
					((NtpTimeProvider) iiwaConfig.getTimeProvider()).updateTime();		
				}
				publisher.publishCurrentState(robot, motion); //to confirm the connection in Ubuntu side
				
		       if (subscriber.currentCommandType != null) {
		    	   configureSmartServoLock.lock(); // the service could stop the motion and restart it

		    	   switch (subscriber.currentCommandType) {
		    	   case CARTESIAN_POSE: { 
		    		   PoseStamped commandPosition = subscriber.getCartesianPose();
		    		   
			           goalFrame.setX(commandPosition.getPose().getPosition().getX());
			           goalFrame.setY(commandPosition.getPose().getPosition().getY());
			           goalFrame.setZ(commandPosition.getPose().getPosition().getZ());
			           
			           goalFrame.setAlphaRad(commandPosition.getPose().getOrientation().getX());
			           goalFrame.setBetaRad(commandPosition.getPose().getOrientation().getY());
			           goalFrame.setGammaRad(commandPosition.getPose().getOrientation().getZ());
			           
			           rel_vel = commandPosition.getPose().getOrientation().getW()/5000000000L;
			           
			           force_data = robot.getExternalForceTorque(robot.getFlange());
			           force_vec = force_data.getForce();
			           forceInZ = force_vec.getZ();
			           
			           ///////non-contact state : robot base coordinate
			           if (robot.isReadyToMove()&& (forceInZ > force_threshold)) {			        	   
			        	   if(contact) //contact == true
			        	   {
			        		   // Create a new smart servo motion
			        		   motion = new SmartServo(robot.getCurrentJointPosition()); //1-1
			        		   motion.setMinimumTrajectoryExecutionTime(8e-3);
			        		   motion.setJointVelocityRel(configuration.getDefaultRelativeJointSpeed());
			        		   motion.setTimeoutAfterGoalReach(300);
			        		   
			        		   getLogger().info("No Contact");			        		   
			        		   tool.getDefaultMotionFrame().moveAsync(motion);//1-2: Starting the SmartServo in position control mode
			        		   SSLR.stopMotion(); //2-5: Stop the SmartServoLIN motion			        		   
			        		   SSR = motion.getRuntime();//1-3: Get the runtime of the SmartServo motion

			        		   contact = false;
			        	   }			        	   			   
			        	   SSR.setDestination(goalFrame);//1-4 
			           }

			           ///////contact state : tool coordinate
			           else {
			        	   if(!contact) //contact == false
			        	   {
			        		   // Create a new smart servo linear motion
			        		   motion_lin = new SmartServoLIN(robot.getCurrentCartesianPosition(robot.getFlange())); //2-1
			        		   motion_lin.setMinimumTrajectoryExecutionTime(20e-3);
			        			
			        		   getLogger().info("Contact");
				               robot.getFlange().moveAsync(motion_lin);//2-2: Starting the SmartServoLIN in position control mode
			        		   motion.getRuntime().stopMotion(); //1-5: Stop the SmartServo motion
			        		   SSLR = motion_lin.getRuntime(); //2-3: Get the runtime of the SmartServoLIN motion
				              
				               timing = new StatisticTimer();
				               contact = true;
				        	   //ThreadUtil.milliSleep(1000);   
			        	   }
			        	   
			        	   aFrame = SSLR.getCurrentCartesianDestination(robot.getFlange());
			               long startTimeStamp = System.nanoTime();
			               double setx = 0;
			               while (true)
			               {
			                   final OneTimeStep aStep = timing.newTimeStep(); //starting an individual measurement
			                   ThreadUtil.milliSleep(milliSleepToEmulateComputationalEffort);

			                   // Update the smart servo LIN runtime
			                   SSLR.updateWithRealtimeSystem();
			                   double curTime = System.nanoTime() - startTimeStamp;

			                   // Compute the sine function
			                   Frame destFrame = new Frame(aFrame);
			                   setx += (curTime)*rel_vel;
			                   destFrame.setX(setx);
			                   getLogger().info("setx: " + setx);

			                   // Set new destination
			                   SSLR.setDestination(destFrame);
			                   aStep.end(); //stop the measurement
			                   
			                   force_data = robot.getExternalForceTorque(robot.getFlange());
					           force_vec = force_data.getForce();
					           forceInZ = force_vec.getZ();
					           
					           if(forceInZ >= 0)
					        	   break;
			               }
			        	   getLogger().info("Finish For loop");
			                
			           }
			           
		    	   }		    	   
		    	   break;
		         
		         case JOINT_POSITION: {
		           iiwa_msgs.JointPosition commandPosition = subscriber.getJointPosition();
		           JointPosition jp = helper.rosJointPositionToKuka(commandPosition);
		           
		           if (robot.isReadyToMove())
		             motion.getRuntime().setDestination(jp);
		         }
		         break;
		
		         default:
		         	throw new UnsupportedControlModeException();
		         }
		
		         configureSmartServoLock.unlock();
		       }
		
		     }
		     
		   } catch (Exception ex) {
		     getLogger().info("ROS loop aborted. " + ex.toString());
		     getLogger().info("goal pos: " + goalFrame.getX() + ", "+ goalFrame.getY() + ", "+ goalFrame.getZ());
		     getLogger().info("goal ori: " + goalFrame.getAlphaRad() + ", "+ goalFrame.getBetaRad() + ", "+ goalFrame.getGammaRad());
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
	public void dispose() { // The ROS nodes are killed.
		if (nodeMainExecutor != null && publisher != null && subscriber != null) {
			nodeMainExecutor.shutdownNodeMain(publisher);
			nodeMainExecutor.shutdownNodeMain(subscriber);
			nodeMainExecutor.shutdownNodeMain(configuration);
			getLogger().info("ROS nodes have been terminated by Garbage Collection.");	
		}
		super.dispose();
	}
}
