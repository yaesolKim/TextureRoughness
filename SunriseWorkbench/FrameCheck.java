//test frame orientation without ros node connection
package hapticRendering;

//ROS imports
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
import com.kuka.roboticsAPI.motionModel.IServoRuntime; //Iservo!!!
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.ServoMotion;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;

import com.kuka.roboticsAPI.geometricModel.Frame; //frame
import com.kuka.common.ThreadUtil; //sleep

public class FrameCheck extends RoboticsAPIApplication {
	private Controller		cabinet;
	private LBR 			robot;
	private Tool 			tool;	
	private SmartServo 		motion;
	private Lock 			configureSmartServoLock = new ReentrantLock();
	private double 			initial_velocity = 0.05;
	private double			force_threshold = -1.5; //-2: cannot contact collision
	private IServoRuntime 	theServoRuntime;
	private boolean 		initSuccessful = false;
	private boolean 		debug = false;
	
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
		
		//mot.setMinimumTrajectoryExecutionTime(100e-3);
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
		tool.attachTo(robot.getFlange());
		
		//getLogger().info("move tool with initial vel: " + initial_velocity );  
		//tool.move(new PTP(new JointPosition(0, 0.523599, 0, -1.5708, 0, -0.523599, 0 )).setJointVelocityRel(initial_velocity));
		
		// SmartServo configuration service callback
		subscriber.setConfigureSmartServoCallback(new ServiceResponseBuilder<iiwa_msgs.ConfigureSmartServoRequest, iiwa_msgs.ConfigureSmartServoResponse>() {
			   @Override
		  public void build(ConfigureSmartServoRequest req, ConfigureSmartServoResponse resp) throws ServiceException {
		    // we can change the parameters if it is the same type of control strategy
		    // otherwise we have to stop the motion, replace it and start it again
		    try {
		    	if (motion.getMode() != null && isSameControlMode(motion.getMode(), req.getMode())) {
		    		motion.getRuntime().changeControlModeSettings(buildMotionControlMode(req.getMode()));	
		    	}
		    	else {
		    		configureSmartServoLock.lock();
		    		SmartServo oldmotion = motion;
		    		ServoMotion.validateForImpedanceMode(robot);
		    		motion = configureSmartServoMotion(req.getMode()); //1
		    		tool.getDefaultMotionFrame().moveAsync(motion); //2
		    		theServoRuntime = motion.getRuntime();//3
		        
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
		    	} else {
		    		resp.setError("UNKNOWN ERROR");
		    	}		    	
		    	return;
		    }
		    
		    resp.setSuccess(true);
		    
		}
		}); // End of SmartServo configuration service callback
		
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
		} catch (InterruptedException e1) {
			e1.printStackTrace();
			return; 
		}
		
		motion = new SmartServo(robot.getCurrentJointPosition()); //1
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
		
		ThreadUtil.milliSleep(2000); 
		
		getLogger().info("default frame: " + tool.getDefaultMotionFrame().getName());
		tool.getDefaultMotionFrame().moveAsync(motion);//2
		
		theServoRuntime = motion.getRuntime();//3		
		
		ThreadUtil.milliSleep(5000); 
		
		
		Frame current = theServoRuntime.getCurrentCartesianDestination(tool.getDefaultMotionFrame());
		double a = current.getAlphaRad();
		double b = current.getBetaRad();
		double c = current.getGammaRad();
		getLogger().info("orientation: " + a + "," +b + "," +c);
		
		Frame goalFrame = theServoRuntime.getCurrentCartesianPosition(tool.getDefaultMotionFrame());
		
		goalFrame.setX(812.0);
		goalFrame.setY(0.0);
        	goalFrame.setZ(487.0);
        
        	goalFrame.setAlphaRad(1.5708);
        	goalFrame.setBetaRad(1.5708);
        	goalFrame.setGammaRad(1.5708);
        
		publisher.setPublishJointStates(configuration.getPublishJointStates());
		theServoRuntime.setDestination(goalFrame);//4
		getLogger().info("Set destination 1");
		ThreadUtil.milliSleep(5000);
		
		goalFrame.setAlphaRad(1.5708);
        	goalFrame.setBetaRad(1.178697);
        	//goalFrame.setGammaRad(1.5708);
        	theServoRuntime.setDestination(goalFrame);//4
		getLogger().info("Set destination 2");
		ThreadUtil.milliSleep(5000);
        
		
		goalFrame.setAlphaRad(1.5708);
		goalFrame.setBetaRad(0.785398);
		//goalFrame.setGammaRad(1.5708);
		theServoRuntime.setDestination(goalFrame);//4
		getLogger().info("Set destination 3");
		ThreadUtil.milliSleep(5000);

		goalFrame.setAlphaRad(1.5708);
		goalFrame.setBetaRad(0.3926881);
		//goalFrame.setGammaRad(1.5708);
		theServoRuntime.setDestination(goalFrame);//5
		getLogger().info("Set destination 4");
		ThreadUtil.milliSleep(5000);
		
		goalFrame.setAlphaRad(1.5708);
		goalFrame.setBetaRad(0);
		//goalFrame.setGammaRad(1.5708);
		theServoRuntime.setDestination(goalFrame);//4
		getLogger().info("Set destination 5");
		ThreadUtil.milliSleep(5000);
		
		goalFrame.setAlphaRad(1.5708);
		goalFrame.setBetaRad(1.5708);
		theServoRuntime.setDestination(goalFrame);//4
		getLogger().info("Set destination 1 again");
		ThreadUtil.milliSleep(5000);
		
		goalFrame.setAlphaRad(0.785398);
        	goalFrame.setBetaRad(1.5708);
        	theServoRuntime.setDestination(goalFrame);//4
		getLogger().info("Set destination 2");
		ThreadUtil.milliSleep(5000);
				
		goalFrame.setAlphaRad(0.785398);
		goalFrame.setBetaRad(1.178697);
		theServoRuntime.setDestination(goalFrame);//4
		getLogger().info("Set destination 2");
		ThreadUtil.milliSleep(5000);
		
		goalFrame.setAlphaRad(0.785398);
		goalFrame.setBetaRad(0.785398);
		theServoRuntime.setDestination(goalFrame);//4
		getLogger().info("Set destination 2");
		ThreadUtil.milliSleep(5000);
				
		goalFrame.setAlphaRad(0.785398);
		goalFrame.setBetaRad(0.3926881);
		theServoRuntime.setDestination(goalFrame);//4
		getLogger().info("Set destination 2");
		ThreadUtil.milliSleep(5000);
		goalFrame.setAlphaRad(0.785398);
		goalFrame.setBetaRad(0);
		theServoRuntime.setDestination(goalFrame);//4
		getLogger().info("Set destination 2");
		ThreadUtil.milliSleep(5000);
		
		
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
