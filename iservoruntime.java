package hapticRendering;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.util.ArrayList;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import additionalFunction.WriteFrameToAPIdataXML;
import application.TBD;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.ITaskLogger;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.IServoRuntime;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.persistenceModel.IPersistenceEngine;
import com.kuka.roboticsAPI.persistenceModel.XmlApplicationDataSource;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class iservoruntime extends RoboticsAPIApplication {
	private Controller	cabinet;
	private LBR			robot;
	private Tool		tool;
	private Frame 		tool_link_ee;
	private SmartServo 	motion;
	private Lock 		configureSmartServoLock = new ReentrantLock();
	private double 		initial_velocity = 0.1;
	private boolean 	initSuccessful = false;
	private boolean 	debug = false;
	private static ITaskLogger	logger;
	private IServoRuntime theServoRuntime;
	
	// frames & position
	JointPosition home = new JointPosition(0, Math.toRadians(30), 0, -Math.toRadians(60), 0, Math.toRadians(90), 0);

	public void initialize() {
		cabinet = getController("KUKA_Sunrise_Cabinet_1");
		robot = getContext().getDeviceFromType(LBR.class);
		logger = getLogger();
		tool = getApplicationData().createFromTemplate("tool");
		tool.attachTo(robot.getFlange());
		
		tool.move(new PTP(new JointPosition(0, 0.523599, 0, -1.5708, 0, -0.523599, 0 )).setJointVelocityRel(initial_velocity));	
	}

	public void run() {
		getLogger().info("---RUN---");
		
		motion = new SmartServo(robot.getCurrentJointPosition());//1
		motion.setMinimumTrajectoryExecutionTime(8e-3);
		motion.setJointVelocityRel(initial_velocity);
		motion.setTimeoutAfterGoalReach(300);

		tool_link_ee = robot.getCurrentCartesianPosition(tool.getFrame("/tool_link_ee"));
		getLogger().info("tool position: " + tool_link_ee.getX() + ", " + tool_link_ee.getY() + ", " + tool_link_ee.getZ());
		
		tool.getDefaultMotionFrame().moveAsync(motion);//2
		
		getLogger().info("A");
		
		theServoRuntime = motion.getRuntime();//3
		
		getLogger().info("B");
		
		//Frame startFrame = robot.getCurrentCartesianPosition(tool.getDefaultMotionFrame(), World.Current.getRootFrame());
		//getLogger().info("x:" + startFrame.getX()+ " y:" + startFrame.getY()+"z:" + startFrame.getZ());
		//Frame new_frame = (new Frame(startFrame));
		//new_frame.setX(810).setY(0).setZ(500);
		
		//Frame currf = theServoRuntime.getCurrentCartesianPosition(tool_link_ee);
		Frame currf = theServoRuntime.getCurrentCartesianPosition(tool.getDefaultMotionFrame());
		getLogger().info("current x:" + currf.getX()+ " y:" + currf.getY()+"z:" + currf.getZ());
		
		currf.setX(812);
		currf.setY(0);
		currf.setZ(500);
		
		getLogger().info("C");
		
		while (true) {
			
			if (robot.isReadyToMove()) {
				getLogger().info("D");
	        	theServoRuntime.setDestination(currf);//4
				getLogger().info("E");
	        }
		}

	}
	
	public static void main(String[] args) {
		iservoruntime app = new iservoruntime();
		app.runApplication();
	}
	
}
