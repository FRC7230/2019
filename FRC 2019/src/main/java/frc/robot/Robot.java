/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import org.usfirst.frc.team7230.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj.CameraServer;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PIDOutput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this	 class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */

public class Robot extends IterativeRobot {
	//private Pixy2 pixy = new Pixy2(1);
	private Encoder
	enc_0 = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	private Encoder
	enc_1 = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
		private DifferentialDrive m_robotDrive //main
				= new DifferentialDrive(new Spark(0), new Spark(2));
		private DifferentialDrive s_robotDrive //secondary
	= new DifferentialDrive(new Spark(1), new Spark(3));
	private TalonSRX l_intake = new TalonSRX(2); 
	private TalonSRX r_intake = new TalonSRX(1);
	private VictorSPX l_output = new VictorSPX(2); 
	private VictorSPX r_output = new VictorSPX(1); 
	private Joystick m_stick = new Joystick(0);
	private Joystick m_grab = new Joystick(1);
	private Timer m_timer = new Timer();
	EncoderPIDSource eSource = new EncoderPIDSource(enc_0, enc_1);
	EncoderPIDOutput eOutput = new EncoderPIDOutput();
	private PIDController m_robotPID = new PIDController(1,1,1,1,eSource,eOutput);
	SendableChooser<String> autoPositionChooser;
	private Button Button_1 = new JoystickButton(m_grab,5),
			 Button_2 = new JoystickButton(m_grab,6);
	private	 NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	private	 NetworkTableEntry tx = table.getEntry("tx");
	private  NetworkTableEntry ty = table.getEntry("ty");
	private  NetworkTableEntry ta = table.getEntry("ta");
	private  NetworkTableEntry tv = table.getEntry("tv");
	private UsbCamera camera1 = new UsbCamera("camera_1",0);
	//private CameraServer camera2 = new UsbCamera(2);

			 

	
	


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
	

		
		enc_0.setMaxPeriod(.1);
		enc_0.setMinRate(10);
		enc_0.setDistancePerPulse(0.94247779607/12.75*1.21);
		enc_0.setReverseDirection(true);
		enc_0.setSamplesToAverage(7);
		enc_1.setMaxPeriod(.1);
		enc_1.setMinRate(10);
		enc_1.setDistancePerPulse(-0.94247779607/12.75*1.21);
		enc_1.setReverseDirection(true);
		enc_1.setSamplesToAverage(7);
		enc_0.reset();
		enc_1.reset();
		m_robotPID.setPercentTolerance(.01);
		m_robotPID.setSetpoint(0);
		m_robotPID.reset();
		double b = tv.getDouble(0.0);
		double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
		double area = ta.getDouble(0.0);
		new Thread(() -> {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(640, 480);
			
			CvSink cvSink = CameraServer.getInstance().getVideo();
			CvSource outputStream = CameraServer.getInstance().putVideo("camera1", 640, 480);
			
			Mat source = new Mat();
			Mat output = new Mat();
			
			while(!Thread.interrupted()) {
				cvSink.grabFrame(source);
				Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
				outputStream.putFrame(output);
			}
		}).start();
}
	

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

	double b = tv.getDouble(0.0);
	double x = tx.getDouble(0.0);
	double y = ty.getDouble(0.0);
	double area = ta.getDouble(0.0);
	
	SmartDashboard.putNumber("LimelightTarget", b);
	SmartDashboard.putNumber("LimelightX", x);
	SmartDashboard.putNumber("LimelightY", y);
	SmartDashboard.putNumber("LimelightArea", area);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
   
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
    }
  

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    double Y= m_stick.getY();
		double X=m_stick.getX();

	
    
		if(-.1>Y&&Y>-.3){
				
			m_robotDrive.arcadeDrive(-.3,.58*Math.tan(X));
			s_robotDrive.arcadeDrive(-.3,.58*Math.tan(X));
		
		}
		else if (.1>Y&&Y>.3){
			
			m_robotDrive.arcadeDrive(.3,.58*Math.tan(X));
			s_robotDrive.arcadeDrive(.3,.58*Math.tan(X));
			
		}
		
		else {
			
		m_robotDrive.arcadeDrive(Y,/*+(a/4))*/.58*Math.tan(X));
		s_robotDrive.arcadeDrive(Y,/*+(a/4))*/.58*Math.tan(X));
		
		}
		
	}
  

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
