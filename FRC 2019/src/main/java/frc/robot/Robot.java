/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Pixy2.LinkType;
import frc.robot.links.Link;
import frc.robot.links.SPILink;
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

import java.awt.Color;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PIDOutput;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.buttons.POVButton;


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
	private TalonSRX l_elevator = new TalonSRX(0); //wavs 0
	private TalonSRX r_elevator = new TalonSRX(1); //was 1
	private VictorSPX intake = new VictorSPX(2); 
	private Joystick m_stick = new Joystick(0);
	private Joystick m_grab = new Joystick(1);
	private Timer m_timer = new Timer(); 
	EncoderPIDSource eSource = new EncoderPIDSource(enc_0, enc_1);
	EncoderPIDOutput eOutput = new EncoderPIDOutput();
	private PIDController m_robotPID = new PIDController(1,1,1,1,eSource,eOutput);
	SendableChooser<String> autoPositionChooser;
	private Button Button_6 = new JoystickButton(m_stick, 6),
			 Button_7 = new JoystickButton(m_stick,7),
			 Button_8 = new JoystickButton(m_stick,8),
			 Button_9 = new JoystickButton(m_stick,9),
			 Button_11 = new JoystickButton(m_stick,11),
			 Button_10 = new JoystickButton(m_stick,10),
			 Button_1 = new JoystickButton(m_stick,1),
			 Button_2 = new JoystickButton(m_stick, 2),
			 Button_12 = new JoystickButton(m_stick,12),
			 Button_4 = new JoystickButton(m_stick, 4),
			 Button_3 = new JoystickButton(m_stick, 3),
			 Button_5 = new JoystickButton(m_stick, 5),
			 Button_7_2 = new JoystickButton(m_grab,7);
			 	private POVButton POVButton_0 = new POVButton(m_stick, 0),
	POVButton_1 = new POVButton(m_stick, 360),
	POVButton_3 = new POVButton(m_stick, 90),
	POVButton_5 = new POVButton(m_stick, 180),
	POVButton_7 = new POVButton(m_stick, 270);
	private PowerDistributionPanel PDP = new PowerDistributionPanel(0);

	private	 NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	private	 NetworkTableEntry tx = table.getEntry("tx");
	private  NetworkTableEntry ty = table.getEntry("ty");
	private  NetworkTableEntry ta = table.getEntry("ta");
	private  NetworkTableEntry tv = table.getEntry("tv");
	//private UsbCamera camera1 = new UsbCamera("camera0",0);
	private Pixy2 ourPixy = Pixy2.createInstance(LinkType.SPI);
	//private CameraServer camera2 = new UsbCamera(2);

	private Compressor compressor = new Compressor();
	//private DoubleSolenoid solenoid_1 = new DoubleSolenoid(0,1);
	private DoubleSolenoid sSolenoid = new DoubleSolenoid(0,1);
	Preferences prefs;
	double elevator_speed;
	double drive_speed;
	double intake_speed;
	double fine_drive;
	double fine_turn;
	double turn_speed;


			 

	
	


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
	

		ourPixy.init();
		ourPixy.getFPS();
		System.out.print(ourPixy.getFPS());
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
		table.getEntry("camMode").setNumber(1);


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
	SmartDashboard.putNumber("POV", m_stick.getPOV());
	SmartDashboard.putNumber("LimelightTarget", b);
	SmartDashboard.putNumber("LimelightX", x);
	SmartDashboard.putNumber("LimelightY", y);
	SmartDashboard.putNumber("LimelightArea", area);
	SmartDashboard.putBoolean("Pressure", compressor.getPressureSwitchValue());
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


  //elevator control code
  public void elevator()
  {
	prefs = Preferences.getInstance();

	  if(PDP.getCurrent(14) < 60)
	  { 
	  	if(m_stick.getRawAxis(5)>.1 || m_stick.getRawAxis(5)<-.14)
	  	{
			r_elevator.set(ControlMode.PercentOutput,-m_stick.getRawAxis(5)* prefs.getDouble("elevator_speed", 1.0));
			l_elevator.set(ControlMode.PercentOutput,-m_stick.getRawAxis(5)* prefs.getDouble("elevator_speed", 1.0));
		}
		else if(m_grab.getRawAxis(0)>.1 || m_grab.getRawAxis(0)<-.14)
	  	{
			r_elevator.set(ControlMode.PercentOutput,-m_grab.getRawAxis(0)* prefs.getDouble("elevator_speed", 1.0));
			l_elevator.set(ControlMode.PercentOutput,-m_grab.getRawAxis(0)* prefs.getDouble("elevator_speed", 1.0));
	  	}
		else
		{
			r_elevator.set(ControlMode.PercentOutput,0);
			l_elevator.set(ControlMode.PercentOutput,0);
		}
	  }
	  else
	  {
		r_elevator.set(ControlMode.PercentOutput,0);
		l_elevator.set(ControlMode.PercentOutput,0);
	  }
  }


  //fine control using the Dpad
 

  //robot limelight and default drive code
  public void drive()
  {
	prefs = Preferences.getInstance();
	double X=m_stick.getX()* prefs.getDouble("turn_speed", 1.0);
	double Y= m_stick.getY()* prefs.getDouble("drive_speed", 1.0);
	double b = tv.getDouble(0.0);
	double x = tx.getDouble(0.0);
	double y = ty.getDouble(0.0);
	double area = ta.getDouble(0.0);
	
	SmartDashboard.putNumber("LimelightTarget", b);
	SmartDashboard.putNumber("LimelightX", x);
	SmartDashboard.putNumber("LimelightY", y);
	SmartDashboard.putNumber("LimelightArea", area);

	/*if(Button_1.get() == true && b==1)
	{
		if(m_stick.getRawAxis(2)==0)
		{
		intake.set(ControlMode.PercentOutput, .2);
		}
		if(x>1)
		{
		m_robotDrive.arcadeDrive(.7,.045*x-.045);
		s_robotDrive.arcadeDrive(.7,.045*x-.045);
		//System.out.println("x = " + x);
		}
		else if(x<1)
		{
		m_robotDrive.arcadeDrive(.7,.045*x+.045);
		s_robotDrive.arcadeDrive(.7,.045*x+.045);
		
		}
		else
		{
			m_robotDrive.arcadeDrive(.7,0 );
			s_robotDrive.arcadeDrive(.7,0 );
		}
	}
	else if(Button_2.get() == true && b==1)
	{
		
		if(x>1)
		{
		m_robotDrive.arcadeDrive(0,.045*x-.045);
		s_robotDrive.arcadeDrive(0,.045*x-.045);
	
		}
		else if(x<1)
		{
		m_robotDrive.arcadeDrive(0,.045*x+.045);
		s_robotDrive.arcadeDrive(0,.045*x+.045);
		
		}
		else
		{
			m_robotDrive.arcadeDrive(0,0 );
			s_robotDrive.arcadeDrive(0,0 );
		}
	}
*/

	
	if (m_stick.getPOV() == -1)
	{
		if(Y>=0){
			m_robotDrive.arcadeDrive(Y,X);
			s_robotDrive.arcadeDrive(Y,X);
		}
		if(Y<=0){
			m_robotDrive.arcadeDrive(Y,X);
			s_robotDrive.arcadeDrive(Y,X);
		}
	}
	else { 
		if(m_stick.getPOV() == 0.0 )
		{
			m_robotDrive.arcadeDrive(-prefs.getDouble("fine_drive",.55),0);
			s_robotDrive.arcadeDrive(-prefs.getDouble("fine_drive",.55),0);
		}
		
		if(m_stick.getPOV() == 180.0 )
		{
			m_robotDrive.arcadeDrive(prefs.getDouble("fine_drive",.55),0);
			s_robotDrive.arcadeDrive(prefs.getDouble("fine_drive",.55),0);
		}
		if(m_stick.getPOV() == 270.0 )
		{
			m_robotDrive.arcadeDrive(0,-prefs.getDouble("fine_turn",.55));
			s_robotDrive.arcadeDrive(0,-prefs.getDouble("fine_turn",.55));
		}
		
		if(m_stick.getPOV() == 90.0 )
		{
			m_robotDrive.arcadeDrive(0,prefs.getDouble("fine_turn",.55));
			s_robotDrive.arcadeDrive(0,prefs.getDouble("fine_turn",.55));
		}
	}
  }



  //intake control code 
  public void intake()
  {
	double b = tv.getDouble(0.0);
	if(m_stick.getRawAxis(2)==0 && Button_1.get() == true)
		{
		intake.set(ControlMode.PercentOutput, .3);
		}
	else
	{
		if(m_stick.getRawAxis(2) > .1)
		{
			intake.set(ControlMode.PercentOutput, m_stick.getRawAxis(2)* prefs.getDouble("intake_speed", 0.7));
		}	
		else if (m_grab.getRawAxis(1) > .1)
		{
			intake .set(ControlMode.PercentOutput, m_stick.getRawAxis(1)* prefs.getDouble("intake_speed", 0.7));
		}

		else if(m_stick.getRawAxis(3) > .1)
		{
			intake.set(ControlMode.PercentOutput, -																																																																																				m_stick.getRawAxis(3)* prefs.getDouble("intake_speed", 0.7));
		}	
		else if(m_grab.getRawAxis(1) < -.1)
		{
			intake.set(ControlMode.PercentOutput, m_stick.getRawAxis(1)* prefs.getDouble("intake_speed", 0.7));
		}	
		else
		{
			intake.set(ControlMode.PercentOutput, 0);	
		}
	}
	

	
		
	
  }



   // pnuematics control code
  public void pnuematics()
  {
	if(Button_8.get() == true)
		compressor.start();
	if(Button_7.get() == true || compressor.getPressureSwitchValue() == true)
	{	
		compressor.stop();
	}
	
	if(Button_6.get() == true)
	{
		sSolenoid.set(Value.kReverse);
	}
	else if (Button_5.get() == true)
	{
		sSolenoid.set(Value.kForward);
	}
	else 
	{
		sSolenoid.set(Value.kOff);
	}	
  }



 
  public void robotCode() {
	elevator();
	drive();
	intake();
	pnuematics();
	
	if(Button_7.get()==true)
	{
		table.getEntry("pipeline").setNumber(0);
		table.getEntry("camMode").setNumber(0);
	}
	else if(Button_3.get() == true)
	{
		table.getEntry("pipeline").setNumber(1);
		table.getEntry("camMode").setNumber(0);
	}
	else if(Button_4.get() == true)
	{
		table.getEntry("pipeline").setNumber(2);
		table.getEntry("camMode").setNumber(0);
	}
  }
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() 
  {
	 robotCode();
  }
  

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
    {
	
		robotCode();
	}	
	
  

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
