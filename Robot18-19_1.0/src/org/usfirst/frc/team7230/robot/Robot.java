/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Mike Polucha was here.
//Swetha has made a presence in the universe.

package org.usfirst.frc.team7230.robot;

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
//import org.usfirst.frc.team7230.robot.commands.ExampleCommand;
import org.usfirst.frc.team7230.robot.subsystems.ExampleSubsystem;

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
	
	
	
	private enum autoModes {FIRST_RUN, TURN, RETURN, DONE, REST, CUBE_UP};
	
	private autoModes autoMode;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		autoPositionChooser = new SendableChooser<String>();
		autoPositionChooser.addDefault("Left", "Left");
		autoPositionChooser.addObject("Middle", "Middle");
		autoPositionChooser.addObject("Right", "Right");
		SmartDashboard.putData("Robot Position", autoPositionChooser);
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
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		enc_0.reset();
	SmartDashboard.putNumber("Distance Traveled0", enc_0.getDistance());
	enc_1.reset();
	SmartDashboard.putNumber("Distance Traveled1", enc_1.getDistance());
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		if(gameData.charAt(0) == 'L')
		{
			System.out.println("The scale is on the left");
			//Put left auto code here
		}
		else {
			System.out.println("The scale is on the right");
			//Put right auto code here
		}
		
		String choice = autoPositionChooser.getSelected();
		
		System.out.println("Initializing plan [" + choice + "] ");
		
		m_timer.reset();
		m_timer.start();
		autoMode = autoModes.FIRST_RUN;

		
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)-+
		
	}
	

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putNumber("Distance Traveled0", enc_0.getDistance());
		SmartDashboard.putNumber("Distance Traveled1", enc_1.getDistance());
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		double H=1, K=1,F=1/*stand in for user input*/, G=1/* stand in for user input*/;
		
		/* if ((enc_0.getDistance()+enc_1.getDistance()) < F-H) {
			m_robotDrive.arcadeDrive(.65,m_robotPID.getError()*5);
			s_robotDrive.arcadeDrive(.65,m_robotPID.getError()*5);	 
		 }
		 else if (enc_0.getDistance() < 45) {
			 m_robotDrive.arcadeDrive(0,50);
			s_robotDrive.arcadeDrive(0,50);	 
			m_robotPID.reset();
			enc_0.reset();
			enc_1.reset();
		 }
		 if ((enc_0.getDistance()+enc_1.getDistance()) < G-K) {
			 m_robotDrive.arcadeDrive(.65,m_robotPID.getError()*5);
			s_robotDrive.arcadeDrive(.65,m_robotPID.getError()*5);	 
		 }
		 
		 else {
			 m_robotDrive.stopMotor();
			 s_robotDrive.stopMotor();
		 } */
		
		if ((gameData.charAt(0) == 'L'&& autoPositionChooser.getSelected() == "Left") ||
			(gameData.charAt(0) == 'R'&& autoPositionChooser.getSelected() == "Right" ))
		{
			
			if(autoMode==autoModes.FIRST_RUN) {
			m_timer.reset();
			
				if ((enc_0.getDistance()+enc_1.getDistance())/2 > -90) {
			
					m_robotDrive.arcadeDrive(.65,m_robotPID.getError()*5);
					s_robotDrive.arcadeDrive(.65,m_robotPID.getError()*5);
				
				}
				else {
					
				m_robotDrive.stopMotor();
				s_robotDrive.stopMotor();
				m_timer.reset();
				m_robotPID.reset();
				enc_1.reset();
				enc_0.reset();
				autoMode = autoModes.CUBE_UP;
				
				}	
			}
			
			else if(autoMode==autoModes.CUBE_UP) {
		
				if (1< m_timer.get()&& m_timer.get() < 2.9) {
						
					l_output.set(ControlMode.PercentOutput, 0.35);
					r_output.set(ControlMode.PercentOutput,-0.35);
				
				}
					
				else {
							
					l_output.set(ControlMode.PercentOutput, .00);
					r_output.set(ControlMode.PercentOutput, .00);
					
				}
			}
		}
		
	
		
		
	
		else {
			if(autoMode==autoModes.FIRST_RUN) {
				m_timer.reset();
		
				if ((enc_0.getDistance()+enc_1.getDistance())/2 > -90) {
			
					m_robotDrive.arcadeDrive(.65,m_robotPID.getError()*5);
					s_robotDrive.arcadeDrive(.65,m_robotPID.getError()*5);
				
				}
				
				else {
					
				m_robotDrive.stopMotor();
				s_robotDrive.stopMotor();
				
				}
			}
		}
	}
					
		

	@Override
	public void teleopInit() {
		enc_0.reset();
		SmartDashboard.putNumber("Distance Traveled0", enc_0.getDistance());
		enc_1.reset();
		SmartDashboard.putNumber("Distance Traveled1", enc_1.getDistance());
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		
		}
	

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putNumber("Distance Traveled0", enc_0.getDistance());
		SmartDashboard.putNumber("Distance Traveled1", enc_1.getDistance());
		
		double Y= m_stick.getY();
		double X=m_stick.getX();
	
// conveyor
		if(Button_1.get()==true) {
			
			l_output.set(ControlMode.PercentOutput, .6);
			r_output.set(ControlMode.PercentOutput,-.6);
			
		}
		
		
		
		 else if (Button_2.get()==true) {
			 
			l_output.set(ControlMode.PercentOutput, -.6);
			r_output.set(ControlMode.PercentOutput,.6);
			
		}
		
		else {
			
			l_output.set(ControlMode.PercentOutput, 0.00);
			r_output.set(ControlMode.PercentOutput,00);
			
		}
		
		
		//driving
		
		
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