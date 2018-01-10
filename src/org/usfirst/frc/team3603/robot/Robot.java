package org.usfirst.frc.team3603.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends IterativeRobot {
	
	//All of these are individual speed controllers
	WPI_TalonSRX leftFront = new WPI_TalonSRX(1);
	WPI_TalonSRX leftMiddle = new WPI_TalonSRX(2);
	WPI_TalonSRX leftBack = new WPI_TalonSRX(3);
	WPI_TalonSRX rightFront = new WPI_TalonSRX(4);
	WPI_TalonSRX rightMiddle = new WPI_TalonSRX(5);
	WPI_TalonSRX rightBack = new WPI_TalonSRX(6);
	
	//This groups the speed controllers into left and right
	SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftMiddle, leftBack);
	SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightMiddle, rightBack);
	
	//This groups them into the new type of RobotDrive
	DifferentialDrive mainDrive = new DifferentialDrive(left, right);
	
	Joystick joy1 = new Joystick(0); //Large twist-axis joystick
	
	DriverStation matchInfo = DriverStation.getInstance(); //Field data object
	
	String sides; //A string to store the switch and scale colors
	Alliance alliance; //An Alliance object to store the alliance
	int position; //An integer to store the starting position
	
	@Override
	public void robotInit() {
		left.setInverted(true); //Invert the left speed controllers
		mainDrive.setSafetyEnabled(false); //Disable safety
	}
	@Override
	public void autonomousInit() {
		sides = matchInfo.getGameSpecificMessage(); //Get the switch and scale colors
		position = matchInfo.getLocation(); //Get the robot's position
		alliance = matchInfo.getAlliance(); //Get the alliance color
	}
	@Override
	public void autonomousPeriodic() {
		
	}
	@Override
	public void teleopPeriodic() {
		double y = Math.pow(joy1.getRawAxis(0), 3); //Double to store the joystick's y axis
		double rot = Math.pow(joy1.getRawAxis(1), 3); //Double to store the joystick's x axis
		if(Math.abs(y) >= 0.05 || Math.abs(rot) >= 0.05) { //Thresholding function
			mainDrive.arcadeDrive(y, rot); //Arcade drive with the joystick's axis
		}
		read(); //Read from sensors and put them on the SmartDashboard
	}
	
	void read() {
	}
	@Override
	public void testPeriodic() {
	}
}