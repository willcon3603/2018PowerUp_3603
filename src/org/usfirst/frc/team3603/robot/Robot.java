package org.usfirst.frc.team3603.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	DoubleSolenoid.Value out = DoubleSolenoid.Value.kForward;
	DoubleSolenoid.Value in = DoubleSolenoid.Value.kReverse;
	
	//All of these are individual speed controllers
	WPI_TalonSRX leftFront = new WPI_TalonSRX(1);
	Spark yo = new Spark(0);
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
	Joystick joy2 = new Joystick(1);
	Ultrasonic ultrasonic = new Ultrasonic(0);
	
	Servo leftRamp = new Servo(0);
	Servo rightRamp = new Servo(1);
	DoubleSolenoid pusher = new DoubleSolenoid(0, 1);
	WPI_TalonSRX lift = new WPI_TalonSRX(7);
	
	DriverStation matchInfo = DriverStation.getInstance(); //Field data object
	
	String sides; //A string to store the switch and scale colors
	int position; //An integer to store the starting position
	char scalePos;
	char switchPos;
	AutonType autonMode;
	int step;
	
	@Override
	public void robotInit() {
		leftRamp.set(0);
		rightRamp.set(0);
		left.setInverted(true); //Invert the left speed controllers
		mainDrive.setSafetyEnabled(false); //Disable safety
		
		Thread read = new Thread(() -> {
			while(true) {
				read();
				Timer.delay(0.01);
			}
		});
		read.start();
	}
	@Override
	public void autonomousInit() {
		step = 0;
		sides = matchInfo.getGameSpecificMessage(); //Get the switch and scale colors
		switchPos = sides.charAt(0);
		scalePos = sides.charAt(1);
		position = matchInfo.getLocation(); //Get the robot's position
		
		if(position == 1 && switchPos == 'L') {//If we can go for the left switch...
			autonMode = AutonType.leftSwitch;
		} else if(position == 3 && switchPos == 'R') {//If we can go for the right switch
			autonMode = AutonType.rightSwitch;
		} else if(position == 1 && scalePos == 'L') {//If we can go for the left scale
			autonMode = AutonType.leftScale;
		} else if(position == 3 && scalePos == 'R') {//If we can go for the right scale
			autonMode = AutonType.rightScale;
		} else if(position == 2) {//If we are in position two
			autonMode = AutonType.middle;
		} else {//If none of those are true
			autonMode = AutonType.straight;
		}
	}
	@Override
	public void autonomousPeriodic() {
		switch(autonMode) {
		case straight:
			straight(); //Drive straight for auton
			break;
		case leftSwitch:
			leftSwitch(); //Go to the left side of the switch 
			break;
		case rightSwitch:
			rightSwitch(); //Go to the right side of the switch
			break;
		case middle:
			middle(); //Curve slightly to the right
			break;
		case leftScale:
			leftScale(); //Go to the left side of the scale
			break;
		case rightScale:
			rightScale(); //Go to the right side of the scale
			break;
		}
	}
	
	@Override
	public void teleopPeriodic() {
		double y = Math.pow(joy1.getRawAxis(0), 3); //Double to store the joystick's y axis
		double rot = Math.pow(joy1.getRawAxis(1), 3); //Double to store the joystick's x axis
		if(Math.abs(y) >= 0.05 || Math.abs(rot) >= 0.05) { //Thresholding function
			mainDrive.arcadeDrive(y, rot); //Arcade drive with the joystick's axis
		}
		if(Timer.getMatchTime() <= 30 && joy1.getRawButton(2)) {
			leftRamp.set(1);
			rightRamp.set(1);
		}
		if(joy1.getRawButton(1)) {
			pusher.set(out);
		} else {
			pusher.set(in);
		}
		if(Math.abs(joy2.getRawAxis(1)) >= 0.05) {
			lift.set(joy2.getRawAxis(1));
		}
	}
	
	void read() {
		SmartDashboard.putNumber("Ultrasonic distance", ultrasonic.get());
	}
	
	@Override
	public void testPeriodic() {
	}
	
	enum AutonType {
		rightScale, leftScale, rightSwitch, leftSwitch, straight, middle
	}
	
	void straight() {
		double distance = ultrasonic.get();
		if(distance < 120) {
			drive(0.75);
		} else {
			drive(0);
		}
	}
	void rightScale() {
	}
	void leftScale() {
	}
	void rightSwitch() {
	}
	void leftSwitch() {
	}
	void middle() {
	}
	void drive(double speed) {
		mainDrive.arcadeDrive(speed, 0);
	}
	void turn(double speed) {
		mainDrive.arcadeDrive(0, speed);
	}
	void place() {
	}
}
