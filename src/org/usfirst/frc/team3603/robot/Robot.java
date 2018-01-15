package org.usfirst.frc.team3603.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	DoubleSolenoid.Value out = DoubleSolenoid.Value.kForward;
	DoubleSolenoid.Value in = DoubleSolenoid.Value.kReverse;
	
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
	
	
	WPI_TalonSRX leftHolder = new WPI_TalonSRX(1);//Leftholder speedcontroller
	WPI_TalonSRX rightHolder = new WPI_TalonSRX(2);//Rightholder speedcontroller
	Joystick joy1 = new Joystick(0); //Large twist-axis joystick
	Joystick joy2 = new Joystick(1);
	Ultrasonic ultrasonic = new Ultrasonic(0);
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	Servo leftRamp = new Servo(0);// Left ramp sevo
	Servo rightRamp = new Servo(1);//Right ramp servo
	DoubleSolenoid pusher = new DoubleSolenoid(0, 1);//Piston neumatics
	WPI_TalonSRX lift = new WPI_TalonSRX(7); //lift speedcontroller
	
	DriverStation matchInfo = DriverStation.getInstance();
	
	String sides; //A string to store the switch and scale colors
	int position; //An integer to store the starting position
	char scalePos;
	char switchPos;
	AutonType autonMode;
	int step;
	
	double time = 0;
	
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
			autonMode = AutonType.straight;
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
		double y = Math.pow(joy1.getRawAxis(1), 3); //Double to store the joystick's y axis
		double rot = Math.pow(joy1.getRawAxis(2), 3); //Double to store the joystick's x axis
		if(Math.abs(y) >= 0.05 || Math.abs(rot) >= 0.05) { //Thresholding function
			mainDrive.arcadeDrive(y, rot); //Arcade drive with the joystick's axis
		}
		if(Timer.getMatchTime() <= 30 && joy1.getRawButton(2)) {
			leftRamp.set(1);
			rightRamp.set(1);
		}
		
		if(Math.abs(joy2.getRawAxis(1)) >= 0.05) {
			lift.set(joy2.getRawAxis(1));
		}
		if(joy2.getRawButton(1)) { //If button A is being pressed...
			leftHolder.set(0.5); //Intake cube
			rightHolder.set(-0.5);
		} else if(joy2.getRawButton(3)) { //If button X is being pressed...
			leftHolder.set(0.5); // Rotate cube
			rightHolder.set(0.5);
		} else if(joy2.getRawButton(4)) { //If button Y is being pressed...
			leftHolder.set(-0.5);// Output cube
			rightHolder.set(0.5);
			pusher.set(out);
		} else { 
			pusher.set(in);//If nothing is being pressed pusher is held in
		}
	}
	
	void read() {
		SmartDashboard.putNumber("Ultrasonic distance", ultrasonic.get());
	}
	
	@Override
	public void testPeriodic() {
	}
	
	enum AutonType {
		rightScale, leftScale, rightSwitch, leftSwitch, straight
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
		switch(step) {
		case 1:
			if(ultrasonic.get() < 300) {
				mainDrive.arcadeDrive(1, 0);
			} else {
				step = 2;
			}
			break;
		case 2:
			if(gyro.getAngle() > -90) {
				mainDrive.arcadeDrive(0, -0.3);
			} else {
				step = 3;
			}
			break;
		case 3:
			if(ultrasonic.get() < 12) {
				mainDrive.arcadeDrive(0.2, 0);
				lift.set(0.5);
			} else {
				step = 4;
				lift.set(0);
				time = Timer.getMatchTime();
			}
			break;
		case 4:
			if(time - Timer.getMatchTime() <= 1.0) {
				leftHolder.set (-1);
				rightHolder.set(1);
				pusher.set(out);
			} else {
				step = 5;
				pusher.set(in);
				leftHolder.set(0);
				rightHolder.set(0);
			}
			break;
		}
	}
	void leftScale() {
		switch(step) {
		case 1:
			if(ultrasonic.get() < 300) {
				mainDrive.arcadeDrive(1, 0);
			} else {
				step = 2;
			}
			break;
		case 2:
			if(gyro.getAngle() < 90) {
				mainDrive.arcadeDrive(0, 0.3);
			} else {
				step = 3;
			}
			break;
		case 3:
			if(ultrasonic.get() < 12) {
				mainDrive.arcadeDrive(0.2, 0);
				lift.set(0.5);
			} else {
				step = 4;
				lift.set(0);
				time = Timer.getMatchTime();
			}
			break;
		case 4:
			if(time - Timer.getMatchTime() <= 1.0) {
				leftHolder.set (-1);
				rightHolder.set(1);
				pusher.set(out);
			} else {
				step = 5;
				pusher.set(in);
				leftHolder.set(0);
				rightHolder.set(0);
			}
			break;
		}
	}
	void rightSwitch() {
		switch(step) {
		case 1:
			if(ultrasonic.get() < 168) {
				mainDrive.arcadeDrive(1, 0);
			} else {
				step = 2;
			}
			break;
		case 2:
			if(gyro.getAngle() > -90) {
				mainDrive.arcadeDrive(0, -0.3);
			} else {
				step = 3;
			}
			break;
		case 3:
			if(ultrasonic.get() < 12) {
				mainDrive.arcadeDrive(0.2, 0);
				lift.set(0.5);
			} else {
				step = 4;
				lift.set(0);
				time = Timer.getMatchTime();
			}
			break;
		case 4:
			if(time - Timer.getMatchTime() <= 1.0) {
				leftHolder.set (-1);
				rightHolder.set(1);
				pusher.set(out);
			} else {
				step = 5;
				pusher.set(in);
				leftHolder.set(0);
				rightHolder.set(0);
			}
			break;
		}
	}
	void leftSwitch() {
		switch(step) {
		case 1:
			if(ultrasonic.get() < 168) {
				mainDrive.arcadeDrive(1, 0);
			} else {
				step = 2;
			}
			break;
		case 2:
			if(gyro.getAngle() < 90) {
				mainDrive.arcadeDrive(0, 0.3);
			} else {
				step = 3;
			}
			break;
		case 3:
			if(ultrasonic.get() < 12) {
				mainDrive.arcadeDrive(0.2, 0);
				lift.set(0.5);
			} else {
				step = 4;
				lift.set(0);
				time = Timer.getMatchTime();
			}
			break;
		case 4:
			if(time - Timer.getMatchTime() <= 1.0) {
				leftHolder.set (-1);
				rightHolder.set(1);
				pusher.set(out);
			} else {
				step = 5;
				pusher.set(in);
				leftHolder.set(0);
				rightHolder.set(0);
			}
			break;
		}
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
