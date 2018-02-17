package org.usfirst.frc.team3603.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	final static DoubleSolenoid.Value out = DoubleSolenoid.Value.kForward; //Piston out value
	static final DoubleSolenoid.Value in = DoubleSolenoid.Value.kReverse; //Piston in value
	
	//All of these are individual speed controllers
	WPI_TalonSRX leftFront = new WPI_TalonSRX(10);
	WPI_TalonSRX leftMiddle = new WPI_TalonSRX(11);
	WPI_TalonSRX leftBack = new WPI_TalonSRX(12);
	WPI_TalonSRX rightFront = new WPI_TalonSRX(4);
	WPI_TalonSRX rightMiddle = new WPI_TalonSRX(5);
	WPI_TalonSRX rightBack = new WPI_TalonSRX(6);
	//This groups the speed controllers into left and right
	SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftMiddle, leftBack);
	SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightMiddle, rightBack);
	//This groups them into the new type of RobotDrive
	DifferentialDrive mainDrive = new DifferentialDrive(left, right);
	
	WPI_TalonSRX leftHolder = new WPI_TalonSRX(9);//Leftholder speedcontroller
	WPI_TalonSRX rightHolder = new WPI_TalonSRX(8);//Rightholder speedcontroller
	WPI_TalonSRX cubeLift = new WPI_TalonSRX(3); //Cube lift speed controller
	WPI_TalonSRX arm = new WPI_TalonSRX(7);
	Servo release = new Servo(0);
	
	Compressor compressor = new Compressor();
	DoubleSolenoid omni = new DoubleSolenoid(0, 1); //Omni solenoid
	DoubleSolenoid shift = new DoubleSolenoid(2, 3);//Transmission solenoid
	
	Joystick joy1 = new Joystick(0); //Large twist-axis joystick
	Joystick joy2 = new Joystick(1); //Xbox controller
	MyEncoder liftEnc = new MyEncoder(cubeLift, false, 1.0); //Encoder for the cube lift
	double mult = (4*Math.PI)/60;
	TouchlessEncoder driveEnc = new TouchlessEncoder(2, mult);
	Encoder armEnc = new Encoder(0, 1, false, EncodingType.k2X);
	WPI_TalonSRX pidStore = new WPI_TalonSRX(1);
	WPI_TalonSRX armStore = new WPI_TalonSRX(2);
	PIDController liftPID = new PIDController(0.001, 0, 0, liftEnc, pidStore);
	PIDController armPID = new PIDController(0.05, 0, 0, armEnc, armStore);
	PressureSensor pressure = new PressureSensor(0);
	CameraServer camera = CameraServer.getInstance();
	AHRS gyro = new AHRS(Port.kMXP);
	PIDController strPID = new PIDController(0.15, 0, 0, gyro, armStore);
	
	DigitalInput slot1 = new DigitalInput(3);
	DigitalInput slot2 = new DigitalInput(4);
	DigitalInput slot3 = new DigitalInput(5);
	int position;
	
	DriverStation matchInfo = DriverStation.getInstance(); //Object to get switch/scale colors
	
	String sides; //A string to store the switch and scale colors
	AutonType autonMode; //Enumerator for the autonomous mode
	int step;
	boolean doOnce = true;
	boolean liftToggle = false;
	double time;
	final static double scaleNeutralHeight = 20000;
	final static double switchHeight = 3000;
	
	@Override
	public void robotInit() {
		pidStore.disable();
		cubeLift.getSensorCollection();
		camera.startAutomaticCapture("cam0", 0);
		compressor.start(); //Start compressor
		
		mainDrive.setSafetyEnabled(false); //Disable safety
		
		liftPID.setOutputRange(-0.7, 0.7);
		armPID.setOutputRange(-0.5, 0.5);
		liftEnc.zero();
	}
	@Override
	public void autonomousInit() {
		strPID.setSetpoint(0);
		driveEnc.reset();
		step = 1; //set the auton step to step 1
		sides = matchInfo.getGameSpecificMessage(); //Get the switch and scale colors
		sides = "RRR";
		
		if(slot1.get()) {
			position = 1;
		} else if(slot2.get()) {
			position = 2;
		} else if(slot3.get()) {
			position = 3;
		} else {
			position = 4;
		}
		if(position == 1) {
			if(sides == "LLL") {
				autonMode = AutonType.leftSwitch;
				liftPID.setSetpoint(switchHeight);
			}
			if(sides == "RRR") {
				autonMode = AutonType.straight;
			}
			if(sides == "LRL") {
				autonMode = AutonType.leftSwitch;
				liftPID.setSetpoint(switchHeight);
			}
			if(sides == "RLR") {
				autonMode = AutonType.leftScale;
				liftPID.setSetpoint(scaleNeutralHeight);
			}
		} else if(position == 2) { //TODO
			autonMode = AutonType.straight;
		} else if(position == 3) {
			if(sides == "LLL") {
				autonMode = AutonType.straight;
			}
			if(sides == "RRR") {
				autonMode = AutonType.rightSwitch;
				liftPID.setSetpoint(switchHeight);
			}
			if(sides == "LRL") {
				autonMode = AutonType.rightScale;
				liftPID.setSetpoint(scaleNeutralHeight);
			}
			if(sides == "RLR") {
				autonMode = AutonType.rightSwitch;
				liftPID.setSetpoint(switchHeight);
			}
		} else if(position == 4) {
			autonMode = AutonType.straight;
		}
		
		autonMode = AutonType.leftMiddle;
		
		liftPID.enable();
		armPID.enable();
		armEnc.reset();
	}
	@Override
	public void autonomousPeriodic() {
		release.set(0.5);
		
		read();
		
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
		case rightMiddle:
			rightMiddle();
			break;
		case leftMiddle:
			leftMiddle();
			break;
		}
		
	}
	
	@Override
	public void teleopPeriodic() {
		release.set(0.5);
		
		/**********
		 * DRIVER *
		 **********/
		
		double sense = -0.5 * joy1.getRawAxis(3) + 0.5;
		double y = Math.pow(joy1.getRawAxis(1), 1); //Double to store the joystick's y axis
		double rot = -Math.pow(joy1.getRawAxis(2), 1)/1.25; //Double to store the joystick's x axis
		if(Math.abs(y) >= 0.05 || Math.abs(rot) >= 0.05 && !joy1.getRawButton(1)) { //Thresholding function
			mainDrive.arcadeDrive(y * sense, rot * sense); //Arcade drive with the joystick's axis
		} else {
			mainDrive.arcadeDrive(0, 0); //Stop if value doesn't meet threshhold
		}
		
		if(joy1.getRawButton(2)) { //Press and hold button 2 for omni wheels
			omni.set(out);
		} else {
			omni.set(in);
		}
		
		if(joy1.getRawButton(3)) { //Press and hold button 3 for transmission
			shift.set(out);
		} else {
			shift.set(in);
		}
		
		
		/***************
		 * MANIPULATOR *
		 ***************/
		
		if(liftEnc.get() >= 20000) {
			joy2.setRumble(RumbleType.kLeftRumble, 0.25);
			joy2.setRumble(RumbleType.kRightRumble, 0.25);
		} else {
			joy2.setRumble(RumbleType.kLeftRumble, 0);
			joy2.setRumble(RumbleType.kRightRumble, 0);
		}
		
		if(doOnce) {
			liftPID.enable();
			doOnce = false;
		}
		if(Math.abs(joy2.getRawAxis(1)) >= 0.1) { //Threshhold for cube lift speed
			liftPID.reset();
			cubeLift.set(joy2.getRawAxis(1));
			liftPID.setSetpoint(liftEnc.get());
			doOnce = true;
		} else if(joy2.getRawButtonReleased(1)) {
			liftPID.reset();
			doOnce = true;
			liftToggle = !liftToggle;
			if(liftToggle) {
				liftPID.setSetpoint(scaleNeutralHeight);
			} else {
				liftPID.setSetpoint(0);
			}
		} else {
			cubeLift.set(-liftPID.get());
		}
		
		if(Math.abs(joy2.getRawAxis(5)) >= 0.1) { //Threshhold for cube lift speed
			arm.set(joy2.getRawAxis(5));
			armPID.setSetpoint(armEnc.get());
			armPID.enable();
		} else {
			arm.set(armPID.get());
		}
		
		
		if(Math.abs(joy2.getRawAxis(2)) >= 0.25) { //If the left trigger is pulled...
			leftHolder.set(0.85); //Input cube
			rightHolder.set(0.85);
		} else if(Math.abs(joy2.getRawAxis(3)) >= 0.25) { //If right trigger is pulled...
			leftHolder.set(-0.75);// Output cube
			rightHolder.set(-0.75);
		} else if(joy2.getRawButton(5)) { //If left bumper is pressed...
			leftHolder.set(-0.75); // Rotate cube
			rightHolder.set(0.75);
		} else if(joy2.getRawButton(6)) { //If right bumper is pressed...
			leftHolder.set(0.75); // Rotate cube
			rightHolder.set(-0.75);
		} else if(joy2.getRawButton(4)){ //If nothing is pressed...
			leftHolder.set(-0.33);// Output cube
			rightHolder.set(-0.33);
		} else {
			leftHolder.set(0);
			rightHolder.set(0);
		}
			
		read();
	}
	
	void read() {
		SmartDashboard.putNumber("Lift encoder", liftEnc.get());
		SmartDashboard.putNumber("Lift PID", liftPID.get());
		SmartDashboard.putNumber("Lift speed", cubeLift.get());
		if(pressure.get() >= 30) {
			SmartDashboard.putBoolean("usable pressure", true);
		} else {
			SmartDashboard.putBoolean("usable pressure", false);
		}
		SmartDashboard.putNumber("Pressure", pressure.get());
		SmartDashboard.putNumber("Arm Encoder", armEnc.get());
		SmartDashboard.putNumber("Arm PID", armPID.get());
		SmartDashboard.putNumber("Arm speed", arm.get());
		SmartDashboard.putNumber("STRAIGHT PID", strPID.get());
		SmartDashboard.putNumber("Gyro", gyro.getAngle());
		SmartDashboard.putNumber("Drive distance", driveEnc.get());
		
		SmartDashboard.putBoolean("Slot 1", slot1.get());
		SmartDashboard.putBoolean("Slot 2", slot2.get());
		SmartDashboard.putBoolean("Slot 3", slot3.get());
		
		SmartDashboard.putNumber("Status", (driveEnc.read() ? 1 : 0));
		
		SmartDashboard.putString("results", matchInfo.getGameSpecificMessage());
	}
	
	@Override
	public void testPeriodic() {
	}
	
	enum AutonType {
		rightScale, leftScale, rightSwitch, leftSwitch, straight, rightMiddle, leftMiddle
	}
	
	void leftMiddle() {
		switch(step) {
		case 1:
			liftPID.setSetpoint(10000);
			armPID.setSetpoint(75);
			armPID.enable();
			liftPID.enable();
			strPID.enable();
			if(driveEnc.get() < 10) {
				mainDrive.arcadeDrive(-0.75, -strPID.get());
			} else {
				mainDrive.arcadeDrive(0, 0);
				step = 2;
			}
			break;
		case 2:
			if(gyro.getAngle() > -60) {
				mainDrive.arcadeDrive(0, 0.4);
			} else {
				mainDrive.arcadeDrive(0, 0);
				strPID.setSetpoint(-60);
				strPID.enable();
				driveEnc.reset();
				step = 3;
			}
			break;
		case 3:
			cubeLift.set(-liftPID.get());
			arm.set(armPID.get());
			if(driveEnc.get() < 144) {
				mainDrive.arcadeDrive(-0.75, -strPID.get());
			} else {
				mainDrive.arcadeDrive(0, 0);
				step = 4;
			}
			break;
		case 4:
			cubeLift.set(-liftPID.get());
			arm.set(armPID.get());
			if(gyro.getAngle() < 0) {
				mainDrive.arcadeDrive(0, -0.4);
			} else {
				mainDrive.arcadeDrive(0, 0);
				step = 5;
			}
			break;
		case 5:
			cubeLift.set(-liftPID.get());
			arm.set(armPID.get());
			leftHolder.set(-0.5);
			rightHolder.set(-0.5);
			break;
		}
	}
	
	void rightMiddle() {
		strPID.enable();
		shift.set(out);
		switch(step) {
		case 1:
			liftPID.setSetpoint(10000);
			armPID.setSetpoint(75);
			armPID.enable();
			liftPID.enable();
			cubeLift.set(-liftPID.get());
			arm.set(armPID.get());
			if(driveEnc.get() < 83) {
				mainDrive.arcadeDrive(-0.75, -strPID.get());
			} else {
				mainDrive.arcadeDrive(0, 0);
				step = 2;
			}
			break;
		case 2:
			cubeLift.set(-liftPID.get());
			arm.set(armPID.get());
			leftHolder.set(-0.5);
			rightHolder.set(-0.5);
			break;
		}
	}
	
	void straight() {
		double distance = driveEnc.get();
		if(distance < 120) {
			mainDrive.arcadeDrive(-0.75, 0);
		} else {
			mainDrive.arcadeDrive(0, 0);
		}
	}
	
	void rightScale() {
		strPID.enable();
		shift.set(out);
		switch(step) {
		case 1:
			liftPID.setSetpoint(15000);
			armPID.setSetpoint(75);
			armPID.enable();
			liftPID.enable();
			cubeLift.set(-liftPID.get());
			if(driveEnc.get() < 252) {
				mainDrive.arcadeDrive(-0.75, -strPID.get());
			} else {
				mainDrive.arcadeDrive(0, 0);
				step = 2;
				liftPID.setSetpoint(24000);
			}
			break;
		case 2:
			cubeLift.set(-liftPID.get());
			arm.set(armPID.get());
			if(gyro.getAngle() > -35) {
				mainDrive.arcadeDrive(0, 0.4);
			} else {
				step = 3;
				driveEnc.reset();
				mainDrive.arcadeDrive(0, 0);
			}
			break;
		case 3:
			cubeLift.set(-liftPID.get());
			arm.set(armPID.get());
			if(liftEnc.get() < 23000) {
			} else {
				leftHolder.set(-0.75);
				rightHolder.set(-0.75);
			}
		}
	}
	
	void leftSwitch() {
		strPID.enable();
		shift.set(out);
		switch(step) {
		case 1:
			liftPID.setSetpoint(10000);
			armPID.setSetpoint(75);
			armPID.enable();
			liftPID.enable();
			cubeLift.set(-liftPID.get());
			if(driveEnc.get() < 149) {
				mainDrive.arcadeDrive(-0.75, -strPID.get());
			} else {
				mainDrive.arcadeDrive(0, 0);
				step = 2;
			}
			break;
		case 2:
			cubeLift.set(-liftPID.get());
			arm.set(armPID.get());
			if(gyro.getAngle() < 80) {
				mainDrive.arcadeDrive(0, -0.6);
			} else {
				step = 3;
				driveEnc.reset();
				mainDrive.arcadeDrive(0, 0);
			}
			break;
		case 3:
			cubeLift.set(-liftPID.get());
			arm.set(armPID.get());
			if(driveEnc.get() < 9) {
				mainDrive.arcadeDrive(-0.5, 0);
			} else {
				step = 4;
				mainDrive.arcadeDrive(0, 0);
				time = Timer.getMatchTime();
			}
			break;
		case 4:
			cubeLift.set(-liftPID.get());
			arm.set(armPID.get());
			if(Math.abs(time - Timer.getMatchTime()) <= 3.0) {
				leftHolder.set(-0.5);
				rightHolder.set(-0.5);
			} else {
				step = 5;
				leftHolder.set(0);
				rightHolder.set(0);
			}
			break;
		}
	}
	
	void leftScale() {
		strPID.enable();
		shift.set(out);
		switch(step) {
		case 1:
			liftPID.setSetpoint(15000);
			armPID.setSetpoint(75);
			armPID.enable();
			liftPID.enable();
			cubeLift.set(-liftPID.get());
			if(driveEnc.get() < 252) {
				mainDrive.arcadeDrive(-0.75, -strPID.get());
			} else {
				mainDrive.arcadeDrive(0, 0);
				step = 2;
				liftPID.setSetpoint(24000);
			}
			break;
		case 2:
			cubeLift.set(-liftPID.get());
			arm.set(armPID.get());
			if(gyro.getAngle() < 35) {
				mainDrive.arcadeDrive(0, -0.4);
			} else {
				step = 3;
				driveEnc.reset();
				mainDrive.arcadeDrive(0, 0);
			}
			break;
		case 3:
			cubeLift.set(-liftPID.get());
			arm.set(armPID.get());
			if(liftEnc.get() < 23000) {
			} else {
				leftHolder.set(-0.75);
				rightHolder.set(-0.75);
			}
		}
	}
	void rightSwitch() {
		strPID.enable();
		shift.set(out);
		switch(step) {
		case 1:
			liftPID.setSetpoint(10000);
			armPID.setSetpoint(75);
			armPID.enable();
			liftPID.enable();
			cubeLift.set(-liftPID.get());
			if(driveEnc.get() < 149) {
				mainDrive.arcadeDrive(-0.75, -strPID.get());
			} else {
				mainDrive.arcadeDrive(0, 0);
				step = 2;
			}
			break;
		case 2:
			cubeLift.set(-liftPID.get());
			arm.set(armPID.get());
			if(gyro.getAngle() > -80) {
				mainDrive.arcadeDrive(0, 0.6);
			} else {
				step = 3;
				driveEnc.reset();
				mainDrive.arcadeDrive(0, 0);
			}
			break;
		case 3:
			cubeLift.set(-liftPID.get());
			arm.set(armPID.get());
			if(driveEnc.get() < 9) {
				mainDrive.arcadeDrive(-0.5, 0);
			} else {
				step = 4;
				mainDrive.arcadeDrive(0, 0);
				time = Timer.getMatchTime();
			}
			break;
		case 4:
			cubeLift.set(-liftPID.get());
			arm.set(armPID.get());
			if(Math.abs(time - Timer.getMatchTime()) <= 3.0) {
				leftHolder.set(-0.5);
				rightHolder.set(-0.5);
			} else {
				step = 5;
				leftHolder.set(0);
				rightHolder.set(0);
			}
			break;
		}
	}
	
	
}