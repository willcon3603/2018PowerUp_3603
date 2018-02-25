package org.usfirst.frc.team3603.robot;

import java.util.Random;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
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
	WPI_TalonSRX arm = new WPI_TalonSRX(7); //Arm speed controller
	Servo release = new Servo(0); //Servo for the arm release
	
	Compressor compressor = new Compressor(); //Air compressor
	DoubleSolenoid omni = new DoubleSolenoid(0, 1); //Omni solenoid
	DoubleSolenoid shift = new DoubleSolenoid(2, 3);//Transmission solenoid
	
	Joystick joy1 = new Joystick(0); //Large twist-axis joystick
	Joystick joy2 = new Joystick(1); //Xbox controller
	MyEncoder liftEnc = new MyEncoder(cubeLift, false, 1.0); //Encoder for the cube lift
	double mult = (4*Math.PI)/60; //Multiplier for the touchless encoder
	TouchlessEncoder driveEnc = new TouchlessEncoder(2, mult); //Touchless encoder
	Encoder armEnc = new Encoder(0, 1, false, EncodingType.k2X); //Arm angle encoder
	WPI_TalonSRX pidStore = new WPI_TalonSRX(1); //Speed controller for setting up liftPID
	WPI_TalonSRX armStore = new WPI_TalonSRX(2); //Speed controller for setting up armPID
	PIDController liftPID = new PIDController(0.001, 0, 0, liftEnc, pidStore); //PID controller for lift
	PIDController armPID = new PIDController(0.05, 0, 0, armEnc, armStore); //PID controller for arm
	PressureSensor pressure = new PressureSensor(0); //Pressure sensor
	CameraServer camera = CameraServer.getInstance(); //Camera
	AHRS gyro = new AHRS(Port.kMXP); //NavX
	PIDController strPID = new PIDController(0.15, 0, 0, gyro, armStore); //PID controller for driving straight
	
	DigitalInput slot1 = new DigitalInput(3); //Digital inputs for the auton switch
	DigitalInput slot2 = new DigitalInput(4);
	DigitalInput slot3 = new DigitalInput(5);
	int position; //Auton switch position
	
	DriverStation matchInfo = DriverStation.getInstance(); //Object to get switch/scale colors
	
	String sides; //A string to store the switch and scale colors
	AutonType autonMode; //Enumerator for the autonomous mode
	int step; //The auton step
	boolean doOnce = true; //Boolean to only enable the liftPID once at a time
	double time; //Double to keep track of initial times
	final static double scaleNeutralHeight = 20200;//Double for scale encoder position //TODO change this number if the lift goes too height/low
	final static double switchHeight = 3000;//Double for the switch encoder position TODO change this number if the lift is too high/low for the switch
	
	
	@Override
	public void robotInit() {
		pidStore.disable();//Disable the PID store
		armStore.disable(); //Disable the PID store
		cubeLift.getSensorCollection();
		camera.startAutomaticCapture("cam0", 0); //Start the camera server
		compressor.start(); //Start compressor
		
		mainDrive.setSafetyEnabled(false); //Disable safety
		
		liftPID.setOutputRange(-0.7, 0.7); //Set the range of speeds for the lift PID
		armPID.setOutputRange(-0.5, 0.5); //Set the range of speeds for the arm PID
		liftEnc.zero(); //Zero out the lift encoder
	}
	@Override
	public void autonomousInit() {
		strPID.setSetpoint(0); //Set the setpoint of the drive straight PID to 0 degrees
		driveEnc.reset(); //Set the touchless encoder to 0
		gyro.reset(); //Set the gyro angle to 0
		step = 1; //set the auton step to step 1
		sides = matchInfo.getGameSpecificMessage(); //Get the switch and scale colors
		
		Random generator = new Random(); //This chunk is all code to randomize the field TODO delete this chunk BEFORE competition
		int randomScene = generator.nextInt(3);
		switch(randomScene) {
		case 0:
			sides = "LLL";
			break;
		case 1:
			sides = "RRR";
			break;
		case 2:
			sides = "LRL";
			break;
		case 3:
			sides = "RLR";
			break;
		}
		
		if(slot1.get()) { //Logic to find the auton rotating switch position
			position = 1;
		} else if(slot2.get()) {
			position = 2;
		} else if(slot3.get()) {
			position = 3;
		} else {
			position = 4;
		}
		
		if(position == 1) { //If we are in position 1...
			if(sides == "LLL") {//If the switch and scale is on our side...
				autonMode = AutonType.leftSwitch;//Set the auton mode to the left side of the switch
				liftPID.setSetpoint(switchHeight);//Set the lift PID to switch height
			} else if(sides == "RRR") {//If neither the switch or scale are on our side...
				autonMode = AutonType.straight; //Cross the auto line
			} else if(sides == "LRL") {//If only the switch is on our side...
				autonMode = AutonType.leftSwitch;//Set the auton mode to the left side of the switch
				liftPID.setSetpoint(switchHeight);//Set the lift PID to switch height
			} else if(sides == "RLR") {//If only the scale is on our side...
				autonMode = AutonType.leftScale;//Set the auton mode to the left scale
				liftPID.setSetpoint(scaleNeutralHeight);//Set the lift PID to scale height
			}
		} else if(position == 2) { //If we are in position 2...
			if(sides == "LLL" || sides == "LRL") {//If the switch is on the left...
				autonMode = AutonType.leftMiddle;//Set the auton mode to left middle
			} else if(sides == "RLR" || sides == "RRR") {//If the switch is on the right side...
				autonMode = AutonType.rightMiddle;//Set the auton mode to right middle
			}
		} else if(position == 3) {//If we are in position 3...
			if(sides == "LLL") {//If neither the switch or scale is on our side...
				autonMode = AutonType.straight;//Set the auton mode to straight
			} else if(sides == "RRR") {//If both the switch and the scale are on our side...
				autonMode = AutonType.rightSwitch;//Do the switch
				liftPID.setSetpoint(switchHeight);
			} else if(sides == "LRL") {//If only the scale is on our side...
				autonMode = AutonType.rightScale;//Do the scale
				liftPID.setSetpoint(scaleNeutralHeight);
			} else if(sides == "RLR") {//If only the switch is on our side...
				autonMode = AutonType.rightSwitch;//Do the switch
				liftPID.setSetpoint(switchHeight);
			}
		} else if(position == 4) {//If the auton switch is in position 4...
			autonMode = AutonType.straight;//Override and drive straight
		}
		
		liftPID.enable();//Enable the liftPID
		armPID.enable();//Enable the armPID
		armEnc.reset();//Reset the arm encoder
	}
	@Override
	public void autonomousPeriodic() {
		release.set(0.5);//Release the arm by raising the servo
		
		read();//Read from sensors and put the info on the smart dashboard
		
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
			rightMiddle(); //Go to the front right side of the switch
			break;
		case leftMiddle:
			leftMiddle(); //Go to the front left side of the switch
			break;
		}
		
	}
	
	@Override
	public void teleopPeriodic() {
		
		/**********
		 * DRIVER *
		 **********/
		
		double sense = -0.5 * joy1.getRawAxis(3) + 0.5;//Sensitivity coefficient determined by the little axis on the big joystick
		double y = Math.pow(joy1.getRawAxis(1), 1); //Double to store the joystick's y axis
		double rot = -Math.pow(joy1.getRawAxis(2), 1)/1.25; //Double to store the joystick's x axis
		if(Math.abs(y) >= 0.05 || Math.abs(rot) >= 0.05 && !joy1.getRawButton(1)) { //Thresholding function
			mainDrive.arcadeDrive(y * sense, rot * sense); //Arcade drive with the joystick's axis
		} else {
			mainDrive.arcadeDrive(0, 0); //Stop if value doesn't meet threshhold
		}
		
		if(joy1.getRawButton(2)) { //Press and hold button 2 for omni wheels
			omni.set(out);//Set the omni piston to out
		} else {
			omni.set(in);//Set the omni piston to in
		}
		
		if(joy1.getRawButton(3)) { //Press and hold button 3 for transmission
			shift.set(out);//Set the transmission piston to out (high gear)
		} else {
			shift.set(in); //Set the transmission piston to in (low gear)
		}
		
		
		/***************
		 * MANIPULATOR *
		 ***************/
		
		if(liftEnc.get() >= 20000) {//If the lift is above encoder position 200000, vibrate the controller
			joy2.setRumble(RumbleType.kLeftRumble, 0.25);
			joy2.setRumble(RumbleType.kRightRumble, 0.25);
		} else {
			joy2.setRumble(RumbleType.kLeftRumble, 0);
			joy2.setRumble(RumbleType.kRightRumble, 0);
		}
		
		if(doOnce) {//If the doOnce boolean is true...
			liftPID.enable();//Enable the liftPID
			doOnce = false;//Set the boolean to false
		}
		if(Math.abs(joy2.getRawAxis(1)) >= 0.1) { //If axis 1 is off-center...
			liftPID.reset();//Reset the liftPID
			cubeLift.set(joy2.getRawAxis(1));//Set the lift speed to the axis reading
			liftPID.setSetpoint(liftEnc.get());//Set the lift PID setpoint to the current encoder value, so it locks into place when the joystick isn't being touched
			doOnce = true;//Set the doOnce boolean to true so it will only enable it once
		} else if(joy2.getRawButton(1)) {//If Troy presses the A button...
			liftPID.reset();//reset the liftPID
			doOnce = true;//Set the boolean to true
			liftPID.setSetpoint(scaleNeutralHeight);//Set the setpoint to the bar height, so they can try climbing
		} else {//If nothing is being pressed...
			cubeLift.set(-liftPID.get());//Set the lift speed to the opposite of the liftPID
		}
		
		if(Math.abs(joy2.getRawAxis(5)) >= 0.1) { //If axis 5 is off-center...
			arm.set(joy2.getRawAxis(5));//Set the arm motor to the axis reading
			armPID.setSetpoint(armEnc.get());//Set the armPID setpoint to the current encoder reading, so that it locks in to place
			armPID.enable();//Enable the PID
		} else {//If the joystick isn't being touched...
			arm.set(armPID.get());//Set the arm motor to the armPID
		}
		
		
		if(Math.abs(joy2.getRawAxis(2)) >= 0.25) { //If the left trigger is pulled...
			leftHolder.set(0.85); //Input cube TODO change these numbers if the intake speed is too fast/slow
			rightHolder.set(0.85);
		} else if(Math.abs(joy2.getRawAxis(3)) >= 0.25) { //If right trigger is pulled...
			leftHolder.set(-0.33);//Soft spit TODO change these numbers if the grabber motors are too fast/slow
			rightHolder.set(-0.33);
		} else if(joy2.getRawButton(5)) { //If left bumper is pressed...
			leftHolder.set(-0.75); // Rotate cube
			rightHolder.set(0.75);
		} else if(joy2.getRawButton(6)) { //If right bumper is pressed...
			leftHolder.set(0.75); // Rotate cube
			rightHolder.set(-0.75);
		} else if(joy2.getRawButton(4)){ //If the Y button is pressed...
			leftHolder.set(-0.75);//Hard spit
			rightHolder.set(-0.75);
		} else {
			leftHolder.set(0);
			rightHolder.set(0);
		}
			
		read();//Read from sensors
	}
	
	void read() {//This puts data onto the smart dashboard
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
	
	enum AutonType {//A list of different auton types
		rightScale, leftScale, rightSwitch, leftSwitch, straight, rightMiddle, leftMiddle
	}
	
	void leftMiddle() {
		switch(step) {
		case 1://Step one of the left auton
			liftPID.setSetpoint(4000);//Set the liftPID to 4000
			armPID.setSetpoint(75);//Set the armPID to 75
			armPID.enable();//Enable the armPID
			liftPID.enable();//Enable the liftPID
			strPID.enable();//Enable the drive straight PID
			if(driveEnc.get() < 10) {//If the drive distance is less than 10 inches...
				mainDrive.arcadeDrive(-0.75, -strPID.get());//Drive at -3/4 speed and use the PID
			} else {//If the drive distance greater than 10 inches...
				mainDrive.arcadeDrive(0, 0);///Stop
				step = 2;//Set the step to 2
			}
			break;
		case 2://Step 2 of the left middle auton
			if(gyro.getAngle() > -60) {//If the current angle is greater than -60
				mainDrive.arcadeDrive(0, 0.4);//Slowly turn left
			} else { //If it has reached the -60 degree mark...
				mainDrive.arcadeDrive(0, 0);//Stop
				strPID.setSetpoint(-60);//Set the straightPID setpoint to -60
				strPID.enable();//Enable the straight PID
				driveEnc.reset();//Reset the touchless encoder
				step = 3;//Set the step to 3
			}
			break;
		case 3://Step 3 of the left middle auton
			cubeLift.set(-liftPID.get());//Activate the lift with the PID
			arm.set(armPID.get());//Activate the arm with the PID
			if(driveEnc.get() < 144) {//If the robot has driven less than 144 inches...
				mainDrive.arcadeDrive(-0.75, -strPID.get());//Drive at -3/4 and drive straight with PID
			} else {//If the robot has driven more than 144 inches...
				mainDrive.arcadeDrive(0, 0);//Stop
				step = 4;//Set the step to 4
			}
			break;
		case 4://Step 4
			cubeLift.set(-liftPID.get());//Keep the lift in position
			arm.set(armPID.get());//Keep the arm in position
			if(gyro.getAngle() < 0) {//If the current gyro angle is less than 0...
				mainDrive.arcadeDrive(0, -0.4);//Slowly turn right
			} else {//If it has completely turned 60 degrees...
				mainDrive.arcadeDrive(0, 0);//Stop
				step = 5;//Set the step to 5
			}
			break;
		case 5://Step 5
			cubeLift.set(-liftPID.get());//Keep the cube lift in place
			arm.set(armPID.get());//Keep the arm in place
			leftHolder.set(-0.5);//Output the cube into the switch
			rightHolder.set(-0.5);
			break;
		}
	}
	
	void rightMiddle() {//Method to do the right side of the switch when in position 2
		strPID.enable();//Enable the drive straight PID
		shift.set(out);//Go into high gear
		switch(step) {
		case 1:
			liftPID.setSetpoint(10000);//set the lift PID setpoint to 10000 TODO change this if it is too high/low
			armPID.setSetpoint(75);//Set the armPID setpoint to 75
			armPID.enable();//Enable the armPID
			liftPID.enable();//Enable the liftPID
			cubeLift.set(-liftPID.get());//Activate the lift
			arm.set(armPID.get());//Activate the arm
			if(driveEnc.get() < 83) {//If the robot has driven less than 83 inches...
				mainDrive.arcadeDrive(-0.75, -strPID.get());//Drive straight at -3/4 speed
			} else {//If it has driven 83 inches...
				mainDrive.arcadeDrive(0, 0);//Stop
				step = 2;//Set the step to 2
			}
			break;
		case 2:
			cubeLift.set(-liftPID.get());//keep the lift in place
			arm.set(armPID.get());//Keep the arm in place
			leftHolder.set(-0.35);//Output the cube
			rightHolder.set(-0.35);
			break;
		}
	}
	
	void straight() {//Auton the drive straight
		strPID.enable();//Enable the drive straight PID
		if(driveEnc.get() < 83) {//If the robot has driven less than 83 inches...
			mainDrive.arcadeDrive(-0.75, -strPID.get());//Drive straight at -3/4 speed
		} else {//Else
			mainDrive.arcadeDrive(0, 0);//Stop
		}
	}
	
	void rightScale() {//Auton for the right side of the scale
		strPID.enable();//Enable the drive straight PID
		shift.set(out);//Go into high gear
		switch(step) {
		case 1:
			liftPID.setSetpoint(15000);//Set the lift setpoint to 15000
			armPID.setSetpoint(75);//Set the arm setpoint to 75
			armPID.enable();//Enable the armPID
			liftPID.enable();//Enable the liftPID
			cubeLift.set(-liftPID.get());//Activate the lift
			arm.set(armPID.get());//Activate the arm
			if(driveEnc.get() < 252) {//If the robot has driven //TODO finish commenting
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
			break;
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