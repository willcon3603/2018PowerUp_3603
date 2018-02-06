package org.usfirst.frc.team3603.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class MyEncoder {

	WPI_TalonSRX talon;
	boolean inv;
	double multiplier;
	
	public MyEncoder(WPI_TalonSRX input, boolean invert, double mult) {
		talon = input;
		inv = invert;
		multiplier = mult;
	}
	
	public void invert(boolean in) {
		inv = in;
	}
	
	public double get() {
		double distance = talon.getSelectedSensorPosition(0) * multiplier;
		return distance;
	}
}
