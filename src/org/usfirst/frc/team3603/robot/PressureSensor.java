package org.usfirst.frc.team3603.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class PressureSensor {
	
	AnalogInput input;
	public PressureSensor(int inputPin) {
		input = new AnalogInput(inputPin);
	}
	
	public int get() {
		return (int) input.getVoltage() * 50 - 25;
	}
}
