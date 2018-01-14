package org.usfirst.frc.team3603.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class Ultrasonic {

	AnalogInput ultrasonic;
	public Ultrasonic(int pin) {
		ultrasonic = new AnalogInput(pin);
	}
	
	public double get() {
		return (double) ultrasonic.getValue() * 0.125;
	}
}
