package org.usfirst.frc.team3603.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class TouchlessEncoder {
	DigitalInput input;
	int ticks = 0;
	double multiplier;
	boolean current = false;
	boolean previous = false;
	public TouchlessEncoder(int pin, double mult) {
		input = new DigitalInput(pin);
		multiplier = mult;
		Thread tracker = new Thread(() -> {
			while(true) {
				current = input.get();
				if(previous == false && current == true) {
					ticks++;
				}
				previous = current;
			}
		});
		tracker.start();
	}
	
	public void reset() {
		ticks = 0;
	}
	
	public boolean g() {
		return input.get();
	}
	
	public double get() {
		return (double) ticks * multiplier;
	}
}