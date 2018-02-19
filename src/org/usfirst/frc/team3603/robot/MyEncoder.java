package org.usfirst.frc.team3603.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class MyEncoder implements PIDSource {

	WPI_TalonSRX talon;
	boolean inv;
	double multiplier;
	double offset;
	
	public MyEncoder(WPI_TalonSRX input, boolean invert, double mult) {
		talon = input;
		inv = invert;
		multiplier = mult;
		talon.getSensorCollection();
		offset = talon.getSelectedSensorPosition(0);
	}
	
	public void invert(boolean in) {
		inv = in;
	}
	
	public double get() {
		double distance = talon.getSelectedSensorPosition(0) * multiplier - offset;
		distance = inv ? -distance : distance;
		return distance;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return PIDSourceType.kDisplacement;
	}
	
	public void zero() {
		offset = talon.getSelectedSensorPosition(0);
	}

	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		return get();
	}
}