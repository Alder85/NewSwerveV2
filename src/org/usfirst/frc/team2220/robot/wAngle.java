package org.usfirst.frc.team2220.robot;

import edu.wpi.first.wpilibj.*;

/*
 * PID Motor control for the rotation of a module
 */
public class wAngle {

	private final double kP;
	private CANTalon talon;
	private AnalogInput encoder;
	private double setpoint;
	private boolean reversed = false;
	private double offset;
	private double cap = 1.0;
	private double deadZone = 10;
	
	
	
	/*
	 * Initialize stuff
	 */
	public wAngle(double p, CANTalon tal, AnalogInput ai)
	{
		kP = p;
		talon = tal;
		encoder = ai;	
	}
	
	/*
	 * Sets the setpoint to a value between (-180..180)
	 */
	public void setSetpoint(double x)
	{
		setpoint = x;
	}
	
	/*
	 * Adds an offset to the motor val
	 */
	public void setOffset(double x)
	{
		offset = -x;
	}
	
	
	/*
	 * Gets the error value from the encoder
	 * Scales from (0..5) to (-180..180)
	 */
	public double getError()
	{
		double out = encoder.getAverageVoltage() + offset;
		if(out > 5)
			out -= 5;
		if(out < -5);
			out += 5;
		out -= 2.5;
		out *= 72; //scaling
		out -= setpoint; 
		while(out > 180)
			out -= 360;
		while(out < -180) //corrects for looping around
			out += 360;
		
		return out;
	}
	/*
	 * Calculate wheel values
	 */
	public void calculate()
	{
		double err = getError(); //(-180..180)
		double aP = kP * err;
		double out = aP;
		if(err < deadZone && err > -deadZone)
			runWheel(0);
		else
			runWheel(out); //cannot go below 0, upper limit based on PID constants
	}
	
	/*
	 * Runs the wheel, a value between 0 and cap;
	 */
	public void runWheel(double val)
	{
		if(val > cap)
			val = cap; 
		if(!reversed)
			talon.set(val);
		else
			talon.set(-val);
	}
	
	public double getTalonSpeed()
	{
		return talon.get();
	}
	/*
	 * kinda obvious
	 */
	public void reverse()
	{
		reversed = !reversed;
	}
	
	
}
