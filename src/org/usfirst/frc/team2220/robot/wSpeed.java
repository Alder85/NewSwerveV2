package org.usfirst.frc.team2220.robot;

import edu.wpi.first.wpilibj.*;

/*
 * PID Motor control for speed of a module
 */
public class wSpeed {
	
	private final double kP;
	private double plusM, desiredRPS, currentRPS;
	private CANTalon talon;
	private Encoder encoder;
	private boolean reversed = false;
	private boolean inputReversed = false;
	private double cap = .5;
	private Timer time = new Timer();
	
	/*
	 * Initialize stuff
	 */
	public wSpeed(double p, CANTalon tal, Encoder ai)
	{
		kP = p;
		talon = tal;
		encoder = ai;
	}
	
	public double getError()
	{
		startRPS();
		Timer.delay(0.005);	
		endRPS();
		double temp = desiredRPS - currentRPS;
		if(inputReversed)
			return -temp;
		return temp;
	}
	
	public double getRPS()
	{
		
		startRPS();
		Timer.delay(0.005);	
		endRPS();
		return currentRPS;
	}
	
	public double getTalonSpeed()
	{
		return talon.get();
	}
	
	public void calculate()
	{
		plusM = getError() * kP;
		if(desiredRPS == 0)
			runWheel(0);
		else
			runWheel(talon.get() + plusM);
	}
	public void reverse()
	{
		reversed = !reversed;
	}
	
	public void runWheel(double val)
	{
		
		if(val > cap)
			val = cap;
		else if(val < -cap)
			val = -cap;
			
		if(!reversed)
			talon.set(val);
		else
			talon.set(-val);
	}
	
	public void reverseInput()
	{
		inputReversed = !inputReversed;
	}
	
	public void setRPS(double in)
	{
		desiredRPS = in;
	}
	
	
	
	public void startRPS()
	{
		encoder.reset();
		time.reset();
		time.start();
	}
	
	public void endRPS()
	{
		currentRPS = (encoder.getDistance() / 256) / time.get();
	}
}
