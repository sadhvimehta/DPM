package ca.mcgill.ecse211.lab5;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread{
	
	  // robot position
	  private double x;
	  private double y;
	  private double theta;
	  private int currentLeftMotorTachoCount;
	  private int currentRightMotorTachoCount;
	  private int lastLeftMotorTachoCount;
	  private int lastRightMotorTachoCount;
	  private EV3LargeRegulatedMotor leftMotor;
	  private EV3LargeRegulatedMotor rightMotor;

	  private static final long ODOMETER_PERIOD = 25; /*odometer update period, in ms*/

	  private Object lock; /*lock object for mutual exclusion*/

	  // default constructor
	  public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
	    this.leftMotor = leftMotor;
	    this.rightMotor = rightMotor;
	    this.x = 0.0;
	    this.y = 0.0;
	    this.theta = 0.0;
	    this.leftMotor.resetTachoCount();
	    this.rightMotor.resetTachoCount();
	    this.lastLeftMotorTachoCount = this.leftMotor.getTachoCount();
	    this.lastRightMotorTachoCount = this.rightMotor.getTachoCount();
	    lock = new Object();
	  }

}
