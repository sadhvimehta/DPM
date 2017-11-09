package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Performs falling edge localization
 *
 */
public class FallingEdgeUSLocalization implements UltrasonicController{ 

	/**
	 * List of localization types
	 */
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE}
	/**
	 *  List of angles detected: alpha is first angle detected, beta is second angle detected.
	 *
	 */
	public enum AngleType {ALPHA, BETA}
	/**
	 * Rotation speed while performing US localization
	 */
	public static int ROTATE_SPEED = 100;
	/**
	 * Degrees corresponding to a full circle
	 */
	public static int FULL_CIRCLE = 360;
	/**
	 * Distance between robot and wall
	 */
	private static final double DISTANCE_WALL = 30;
	/**
	 * Margin of error for US sensor
	 */
	private static final double NOISE_MARGIN = 3;
	/**
	 * Angle to be added for theta correction
	 */
	private double deltaHeading;

	
	private Odometer odo;
	private SampleProvider usSensor;
	private float[] usData;
	private LocalizationType locType;
	private Navigation navigation;
	private EV3LargeRegulatedMotor leftMotor,rightMotor;
	
	//Constructor
	public FallingEdgeUSLocalization(Odometer odo, SampleProvider usSensor, float[] usData, LocalizationType locType,EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor, Navigation navigation) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locType = locType;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.navigation = navigation;
	}

	/**
	 * Performs actual US localization
	 */
	public void doLocalization() {
		double angleA, angleB;
			
			//check if the robot is face to the wall, if the robot is face to the wall
			//turn 180 degrees
			if (readUSData()< DISTANCE_WALL+10) {
				this.leftMotor.setSpeed(ROTATE_SPEED);
				this.rightMotor.setSpeed(ROTATE_SPEED);	
				this.leftMotor.rotate(Navigation.convertAngle(CaptureFlagMain.WHEEL_RADIUS,CaptureFlagMain.TRACK,180),true);
				this.rightMotor.rotate(-Navigation.convertAngle(CaptureFlagMain.WHEEL_RADIUS,CaptureFlagMain.TRACK,180),false);
				odo.setTheta(0);
			}

			// get two falling edge angle
			angleA = getAngleFallingEdge(AngleType.ALPHA);
			angleB = getAngleFallingEdge(AngleType.BETA);


			//calculate heading
			deltaHeading =  calculateHeading(angleA,angleB);

			double convertedDeltaTheta = (Math.toRadians(deltaHeading) - Math.PI);
			//turn to origin
			navigation.turnTo(convertedDeltaTheta);
			
			//set the theta to 0
			odo.setTheta(0);
			Sound.beep();
		
	}

	/**
	 * Method to get falling edge angles
	 * @param angleType angle one wishes to detect
	 * @return angle when wall is detected in radians
	 */
	private double getAngleFallingEdge(AngleType angleType) {

		if (angleType == AngleType.ALPHA){
			//rotate our robot clockwise until a wall is detected
			while (readUSData() < DISTANCE_WALL + NOISE_MARGIN) {
				setSpeeds(ROTATE_SPEED, -ROTATE_SPEED);
			}
			while (readUSData() > DISTANCE_WALL) {
				setSpeeds(ROTATE_SPEED, -ROTATE_SPEED);
			}
		}
		else if (angleType == AngleType.BETA) {
			//rotate our robot counter-clockwise until a wall is detected
			while (readUSData() < DISTANCE_WALL + NOISE_MARGIN) {
				setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);
			}
			while (readUSData() > DISTANCE_WALL) {
				setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);
			}
		}

		//stop the motors and return the falling edge angle
		stopMotor();
		Sound.beep();
		return Math.toDegrees(odo.getTheta());
	}
	
	/**
	 * Calculates deltaTheta
	 * @param angleA back wall angle (alpha)
	 * @param angleB front wall angle (beta)
	 * @return heading to add to current theta
	 */
	private double calculateHeading( double angleA, double angleB ) {
		double deltaHeading = 0;
		if(angleA > angleB){
			deltaHeading = Math.abs(220 - ((angleA + angleB)/2.0));
		} else {
			deltaHeading = Math.abs(40 - ((angleA + angleB)/2.0));
		}
		return deltaHeading;
	}
	
	/**
	 * Method to set speed of motors
	 * @param lSpd left motor speed
	 * @param rSpd right motor speed
	 */
	 public void setSpeeds(int lSpd, int rSpd) {
			this.leftMotor.setSpeed(lSpd);
			this.rightMotor.setSpeed(rSpd);
			if (lSpd < 0)
				this.leftMotor.backward();
			else
				this.leftMotor.forward();
			if (rSpd < 0)
				this.rightMotor.backward();
			else
				this.rightMotor.forward();
		}
	 
	 /**
	  * Method to stop motors synchronously
	  */
	 public void stopMotor() {
			this.leftMotor.stop(true);
			this.rightMotor.stop(false);
		}


	@Override
	public void processUSData(float usData) {
		//TODO: add body
	}

	@Override
	public float readUSData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0]*100;	
		return distance > 100 ? 100 : distance;
	}

}
