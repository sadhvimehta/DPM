package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.main.Main;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * <h1>FallingEdgeUSLocalization</h1>
 *
 * <p style="text-indent: 30px">
 */
public class FallingEdgeUSLocalization implements UltrasonicController{ 
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE}
	public enum AngleType {ALPHA, BETA}
	public static int ROTATION_SPEED = 100;
	public static int FULL_CIRCLE = 360;
	private static final double DISTANCE_WALL = 30, NOISE_MARGIN = 3;
	private static final int ROTATE_SPEED = 100;
	private double deltaHeading; //the angle to be added to the odometer angle to correct it

	
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


	public void doLocalization() {
		double angleA, angleB;
			
			//check if the robot is face to the wall, if the robot is face to the wall
			//turn 180 degrees
			if (readUSData()< DISTANCE_WALL+10) {
				this.leftMotor.setSpeed(ROTATE_SPEED);
				this.rightMotor.setSpeed(ROTATE_SPEED);	
				this.leftMotor.rotate(Navigation.convertAngle(Main.WHEEL_RADIUS,Main.TRACK,180),true);
				this.rightMotor.rotate(-Navigation.convertAngle(Main.WHEEL_RADIUS,Main.TRACK,180),false);
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

	//method to get first falling edge angle
	//Angle A - Angle where sensor first detects the back wall
	private double getAngleFallingEdge(AngleType angleType) {

		if (angleType == AngleType.ALPHA){
			//rotate our robot clockwise until a wall is detected
			while (readUSData() < DISTANCE_WALL + NOISE_MARGIN) {
				setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
			}
			while (readUSData() > DISTANCE_WALL) {
				setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
			}
		}
		else if (angleType == AngleType.BETA) {
			//rotate our robot counter-clockwise until a wall is detected
			while (readUSData() < DISTANCE_WALL + NOISE_MARGIN) {
				setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
			}
			while (readUSData() > DISTANCE_WALL) {
				setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
			}
		}

		//stop the motors and return the falling edge angle
		stopMotor();
		Sound.beep();
		return Math.toDegrees(odo.getTheta());
	}
	
	//method the calculate deltaHeading
	private double calculateHeading( double angleA, double angleB ) {
		double deltaHeading = 0;
		if(angleA > angleB){
			deltaHeading = Math.abs(220 - ((angleA + angleB)/2.0));
		} else {
			deltaHeading = Math.abs(40 - ((angleA + angleB)/2.0));
		}
		return deltaHeading;
	}
	
	 //method to set Speeds of motors
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
	 
	 //method to stop the motors
	 public void stopMotor() {
			this.leftMotor.stop(true);
			this.rightMotor.stop(false);
		}
	 
	//method that turns the rpbot to a specific angle, so it compares between the angle we had and the angle we want to go to
	//inputs dest angle
	public void turnToDestintaionAngle(double angle) {

		double delta = angle - this.odo.getTheta();
		
		if (delta < -Math.PI) {
			delta +=2*Math.PI;
		}
			
		if (delta > Math.PI) {
			delta -= 2*Math.PI;
		}
			
		if(delta ==0 || delta == 2*Math.PI) {
			delta = 0;
		}
			
		this.leftMotor.setSpeed(ROTATE_SPEED);
		this.rightMotor.setSpeed(ROTATE_SPEED);	

		this.leftMotor.rotate(convertAngle(Main.WHEEL_RADIUS,Main.TRACK,Math.toDegrees(delta)),true);
		this.rightMotor.rotate(-convertAngle(Main.WHEEL_RADIUS,Main.TRACK,Math.toDegrees(delta)),false);
		Sound.beep();
			
	}
		
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
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
