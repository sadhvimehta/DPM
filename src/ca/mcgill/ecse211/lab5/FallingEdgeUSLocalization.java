package ca.mcgill.ecse211.lab5;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class FallingEdgeUSLocalization extends Thread {
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE};
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
	public FallingEdgeUSLocalization(Odometer odo, SampleProvider usSensor, float[] usData, LocalizationType locType,EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locType = locType;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;		
	}
	
	
	public void run() {
		doLocalization();
	}
	
	
	public void doLocalization() {
		double angleA, angleB;
		
		//check the localization type
		//if the localization type is falling edge
		if(locType == LocalizationType.FALLING_EDGE) {
			
			//check if the robot is face to the wall, if the robot is face to the wall
			//turn 180 degrees
			if (getFilteredData()< DISTANCE_WALL+10) {
				this.leftMotor.setSpeed(ROTATE_SPEED);
				this.rightMotor.setSpeed(ROTATE_SPEED);	
				this.leftMotor.rotate(navigation.convertAngle(LabFiveMain.WHEEL_RADIUS,LabFiveMain.TRACK,180),true);
				this.rightMotor.rotate(-navigation.convertAngle(LabFiveMain.WHEEL_RADIUS,LabFiveMain.TRACK,180),false);
				odo.setTheta(0);
			}
			
			// get two falling edge angle
			angleA = getAngleAFallingEdge();
			angleB = getAngleBFallingEdge();

			//calculate heading
			deltaHeading =  calculateHeading(angleA,angleB);
			
			//turn to origin
			navigation.turnTo(Math.toRadians(deltaHeading)-Math.PI);
			
			//set the theta to 0
			odo.setTheta(0);
			Sound.beep();
		}
		
		//if the localization type is rising edge
		else {

			//check if the robot is face to the wall, if the robot is not face to the wall
			//turn 180 degrees
			if (getFilteredData()> DISTANCE_WALL) {
				this.leftMotor.setSpeed(ROTATE_SPEED);
				this.rightMotor.setSpeed(ROTATE_SPEED);	
				this.leftMotor.rotate(navigation.convertAngle(LabFiveMain.WHEEL_RADIUS,LabFiveMain.TRACK,180),true);
				this.rightMotor.rotate(-navigation.convertAngle(LabFiveMain.WHEEL_RADIUS,LabFiveMain.TRACK,180),false);
				odo.setTheta(0);
			}
			
			//get tow rising edge angle
			angleA = getAngleARisingEdge();
			angleB = getAngleBRisingEdge();
			
			//calculate heading
			deltaHeading =  calculateHeading(angleA,angleB);
			
			//turn to origin 
			navigation.turnTo(Math.toRadians(deltaHeading));
			
			//set theta to 0
			odo.setTheta(0);
			Sound.beep();
			
		}
		

		
	}
	
	//method to get first falling edge angle
	//Angle A - Angle where sensor first detects the back wall
	private double getAngleAFallingEdge() {
		
		//rotate our robot clockwise until a wall is detected
		while(getFilteredData()< DISTANCE_WALL+NOISE_MARGIN) {
			setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
		}
		while(getFilteredData()> DISTANCE_WALL) {
			setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);			
		}
		
		//stop the motors and return the falling edge angle
		stopMotor();
		Sound.beep();
		return Math.toDegrees(odo.getTheta());
	}
	
	//method to get second falling edge angle
	//Angle B - Angle where sensor detects the left wall
	private double getAngleBFallingEdge() {
		
		//rotate our robot counter-clockwise until a wall is detected
		while(getFilteredData()< DISTANCE_WALL+NOISE_MARGIN) {
			setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
		}
		while(getFilteredData()> DISTANCE_WALL) {
			setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);			
		}
		
		//stop the motors and return the falling edge angle
		stopMotor();
		Sound.beep();
		return Math.toDegrees(odo.getTheta());
	}
		
	// method to get distance from the wall
	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0]*100;	
		return distance > 100 ? 100 : distance;
	}

	//method to get first rising edge angle
	//Angle A - Angle where sensor detects the back wall
	private double getAngleARisingEdge() {
		
		// rotate our robot counter-clockwise until a wall is detected
		while ( getFilteredData() > DISTANCE_WALL - NOISE_MARGIN ) {
			setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
		}
		while ( getFilteredData() < DISTANCE_WALL ) {
			setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);

		}
		
		// stop the motors and return the rising edge angle
		stopMotor();
		Sound.beep();		
		return  Math.toDegrees(odo.getTheta());
	}
	
	//method to get second rising edge angle
	//Angle B- Angle where sensor detects the left wall
	private double getAngleBRisingEdge() {
		
		// rotate our robot clockwise until a wall is detected
		while ( getFilteredData() > DISTANCE_WALL - NOISE_MARGIN ) {
			setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
		}

		while ( getFilteredData() < DISTANCE_WALL ) {
			setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
		}
		
		// stop the motors and return the rising edge angle
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
			deltaHeading = Math.abs(40 - (angleA + angleB)/2.0);
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

}
