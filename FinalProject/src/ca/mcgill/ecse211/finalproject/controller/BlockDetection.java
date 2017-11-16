package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.sensor.LightController;
import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Contains all methods necessary to detect the enemy's flag based on light intensity readings. <br>
 * It is is instantiated within the controller and its respective methods are called upon by the controller as well.
 *
 */
public class BlockDetection implements UltrasonicController, LightController{
	/**
	 * Navigation which contains basic methods of moving our robot.
	 */
	private Navigation navigation;
	/**
	 * Odometer which calculates robot's position using odometry.

	 */
	private Odometer odometer;
	/**
	 * Left motor of the robot.
	 */
	private EV3LargeRegulatedMotor leftMotor;
	/**
	 * Right motor of the robot.
	 */
	private EV3LargeRegulatedMotor rightMotor;
	/**
	 * Sample provider of the color sensor used to fetch light sensor's readings.
	 */
	private SampleProvider csSensor;
	/**
	 * Array containing data obtained from light sensor.
	 */
    private float[] csData;
    /**
     * Buffer that contains light intensity readings from range of distances whichh will be used to identify enemy's flag.
     */
    private float[] intensityBuffer;

	/**
	 * Constructor of the class BlockDetection, which links the parameters to the class variables.
	 */
	public BlockDetection(Navigation navigation,
			              Odometer odometer,
	                      EV3LargeRegulatedMotor leftMotor,
	                      EV3LargeRegulatedMotor rightMotor,
	                      SampleProvider csSensor,
	                      float[] csData){
		this.navigation = navigation;
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.csSensor = csSensor;
		this.csData = csData;
	}

	/**
	 * Main method of this class which will contain logic to go about finding the opponents flag. 
	 * It navigates robot to the flag zone and then instructs it to looks for blocks and identify enemy flag.
	 * This method also represents one of the states that controller will be in along the competition.
	 */
	//TODO: complete this method
	public void findFlag() {
		 // go to flag zone
        navigation.travelTo(CaptureFlagMain.UR_search_x, CaptureFlagMain.LL_search_y);
        
        
	}
	
	/**
	 * Determines color of detected block using intensityBuffer values. 
	 * This method will therefore help to determine if block found is indeed the flag of the opposing team.
	 * @return number representing color of detected block
	 */
	private int determineColor(){
		return 0;
	}
	
	/**
	 * Provides the intensity of block from various distances to populate intensity buffer. 
	 * This method is called upon once a block has been encountered and values are needed to determine its color.
	 */
	private void populateIntensityBuffer(){
		
	}

	/**
	 * Performs any processing of light sensor data.
	 *
	 * @param lsData light intensity reading
	 */
	@Override
	public void processLSData(float lsData) {
		// TODO Auto-generated method stub
		
	}

	/**
	 * Retrieves intensity read by light sensor
	 *
	 * @return light sensor reading
	 */
	@Override
	public float readLSData() {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * Performs any processing of ultrasonic sensor data.
	 *
	 * @param usData ultrasonic sensor reading
	 */
	@Override
	public void processUSData(float usData) {
		// TODO Auto-generated method stub
		
	}


	@Override
	public float readUSData() {
		// TODO Auto-generated method stub
		return 0;
	}

}
