package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import lejos.robotics.geometry.Point2D;
import lejos.robotics.geometry.Point2D.Double;
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

    // below correspond to corners of search region.
    // corner zero is lower left corner
    // corner one is lower right corner
    // corner two is upper right corner
    // corner three is upper left corner
    private double cornerZero_x;
    private double cornerZero_y;
    private double cornerOne_x;
    private double cornerOne_y;
    private double cornerTwo_x;
    private double cornerTwo_y;
    private double cornerThree_x;
    private double cornerThree_y;


    
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
		// initialize corners of search zone
		cornerZero_x = CaptureFlagMain.LL_search_x;
		cornerZero_y = CaptureFlagMain.LL_search_y;
		cornerOne_x = CaptureFlagMain.UR_search_x;
		cornerOne_y = CaptureFlagMain.LL_search_y;
		cornerTwo_x = CaptureFlagMain.UR_search_x;
		cornerTwo_y = CaptureFlagMain.UR_search_y;
		cornerThree_x = CaptureFlagMain.LL_search_x;
		cornerThree_y = CaptureFlagMain.UR_search_y;
		


	}

	/**
	 * Main method of this class which will contain logic to go about finding the opponents flag. 
	 * It navigates robot to the flag zone and then instructs it to looks for blocks and identify enemy flag.
	 * This method also represents one of the states that controller will be in along the competition.
	 */
	//TODO: complete this method
	public void findFlag() {
		// find corner closest to robot's current position and travel to it(after dismounting zipline)
		System.out.println("lower left search (x) : " + CaptureFlagMain.LL_mysearch_x);
		System.out.println("lower left search (y) : " + CaptureFlagMain.LL_mysearch_y);
		System.out.println("upper right search (x) : " + CaptureFlagMain.UR_mysearch_x);
		System.out.println("upper right search (y) : " + CaptureFlagMain.UR_mysearch_y);
		int closestCorner = 0;
		double shortestDistance = distanceBetweenPoint(CaptureFlagMain.ziplineOther_red_x, CaptureFlagMain.ziplineOther_red_y, this.cornerZero_x, this.cornerZero_y);
		if(distanceBetweenPoint(CaptureFlagMain.ziplineOther_red_x, CaptureFlagMain.ziplineOther_red_y, this.cornerOne_x, this.cornerOne_y) < shortestDistance){
			shortestDistance = distanceBetweenPoint(CaptureFlagMain.ziplineOther_red_x, CaptureFlagMain.ziplineOther_red_y, this.cornerOne_x, this.cornerOne_y);
			closestCorner = 1;
		}
		if((distanceBetweenPoint(CaptureFlagMain.ziplineOther_red_x, CaptureFlagMain.ziplineOther_red_y, this.cornerTwo_x, this.cornerTwo_y) < shortestDistance)){
			shortestDistance = distanceBetweenPoint(CaptureFlagMain.ziplineOther_red_x, CaptureFlagMain.ziplineOther_red_y, this.cornerTwo_x, this.cornerTwo_y);
			closestCorner = 2;
		}
		if((distanceBetweenPoint(CaptureFlagMain.ziplineOther_red_x, CaptureFlagMain.ziplineOther_red_y, this.cornerThree_x, this.cornerThree_y) < shortestDistance)){
			shortestDistance = distanceBetweenPoint(CaptureFlagMain.ziplineOther_red_x, CaptureFlagMain.ziplineOther_red_y, this.cornerThree_x, this.cornerThree_y);
			closestCorner = 3;
		}
		System.out.println("closest corner: " + closestCorner);
		/*if(closestCorner == 0)
			navigation.travelToUpdate(this.cornerZero_x, this.cornerZero_y);
		else if(closestCorner == 1)
			navigation.travelToUpdate(this.cornerOne_x, this.cornerOne_y);
		else if(closestCorner == 2)
			navigation.travelToUpdate(this.cornerTwo_x, this.cornerTwo_y);
		else
			navigation.travelToUpdate(this.cornerThree_x, this.cornerThree_y);*/
	}
	/**
	 * Method that is used to find distance to all four corners of the search region
	 * @param x x-coordinate of desired corner
	 * @param y y-coordinate of desired corner
	 * @return distance to desired corner
	 */
	public double distanceBetweenPoint(double x1, double y1, double x2, double y2){
		x1 = x1 * 30.48;
		x2 = x2 * 30.48;
		y1 = y1 * 30.48;
		y2 = y2 * 30.48;
		Point2D.Double currentPosition = new Point2D.Double(x1, y1);
		Point2D.Double desiredPosition = new Point2D.Double(x2, y2);
		double distance = currentPosition.distance(desiredPosition); // calculates distance between the two point
		System.out.println(distance);
		return distance;
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
