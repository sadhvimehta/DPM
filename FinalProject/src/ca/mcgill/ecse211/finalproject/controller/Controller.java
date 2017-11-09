package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.odometry.OdometryCorrection;
import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Controls sequence of events between block detection, navigation, obstacle avoidance, and zipline traversal.
 *
 * The class Controller is started in main. Controller takes care of localizing the robot, and deciding
 * and controlling the state of the robot to accomplish the tasks at hand to complete the challenge.
 *
 */
public class Controller extends Thread implements UltrasonicController{
	/**
	 * Navigation class which contains basic methods of moving our robot
	 */
	private Navigation navigation;
	/**
	 * OdometryCorrection class which corrects the odometer as the robot is running to make sure our odometer is correct
	 */
	private OdometryCorrection odometryCorrection;
	/**
	 * FallingEdgeUSLocalization class which localizes with an ultrasonic sensor
	 */
    private FallingEdgeUSLocalization usl;
	/**
	 * LightLocalization class which localizes with the light sensor
	 */
    private LightLocalization lightLocalization;
	/**
	 * ZiplineTraversal class which helps the robot traverse the zipline
	 */
	private ZiplineTraversal ziplineTraversal;
	/**
	 * RiverTraversal class which helps the robot traverse the river
	 */
	private RiverTraversal riverTraversal;

	/**
	 * Constructor of the class Controller which uses the parameters to instantiate the classes which will be controlled
	 */
    public Controller(Odometer odometer,
                      SampleProvider usValue,
                      float[] usData,
                      SampleProvider csValue,
                      float[] csData,
                      EV3LargeRegulatedMotor leftMotor,
                      EV3LargeRegulatedMotor rightMotor,
                      EV3LargeRegulatedMotor ziplineMotor,
                      double WHEEL_RADIUS,
                      double TRACK
                      ){

        this.navigation = new Navigation(odometer, leftMotor, rightMotor, WHEEL_RADIUS, TRACK);


        this.usl = new FallingEdgeUSLocalization(odometer, usValue, usData, FallingEdgeUSLocalization.LocalizationType.FALLING_EDGE, leftMotor, rightMotor, navigation);

        this.lightLocalization = new LightLocalization(navigation, odometer, leftMotor, rightMotor, csValue, csData);

        //TODO:uncomment below
        //this.odometryCorrection = new OdometryCorrection(odometer, csValue, csData);
        
        this.ziplineTraversal = new ZiplineTraversal(navigation, odometer, leftMotor, rightMotor, ziplineMotor, usValue, usData);

        //TODO: implement river traversal
	    //this.riverTraversal = new RiverTraversal();

    }

	/**
	 * Method which dictates what the controller thread will do. This method also includes the state machine which helps
	 * with the logic of what state the robot should be depending on factors.
	 */
	public void run() {
    	usl.doLocalization();

        lightLocalization.doLocalization();

        //TODO: uncomment below
        //odometryCorrection.start();
        //ziplineTraversal.doTraversal();
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

	/**
	 * Retrieves distance read by ultrasonic sensor
	 *
	 * @return ultrasonic sensor reading
	 */
	@Override
	public float readUSData() {
		// TODO Auto-generated method stub
		return 0;
	}

}
