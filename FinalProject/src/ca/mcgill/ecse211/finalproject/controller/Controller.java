package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.odometry.OdometryCorrection;
import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/** *
 * Controller takes care of all actions required to
 * complete the course and the sequence of these actions. <br>
 * This includes localizing the robot, 
 * navigation of robot, zipline traversal, river traversal, and finally starting the block detection. <br>
 * It is important to note that this class is not a thread as it simply controls the sequence of the robot's actions
 * by calling the respective methods from the respective classes to finish the obstacle course. <br>
 * To view the state machine diagram which describes how this class operates, please open the link below in a new tab and zoom into
 * the controller class: <br>
 * <a href="https://www.dropbox.com/s/8m3zkmdv4t1gw5t/stateDiagram.jpg?dl=0"> State Machine Diagram </a>
 *
 */
public class Controller{
	/**
	 * Navigation which contains basic methods of moving our robot to input points and to travel in square-like fashion.
	 */
	private Navigation navigation;
	/**
	 * Odometer which calculates robot's position using odometry.
	 */
	
	private Odometer odometer;
	/**
	 * OdometryCorrection which corrects the odometer through line detection as the robot is moving to ensure  odometer is correct.
	 */
	private OdometryCorrection odometryCorrection;
	/**
	 * FallingEdgeUSLocalization which localizes with an ultrasonic sensor at the start of obstacle course.
	 */
    private FallingEdgeUSLocalization usl;
	/**
	 * LightLocalization which localizes with the light sensor.
	 */
    private LightLocalization lightLocalization;
	/**
	 * ZiplineTraversal which helps the robot traverse the zipline.
	 */
	private ZiplineTraversal ziplineTraversal;
	/**
	 * RiverTraversal which helps the robot traverse the river.
	 */
	private RiverTraversal riverTraversal;
	/**
	 * BlockDetection which helps robot eliminate blocks that are not the enemy's flag and help detect enemy's flag.
	 */
	private BlockDetection blockDetection;
	

	/**
	 * Constructor of the class Controller which uses the parameters to instantiate the classes which will be controlled.
	 */
    public Controller(Odometer odometer,
                      SampleProvider usValue,
                      float[] usData,
                      SampleProvider csValue,
                      float[] csData,
                      EV3LargeRegulatedMotor leftMotor,
                      EV3LargeRegulatedMotor rightMotor,
                      EV3LargeRegulatedMotor ziplineMotor
                      ){

        this.navigation = new Navigation(odometer, leftMotor, rightMotor);

    	this.odometer = odometer;

        this.usl = new FallingEdgeUSLocalization(odometer, usValue, usData, FallingEdgeUSLocalization.LocalizationType.FALLING_EDGE, leftMotor, rightMotor, navigation);

        this.lightLocalization = new LightLocalization(navigation, odometer, leftMotor, rightMotor, csValue, csData);

        this.odometryCorrection = new OdometryCorrection(odometer, lightLocalization);
        
        this.ziplineTraversal = new ZiplineTraversal(navigation, odometer, lightLocalization, leftMotor, rightMotor, ziplineMotor);

      //TODO: implement block detection
        this.blockDetection = new BlockDetection(navigation, odometer, leftMotor, rightMotor, csValue, csData);
        
        //TODO: implement river traversal
	    this.riverTraversal = new RiverTraversal(navigation, odometer, leftMotor, rightMotor);

    }

	/**
	 * Method which dictates in what order Controller should call the necessary methods
	 * required for robot to complete the course. This method also includes the state machine linked above which helps
	 * with the logic of what state the robot should be in depending on the surrounding factors and its progress in the 
	 * completion of the course.
	 */
	public void startCourseTraversal() {
		// perform ultrasonic falling edge localization
    	usl.doLocalization();
    	// perform initial light localization
        lightLocalization.doLocalization();
        
        if(CaptureFlagMain.teamColor == "Green"){
        	// travel to zipline and traverse it
        	ziplineTraversal.doTraversal();        
        	// find flag
        	// TODO: complete findFlag method
        	blockDetection.findFlag();
        	// traverse the river
        	// TODO: complete the doTraversal method
        	//riverTraversal.doTraversal();
        	//then go back to origin
        	//navigation.returnToOrigin();
        }
    }

}
