package ca.mcgill.ecse211.finalproject.controller;

import java.util.ArrayList;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.sensor.LightController;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import sun.awt.geom.AreaOp;


/** *
 * LightLocalization class is responsible for localizing the robot through line detection getting the derivative of the light level. <br>
 * This allows to flag if a line is passed regardless of the initial light source. <br>
 * While spinning, the angles at which the lines are crossed are noted and used to calculate the X, Y, and Theta offset of the robot.
 *
 */
public class LightLocalization implements LightController {
	/**
	 * Navigation which contains basic methods of moving our robot.
	 */
    private Navigation navigation;
	/**
	 * Odometer which keeps track of where the robot is positioned based on wheel movement.
	 */
    private Odometer odometer;
	/**
	 * Sample provider of the color sensor used to fetch light sensor's readings.
	 */
    private SampleProvider csSensor;
	/**
	 * Array containing data obtained from light sensor.
	 */
    private float[] csData;
	/**
	 * Variables which store the current and previous light value to calculate the instantaneous differentiation.
	 */
	private double lightValueCurrent, lightValuePrev;
	/**
	 * Variable which is the difference of the angles between the positive X axis and negative X axis.
	 */
	private double thetaX;
	/**
	 * Variable which is the difference of the angles between the positive Y axis and negative Y axis.
	 */
	private double thetaY;
	/**
	 * Variable which is the calculated offset of the robot in X direction.
	 */
    private double positionX;
	/**
	 * Variable which is the calculated offset of the robot in Y direction.
	 */
    private double positionY;
	/**
	 * Variable which is the calculated angle offset of the robot.
	 */
    private double dT;
	/**
	 * Variable which is the start time of the method.
	 */
	private long startTime;
    /**
     * Stores difference between previous and current light sensor readings and is used in line detection algorithm.
     */
    private double dCdt;
	/**
	 * Variables that hold the values of the start and end of sensor polling.
	 */
	private long correctionStart, correctionEnd;
    /**
     * Buffer that stores light sensor readings for differential line detection algorithm.
     */
    private ArrayList<Double> lastNValue = new ArrayList<>(); // stores last 40 cs readings

    /**
     * Boolean that indicates if robot is at origin or not.
     */
    private boolean atApproxOrigin = false;
    
    /**
     * Counter that determines number of lines detected.
     */
    private int lineCounter;
    /**
     * Buffer that stores angles at which lines are detected.
     */
    private double[] saveLineAngles;
    
    /**
     * Threshold to be passed to confirm line was detected
     */
    private final float LIGHT_DIFF_THRESHOLD = 40;
    /**
     * Distance to move to origin.
     */
    private final double SENSOR_TO_WHEEL = 14.0; //distance between the wheels and the sensor
    /**
     * Period for which sensor reading must be taken.
     */
    private static final long LOOP_TIME = 5;
    /**
     * Boolean indicating line detection has started.
     */
    private boolean firstpass = true;
	/**
	 * Left and right motor of the robot.
	 */
    private EV3LargeRegulatedMotor leftMotor, rightMotor;
    
    /**
     * Boolean indicating light localization is happening for zipline mounting. <br>
     * This boolean is used to communicate between ZiplineTraversal class and LightLocalization class.
     */
    public boolean zipLineLocalization = false;
    /**
     * Boolean indicating light localization is happening for zipline dismounting. <br>
     * This boolean is used to communicate between ZiplineTraversal class and LightLocalization class.
     */
    public boolean endZipLineLocalization = false;
    /**
     * Variable indicating approx value of light sensorr when line is detected.
     */
	public int lightThreshold = 38;
    

	/**
	 * Constructor which links the parameters to the class variables.
	 */
    public LightLocalization(Navigation navigation,
                             Odometer odo,
                             EV3LargeRegulatedMotor leftMotor,
                             EV3LargeRegulatedMotor rightMotor,
                             SampleProvider csSensor,
                             float[] csData) {
        this.odometer = odo;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.navigation = navigation;
        this.csSensor = csSensor;
        this.csData = csData;

    }
    
    /**
     * Main method which localizes the robot through the line detection algorithm that relies on light intensity differentiation.
     */
    public void doLocalization() {
    	// do not perform odometry correction when light localizing
    	CaptureFlagMain.doCorrection = false;
    	// if either ziplineLocalization or endZiplineLocalizatioin are true, don't go to the origin
    	if((!zipLineLocalization) && (!endZipLineLocalization)){ 
    		// get the robot close to where the origin is
            goToEstimateOrigin();
    	}

	    /*this.leftMotor.setSpeed(Navigation.ROTATE_SPEED);
	    this.rightMotor.setSpeed(Navigation.ROTATE_SPEED * constant);
    	while(!pastline()) {
		    navigation.turnCCW(360);
	    }
	    // stops ??
	    this.leftMotor.stop(true);
	    this.rightMotor.stop(true);*/

        // turn around the origin and detect the lines
        checkLines();
        
        // calculate positional offset
        calculatePosition();
    }
    
    /**
     * Method that moves robot to origin to commence light localization.
     */
    private void goToEstimateOrigin() {

        //turn 45 degrees to face origin (0,0)
        this.leftMotor.setSpeed(Navigation.ROTATE_SPEED);
        this.rightMotor.setSpeed((int) (Navigation.ROTATE_SPEED * CaptureFlagMain.balanceConstant));
        this.leftMotor.rotate(Navigation.convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, 45), true);
        this.rightMotor.rotate(-Navigation.convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, 45), false);


        //then go straight
        this.leftMotor.setSpeed(Navigation.FORWARD_SPEED);
        this.rightMotor.setSpeed((int) (Navigation.FORWARD_SPEED * CaptureFlagMain.balanceConstant));
        this.leftMotor.forward();
        this.rightMotor.forward();

        while (!atApproxOrigin) { //boolean to check if we have arrived or not
            lightValueCurrent = readLSData(); //update data

            //If the difference in colour intensity is bigger than a chosen threshold, a line was detected
            if (lightValueCurrent <= lightThreshold) {
                leftMotor.stop(true);
                rightMotor.stop(true);
                atApproxOrigin = true;
                Sound.beep();
            }
        }
        //go backwards so the front wheels are at the origin and not the sensor
        this.leftMotor.setSpeed(Navigation.FORWARD_SPEED);
        this.rightMotor.setSpeed((int) (Navigation.FORWARD_SPEED * CaptureFlagMain.balanceConstant));
        this.leftMotor.rotate(-Navigation.convertDistance(CaptureFlagMain.WHEEL_RADIUS, SENSOR_TO_WHEEL), true);
        this.rightMotor.rotate(-Navigation.convertDistance(CaptureFlagMain.WHEEL_RADIUS, SENSOR_TO_WHEEL), false);

    }
    
    /**
     * Method responsible for robot to rotate and detect lines to be able to calcualte positional offset.
     */
    private void checkLines() {
        //int lastLTC = leftMotor.getTachoCount();
        //int lastRTC = rightMotor.getTachoCount();
    	
    	//it turns anti clockwise, so 1st line it sees in neg y, then pos x, then pos y, then neg x

        //Set up variables
        lineCounter = 0;
        saveLineAngles = new double[4];

        startTime = System.currentTimeMillis();
        this.leftMotor.setSpeed(Navigation.ROTATE_SPEED);
        this.rightMotor.setSpeed((int) (Navigation.ROTATE_SPEED * CaptureFlagMain.balanceConstant));
        this.leftMotor.rotate(Navigation.convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, 400), true);
        this.rightMotor.rotate(-Navigation.convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, 400), true);

        /*try {
	        Thread.sleep(1000);
        }
        catch (Exception e ) {}*/

        //Runs until it has detected 4 lines
        while (lineCounter < 4) {
            if (pastline()) {
                //Store angles in variable for future calculations
                saveLineAngles[lineCounter] = this.odometer.getTheta();
                Sound.beep();
                lineCounter++;
            }
        }

        //int totalLTC = leftMotor.getTachoCount() - lastLTC;
        //int totalRTC = rightMotor.getTachoCount() - lastRTC;
	    /*this.leftMotor.stop(true);
	    this.rightMotor.stop(false);*/
        //int averageTC = (Math.abs(totalLTC) + Math.abs(totalRTC))/2;
        //CaptureFlagMain.TRACK = 2 * (CaptureFlagMain.WHEEL_RADIUS * averageTC) / (360);
    }
    
    /**
     * Method responsible to calculate positional offset of robot from true position.
     */
    private void calculatePosition() {
        //Trigonometry calculations from tutorial
        thetaY = saveLineAngles[3] - saveLineAngles[1]; //Y+ - Y-
        thetaX = saveLineAngles[2] - saveLineAngles[0];    //X+ - X-

        positionX = -SENSOR_TO_WHEEL * Math.cos((thetaY) / 2);
        positionY = -SENSOR_TO_WHEEL * Math.cos((thetaX) / 2);


        dT = Math.toRadians(270.00) + (thetaY / 2) - saveLineAngles[3]; //y-

        double newTheta = this.odometer.getTheta() + dT;
        if(120.00 <= Math.toDegrees(newTheta) && Math.toDegrees(newTheta) <= 220.00){
        	newTheta = newTheta + Math.PI;
        	if(newTheta >= (2*Math.PI))
        		newTheta = newTheta - 2*Math.PI;
        }
        
        if(zipLineLocalization){
        	this.odometer.setX(positionX + (CaptureFlagMain.ziplineOther_green_x *30.48));
        	this.odometer.setY(positionY + (CaptureFlagMain.ziplineOther_green_y *30.48));
        	this.odometer.setTheta(newTheta);
        }
        else if(endZipLineLocalization) {
	        this.odometer.setX(positionX + (CaptureFlagMain.ziplineOther_red_x * 30.48));
	        this.odometer.setY(positionY + (CaptureFlagMain.ziplineOther_red_y * 30.48));
	        this.odometer.setTheta(newTheta);
        }
       //Updates odometer to actual values depending on corner!
        else if(CaptureFlagMain.startingCorner == 0){
        	this.odometer.setX(positionX + (1.00*30.48));
        	this.odometer.setY(positionY + (1.00*30.48));
        	this.odometer.setTheta(newTheta);
        }
        
        else if(CaptureFlagMain.startingCorner == 1){
        	positionX = Math.abs(positionX);
        	this.odometer.setX(positionX + (7.00*30.48));
        	this.odometer.setY(positionY + (1.00*30.48));
        	this.odometer.setTheta(newTheta + Math.toRadians(270.00));
        	
        }
        else if(CaptureFlagMain.startingCorner == 2){
        	positionX = Math.abs(positionX);
        	positionY = Math.abs(positionY);
        	this.odometer.setX(positionX + (7.00*30.48));
        	this.odometer.setY(positionY + (7.00*30.48));
        	this.odometer.setTheta(newTheta + Math.toRadians(180.00));
        }
        else if(CaptureFlagMain.startingCorner == 3){
        	positionY = Math.abs(positionY);
        	this.odometer.setX(positionX + (1.00*30.48));
        	this.odometer.setY(positionY + (7.00*30.48));
        	this.odometer.setTheta(newTheta + Math.toRadians(90.00));
        }
    }

    /**
     * Method adds in values to the {@link #lastNValue} with logic.
     */
    public void lastNValueAdd(double value) {
        // the array does not take chucks of the values and risk to miss a big difference.
        // Instead we slide along
        if (this.lastNValue.size() > 100) {
            // the oldest value is removed to make space

            this.lastNValue.remove(0);
            this.lastNValue.add(value);
        } else {
            // if the array happens to be less that, simply add them

            this.lastNValue.add(value);
        }
    }

    /**
     * Method responsible for performing line detection algorithm
     */
    public boolean pastline() {
    	if(readLSData() <= lightThreshold){
    		//Sound.setVolume(30);
    		//Sound.beep();
    		return true;
    	}
    	else
    		return false;
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
     * Retrieves intensity read by light sensor.
     *
     * @return light sensor reading
     */
	@Override
	public float readLSData() {
        //correctionStart = System.currentTimeMillis();

        csSensor.fetchSample(csData, 0);
        float color = csData[0] * 100;

        // the correctionstart and correctionend are to make sure that a value is taken once every
        // LOOP_TIME
        /*correctionEnd = System.currentTimeMillis();
        if (correctionEnd - correctionStart < LOOP_TIME) {
            try {
                Thread.sleep(LOOP_TIME - (correctionEnd - correctionStart));
            } catch (InterruptedException e) {
            }
        }*/

        return color;
	}


}

