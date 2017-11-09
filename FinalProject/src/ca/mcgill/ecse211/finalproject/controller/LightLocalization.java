package ca.mcgill.ecse211.finalproject.controller;

import java.util.ArrayList;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.sensor.LightController;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;


/**
 * Performs light localization
 *
 */
public class LightLocalization implements LightController {

    private Navigation navigation;
    private Odometer odo;
    private SampleProvider csSensor;
    private float[] csData;
    private double lightValueCurrent, lightValuePrev; // store current cs value, previous cs value
    private double thetaX;
    private double thetaY;
    private double positionX;
    private double positionY;
    private double dT;
    private long startTime;
    /**
     * Stores difference between previous and current light sensor readings
     */
    private double dCdt; // store difference b/w prev and current cs
    private long correctionStart, correctionEnd;
    /**
     * Buffer that stores light sensor readings for differential line detection algorithm
     */
    private ArrayList<Double> lastNValue = new ArrayList<>(); // stores last 40 cs readings

    /**
     * Boolean that indicates if at origin or not
     */
    private boolean atApproxOrigin = false;
    
    /**
     * Counter that determines number of lines detected
     */
    private int lineCounter;
    /**
     * Buffer that stores angles at which lines are detected
     */
    private double[] saveLineAngles;
    
    /**
     * Threshold to be passed to confirm line was detected
     */
    private final float LIGHT_DIFF_THRESHOLD = 40;
    /**
     * Distance to move to origin
     */
    private final double SENSOR_TO_WHEEL = 14.0; //distance between the wheels and the sensor
    /**
     * Period for which sensor reading must be taken
     */
    private static final long LOOP_TIME = 10;
    /**
     * Boolean indicating line detection has started
     */
    private boolean firstpass = true;

    private EV3LargeRegulatedMotor leftMotor, rightMotor;


    public LightLocalization(Navigation navigation, Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, SampleProvider csSensor, float[] csData) {
        this.odo = odo;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.navigation = navigation;
        this.csSensor = csSensor;
        this.csData = csData;

    }
    
    /**
     * Method that performs actual light localization
     */
    public void doLocalization() {
        //1st, get the robot close to where the origin is
    	System.out.println("Hello");
        goToEstimateOrigin();

        //2nd, turn around the origin and detect the lines
        checkLines();

        calculatePosition();

        navigation.travelTo(0, 0);

    }
    
    /**
     * Method that moves robot to origin to commence localization
     */
    private void goToEstimateOrigin() {

        //turn 45 degrees to face origin (0,0)
        this.leftMotor.setSpeed(Navigation.ROTATE_SPEED);
        this.rightMotor.setSpeed(Navigation.ROTATE_SPEED);
        this.leftMotor.rotate(Navigation.convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, 45), true);
        this.rightMotor.rotate(-Navigation.convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, 45), false);


        //then go straight
        this.leftMotor.setSpeed(Navigation.FORWARD_SPEED);
        this.rightMotor.setSpeed(Navigation.FORWARD_SPEED);
        this.leftMotor.forward();
        this.rightMotor.forward();

        while (!atApproxOrigin) { //boolean to check if we have arrived or not
            lightValueCurrent = readLSData(); //update data

            //If the difference in colour intensity is bigger than a chosen threshold, a line was detected
            if (lightValueCurrent <= 38) { //TODO: change this to a non absolute value
                leftMotor.stop(true);
                rightMotor.stop(true);
                atApproxOrigin = true;
                Sound.beep();
            }
        }
        //go backwards so the front wheels are at the origin and not the sensor
        this.leftMotor.setSpeed(Navigation.FORWARD_SPEED);
        this.rightMotor.setSpeed(Navigation.FORWARD_SPEED);

        this.leftMotor.rotate(-Navigation.convertDistance(CaptureFlagMain.WHEEL_RADIUS, SENSOR_TO_WHEEL), true);
        this.rightMotor.rotate(-Navigation.convertDistance(CaptureFlagMain.WHEEL_RADIUS, SENSOR_TO_WHEEL), false);
        System.out.println("Reversing from origin " + odo.getX() + " " + odo.getY());
        System.out.println("Theta: " + odo.getTheta());

    }
    
    /**
     * Method responsible for robot to rotate and detect lines
     */
    private void checkLines() {
        //it turns anti clockwise, so 1st line it sees in neg y, then pos x, then pos y, then neg x

        //Set up variables
        lineCounter = 0;
        saveLineAngles = new double[4];

        startTime = System.currentTimeMillis();
        this.leftMotor.setSpeed(Navigation.ROTATE_SPEED);
        this.rightMotor.setSpeed(Navigation.ROTATE_SPEED);
        this.leftMotor.rotate(Navigation.convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, 360), true);
        this.rightMotor.rotate(-Navigation.convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, 360), true);

        //Runs until it has detected 4 lines
        while (lineCounter < 4) {
            lightValueCurrent = readLSData();

            if (firstpass) {
                lightValuePrev = lightValueCurrent;
                firstpass = false;
            }
            // rather than check the value of light captured, we are checking the instaneous
            // differentiation
            dCdt = lightValueCurrent - lightValuePrev;
            lightValuePrev = lightValueCurrent;

            // add the differentiation of the color to the array
            lastNValueAdd(dCdt);
            if (pastline()) {
                //Store angles in variable for future calculations
                saveLineAngles[lineCounter] = this.odo.getTheta();
                Sound.beep();
                lineCounter++;
            }
        }

        this.leftMotor.stop(true);
        this.rightMotor.stop(true);

    }
    
    /**
     * Method responsible to calculate positional offset of robot from true position
     */
    private void calculatePosition() {
        //Trigonometry calculations from tutorial
        thetaY = saveLineAngles[3] - saveLineAngles[1]; //Y+ - Y-
        thetaX = saveLineAngles[2] - saveLineAngles[0];    //X+ - X-

        positionX = -SENSOR_TO_WHEEL * Math.cos((thetaY) / 2);
        positionY = -SENSOR_TO_WHEEL * Math.cos((thetaX) / 2);


        dT = Math.toRadians(270.00) + (thetaY / 2) - saveLineAngles[3]; //y-

        //Updates odometer to actual values depending on corner!
        if(CaptureFlagMain.startingCorner == 0){
        	this.odo.setX(positionX + (1.00*30.48));
        	this.odo.setY(positionY + (1.00*30.48));
        	this.odo.setTheta(this.odo.getTheta() + dT);
        }
        
        else if(CaptureFlagMain.startingCorner == 1){
        	positionX = Math.abs(positionX);
        	this.odo.setX(positionX + (7.00*30.48));
        	this.odo.setY(positionY + (1.00*30.48));
        	this.odo.setTheta(this.odo.getTheta() + dT + Math.toRadians(270.00));
        	
        }
        else if(CaptureFlagMain.startingCorner == 2){
        	positionX = Math.abs(positionX);
        	positionY = Math.abs(positionY);
        	this.odo.setX(positionX + (7.00*30.48));
        	this.odo.setY(positionY + (7.00*30.48));
        	this.odo.setTheta(this.odo.getTheta() + dT + Math.toRadians(180.00));
        }
        else{
        	positionY = Math.abs(positionY);
        	this.odo.setX(positionX + (1.00*30.48));
        	this.odo.setY(positionY + (7.00*30.48));
        	this.odo.setTheta(this.odo.getTheta() + dT + Math.toRadians(90.00));
        }
        //TODO: remove these prints
        System.out.println("dt:" + dT);
        System.out.println("x:" + this.odo.getX());
        System.out.println("y:" + this.odo.getY());
        System.out.println("theta:" + this.odo.getTheta());

    }

    /**
     * Method adds in values to the {@link #lastNValue} with logic
     */
    public void lastNValueAdd(double value) {
        // the array does not take chucks of the values and risk to miss a big difference.
        // Instead we slide along
        if (this.lastNValue.size() > 40) {
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
        double biggest = -100;
        double smallest = 100;

        // since crossing a line causes a drop and a rise in the derivative, the filter
        // only considers a line crossed if the biggest value is higher than some threshold
        for (int i = 0; i < this.lastNValue.size(); i++) {
            // and the reverse for the lowest value, thus creating one beep per line
            if (this.lastNValue.get(i) > biggest) {
                biggest = this.lastNValue.get(i);
            }
            if (this.lastNValue.get(i) < smallest) {
                smallest = this.lastNValue.get(i);
            }
        }

        // if a sample is considered to be a line, the array is cleared as to not retrigger an other
        // time
        if (biggest > 5 && smallest < -3) {
            this.lastNValue.clear();
            Sound.setVolume(30);
            Sound.beep();
            return true;
        } else {
            return false;
        }
    }

	@Override
	public void processLSData(float lsData) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public float readLSData() {
        correctionStart = System.currentTimeMillis();

        csSensor.fetchSample(csData, 0);
        float color = csData[0] * 100;

        // the correctionstart and correctionend are to make sure that a value is taken once every
        // LOOP_TIME
        correctionEnd = System.currentTimeMillis();
        if (correctionEnd - correctionStart < LOOP_TIME) {
            try {
                Thread.sleep(LOOP_TIME - (correctionEnd - correctionStart));
            } catch (InterruptedException e) {
            }
        }

        return color;
	}


}

