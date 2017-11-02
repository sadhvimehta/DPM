package ca.mcgill.ecse211.finalproject.controller;

import java.util.ArrayList;

import ca.mcgill.ecse211.finalproject.main.Main;
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
 * <h1>LightLocalization</h1>
 *
 * <p style="text-indent: 30px">
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
    private double dCdt; // store difference b/w prev and current cs
    private long correctionStart, correctionEnd;
    private ArrayList<Double> lastNValue = new ArrayList<>(); // stores last 40 cs readings


    private boolean atApproxOrigin = false;

    private int lineCounter;
    private double[] saveLineAngles;

    private final float LIGHT_DIFF_THRESHOLD = 40;
    private static final int WAIT_PERIOD = 1000; //in milliseconds
    private final double SENSOR_TO_WHEEL = 14.0; //distance between the wheels and the sensor
    private static final long LOOP_TIME = 10;
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

    public void doLocalization() {
        //1st, get the robot close to where the origin is

        goToEstimateOrigin();

        //2nd, turn around the origin and detect the lines
        checkLines();

        calculatePosition();

        navigation.travelTo(0, 0);

    }

    private void goToEstimateOrigin() {

        //turn 45 degrees to face origin (0,0)
        this.leftMotor.setSpeed(Navigation.ROTATE_SPEED);
        this.rightMotor.setSpeed(Navigation.ROTATE_SPEED);
        this.leftMotor.rotate(Navigation.convertAngle(Main.WHEEL_RADIUS, Main.TRACK, 45), true);
        this.rightMotor.rotate(-Navigation.convertAngle(Main.WHEEL_RADIUS, Main.TRACK, 45), false);


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

        this.leftMotor.rotate(-Navigation.convertDistance(Main.WHEEL_RADIUS, SENSOR_TO_WHEEL), true);
        this.rightMotor.rotate(-Navigation.convertDistance(Main.WHEEL_RADIUS, SENSOR_TO_WHEEL), false);
        System.out.println("Reversing from origin " + odo.getX() + " " + odo.getY());
        System.out.println("Theta: " + odo.getTheta());

    }

    private void checkLines() {
        //it turns anti clockwise, so 1st line it sees in neg y, then pos x, then pos y, then neg x

        //Set up variables
        lineCounter = 0;
        saveLineAngles = new double[4];

        startTime = System.currentTimeMillis();
        this.leftMotor.setSpeed(Navigation.ROTATE_SPEED);
        this.rightMotor.setSpeed(Navigation.ROTATE_SPEED);
        this.leftMotor.rotate(Navigation.convertAngle(Main.WHEEL_RADIUS, Main.TRACK, 360), true);
        this.rightMotor.rotate(-Navigation.convertAngle(Main.WHEEL_RADIUS, Main.TRACK, 360), true);

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

    private void calculatePosition() {
        //Trigonometry calculations from tutorial
        thetaY = saveLineAngles[3] - saveLineAngles[1]; //Y+ - Y-
        thetaX = saveLineAngles[2] - saveLineAngles[0];    //X+ - X-

        positionX = -SENSOR_TO_WHEEL * Math.cos((thetaY) / 2);
        positionY = -SENSOR_TO_WHEEL * Math.cos((thetaX) / 2);


        dT = Math.toRadians(270.00) + (thetaY / 2) - saveLineAngles[3]; //y-

        //Updates odometer to actual values
        this.odo.setX(positionX);
        this.odo.setY(positionY);
        this.odo.setTheta(this.odo.getTheta() + dT);
        //TODO: remove these prints
        System.out.println("dt:" + dT);
        System.out.println("x:" + this.odo.getX());
        System.out.println("y:" + this.odo.getY());

    }

    /**
     * method adds in values to the array with logic
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
     * this method indicates whether a line has actually been passed over
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

        // the correctionstart and corredtionend are to make sure that a value is taken once every
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

