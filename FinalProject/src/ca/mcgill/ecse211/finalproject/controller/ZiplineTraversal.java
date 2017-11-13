package ca.mcgill.ecse211.finalproject.controller;

import com.sun.org.apache.bcel.internal.classfile.Constant;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Responsible for handling traversal of zipline
 */
public class ZiplineTraversal implements UltrasonicController{
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private Odometer odometer;
    private LightLocalization lightLocalization;
    private EV3LargeRegulatedMotor ziplineMotor;
    private Navigation navigation;
    private SampleProvider usSensor;
    private float[] usData;
    private static final double DISTANCE_WALL = 20;
    private static final double SIDE_SQUARE = 30.48;
    private double ZIPLINE_LENGTH = 123.4;
    private double ZIPLINE_NOISE_LENGTH = -10.00; // used to adjust bottom motors due to inital delay during mount

    public ZiplineTraversal(Navigation navigation,
                            Odometer odometer,
                            LightLocalization lightLocalization,
                            EV3LargeRegulatedMotor leftMotor,
                            EV3LargeRegulatedMotor rightMotor,
                            EV3LargeRegulatedMotor ziplineMotor,
                            SampleProvider usSensor,
                            float[] usData) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.odometer = odometer;
        this.lightLocalization = lightLocalization;
        this.ziplineMotor = ziplineMotor;
        this.navigation = navigation;
        this.usSensor = usSensor;
        this.usData = usData;
    }
    
    /**
     * Method responsible for going to zipline and performing zipline traversal
     */
    public void doTraversal() {

        // first go to the premount in a square-like fashion
    	navigation.travelToPremount();
    	
        //localise to correct our angle and position
        lightLocalization.zipLineLocalization = true;
        navigation.turnTo(Math.toRadians(45)); //turn to 45 to ensure we cross the correct lines during localization
        lightLocalization.doLocalization();
        
        // then go to face the mount
        navigation.travelTo(CaptureFlagMain.ziplineEndPoint_green_x, CaptureFlagMain.ziplineEndPoint_green_y);
        
        // possible lee way for zipline motor to latch on to zipline
        //navigation.advance((long) 10.00, false);
        
        // mount the zipline
        leftMotor.setSpeed(Navigation.FORWARD_SPEED);
        rightMotor.setSpeed((int) (Navigation.FORWARD_SPEED * CaptureFlagMain.balanceConstant));
        ziplineMotor.setSpeed(Navigation.FORWARD_SPEED *2);
        
       // travel approximate length of zipline (negative b/c motor attached backwards)
        ziplineMotor.rotate(-Navigation.convertDistance(CaptureFlagMain.ZIPLINE_WHEEL_RADIUS, ZIPLINE_LENGTH), true);
        navigation.advance((long) (ZIPLINE_LENGTH), false); // ends up on roughly on zipline red other point
    }

	@Override
	public void processUSData(float usData) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public float readUSData() {
		usSensor.fetchSample(usData, 0);
        float distance = usData[0] * 100;
        return distance > 100 ? 100 : distance;
	}
}
