package ca.mcgill.ecse211.finalproject.controller;

import com.sun.org.apache.bcel.internal.classfile.Constant;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Responsible for handling traversal of zipline by arriving to premount point, localizing at premount point,
 * mounting zipline, traversing it, and localize after dismount.
 */
public class ZiplineTraversal{
	/**
	 * left motor of robot.
	 */
    private EV3LargeRegulatedMotor leftMotor;
    /**
     * right motor of robot.
     */
    private EV3LargeRegulatedMotor rightMotor;
    /**
     * Odometer which is responsible for calculating robot's current position using odometry.
     */
    private Odometer odometer;
    /**
     * LightLocalization which is responsible for mounting at premount point postdismount point.
     */
    private LightLocalization lightLocalization;
    /**
     * zipline motor of robot.
     */
    private EV3LargeRegulatedMotor ziplineMotor;
    /**
	 * Navigation which contains basic methods of moving our robot.
	 */
    private Navigation navigation;
    /**
     * length of zipline in cm.
     */
    private double ZIPLINE_LENGTH = 123.4;
    /**
	 * Constructor for the class ZiplineTraversal which links parameters to class variables.
     */
    public ZiplineTraversal(Navigation navigation,
                            Odometer odometer,
                            LightLocalization lightLocalization,
                            EV3LargeRegulatedMotor leftMotor,
                            EV3LargeRegulatedMotor rightMotor,
                            EV3LargeRegulatedMotor ziplineMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.odometer = odometer;
        this.lightLocalization = lightLocalization;
        this.ziplineMotor = ziplineMotor;
        this.navigation = navigation;
    }
    
    /**
     * Method responsible for going to zipline and performing zipline traversal.
     */
    public void doTraversal() {

        // first go to the premount in a square-like fashion
    	navigation.travelToPremount();
    	
        //localise to correct our angle and position
        lightLocalization.zipLineLocalization = true;
        navigation.turnTo(Math.toRadians(45)); //turn to 45 to ensure we cross the correct lines during localization
        lightLocalization.doLocalization();
        lightLocalization.zipLineLocalization = false;
        
        // then go to face the mount
        navigation.travelTo(CaptureFlagMain.ziplineEndPoint_green_x, CaptureFlagMain.ziplineEndPoint_green_y);
        
        // mount the zipline
        leftMotor.setSpeed(Navigation.FORWARD_SPEED);
        rightMotor.setSpeed((int) (Navigation.FORWARD_SPEED * CaptureFlagMain.balanceConstant));
        ziplineMotor.setSpeed(Navigation.FORWARD_SPEED *2);
        
       // travel approximate length of zipline (negative b/c motor attached backwards)

        ziplineMotor.rotate(-Navigation.convertDistance(CaptureFlagMain.ZIPLINE_WHEEL_RADIUS, ZIPLINE_LENGTH * 1.50), true);
        navigation.travelTo(CaptureFlagMain.ziplineEndPoint_red_x, CaptureFlagMain.ziplineEndPoint_red_y);


        // localize after it descends zipline
        lightLocalization.endZipLineLocalization = true;
        ziplineMotor.stop();
        navigation.turnTo(Math.toRadians(45));
        lightLocalization.doLocalization();
        lightLocalization.endZipLineLocalization = false;
    }

}
