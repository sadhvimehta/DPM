package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.main.Main;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Responsible for handling traversal of zipline
 */
public class ZiplineTraversal implements UltrasonicController{
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private Odometer odometer;
    private EV3LargeRegulatedMotor ziplineMotor;
    private Navigation navigation;
    private SampleProvider usSensor;
    private float[] usData;
    private static final double DISTANCE_WALL = 20;
    private static final double SIDE_SQUARE = 30.48;

    public ZiplineTraversal(Navigation navigation,
                            Odometer odometer,
                            EV3LargeRegulatedMotor leftMotor,
                            EV3LargeRegulatedMotor rightMotor,
                            EV3LargeRegulatedMotor ziplineMotor,
                            SampleProvider usSensor,
                            float[] usData) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.odometer = odometer;
        this.ziplineMotor = ziplineMotor;
        this.navigation = navigation;
        this.usSensor = usSensor;
        this.usData = usData;
    }
    
    /**
     * Method responsible for performing zipline traversal
     */
    public void doTraversal() {
        // first go to the premount
    	navigation.travelTo(Main.ziplineOther_green_x, Main.ziplineOther_green_y);

        // then go to face the mount
        navigation.travelTo(Main.ziplineEndPoint_green_x, Main.ziplineEndPoint_green_y);

        // finds approximate length of zipline
        double deltax = Main.ziplineEndPoint_red_x * SIDE_SQUARE - odometer.getX();
        double deltay = Main.ziplineEndPoint_green_y * SIDE_SQUARE - odometer.getY();
        double h = Math.sqrt(Math.pow(deltax, 2) + Math.pow(deltay, 2));
        
        // mount the zipline
        leftMotor.setSpeed(Navigation.FORWARD_SPEED);
        rightMotor.setSpeed(Navigation.FORWARD_SPEED);
        ziplineMotor.setSpeed(Navigation.FORWARD_SPEED);
        
        // travel approximate length of zipline
        ziplineMotor.rotate(Navigation.convertDistance(Main.WHEEL_RADIUS, h), true);
        navigation.advance((long) h, false);
        
        navigation.travelTo(Main.ziplineOther_red_x, Main.ziplineOther_red_y);
        
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
