package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.main.Main;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Responsible for handling traversal of zipline
 */
public class ZiplineTraversal {
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private Odometer odometer;
    private EV3LargeRegulatedMotor ziplineMotor;
    private Navigation navigation;
    private SampleProvider usSensor;
    private float[] usData;
    private static final double DISTANCE_WALL = 20;

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

    public void doTraversal() {
        // first go to the premount in a rectangular fashion
        navigation.travelTo(Main.xPreMount, odometer.getY());
        navigation.travelTo(Main.xPreMount, Main.yPreMount);

        // then go to face the mount
        navigation.travelTo(Main.xMount, Main.yMount);

        // then mount the zipline
        leftMotor.setSpeed(Navigation.FORWARD_SPEED);
        rightMotor.setSpeed(Navigation.FORWARD_SPEED);
        ziplineMotor.setSpeed(Navigation.FORWARD_SPEED);

        // if sensor detects wall or not
        if (getFilteredData() < DISTANCE_WALL) {
            leftMotor.stop(true);
            rightMotor.stop(true);
            ziplineMotor.stop();
        }
    }

    // method to get distance from the wall
    private float getFilteredData() {
        usSensor.fetchSample(usData, 0);
        float distance = usData[0] * 100;
        return distance > 100 ? 100 : distance;
    }
}
