package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * <h1>RiverTraversal</h1>
 *
 * <p style="text-indent: 30px">
 */
public class RiverTraversal {
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private Odometer odometer;
    private Navigation navigation;
    private static final double DISTANCE_WALL = 20;

    public RiverTraversal(Navigation navigation,
                            Odometer odometer,
                            EV3LargeRegulatedMotor leftMotor,
                            EV3LargeRegulatedMotor rightMotor) {
        this.navigation = navigation;
        this.odometer = odometer;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    public void doTraversal() {
        //TODO: add body
    }
}
