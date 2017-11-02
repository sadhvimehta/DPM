package ca.mcgill.ecse211.finalproject.controller;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

import java.util.ArrayList;

import ca.mcgill.ecse211.finalproject.main.Main;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;

/**
 * <h1>Navigation</h1>
 *
 * <p style="text-indent: 30px">
 */
public class Navigation{

    private static final double SIDE_SQUARE = 30.48;
    public static final int FORWARD_SPEED = 250;
    public static final int ROTATE_SPEED = 100;
    private double theta;
    private static double WHEEL_RADIUS;
    private static double TRACK;
    private boolean isNavigating;
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private Odometer odometer;

    private ArrayList<Integer[]> map = new ArrayList<>();

    public Navigation(
            Odometer odometer,
            EV3LargeRegulatedMotor leftMotor,
            EV3LargeRegulatedMotor rightMotor,
            double WHEEL_RADIUS,
            double TRACK) {

        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.odometer = odometer;
        this.WHEEL_RADIUS = WHEEL_RADIUS;
        this.TRACK = TRACK;

        for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[]{leftMotor, rightMotor}) {
            motor.stop();
            motor.setAcceleration(3000);
        }
    }

    public void travelTo(double x, double y) {
        isNavigating = true;
        // this finds the change in x and y necessary to get to the target point
        double deltax = x * SIDE_SQUARE - odometer.getX();
        double deltay = y * SIDE_SQUARE - odometer.getY();

        // this finds the longs side which is the distance the robot must travel
        double h = Math.sqrt(Math.pow(deltax, 2) + Math.pow(deltay, 2));

        // this part creates a triangle and finds the angle which the robot must face to get to its
        // point
        if (deltax == 0 || deltay == 0) {
            if (deltax == 0) {
                if (deltay < 0) {
                    theta = Math.PI;
                } else if (deltay > 0) {
                    theta = 0;
                }
            } else if (deltay == 0) {
                if (deltax < 0) {
                    theta = 3 * Math.PI / 2;
                } else if (deltax > 0) {
                    theta = Math.PI / 2;
                }
            }
        } else if (deltay > 0) {
            if (deltax > 0) {
                theta = Math.PI / 2 - Math.asin(deltay / h);
            } else if (deltax < 0) {
                theta = 3 * Math.PI / 2 + Math.asin(deltay / h);
            }
        } else if (deltay < 0) {
            if (deltax > 0) {
                theta = Math.PI - Math.asin(deltax / h);
            } else if (deltax < 0) {
                theta = Math.PI + Math.asin(Math.abs(deltax) / h);
            }
        }

        // the robot will only turn if it is not facing the angle it must face to get to the point
        if (Math.toDegrees(odometer.getTheta()) != theta) {
            turnTo(theta);
        }

        // the robot then rolls foward to get to the point
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED);
        leftMotor.rotate(convertDistance(WHEEL_RADIUS, h), true);
        rightMotor.rotate(convertDistance(WHEEL_RADIUS, h), false);

        leftMotor.stop();
        rightMotor.stop();

        isNavigating = false;
    }

    public static int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }

    public static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }

    public void turnTo(double theta) {
        // the robot checks the angle it must travel to
        double travelangle = theta - odometer.getTheta();

        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);

        // this is just to make sure that the robot takes the most optimal route, thus always turning
        // 180 or less
        if (Math.abs(travelangle) < Math.PI) {
            if (travelangle > 0) {
                leftMotor.rotate(
                        convertAngle(WHEEL_RADIUS, TRACK, Math.toDegrees(Math.abs(travelangle))), true);
                rightMotor.rotate(
                        -convertAngle(WHEEL_RADIUS, TRACK, Math.toDegrees(Math.abs(travelangle))), false);
            } else if (travelangle < 0) {
                leftMotor.rotate(
                        -convertAngle(WHEEL_RADIUS, TRACK, Math.toDegrees(Math.abs(travelangle))), true);
                rightMotor.rotate(
                        convertAngle(WHEEL_RADIUS, TRACK, Math.toDegrees(Math.abs(travelangle))), false);
            }
        } else if (Math.abs(travelangle) > Math.PI) {
            if (travelangle > Math.PI) {
                leftMotor.rotate(
                        -convertAngle(
                                WHEEL_RADIUS,
                                TRACK,
                                Math.toDegrees(Math.abs(2 * Math.PI - Math.abs(travelangle)))),
                        true);
                rightMotor.rotate(
                        convertAngle(
                                WHEEL_RADIUS,
                                TRACK,
                                Math.toDegrees(Math.abs(2 * Math.PI - Math.abs(travelangle)))),
                        false);
            } else if (travelangle < -Math.PI) {
                leftMotor.rotate(
                        convertAngle(
                                WHEEL_RADIUS,
                                TRACK,
                                Math.toDegrees(Math.abs(2 * Math.PI - Math.abs(travelangle)))),
                        true);
                rightMotor.rotate(
                        -convertAngle(
                                WHEEL_RADIUS,
                                TRACK,
                                Math.toDegrees(Math.abs(2 * Math.PI - Math.abs(travelangle)))),
                        false);
            }
        }
    }

    public boolean isNavigating() {
        return isNavigating;
    }

    public void turnCW(long degree) {
        leftMotor.rotate(
                convertAngle(Main.WHEEL_RADIUS, Main.TRACK, degree), true);
        rightMotor.rotate(
                -convertAngle(Main.WHEEL_RADIUS, Main.TRACK, degree), true);
    }

    public void turnCCW(long degree) {
        leftMotor.rotate(
                -convertAngle(Main.WHEEL_RADIUS, Main.TRACK, degree), true);
        rightMotor.rotate(
                convertAngle(Main.WHEEL_RADIUS, Main.TRACK, degree), true);
    }

    public void advance(long distance, boolean immediateReturn) {
        leftMotor.rotate(convertDistance(WHEEL_RADIUS, distance), true);
        rightMotor.rotate(convertDistance(WHEEL_RADIUS, distance), immediateReturn);
    }
}
