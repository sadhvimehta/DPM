package ca.mcgill.ecse211.lab5;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

import java.util.ArrayList;

public class Navigation {
    private static final double SIDE_SQUARE = 30.48;
    private static final int FOWARD_SPEED = 250;
    private static final int ROTATE_SPEED = 100;

    private double theta;
    private double wheelRadius;
    private double wheelWidth;
    private boolean isNavigating;
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private Odometer odometer;

    private ArrayList<Integer[]> map = new ArrayList<>();

    public Navigation(
            Odometer odometer,
            EV3LargeRegulatedMotor leftMotor,
            EV3LargeRegulatedMotor rightMotor,
            double wheelRadius,
            double wheelWidth) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.odometer = odometer;
        this.wheelRadius = wheelRadius;
        this.wheelWidth = wheelWidth;

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
        leftMotor.setSpeed(FOWARD_SPEED);
        rightMotor.setSpeed(FOWARD_SPEED);
        leftMotor.rotate(convertDistance(wheelRadius, h), true);
        rightMotor.rotate(convertDistance(wheelRadius, h), false);

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
                        convertAngle(wheelRadius, wheelWidth, Math.toDegrees(Math.abs(travelangle))), true);
                rightMotor.rotate(
                        -convertAngle(wheelRadius, wheelWidth, Math.toDegrees(Math.abs(travelangle))), false);
            } else if (travelangle < 0) {
                leftMotor.rotate(
                        -convertAngle(wheelRadius, wheelWidth, Math.toDegrees(Math.abs(travelangle))), true);
                rightMotor.rotate(
                        convertAngle(wheelRadius, wheelWidth, Math.toDegrees(Math.abs(travelangle))), false);
            }
        } else if (Math.abs(travelangle) > Math.PI) {
            if (travelangle > Math.PI) {
                leftMotor.rotate(
                        -convertAngle(
                                wheelRadius,
                                wheelWidth,
                                Math.toDegrees(Math.abs(2 * Math.PI - Math.abs(travelangle)))),
                        true);
                rightMotor.rotate(
                        convertAngle(
                                wheelRadius,
                                wheelWidth,
                                Math.toDegrees(Math.abs(2 * Math.PI - Math.abs(travelangle)))),
                        false);
            } else if (travelangle < -Math.PI) {
                leftMotor.rotate(
                        convertAngle(
                                wheelRadius,
                                wheelWidth,
                                Math.toDegrees(Math.abs(2 * Math.PI - Math.abs(travelangle)))),
                        true);
                rightMotor.rotate(
                        -convertAngle(
                                wheelRadius,
                                wheelWidth,
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
                convertAngle(LabFiveMain.WHEEL_RADIUS, LabFiveMain.TRACK, degree), true);
        rightMotor.rotate(
                -convertAngle(LabFiveMain.WHEEL_RADIUS, LabFiveMain.TRACK, degree), true);
    }

    public void turnCCW(long degree) {
        leftMotor.rotate(
                -convertAngle(LabFiveMain.WHEEL_RADIUS, LabFiveMain.TRACK, degree), true);
        rightMotor.rotate(
                convertAngle(LabFiveMain.WHEEL_RADIUS, LabFiveMain.TRACK, degree), true);
    }

    public void advance(long distance) {
        leftMotor.rotate(convertDistance(wheelRadius, distance), true);
        rightMotor.rotate(convertDistance(wheelRadius, distance), false);
    }
}
