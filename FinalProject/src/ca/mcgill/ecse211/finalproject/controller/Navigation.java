package ca.mcgill.ecse211.finalproject.controller;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;

import java.util.ArrayList;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;

/**
 * Class responsible for the moving of the robot throughout the competition. This means that it contains all methods
 * responsible for moving robot by desired distances, rotating by desired angles, and traveling to desired points.
 * The travelTo and turnTo go hand in hand as we need the robot to face the right way to go to the right location.
 * The convert methods convert distances and angles with which the lejos motor methods can work with. Other important
 * methods, travelToPremount and returnToOrigin, are more situational methods that are used for a specific part of the
 * competition.
 *
 */
public class Navigation{

    public static final double SIDE_SQUARE = 30.48;
    public static final int FORWARD_SPEED = 250;
    public static final int ROTATE_SPEED = 200;
    private double theta;
    private boolean isNavigating;
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private Odometer odometer;
    private ArrayList<Integer[]> map = new ArrayList<>();

    /**
	 * Constructor for the class Navigation which links parameters to class variables.
     */
    public Navigation(
            Odometer odometer,
            EV3LargeRegulatedMotor leftMotor,
            EV3LargeRegulatedMotor rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.odometer = odometer;

        for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[]{leftMotor, rightMotor}) {
            motor.stop();
            motor.setAcceleration(3000);
        }
        
    }
    /**
     * Method responsible to make robot travel to a certain point.
     * @param x x-coordinate to travel to.
     * @param y y-coordinate to travel to.
     */
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

        // the robot then rolls forward to get to the point
        leftMotor.setSpeed(FORWARD_SPEED);

        rightMotor.setSpeed((int) (FORWARD_SPEED * CaptureFlagMain.balanceConstant));
        leftMotor.rotate(convertDistance(CaptureFlagMain.WHEEL_RADIUS, h), true);
        rightMotor.rotate(convertDistance(CaptureFlagMain.WHEEL_RADIUS, h), false);

        /*leftMotor.stop(true);
        rightMotor.stop(false);*/
        
        isNavigating = false; 
    }
    /**
     * Method that converts distance to travel to required wheel rotations.
     * @param radius wheel radius.
     * @param distance distance to travel.
     * @return wheel rotations in degrees.
     */
    public static int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }
    
    /**
     * Method that converts angle to rotate by to required wheel rotations.
     * @param radius wheel radius.
     * @param width distance between the wheels.
     * @param angle angle to rotate by in degrees.
     * @return wheel rotations in degrees.
     */
    public static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }
    
    /**
     * Method responsible to rotate robot by minimal angle to face a certain orientation.
     * @param theta angle to rotate by in radians.
     */
    public void turnTo(double theta) {
        // the robot checks the angle it must travel to
        double travelangle = theta - odometer.getTheta();

        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed((int) (ROTATE_SPEED * CaptureFlagMain.balanceConstant));

        // this is just to make sure that the robot takes the most optimal route, thus always turning
        // 180 or less
        if (Math.abs(travelangle) < Math.PI) {
            if (travelangle > 0) {
                leftMotor.rotate(
                        convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, Math.toDegrees(Math.abs(travelangle))), true);
                rightMotor.rotate(
                        -convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, Math.toDegrees(Math.abs(travelangle))), false);
            } else if (travelangle < 0) {
                leftMotor.rotate(
                        -convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, Math.toDegrees(Math.abs(travelangle))), true);
                rightMotor.rotate(
                        convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, Math.toDegrees(Math.abs(travelangle))), false);
            }
        } else if (Math.abs(travelangle) > Math.PI) {
            if (travelangle > Math.PI) {
                leftMotor.rotate(
                        -convertAngle(
                                CaptureFlagMain.WHEEL_RADIUS,  CaptureFlagMain.TRACK,
                                Math.toDegrees(Math.abs(2 * Math.PI - Math.abs(travelangle)))),
                        true);
                rightMotor.rotate(
                        convertAngle(
                                CaptureFlagMain.WHEEL_RADIUS,  CaptureFlagMain.TRACK,
                                Math.toDegrees(Math.abs(2 * Math.PI - Math.abs(travelangle)))),
                        false);
            } else if (travelangle < -Math.PI) {
                leftMotor.rotate(
                        convertAngle(
                                CaptureFlagMain.WHEEL_RADIUS,  CaptureFlagMain.TRACK,
                                Math.toDegrees(Math.abs(2 * Math.PI - Math.abs(travelangle)))),
                        true);
                rightMotor.rotate(
                        -convertAngle(
                                CaptureFlagMain.WHEEL_RADIUS,  CaptureFlagMain.TRACK,
                                Math.toDegrees(Math.abs(2 * Math.PI - Math.abs(travelangle)))),
                        false);
            }
        }
        //leftMotor.stop(true);
        //rightMotor.stop(false);
    }
    
    /**
     * Method that determines if robot is still navigating.
     * @return isNavigating boolean that is either true or false.
     */
    public boolean isNavigating() {
        return isNavigating;
    }
    
    /**
     * Method that rotates robot clockwise by certain degrees.
     * @param degree angle to rotate by.
     */
    public void turnCW(long degree) {
        leftMotor.rotate(
                convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, degree), true);
        rightMotor.rotate(
                -convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, degree), true);
    }
    
    /**
     * Method that rotates robot counterclockwise by certain degrees.
     * @param degree angle to rotate by.
     */
    public void turnCCW(long degree) {
        leftMotor.rotate(
                -convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, degree), true);
        rightMotor.rotate(
                convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, degree), true);
    }
    
    /**
     * Method that makes robot move by desired distance.
     * @param distance distance to move by.
     * @param immediateReturn boolean that determines synchronization of motors.
     */
    public void advance(long distance, boolean immediateReturn) {
    	leftMotor.setSpeed(FORWARD_SPEED);
    	rightMotor.setSpeed((int) (FORWARD_SPEED * CaptureFlagMain.balanceConstant)); 
        leftMotor.rotate(convertDistance(CaptureFlagMain.WHEEL_RADIUS, distance), true);
        rightMotor.rotate(convertDistance(CaptureFlagMain.WHEEL_RADIUS, distance), immediateReturn);
        //leftMotor.stop(true);
        //rightMotor.stop(false);
    }
    
    /**
     * Method responsible for making robot move to a premount point in right angles (imitates square driver). <br>
     * It also handles whether to travel in x-direction or y-direction first to ensure no collision with zipline.
     */
    public void travelToPremount(){
    	// setting up for odometry correction (go to middle of first square)
    	if(CaptureFlagMain.startingCorner == 0)
    		travelTo(0.50, 0.50);
    	else if(CaptureFlagMain.startingCorner == 1)
    		travelTo(7.50, 0.50);
    	else if(CaptureFlagMain.startingCorner == 2)
    		travelTo(7.50, 7.50);
    	else if(CaptureFlagMain.startingCorner == 3)
    		travelTo(0.50, 7.50);
    	
    	// below, navigation to premount
	    double premountpointX;
	    double premountpointY;

	    boolean isVertical = false;
	    boolean isHorizontal = false;
	    boolean isDiagonal = false;

    	if(CaptureFlagMain.ziplineOther_green_x == CaptureFlagMain.ziplineEndPoint_green_x) {
    		premountpointX = CaptureFlagMain.ziplineOther_green_x;
        	if (CaptureFlagMain.ziplineOther_green_y > CaptureFlagMain.ziplineEndPoint_green_y) {
        		premountpointY = CaptureFlagMain.ziplineOther_green_y + 0.5;
	        }
	        else {
        		premountpointY = CaptureFlagMain.ziplineOther_green_y - 0.5;
	        }
		    isVertical = true;
        }
        else if(CaptureFlagMain.ziplineOther_green_y == CaptureFlagMain.ziplineEndPoint_green_y) {
    		premountpointY = CaptureFlagMain.ziplineOther_green_y;
		    if (CaptureFlagMain.ziplineOther_green_x > CaptureFlagMain.ziplineEndPoint_green_x) {
			    premountpointX = CaptureFlagMain.ziplineOther_green_x + 0.5;
		    } else {
			    premountpointX = CaptureFlagMain.ziplineOther_green_x - 0.5;
		    }
		    isHorizontal = true;
	    }
	    else {
    		//case where the zipline is at an angle
	        if(CaptureFlagMain.ziplineOther_green_x < CaptureFlagMain.ziplineEndPoint_green_x && CaptureFlagMain.ziplineOther_green_y < CaptureFlagMain.ziplineEndPoint_green_y) {
	        	premountpointX = CaptureFlagMain.ziplineOther_green_x - 0.5;
	        	premountpointY = CaptureFlagMain.ziplineOther_green_y - 0.5;
	        }
	        else if (CaptureFlagMain.ziplineOther_green_x > CaptureFlagMain.ziplineEndPoint_green_x && CaptureFlagMain.ziplineOther_green_y < CaptureFlagMain.ziplineEndPoint_green_y) {
		        premountpointX = CaptureFlagMain.ziplineOther_green_x + 0.5;
		        premountpointY = CaptureFlagMain.ziplineOther_green_y - 0.5;
	        } else if (CaptureFlagMain.ziplineOther_green_x > CaptureFlagMain.ziplineEndPoint_green_x && CaptureFlagMain.ziplineOther_green_y > CaptureFlagMain.ziplineEndPoint_green_y) {
		        premountpointX = CaptureFlagMain.ziplineOther_green_x + 0.5;
		        premountpointY = CaptureFlagMain.ziplineOther_green_y + 0.5;
	        }
	        else {
		        premountpointX = CaptureFlagMain.ziplineOther_green_x - 0.5;
		        premountpointY = CaptureFlagMain.ziplineOther_green_y + 0.5;
	        }
	        isDiagonal = true;
    	}

	    if(odometer.getX() == premountpointX || odometer.getY() == premountpointY) {
    		travelTo(premountpointX, premountpointY);
	    }
	    else if (isHorizontal) {
    		travelTo(premountpointX, odometer.getY()/SIDE_SQUARE);
    		travelTo(premountpointX, premountpointY);
	    }
	    else if (isVertical) {
    		travelTo(odometer.getX()/SIDE_SQUARE, premountpointY);
    		travelTo(premountpointX, premountpointY);
	    }
	    else if (isDiagonal) {
    		travelTo(premountpointX, odometer.getY()/SIDE_SQUARE);
    		travelTo(premountpointX, premountpointY);
	    }

	    travelTo(CaptureFlagMain.ziplineOther_green_x, CaptureFlagMain.ziplineOther_green_y);

    }
    
    /**
     * Method responsible to return robot to its starting corner after it has located enemy's flag.
     */
    public void returnToOrigin(){
    	
    	 if(CaptureFlagMain.startingCorner == 0){
    		 travelTo( 0 + SIDE_SQUARE /2 , 0+ SIDE_SQUARE/2);	
         }
         
         else if(CaptureFlagMain.startingCorner == 1){
        	 travelTo( 12 - SIDE_SQUARE /2 , 0 + SIDE_SQUARE/2);
         	
         }
         else if(CaptureFlagMain.startingCorner == 2){
        	 travelTo( 12 - SIDE_SQUARE /2 , 12 - SIDE_SQUARE/2);
         }
         else if(CaptureFlagMain.startingCorner == 3){
         	  travelTo( 0 + SIDE_SQUARE /2 , 12 - SIDE_SQUARE/2);
         }
        	
    	
    }
    
    
    
    
    
    
}
