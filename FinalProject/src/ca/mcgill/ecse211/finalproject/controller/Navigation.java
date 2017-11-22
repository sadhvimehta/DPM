package ca.mcgill.ecse211.finalproject.controller;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.geometry.Point2D;

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
        if (odometer.getTheta() != theta) {
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
    	// below, navigation to premount
	    double premountpointX = CaptureFlagMain.ziplineOther_green_x;
	    double premountpointY = CaptureFlagMain.ziplineOther_green_y;

	    boolean isVertical = false;
	    boolean isHorizontal = false;
	    boolean isDiagonal = false;

    	if(CaptureFlagMain.ziplineOther_green_x == CaptureFlagMain.ziplineEndPoint_green_x) {
		    isVertical = true;
        }
        else if(CaptureFlagMain.ziplineOther_green_y == CaptureFlagMain.ziplineEndPoint_green_y) {
		    isHorizontal = true;
	    }
	    else {
    		//case where the zipline is at an angle
	        isDiagonal = true;
    	}
    	// takes care of avoiding home search zone
    	if(CaptureFlagMain.LL_mysearch_x <= premountpointX && premountpointX <= CaptureFlagMain.UR_mysearch_x){
    		travelTo(odometer.getX()/SIDE_SQUARE, premountpointY);
    		travelTo(premountpointX, premountpointY);
    	}
	    else{
	    	if(CaptureFlagMain.LL_mysearch_y <= odometer.getY() && odometer.getY() <= CaptureFlagMain.UR_mysearch_y){
	    		travelTo(odometer.getX()/SIDE_SQUARE, premountpointY);
	    		travelTo(premountpointX, premountpointY);
	    	}
	    	else{
	    		travelTo(premountpointX, odometer.getY()/SIDE_SQUARE);
	    		travelTo(premountpointX, premountpointY);
	    	}
	    }
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
    
	public void travelToUpdate(double x, double y){
				x = x * SIDE_SQUARE;
				y = y * SIDE_SQUARE;
				double currentX = odometer.getX();
				double currentY = odometer.getY();
				Point2D.Double currentPosition = new Point2D.Double(currentX, currentY);
				Point2D.Double desiredPosition = new Point2D.Double(x, y);
				double distanceToTravel = currentPosition.distance(desiredPosition); // calculates distance between the two points
				double differenceInTheta = turnToAngle(x,y);
				turnToUpdate(differenceInTheta);
				
				// drive forward required distance
			    leftMotor.setSpeed(FORWARD_SPEED);
			    rightMotor.setSpeed((int) (FORWARD_SPEED * CaptureFlagMain.balanceConstant));
			    leftMotor.rotate(convertDistance(CaptureFlagMain.WHEEL_RADIUS, distanceToTravel), true);
			    rightMotor.rotate(convertDistance(CaptureFlagMain.WHEEL_RADIUS, distanceToTravel), false);
	}
	
	void turnToUpdate(double differenceInTheta){ // makes robot turn by the minimal angle (in radians)
		if((differenceInTheta >= -(Math.PI)) && (differenceInTheta <= Math.PI)){
			if(differenceInTheta < 0){
				leftMotor.setSpeed(ROTATE_SPEED);
			    rightMotor.setSpeed((int) (ROTATE_SPEED * CaptureFlagMain.balanceConstant));
				leftMotor.rotate(-(convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, Math.toDegrees(-differenceInTheta))), true);
				rightMotor.rotate(convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK, Math.toDegrees(-differenceInTheta)), false);
			}
			else{
				leftMotor.setSpeed(ROTATE_SPEED);
			    rightMotor.setSpeed((int) (ROTATE_SPEED * CaptureFlagMain.balanceConstant));
				rightMotor.rotate(-(convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK,  Math.toDegrees(differenceInTheta))), true);
				leftMotor.rotate(convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK,  Math.toDegrees(differenceInTheta)), false);
			}
		}
		else if(differenceInTheta < -(Math.PI)){
			differenceInTheta = differenceInTheta + (2*Math.PI);
			leftMotor.setSpeed(ROTATE_SPEED);
		    rightMotor.setSpeed((int) (ROTATE_SPEED * CaptureFlagMain.balanceConstant));
			leftMotor.rotate(convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK,  Math.toDegrees(differenceInTheta)), true);
			rightMotor.rotate(-(convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK,  Math.toDegrees(differenceInTheta))), false);
		}
		else if(differenceInTheta > (Math.PI)){
			differenceInTheta = (2*Math.PI) - differenceInTheta;
			leftMotor.setSpeed(ROTATE_SPEED);
		    rightMotor.setSpeed((int) (ROTATE_SPEED * CaptureFlagMain.balanceConstant));
			rightMotor.rotate(convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK,  Math.toDegrees(differenceInTheta)), true);
			leftMotor.rotate(-(convertAngle(CaptureFlagMain.WHEEL_RADIUS, CaptureFlagMain.TRACK,  Math.toDegrees(differenceInTheta))), false);
		}
	}
	
	/**
	 * Calculates angle to turn to for when wanting to travel to a specific point
	 * @param x x-coordinate of desired point
	 * @param y y-coordinate of desired point
	 * @return angle to face to travel to desired point
	 */
	public double turnToAngle(double x, double y){
		double currentX = odometer.getX();
		double currentY = odometer.getY();
		
		// correct orientation
		double updatedTheta = Math.atan2(x - currentX, y - currentY);
		if(updatedTheta < 0){ // Make it follow the 0 - 2*pi convention (not -pi to +pi)
			updatedTheta = ((2.0*Math.PI) + updatedTheta);
		}
		
		double differenceInTheta = (updatedTheta - odometer.getTheta());
		return differenceInTheta;
	}
}
