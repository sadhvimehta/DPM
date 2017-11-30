package ca.mcgill.ecse211.finalproject.controller;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.geometry.Point2D;
import java.util.ArrayList;
import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;

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
    public static final int SECURE_BLOCK_LENGTH = 4;
    private double theta;
    private boolean isNavigating;
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private Odometer odometer;
	private SampleProvider csValue;
	private float[] csData;

    /**
	 * Constructor for the class Navigation which links parameters to class variables.
     */
    public Navigation(Odometer odometer,
                      EV3LargeRegulatedMotor leftMotor,
                      EV3LargeRegulatedMotor rightMotor,
                      SampleProvider csValue,
                      float[] csData) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.odometer = odometer;
        this.csValue = csValue;
        this.csData = csData;

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
    		 travelTo( 0.5,0.5);
         }
         
         else if(CaptureFlagMain.startingCorner == 1){
        	 travelTo( CaptureFlagMain.MAP_SIZE - 0.5 , 0.5);
         	
         }
         else if(CaptureFlagMain.startingCorner == 2){
        	 travelTo( CaptureFlagMain.MAP_SIZE - 0.5, CaptureFlagMain.MAP_SIZE - 0.5);
         }
         else if(CaptureFlagMain.startingCorner == 3){
         	  travelTo( 0.5, CaptureFlagMain.MAP_SIZE - 0.5);
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

	/**
	 *
	 */
	public void travelToWLocalize(double x, double y) {
		double dx = x - Math.round(odometer.getX() / Navigation.SIDE_SQUARE);
		double dy = y - Math.round(odometer.getY() / Navigation.SIDE_SQUARE);

		LightLocalization lightLocalization = new LightLocalization(this, odometer, leftMotor, rightMotor, csValue, csData);

		if (dx > Navigation.SECURE_BLOCK_LENGTH || dy > Navigation.SECURE_BLOCK_LENGTH) {
			if (Math.abs(dx) > Math.abs(dy)) {    // moves horizontally
				if (dx > 0) {                     // moves to the right
					for (int i = 0; i < Math.floor( dx / Navigation.SECURE_BLOCK_LENGTH); i++) {
						travelToUpdate(Math.round(odometer.getX() / Navigation.SIDE_SQUARE) + Navigation.SECURE_BLOCK_LENGTH, y);
						lightLocalization.localizeOnTheMove = true;
						turnTo(Math.toRadians(45)); //turn to 45 to ensure we cross the correct lines during localization
						lightLocalization.doLocalization();
						lightLocalization. localizeOnTheMove = false;
					}
				} else {                           // moves to the left
					for (int i = 0; i < Math.floor(dx / Navigation.SECURE_BLOCK_LENGTH); i++) {
						travelToUpdate(Math.round(odometer.getX() / Navigation.SIDE_SQUARE) - Navigation.SECURE_BLOCK_LENGTH, y);
						lightLocalization.localizeOnTheMove = true;
						turnTo(Math.toRadians(45)); //turn to 45 to ensure we cross the correct lines during localization
						lightLocalization.doLocalization();
						lightLocalization.localizeOnTheMove = false;
					}
				}
			} else {                                // moves vertically
				if (dy > 0) {                     // moves to the right
					for (int i = 0; i < Math.floor(dy / Navigation.SECURE_BLOCK_LENGTH); i++) {
						System.out.println(dy/ Navigation.SECURE_BLOCK_LENGTH); //check going there
						travelToUpdate(x, Math.round(odometer.getY() / Navigation.SIDE_SQUARE) + Navigation.SECURE_BLOCK_LENGTH);
						lightLocalization.localizeOnTheMove = true;
						turnTo(Math.toRadians(45)); //turn to 45 to ensure we cross the correct lines during localization
						lightLocalization.doLocalization();
						lightLocalization.localizeOnTheMove = false;
					}
				} else {                           // moves to the left
					for (int i = 0; i < Math.floor(dy / Navigation.SECURE_BLOCK_LENGTH); i++) {
						travelToUpdate(x, Math.round(odometer.getY() / Navigation.SIDE_SQUARE) - Navigation.SECURE_BLOCK_LENGTH);
						lightLocalization.localizeOnTheMove = true;
						turnTo(Math.toRadians(45)); //turn to 45 to ensure we cross the correct lines during localization
						lightLocalization.doLocalization();
						lightLocalization.localizeOnTheMove = false;
					}
				}

			}
		}
		Sound.setVolume(30);
		Sound.beep();
		travelToUpdate(x, y);
	}
}
