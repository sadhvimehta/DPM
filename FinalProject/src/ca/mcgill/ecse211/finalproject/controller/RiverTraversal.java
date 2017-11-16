package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Responsible for finding a path through shallow water and actual traversal of river using shallow water crossing.
 * This class mainly contains the doTrversal method which takes care of this river traversal to go to the green side.
 *
 */
public class RiverTraversal {
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
	 * Navigation which contains basic methods of moving our robot.
	 */
    private Navigation navigation;
    
    private double xMiddle;
    private double yMiddle;
    
    
    /**
	 * Constructor for the class RiverTraversal which links parameters to class variables.
     */
    public RiverTraversal(Navigation navigation,
                            Odometer odometer,
                            EV3LargeRegulatedMotor leftMotor,
                            EV3LargeRegulatedMotor rightMotor) {
        this.navigation = navigation;
        this.odometer = odometer;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }
    
    /**
     * Method responsible for performing river traversal by first entering through middle of shallow water entry, <br>
     * traveling along this middle until one reaches midpoint of vertical segment's entry of shallow water, <br>
     * then traveling to exit of vertical segment of shallow water.
     */
    public void doTraversal() {
        //first, go to the entry point of the bridge, but stay withn the points, so calculate difference
    	//caculate the middle of the lane
    	yMiddle = CaptureFlagMain.LL_horizontalShallow_y + ((CaptureFlagMain.UR_horizontalShallow_y - CaptureFlagMain.LL_horizontalShallow_y)/2);
    	xMiddle = CaptureFlagMain.LL_verticalShallow_x + ((CaptureFlagMain.UR_verticalShallow_x - CaptureFlagMain.LL_verticalShallow_x)/2);
    	
    	//travel to the entry of the bridge
    	navigation.travelTo(CaptureFlagMain.LL_horizontalShallow_x, yMiddle);
    	
    	//travel to the corner of the bridge
    	navigation.travelTo(xMiddle, yMiddle);
    	
    	//travel to exit of bridge
    	navigation.travelTo(xMiddle, CaptureFlagMain.LL_verticalShallow_y);
    	
    	//end of traversal
    }
}
