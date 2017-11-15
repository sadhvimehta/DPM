package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Responsible for handling traversal of river
 *
 */
public class RiverTraversal {
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private Odometer odometer;
    private Navigation navigation;
    private static final double DISTANCE_WALL = 20;
    
    private double xMiddle;
    private double yMiddle;
    

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
     * Method responsible for performing river traversal
     */
    public void doTraversal() {
        //first, go to the entry point of the bridge, but stay withn the points, so calculate difference
    	//caculate the middle of the lane
    	yMiddle = CaptureFlagMain.LL_horizontalShallow_y + ((CaptureFlagMain.UR_horizontalShallow_y - CaptureFlagMain.LL_horizontalShallow_y)/2);
    	xMiddle = CaptureFlagMain.LL_verticalShallow_x + ((CaptureFlagMain.UR_verticalShallow_x - CaptureFlagMain.LL_verticalShallow_x)/2);
    	
    	//travel to the entry of the bridge
    	navigation.travelTo(CaptureFlagMain.LL_verticalShallow_x, yMiddle);
    	
    	//travel to the corner of the bridge
    	navigation.travelTo(xMiddle, yMiddle);
    	
    	//travel to exit of bridge
    	navigation.travelTo(xMiddle, CaptureFlagMain.LL_verticalShallow_y);
    	
    	//end of traversal
    }
}
