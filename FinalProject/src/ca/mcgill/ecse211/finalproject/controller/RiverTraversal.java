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
    private double cornerZero_x;
    private double cornerZero_y;
    private double cornerOne_x;
    private double cornerOne_y;
    private double cornerTwo_x;
    private double cornerTwo_y;
    private double cornerThree_x;
    private double cornerThree_y;
    private double entryPoint_x;
    private double entryPoint_y;

    private boolean horizontalFirst = false;
    
    
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
     * Method responsible for seeing if point is within red zone boundaries
     */
    private boolean isContainedInRed(int x, int y){
    	
    	// 1st segment: (corner zero to corner one)
    	if(y == cornerZero_y){
    		if(cornerZero_x <= x && x <= cornerOne_x){
    			return true;
    		}
    	}
    	// 2nd segment: (corner one to corner two)
    	else if( x == cornerOne_x){
    		if(cornerOne_y <= y && y <= cornerTwo_y){
    			return true;
    		}
    	}
    	// 3rd segment: (corner two to corner three)
    	else if( y == cornerTwo_y){
    		if(cornerThree_x <= x && x <= cornerTwo_x){
    			return true;
    		}
    	}
    	// 4th segment: (corner three to corner zero)
    	else if( x == cornerThree_x){
    		if(cornerZero_y <= y && y <= cornerThree_y){
    			return true;
    		}
    	}
    	return false;
    }

	/**
	 *
	 */
	private double farthestPoint(double position, double point1, double point2) {
		if (Math.abs(position - point1) > Math.abs(position - point2)) {
			return point1;
		}
		else {
			return point2;
		}
	}
	/**
	 * Method responsible to determine entry point of river.
     */
    private void entryPointRiver(){
    	// initialize corners
    	cornerZero_x = CaptureFlagMain.LL_redZone_x;
		cornerZero_y = CaptureFlagMain.LL_redZone_y;
		cornerOne_x = CaptureFlagMain.UR_redZone_x;
		cornerOne_y = CaptureFlagMain.LL_redZone_y;
		cornerTwo_x = CaptureFlagMain.UR_redZone_x;
		cornerTwo_y = CaptureFlagMain.UR_redZone_y;
		cornerThree_x = CaptureFlagMain.LL_redZone_x;
		cornerThree_y = CaptureFlagMain.UR_redZone_y;
    	// start with vertical segments:
		if((isContainedInRed(CaptureFlagMain.LL_verticalShallow_x, CaptureFlagMain.LL_verticalShallow_y))){
			entryPoint_x = ((CaptureFlagMain.UR_verticalShallow_x + CaptureFlagMain.LL_verticalShallow_x) * 0.5);
			entryPoint_y = CaptureFlagMain.LL_verticalShallow_y;
		}
		else if((isContainedInRed(CaptureFlagMain.UR_verticalShallow_x, CaptureFlagMain.UR_verticalShallow_y))){
			entryPoint_x = ((CaptureFlagMain.UR_verticalShallow_x + CaptureFlagMain.LL_verticalShallow_x) * 0.5);
			entryPoint_y = CaptureFlagMain.UR_verticalShallow_y;
		}
		// now horizontal segments:
		else if((isContainedInRed(CaptureFlagMain.LL_horizontalShallow_x, CaptureFlagMain.LL_horizontalShallow_y))){
			entryPoint_x = CaptureFlagMain.LL_horizontalShallow_x;
			entryPoint_y = ((CaptureFlagMain.LL_horizontalShallow_y + CaptureFlagMain.UR_horizontalShallow_y) * 0.5);
			horizontalFirst = true;
		}
		else{
			entryPoint_x = CaptureFlagMain.UR_horizontalShallow_x;
			entryPoint_y = ((CaptureFlagMain.LL_horizontalShallow_y + CaptureFlagMain.UR_horizontalShallow_y) * 0.5);
			horizontalFirst = true;
		}
    }
    /**
     * Method responsible for performing river traversal by first entering through middle of shallow water entry, <br>
     * traveling along this middle until one reaches midpoint of vertical segment's entry of shallow water, <br>
     * then traveling to exit of vertical segment of shallow water.
     */
    public void doTraversal() {
    	// find entry point
	    entryPointRiver();

	    if (horizontalFirst) {
		    // travel to entry point of shallow water in square like fashion
		    navigation.travelToUpdate(odometer.getX() / 30.48, entryPoint_y);
		    // TODO: add localization here
		    navigation.travelToUpdate(entryPoint_x, entryPoint_y);
	    }
	    else {
		    // travel to entry point of shallow water in square like fashion
		    navigation.travelToUpdate(entryPoint_x, odometer.getY() / 30.48);
		    // TODO: add localization here
		    navigation.travelToUpdate(entryPoint_x, entryPoint_y);
	    }
    	
    	// TODO: add travel to end of first segment of bridge
	    double[] bridgeMiddle = {((CaptureFlagMain.LL_verticalShallow_x + CaptureFlagMain.UR_verticalShallow_x) * 0.5), ((CaptureFlagMain.LL_horizontalShallow_y + CaptureFlagMain.UR_horizontalShallow_y) * 0.5)};

	    navigation.travelToUpdate(bridgeMiddle[0], bridgeMiddle[1]);

    	//TODO: add travel to end of second segment of bridge
	    if (horizontalFirst) {
		    navigation.travelToUpdate(bridgeMiddle[0], farthestPoint(bridgeMiddle[1], CaptureFlagMain.UR_verticalShallow_y, CaptureFlagMain.LL_verticalShallow_y));
	    }
	    else {
	    	navigation.travelToUpdate(farthestPoint(bridgeMiddle[0], CaptureFlagMain.UR_horizontalShallow_x, CaptureFlagMain.LL_horizontalShallow_y), bridgeMiddle[1]);
	    }
    	//end of traversal
    }
}
