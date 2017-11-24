package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import lejos.hardware.Sound;
import lejos.robotics.geometry.Point2D;
import lejos.robotics.geometry.Point2D.Double;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.sensor.LightController;
import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Contains all methods necessary to detect the enemy's flag based on light intensity readings. <br>
 * It is is instantiated within the controller and its respective methods are called upon by the controller as well.
 *
 */
public class BlockDetection{
	/**
	 * Period for which sensor reading must be taken.
	 */
	private static final long LOOP_TIME = 5;
	private double DIST_FAULT = 0.165;  // about 5 cm
	private double INCREMENT = 0.328; // about 10 cm
	/**
	 * Navigation which contains basic methods of moving our robot.
	 */
	private Navigation navigation;
	/**
	 * Odometer which calculates robot's position using odometry.

	 */
	private Odometer odometer;
	/**
	 * Left motor of the robot.
	 */
	private EV3LargeRegulatedMotor leftMotor;
	/**
	 * Right motor of the robot.
	 */
	private EV3LargeRegulatedMotor rightMotor;
	/**
	 * Sample provider of the color sensor used to fetch light sensor's readings.
	 */
	private SampleProvider csSensor;
	/**
	 * Array containing data obtained from light sensor.
	 */
    private float[] csData;
    /**
     * Buffer that contains light intensity readings from range of distances whichh will be used to identify enemy's flag.
     */
    private float[] intensityBuffer;
    
    private LightLocalization lightLocalization;

    // below correspond to corners of search region.
    // corner zero is lower left corner
    // corner one is lower right corner
    // corner two is upper right corner
    // corner three is upper left corner
    private double cornerZero_x;
    private double cornerZero_y;
    private double cornerOne_x;
    private double cornerOne_y;
    private double cornerTwo_x;
    private double cornerTwo_y;
    private double cornerThree_x;
    private double cornerThree_y;

    private double[][] corners = new double[4][2];

    private double[] colorData = new double[3];

	private int closestCorner = 0;

	private String color;
	/**
	 * Variables that hold the values of the start and end of sensor polling.
	 */
	private long correctionStart, correctionEnd;

	private boolean hasflagfound = false;
    
	/**
	 * Constructor of the class BlockDetection, which links the parameters to the class variables.
	 */
	public BlockDetection(Navigation navigation,
			              Odometer odometer,
	                      EV3LargeRegulatedMotor leftMotor,
	                      EV3LargeRegulatedMotor rightMotor,
	                      SampleProvider csSensor,
	                      float[] csData, LightLocalization lightLocalization){
		this.navigation = navigation;
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.csSensor = csSensor;
		this.csData = csData;
		this.lightLocalization = lightLocalization;
		determineColor(CaptureFlagMain.flagColor);
	}

	/**
	 * Main method of this class which will contain logic to go about finding the opponents flag. 
	 * It navigates robot to the flag zone and then instructs it to looks for blocks and identify enemy flag.
	 * This method also represents one of the states that controller will be in along the competition.
	 */
	//TODO: complete this method
	public void findFlag() {
		gotoSearch();
		//search();
	}

	/**
	 * Method that is used to find distance to all four corners of the search region
	 * @param x1 x-coordinate of desired corner
	 * @param y1 y-coordinate of desired corner
	 * @return distance to desired corner
	 */
	public static double distanceBetweenPoint(double x1, double y1, double x2, double y2){
		/*x1 = x1 * 30.48;
		x2 = x2 * 30.48;
		y1 = y1 * 30.48;
		y2 = y2 * 30.48;*/

		for(double point : new double[]{x1, x2, y1, y2}) {
			point *= Navigation.SIDE_SQUARE;
		}

		java.awt.geom.Point2D.Double currentPosition = new java.awt.geom.Point2D.Double(x1, y1);
		java.awt.geom.Point2D.Double desiredPosition = new java.awt.geom.Point2D.Double(x2, y2);
		double distance = currentPosition.distance(desiredPosition); // calculates distance between the two point
		return distance;
	}
	
	/**
	 * Determines color of detected block using intensityBuffer values. 
	 * This method will therefore help to determine if block found is indeed the flag of the opposing team.
	 * @return number representing color of detected block
	 */
	private void determineColor(int number){
		switch (number) {
			case 1:
				color = "Red";
				break;
			case 2:
				color = "Blue";
				break;
			case 3:
				color = "Yellow";
				break;
			case 4:
				color = "White";
				break;
		}
	}

	public boolean foundFlag() {
		correctionStart = System.currentTimeMillis();

		csSensor.fetchSample(csData, 0); //get data from the sensor
		double r = colorData[0] * 100;
		double g = colorData[1] * 100;
		double b = colorData[2] * 100;

		correctionEnd = System.currentTimeMillis();
		if (correctionEnd - correctionStart < LOOP_TIME) {
			try {
				Thread.sleep(LOOP_TIME - (correctionEnd - correctionStart));
			}
			catch (InterruptedException e) {}
		}

		switch (color) {
			case "Red":
				if (g > 0.1 && b > 0.1 && r >= g * 4) {
					hasflagfound = true;
				}
				break;
			case "Blue":
				if (g > 0.1 && b > 0.1 && r > 0.1 && b > 1.6 * r && g > 1.4 * r) {
					hasflagfound = true;
				}
				break;
			case "Yellow":
				if (r > g * 1.4 && r < g * 2 && g > 0.1 && b > 0.1 && g > 2 * b) {
					hasflagfound = true;
				}
				break;
			case "White":
				if (r < g * 1.4 && r > g * 1.1 && g > 0.1 && b > 0.1 && r > 0.1 && g < b * 1.3) {
					hasflagfound = true;
				}
				break;
		}
		return hasflagfound;
	}

	public void gotoSearch() {
		// initialize corners of search zone (do it inside method in order to ensure LL_search_x etc were initialsied by wifi for sure)
		cornerZero_x = CaptureFlagMain.LL_search_x;
		cornerZero_y = CaptureFlagMain.LL_search_y;
		cornerOne_x = CaptureFlagMain.UR_search_x;
		cornerOne_y = CaptureFlagMain.LL_search_y;
		cornerTwo_x = CaptureFlagMain.UR_search_x;
		cornerTwo_y = CaptureFlagMain.UR_search_y;
		cornerThree_x = CaptureFlagMain.LL_search_x;
		cornerThree_y = CaptureFlagMain.UR_search_y;

		corners = new double[][] { {cornerZero_x, cornerZero_y},
								   {cornerOne_x, cornerOne_y},
								   {cornerTwo_x, cornerTwo_y},
								   {cornerThree_x, cornerThree_y}
								 };

		// find corner closest to robot's current position and travel to it(after dismounting zipline)
		double shortestDistance = distanceBetweenPoint(CaptureFlagMain.ziplineOther_red_x, CaptureFlagMain.ziplineOther_red_y, cornerZero_x, cornerZero_y);
		if (distanceBetweenPoint(CaptureFlagMain.ziplineOther_red_x, CaptureFlagMain.ziplineOther_red_y, cornerOne_x, cornerOne_y) < shortestDistance) {
			shortestDistance = distanceBetweenPoint(CaptureFlagMain.ziplineOther_red_x, CaptureFlagMain.ziplineOther_red_y, cornerOne_x, cornerOne_y);
			closestCorner = 1;
		}
		if ((distanceBetweenPoint(CaptureFlagMain.ziplineOther_red_x, CaptureFlagMain.ziplineOther_red_y, cornerTwo_x, cornerTwo_y) < shortestDistance)) {
			shortestDistance = distanceBetweenPoint(CaptureFlagMain.ziplineOther_red_x, CaptureFlagMain.ziplineOther_red_y, cornerTwo_x, cornerTwo_y);
			closestCorner = 2;
		}
		if ((distanceBetweenPoint(CaptureFlagMain.ziplineOther_red_x, CaptureFlagMain.ziplineOther_red_y, cornerThree_x, cornerThree_y) < shortestDistance)) {
			//shortestDistance = distanceBetweenPoint(CaptureFlagMain.ziplineOther_red_x, CaptureFlagMain.ziplineOther_red_y, cornerThree_x, cornerThree_y);
			closestCorner = 3;
		}
		// below, travels to nearest block detection point.
		// if distance >= 4.5 tiles, localizes once during travel to search area.
		if (closestCorner == 0) {
			navigation.travelToUpdate(this.cornerZero_x, this.cornerZero_y);
		} else if (closestCorner == 1) {
			navigation.travelToUpdate(this.cornerOne_x, this.cornerOne_y);
		} else if (closestCorner == 2) {
			navigation.travelToUpdate(this.cornerTwo_x, this.cornerTwo_y);
		} else {
			navigation.travelToUpdate(this.cornerThree_x, this.cornerThree_y);
		}

		//localise to correct our angle and position
		lightLocalization.localizeOnTheMove = true;
		navigation.turnTo(Math.toRadians(45)); //turn to 45 to ensure we cross the correct lines during localization
		lightLocalization.doLocalization();
		lightLocalization.localizeOnTheMove = false;

		double[][] corners = {{cornerZero_x, cornerZero_y},{cornerOne_x, cornerOne_y},{cornerTwo_x, cornerTwo_y},{cornerThree_x, cornerThree_y}};
		
		Sound.setVolume(30);
		Sound.beep();
		Sound.beep();
		Sound.beep();

		//int currentcorner = closestCorner;
	}

	public void search() {
		double starttime = System.currentTimeMillis();
		double[] nextPoint = corners[(closestCorner + 1) % 4];

		if (nextPoint[0] == CaptureFlagMain.MAP_SIZE || nextPoint[0] == 0 || nextPoint[1] == CaptureFlagMain.MAP_SIZE || nextPoint[1] == 0) {
			closestCorner = (closestCorner - 1) % 4;
		//	navigation.travelToUpdate(corners[closestCorner][0], corners[closestCorner][1]);
		}

		nextPoint = corners[(closestCorner + 1) % 4];
		//from og point to other point
		double[] vector = {nextPoint[0] - corners[closestCorner][0], nextPoint[1] - corners[closestCorner][1]};

		double[] alignedPoint = corners[closestCorner];

		/*System.out.println(vector[0] + " " + vector[1]);

		if (vector[0] == 0) {
			navigation.turnToUpdate(navigation.turnToAngle(nextPoint[0], nextPoint[1]));
			navigation.advance((long) (vector[1] * Navigation.SIDE_SQUARE), false);
		} else if (vector[1] == 0) {
			System.out.println("turning to " + navigation.turnToAngle(nextPoint[0], nextPoint[1]));
			navigation.turnToUpdate(navigation.turnToAngle(nextPoint[0], nextPoint[1]));
			navigation.advance((long) (vector[0] * Navigation.SIDE_SQUARE), false);
		}*/

		while (!searchdone(closestCorner, (closestCorner + 1) % 4)) {

			if (vector[0] == 0) {
			navigation.turnToUpdate(navigation.turnToAngle(nextPoint[0], nextPoint[1]));
			navigation.advance((long) (vector[1] * Navigation.SIDE_SQUARE), false);
		} else if (vector[1] == 0) {
			System.out.println("turning to " + navigation.turnToAngle(nextPoint[0], nextPoint[1]));
			navigation.turnToUpdate(navigation.turnToAngle(nextPoint[0], nextPoint[1]));
			navigation.advance((long) (vector[0] * Navigation.SIDE_SQUARE), false);
		}
			
			while(leftMotor.isMoving() || rightMotor.isMoving()) {
				if(foundFlag()) {
					hasflagfound = true;
					leftMotor.stop();
					rightMotor.stop();
					navigation.travelToUpdate(nextPoint[0], nextPoint[1]);
					lightLocalization.localizeOnTheMove = true;
					navigation.turnTo(Math.toRadians(45)); //turn to 45 to ensure we cross the correct lines during localization
					lightLocalization.doLocalization();
					lightLocalization.localizeOnTheMove = false;
					Sound.setVolume(30);
					Sound.beep();
					Sound.beep();
					Sound.beep();
					return;
				}
				else if (System.currentTimeMillis() - starttime > (60 * 1000)) {
					Sound.setVolume(30);
					Sound.beep();
					Sound.beep();
					Sound.beep();
					return;
				}
			}

			if (vector[0] == 0) {
				if (vector[1] > 0) {
					alignedPoint[0] -= INCREMENT;
					nextPoint[0] -= INCREMENT;

				} else {
					alignedPoint[0] += INCREMENT;
					nextPoint[0] += INCREMENT;
				}
			} else if (vector[1] == 0) {
				if (vector[0] > 0) {
					alignedPoint[1] += INCREMENT;
					nextPoint[1] += INCREMENT;
				} else {
					alignedPoint[1] -= INCREMENT;
					nextPoint[1] -= INCREMENT;
				}
			}

			navigation.turnToUpdate(odometer.getTheta() + Math.PI * 0.5);
			navigation.advance((long) INCREMENT, false);
			navigation.turnToUpdate(odometer.getTheta() + Math.PI * 0.5);
			navigation.travelToUpdate(alignedPoint[0],alignedPoint[1]);
			navigation.turnToUpdate(odometer.getTheta() + Math.PI);

			navigation.travelToUpdate(nextPoint[0], nextPoint[1]);
			navigation.travelToUpdate(alignedPoint[0], alignedPoint[1]);
		}

	}

	public boolean searchdone(double currentcorner, double nextcorner) {

		if (corners[(int) currentcorner][0] == corners[(int) nextcorner][0]) {
			if(odometer.getX() / Navigation.SIDE_SQUARE + DIST_FAULT > corners[(int)(nextcorner + 1)%4][0] && odometer.getX() / Navigation.SIDE_SQUARE - DIST_FAULT < corners[(int) (nextcorner + 1) % 4][0]) {
				return true;
			}
		}
		else if (corners[(int) currentcorner][1] == corners[(int) nextcorner][1]) {
			if (odometer.getY() / Navigation.SIDE_SQUARE + DIST_FAULT > corners[(int) (nextcorner + 1) % 4][1] && odometer.getY() / Navigation.SIDE_SQUARE - DIST_FAULT < corners[(int) (nextcorner + 1) % 4][1]) {
				return true;
			}
		}

		return false;
	}
}
