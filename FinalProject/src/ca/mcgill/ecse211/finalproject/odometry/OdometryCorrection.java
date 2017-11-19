package ca.mcgill.ecse211.finalproject.odometry;

import ca.mcgill.ecse211.finalproject.controller.Navigation;
import ca.mcgill.ecse211.finalproject.controller.TwoLightLocalization;
import ca.mcgill.ecse211.finalproject.sensor.LightController;
import ca.mcgill.ecse211.finalproject.controller.LightLocalization;
import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Corrects robot position upon line detection except during light localization.
 *
 */
public class OdometryCorrection extends Thread{
	
	/**
	 * Length of period before correction is performed
	 */
	private static final long CORRECTION_PERIOD = 10;
	/**
	 * Odometer
	 */
	private Odometer odometer;
	/**
	 * Buffer that holds robot's position x,y,theta respectively
	 */
	private double robotPos[] = {0,0,0};
	/**
	 * LightLocalization which provides method to check if a line has been detected.
	 */
	private LightLocalization lightLocalization;
	/**
	 * Boolean indicating whether it is the first correction in x-direction after light localization has been performed.
	 */
	boolean firstXCorrection = true;
	/**
	 * Boolean indicating whether it is the first correction in y-direction after light localization has been performed.
	 */
	boolean firstYCorrection = true;
	/**
	 * Stores current corrected x-position of robot once it encounters a line.
	 */
	double currentXValue;
	/**
	 * Stores current corrected y-position of robot once it encounters a line.
	 */
	double currentYValue;
	/**
	 * Length of tile.
	 */
	double squareSize = 30.48;


	private long correctionStart, correctionEnd, startTime;
	private boolean leftHasPastLine, rightHasPastLine;
	private float[] lastLeftLine, lastRightLine;
	private float radOff;
	private double[] XYOff;
	private TwoLightLocalization twoLightLocalization;
	private Navigation navigation = new Navigation(odometer,CaptureFlagMain.leftMotor, CaptureFlagMain.rightMotor);
	
	// constructor
	/**
	 * Constructor of the class Odometer, which links the parameters to the class variables.
	*/
	public OdometryCorrection(Odometer odometer, LightLocalization lightLocalization, TwoLightLocalization twoLightLocalization) {
	    this.odometer = odometer;
	    this.lightLocalization = lightLocalization;
	    this.twoLightLocalization = twoLightLocalization;
	}
	
	/**
	 * Run method required for thread.
	 */
	/*public void run() {
	    long correctionStart, correctionEnd;

	    while (true) {
	      correctionStart = System.currentTimeMillis();
	      if(CaptureFlagMain.doCorrection == false){
	    	  firstXCorrection = true;
	    	  firstYCorrection = true;
	      }
	      else{
	    	  if(lightLocalization.pastline()){ // ensures a line was detected
	    		  // checks if robot is moving in x direction
	    		  if(((80.00 <= Math.toDegrees(odometer.getTheta())) && (Math.toDegrees(odometer.getTheta()) <= 100.00)) 
	    				  || ((260.00 <= Math.toDegrees(odometer.getTheta())) && (Math.toDegrees(odometer.getTheta()) <= 280.00)) ){
	    			  if(firstXCorrection){
	    				  currentXValue = odometer.getX();
	    				  firstXCorrection = false;
	    			  }
	    			  else{
	    				  // case 1: robot is moving in positive x direction
	    				  if(80.00 <= Math.toDegrees(odometer.getTheta()) && Math.toDegrees(odometer.getTheta()) <= 100.00){
	    						  currentXValue = currentXValue + squareSize;
	    						  odometer.setX(currentXValue);
	    				  }
	    				  // case 2: robot is moving in negative x direction
	    				  else{
	    					  currentXValue = currentXValue - squareSize;
	    					  odometer.setX(currentXValue);
	    				  }
	    			  }
	    		  }
	    		  // checks if robot is moving in y direction
	    		  if(((10.00 <= Math.toDegrees(odometer.getTheta())) && (Math.toDegrees(odometer.getTheta()) >= 350.00)) 
	    				  || ((170.00 <= Math.toDegrees(odometer.getTheta())) && (Math.toDegrees(odometer.getTheta()) <= 190.00))){
	    			  if(firstYCorrection){
	    				  currentYValue = odometer.getY();
	    				  firstYCorrection = false;
	    			  }
	    			  else{
	    				  // case 3: robot is moving in positive y direction
	    				  if((10.00 <= Math.toDegrees(odometer.getTheta())) && (Math.toDegrees(odometer.getTheta()) >= 350.00)){
	    					  currentYValue = currentYValue + squareSize;
	    					  odometer.setY(currentYValue);
	    				  }
	    				  // case 4: robot is moving in negative y direction
	    				  else{
	    					  currentYValue = currentYValue - squareSize;
	    					  odometer.setY(currentYValue);
	    				  }
	    			  }
	    			  
	    		  }
	    		  
	    	  }
	      }
	      // this ensures the odometry correction occurs only once every period
	      correctionEnd = System.currentTimeMillis();
	      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
	        try {
	          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
	        } catch (InterruptedException e) {
	          // there is nothing to be done here because it is not
	          // expected that the odometry correction will be
	          // interrupted by another thread
	        }
	      }
	    }
	  }*/

	public void run() {
		float[] lastBrightness = {0, 0}; // this variable and the next one are used to apply a differentiation filter to the data
		float[] presentBrightness; // the differentiation filter allows the robot to still operate regardless of the light level
		float[] dbdt = {0, 0}; // the instantaneous differentiation of the brightness d/dt(brightness)
		boolean firstpassby = true; // since there is no past value for the first value, this sets helps set the last
		// value for the first time running

		startTime = System.currentTimeMillis();

		while (true) {
			correctionStart = System.currentTimeMillis();

			presentBrightness = twoLightLocalization.getFilteredData();


			if (firstpassby) {
				lastBrightness = presentBrightness; // simply sets the previous brightness to the current brightness if the
				// code has been run for the first time
				firstpassby = false;
			}

			for (int i = 0; i < 2; i++) {
				dbdt[i] = presentBrightness[i] - lastBrightness[i]; // get the current brightness and sets the derivative
			}

			lastBrightness = presentBrightness; // update the last brightness 

			// TODO Place correction implementation here

            /*try {
                BufferedWriter writertemp = new BufferedWriter(new FileWriter("log.txt", true));
                // writertemp.append(String.valueOf(correctionStart - startTime) + ", " +
                // String.valueOf(colorData[0] * 100) + "\n");
                writertemp.append(
                        String.valueOf(correctionStart - startTime) + ", " + String.valueOf(dbdt[1]) + "\n"); // writes to the log file the time and the brightness
                writertemp.close();
            } catch (IOException e) {
                System.console().printf("no error here!");
            }*/

			twoLightLocalization.lastNValueLAdd(dbdt[0]); // adds the derivative to a filter (explained lower down)
			twoLightLocalization.lastNValueRAdd(dbdt[1]);

            /*if (this.pastline()) { // if the filter has sensed that a line has been crossed:
                Sound.beep();
                if (odometer.getTheta() > Math.PI / 4 && odometer.getTheta() < 3 * Math.PI / 4) { // right    // and the robot is moving right
                    odometer.setX(x * 30.48);
                } else if (odometer.getTheta() > 7 * Math.PI / 4 || odometer.getTheta() < Math.PI / 4) { // up       // and the robot is moving up
                    odometer.setY(y * 30.48);
                } else if (odometer.getTheta() > 5 * Math.PI / 4 && odometer.getTheta() < 7 * Math.PI / 4) { // left     // and the robot is moving left
                    odometer.setX(x * 30.48);
                } else if (odometer.getTheta() > 3 * Math.PI / 4 && odometer.getTheta() < 5 * Math.PI / 4) { // down     // and the robot is moving down
                    odometer.setY(y * 30.48);
                }
            }*/

			leftHasPastLine = twoLightLocalization.leftPastline();
			rightHasPastLine = twoLightLocalization.rightPastline();

			if (leftHasPastLine && rightHasPastLine) {
				if (false/*isCloseXYAxis()*/) {
					Sound.setVolume(30);
					Sound.beep();

					radOff = (float) twoLightLocalization.calculateRadOff();
					XYOff = twoLightLocalization.calculateXYOff();

					RegulatedMotor[] synclist = {CaptureFlagMain.rightMotor};
					CaptureFlagMain.leftMotor.synchronizeWith(synclist);
					CaptureFlagMain.leftMotor.startSynchronization();
					CaptureFlagMain.leftMotor.stop(true);
					CaptureFlagMain.rightMotor.stop(true);
					CaptureFlagMain.leftMotor.endSynchronization();

					leftHasPastLine = false;
					rightHasPastLine = false;
					navigation.travelTo(0, 0);
				}
				else if (twoLightLocalization.isCloseXAxis()) {
					// update x
					odometer.setX(twoLightLocalization.closestX());
				}
				else if (twoLightLocalization.isCloseYAxis()) {
					//update y
					odometer.setY(twoLightLocalization.closestY());
				}
			}


			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}

}
