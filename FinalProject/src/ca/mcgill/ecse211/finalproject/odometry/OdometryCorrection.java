package ca.mcgill.ecse211.finalproject.odometry;

import ca.mcgill.ecse211.finalproject.sensor.LightController;
import ca.mcgill.ecse211.finalproject.controller.LightLocalization;
import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * Corrects robot position upon line detection
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
	private LightLocalization lightLocalization;
	boolean firstXCorrection = true;
	boolean firstYCorrection = true;
	double currentXValue;
	double currentYValue;
	double squareSize = 30.48;
	
	// constructor
	public OdometryCorrection(Odometer odometer, LightLocalization lightLocalization) {
    this.odometer = odometer;
    this.lightLocalization = lightLocalization;
	}
	
	/**
	 * Run method required for thread.
	 */
	public void run() {
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
	  }

}
