package ca.mcgill.ecse211.finalproject.display;

import ca.mcgill.ecse211.finalproject.odometry.Odometer;
//import ca.mcgill.ecse211.lab5.UltrasonicController;
import lejos.hardware.lcd.TextLCD;

/**
 * Displays robot X, Y, and Theta positional parameters on LCD. This class was made a thread as we did not want it
 * to momentarly stop or have to call a method constantly in other classes.
 *
 */
public class OdometryDisplay extends Thread{
	
	/**
	 * Duration to display info before refreshing.
	 */
	  private static final long DISPLAY_PERIOD = 250;
	  /**
	   * Odometer whose position must be displayed.
	   */
	  private Odometer odometer;
	  /**
	   * LCD display.
	   */
	  private TextLCD t;
	  //private UltrasonicController cont;
	  
	  /**
		 * Constructor for the class OdometryDisplay which links parameters to class variables.
	     */
	  public OdometryDisplay(Odometer odometer, TextLCD t) {
	    this.odometer = odometer;
	    this.t = t;
	  }

	  /**
	   * Run method required for thread.
	   */
	  public void run() {
	    long displayStart, displayEnd;
	    double[] position = new double[3];

	    // clear the display once
	    t.clear();

	    while (true) {
	      displayStart = System.currentTimeMillis();

	      // clear the lines for displaying odometry information
	      t.drawString("X:              ", 0, 0);
	      t.drawString("Y:              ", 0, 1);
	      t.drawString("T:              ", 0, 2);

	      // get the odometry information
	      odometer.getPosition(position, new boolean[] {true, true, true});

	      // display odometry information
	      for (int i = 0; i < 3; i++) {
	        t.drawString(formattedDoubleToString(position[i], 2), 3, i);
	      }

	      // throttle the OdometryDisplay
	      displayEnd = System.currentTimeMillis();
	      if (displayEnd - displayStart < DISPLAY_PERIOD) {
	        try {
	          Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
	        } catch (InterruptedException e) {
	          // there is nothing to be done here because it is not
	          // expected that OdometryDisplay will be interrupted
	          // by another thread
	        }
	      }
	    }
	  }
	  
	  /**
	   * Converts numeric positions into string to display on LCD of robot.
	   * @param x parameter corresponding to either x, y, or theta position of robot.
	   * @param places index of parameter within array storing position of robot
	   * @return
	   */
	  private static String formattedDoubleToString(double x, int places) {
	    String result = "";
	    String stack = "";
	    long t;

	    // put in a minus sign as needed
	    if (x < 0.0)
	      result += "-";

	    // put in a leading 0
	    if (-1.0 < x && x < 1.0)
	      result += "0";
	    else {
	      t = (long) x;
	      if (t < 0)
	        t = -t;

	      while (t > 0) {
	        stack = Long.toString(t % 10) + stack;
	        t /= 10;
	      }

	      result += stack;
	    }

	    // put the decimal, if needed
	    if (places > 0) {
	      result += ".";

	      // put the appropriate number of decimals
	      for (int i = 0; i < places; i++) {
	        x = Math.abs(x);
	        x = x - Math.floor(x);
	        x *= 10.0;
	        result += Long.toString((long) x);
	      }
	    }

	    return result;
	  }
}
