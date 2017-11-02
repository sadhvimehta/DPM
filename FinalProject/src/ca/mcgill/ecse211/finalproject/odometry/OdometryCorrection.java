package ca.mcgill.ecse211.finalproject.odometry;

import ca.mcgill.ecse211.finalproject.sensor.LightController;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * Corrects robot position upon line detection
 *
 */
public class OdometryCorrection extends Thread implements LightController{
	
	/**
	 * Length of period before correction is performed
	 */
	private static final long CORRECTION_PERIOD = 10;
	/**
	 * Odometer
	 */
	private Odometer odometer;
	/**
	 * Color sensor used to detect lines.
	 */
	private EV3ColorSensor colorSensor;
	/**
	 * Buffer storing color sensor values.
	 */
	private float[] csData;
	/**
	 * Sample provider
	 */
	private SampleProvider csSensor;
	/**
	 * Boolean indicating line detection
	 */
	private boolean lineDetected = false;
	/**
	 * Buffer that holds robot's position x,y,theta respectively
	 */
	private double robotPos[] = {0,0,0};
	
	// constructor
	public OdometryCorrection(Odometer odometer, EV3ColorSensor colorSensor) {
    this.odometer = odometer;
    this.colorSensor = colorSensor;
    csSensor = colorSensor.getRGBMode();
    this.csData = new float [csSensor.sampleSize()];
	}
	
	/**
	 * Run method required for thread.
	 */
	public void run() {
	    long correctionStart, correctionEnd;

	    while (true) {
	      correctionStart = System.currentTimeMillis();

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

	@Override
	public void processLSData(float lsData) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public float readLSData() {
		csSensor.fetchSample(csData, 0); // acquire data
	    return csData[0]; // color intensity read by sensor
	}

}
