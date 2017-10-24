/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class OdometryCorrection extends Thread {
  private static final long CORRECTION_PERIOD = 10;
  private static final double DISTANCE_WHEEL_SENSOR = 14.0;
  private Odometer odometer;
  private float lightValueCurrent,lightValuePrev;
  private SampleProvider samples;
  private double error = 18.0;
  private static Port csPort = LocalEV3.get().getPort("S2");
  private SensorModes csSensor;
  private float[] data;
  private int counterX;
  private int counterY;
  private double DeltaY2;
  private double DeltaY1;
  private static final double tileSize = 30.48;
  

  // constructor
  public OdometryCorrection(Odometer odometer) {
	  this.odometer = odometer;
	  this.csSensor= new EV3ColorSensor(csPort);
	  this.samples = csSensor.getMode("Red"); //red light sensor because we need to measure the intensity of the reflected red light (black vs light wood)
	  this.data = new float[csSensor.sampleSize()];
	  
	  counterX = 0;
	  counterY= 0;
  }

  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    samples.fetchSample(data, 0); //get data from the sensor
	lightValuePrev = data[0]; //save previous value

    while (true) {
      correctionStart = System.currentTimeMillis();

      samples.fetchSample(data,0); //get data everytime
		lightValueCurrent = data[0]; //save new value
		
			if (Math.abs(lightValueCurrent-lightValuePrev) >= 0.08){
				Sound.beep();//if detects a black line
				
				double theta = odometer.getTheta();
				if(theta <= Math.PI/4 || theta >= Math.PI*7/4){
					odometer.setY(counterY*tileSize + DISTANCE_WHEEL_SENSOR);
					counterY++;
				}
				else if (theta>= Math.PI/4 && theta <= Math.PI*3/4){
					odometer.setX(counterX*tileSize + DISTANCE_WHEEL_SENSOR);
					counterX++;
				}
				else if (theta>=Math.PI*3/4 && theta <= Math.PI*5/4) {
					counterY--;
					odometer.setY(counterY*tileSize - DISTANCE_WHEEL_SENSOR);
				}
				else if (theta>=Math.PI*5/4 && theta <= Math.PI*7/4) {
					counterX--;
					odometer.setX(counterX*tileSize - DISTANCE_WHEEL_SENSOR);					
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
