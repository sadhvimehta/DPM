package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class LightLocalization extends Thread {

  private Navigation navigation;
  private Odometer odo;
  private SampleProvider csSensor;
  private float[] csData;
  private double lightValueCurrent, lightValuePrev;
  private double thetaX;
  private double thetaY;
  private double positionX;
  private double positionY;
  private double dT;

  private boolean atApproxOrigin = false;

  private int lineCounter;
  private double[] saveLineAngles;

  private final float LIGHT_DIFF_THRESHOLD = 370;
  private static final int WAIT_PERIOD = 500; // in milliseconds
  private final double SENSOR_TO_WHEEL = 14.0; // distance between the wheels and the sensor
  private final double LINE_OFFSET = SENSOR_TO_WHEEL;

  private EV3LargeRegulatedMotor leftMotor, rightMotor;

  public LightLocalization(
      Navigation navigation,
      Odometer odo,
      EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor,
      SampleProvider csSensor,
      float[] csData) {
    this.odo = odo;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.navigation = navigation;
    this.csSensor = csSensor;
    this.csData = csData;
  }

  public void run() {
    doLocalization();
  }

  public void doLocalization() {
    // 1st, get the robot close to where the origin is

    goToEstimateOrigin();

    // 2nd, turn around the origin and detect the lines
    checkLines();

    calculatePosition();
  }

  // Polls the color sensor
  private float getData() {
    csSensor.fetchSample(csData, 0);
    float color = csData[0] * 1000;

    return color;
  }

  private void goToEstimateOrigin() {

    // turn 45 degrees to face origin (0,0)
    this.leftMotor.setSpeed(Navigation.ROTATE_SPEED);
    this.rightMotor.setSpeed(Navigation.ROTATE_SPEED);
    this.leftMotor.rotate(
        Navigation.convertAngle(LabFiveMain.WHEEL_RADIUS, LabFiveMain.TRACK, 45), true);
    this.rightMotor.rotate(
        -Navigation.convertAngle(LabFiveMain.WHEEL_RADIUS, LabFiveMain.TRACK, 45), false);

    // then go straight
    this.leftMotor.setSpeed(Navigation.FORWARD_SPEED);
    this.rightMotor.setSpeed(Navigation.FORWARD_SPEED);
    this.leftMotor.synchronizeWith(new RegulatedMotor[] {rightMotor});
    this.leftMotor.forward();
    this.rightMotor.forward();
    leftMotor.endSynchronization();

    lightValuePrev = getData(); // save previous value, aka bord colour

    while (!atApproxOrigin) { // boolean to check if we have arrived or not
      lightValueCurrent = getData(); // update data

      // If the difference in colour intensity is bigger than a chosen threshold, a line was
      // detected
      if (lightValueCurrent <= LIGHT_DIFF_THRESHOLD) { // if we detect a line
        // Sound.beep();
        leftMotor.stop(true);
        rightMotor.stop(true);
        atApproxOrigin = true;
        Sound.beep();
      }
      lightValuePrev = lightValueCurrent;
    }
    // go backwards so the front wheels are at the origin and not the sensor
    this.leftMotor.setSpeed(Navigation.FORWARD_SPEED);
    this.rightMotor.setSpeed(Navigation.FORWARD_SPEED);

    this.leftMotor.rotate(-Navigation.convertDistance(LabFiveMain.WHEEL_RADIUS, LINE_OFFSET), true);
    this.rightMotor.rotate(
        -Navigation.convertDistance(LabFiveMain.WHEEL_RADIUS, LINE_OFFSET), false);
  }

  private void checkLines() {
    // it turns anti clockwise, so 1st line it sees in neg y, then pos x, then pos y, then neg x

    // Set up variables
    lineCounter = 0;
    saveLineAngles = new double[4];

    this.leftMotor.setSpeed(Navigation.ROTATE_SPEED);
    this.rightMotor.setSpeed(Navigation.ROTATE_SPEED);
    this.leftMotor.rotate(
        Navigation.convertAngle(LabFiveMain.WHEEL_RADIUS, LabFiveMain.TRACK, 360), true);
    this.rightMotor.rotate(
        -Navigation.convertAngle(LabFiveMain.WHEEL_RADIUS, LabFiveMain.TRACK, 360), true);

    // Runs until it has detected 4 lines
    while (lineCounter < 4) {
      lightValueCurrent = getData();
      System.out.println(odo.getTheta());
      // If the difference in colour intensity is bigger than a chosen threshold, a line was
      // detected
      if (lightValueCurrent <= LIGHT_DIFF_THRESHOLD) {
        // Store angles in variable for future calculations
        saveLineAngles[lineCounter] = this.odo.getTheta();

        Sound.beep();

        lineCounter++;

        // Makes the thread sleep as to not detect the same line twice
        sleepThread();
      }
    }

    this.leftMotor.stop(true);
    this.rightMotor.stop(false);
  }

  private void calculatePosition() {
    // Trigonometry calculations from tutorial
    thetaY = saveLineAngles[3] - saveLineAngles[1]; // Y+ - Y-
    thetaX = saveLineAngles[2] - saveLineAngles[0]; // X+ - X-

    positionX = -SENSOR_TO_WHEEL * Math.cos((thetaY) / 2);
    positionY = -SENSOR_TO_WHEEL * Math.cos((thetaX) / 2);

    dT = Math.toRadians(270.00) + (thetaY / 2) - saveLineAngles[3]; // y-

    // Updates odometer to actual values
    this.odo.setX(positionX);
    this.odo.setY(positionY);
    this.odo.setTheta(this.odo.getTheta() + dT);
  }

  public static void sleepThread() {
    try {
      Thread.sleep(WAIT_PERIOD);
    } catch (InterruptedException e) {
    }
  }
}
