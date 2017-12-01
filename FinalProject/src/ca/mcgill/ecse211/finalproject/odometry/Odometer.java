package ca.mcgill.ecse211.finalproject.odometry;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/** Calculates robot position using odometry */
public class Odometer extends Thread {

  /** X positional parameter */
  private double x;
  /** Y positional parameter */
  private double y;
  /** Theta positional parameter */
  private double theta;
  /** Current left motor tacho count used for odometry calculation */
  private int currentLeftMotorTachoCount;
  /** Current right motor tacho count used for odometry calculation */
  private int currentRightMotorTachoCount;
  /** Previous left motor tacho count used for odometry calculation */
  private int lastLeftMotorTachoCount;
  /** previous left motor tacho count used for odometry calculation */
  private int lastRightMotorTachoCount;
  /** Left motor from {@link CaptureFlagMain} */
  private EV3LargeRegulatedMotor leftMotor;
  /** Right motor from {@link CaptureFlagMain} */
  private EV3LargeRegulatedMotor rightMotor;
  /** Duration of period after which odometer must recalculate position */
  private static final long ODOMETER_PERIOD = 25; /*odometer update period, in ms*/

  /** Lock object for mutual exclusion */
  private Object lock;

  // default constructor
  /** Constructor of the class Odometer, which links the parameters to the class variables. */
  public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.x = 0.0;
    this.y = 0.0;
    this.theta = 0.0;
    this.leftMotor.resetTachoCount();
    this.rightMotor.resetTachoCount();
    this.lastLeftMotorTachoCount = this.leftMotor.getTachoCount();
    this.lastRightMotorTachoCount = this.rightMotor.getTachoCount();
    lock = new Object();
  }

  /** Run method required for thread */
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      synchronized (lock) {
        double distL, distR, deltaD, deltaT, dX, dY;

        currentLeftMotorTachoCount = leftMotor.getTachoCount(); // get tacho counts
        currentRightMotorTachoCount =
            (int) (rightMotor.getTachoCount() / CaptureFlagMain.balanceConstant);
        distL =
            Math.PI
                * CaptureFlagMain.WHEEL_RADIUS
                * (currentLeftMotorTachoCount - lastLeftMotorTachoCount)
                / 180; // compute L and R wheel displacements
        distR =
            Math.PI
                * CaptureFlagMain.WHEEL_RADIUS
                * (currentRightMotorTachoCount - lastRightMotorTachoCount)
                / 180;
        lastLeftMotorTachoCount =
            currentLeftMotorTachoCount; // save tacho counts for next iteration
        lastRightMotorTachoCount = currentRightMotorTachoCount;
        deltaD = 0.5 * (distL + distR); // compute vehicle displacement
        deltaT = (distL - distR) / CaptureFlagMain.TRACK; // compute change in heading
        theta += deltaT; // update heading
        if (theta
            >= (2 * Math.PI)) { // if theta crosses 360, return theta to zero and compute w.r.t zero
          // degrees
          theta = theta - (2 * Math.PI);
        } else if (theta < 0) {
          theta =
              (2 * Math.PI)
                  + theta; // if theta goes below zero, return theta to 360 and compute w.r.t 360
          // degrees
        }
        dX = deltaD * Math.sin(theta); // compute X component of displacement
        dY = deltaD * Math.cos(theta); // compute Y component of displacement
        x = x + dX; // update estimates of X and Y position
        y = y + dY;
      }

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that the odometer will be interrupted by
          // another thread
        }
      }
    }
  }

  public void getPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0]) position[0] = x;
      if (update[1]) position[1] = y;
      if (update[2]) position[2] = theta;
    }
  }

  public double getX() {
    double result;

    synchronized (lock) {
      result = x;
    }

    return result;
  }

  public double getY() {
    double result;

    synchronized (lock) {
      result = y;
    }

    return result;
  }

  public double getTheta() {
    double result;

    synchronized (lock) {
      result = theta;
    }

    return result;
  }

  // mutators
  public void setPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0]) x = position[0];
      if (update[1]) y = position[1];
      if (update[2]) theta = position[2];
    }
  }

  public void setX(double x) {
    synchronized (lock) {
      this.x = x;
    }
  }

  public void setY(double y) {
    synchronized (lock) {
      this.y = y;
    }
  }

  public void setTheta(double theta) {
    synchronized (lock) {
      this.theta = theta;
    }
  }

  /** @return the leftMotorTachoCount */
  public int getLeftMotorTachoCount() {
    return lastLeftMotorTachoCount;
  }

  /** @param leftMotorTachoCount the leftMotorTachoCount to set */
  public void setLeftMotorTachoCount(int leftMotorTachoCount) {
    synchronized (lock) {
      this.lastLeftMotorTachoCount = leftMotorTachoCount;
    }
  }

  /** @return the rightMotorTachoCount */
  public int getRightMotorTachoCount() {
    return lastRightMotorTachoCount;
  }

  /** @param rightMotorTachoCount the rightMotorTachoCount to set */
  public void setRightMotorTachoCount(int rightMotorTachoCount) {
    synchronized (lock) {
      this.lastRightMotorTachoCount = rightMotorTachoCount;
    }
  }
}
