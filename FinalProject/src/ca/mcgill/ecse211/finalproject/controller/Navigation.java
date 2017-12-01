package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.geometry.Point2D;

/**
 * Class responsible for the moving of the robot throughout the competition. This means that it
 * contains all methods responsible for moving robot by desired distances, rotating by desired
 * angles, and traveling to desired points. The travelTo and turnTo go hand in hand as we need the
 * robot to face the right way to go to the right location. The convert methods convert distances
 * and angles with which the lejos motor methods can work with. Other important methods,
 * travelToPremount and returnToOrigin, are more situational methods that are used for a specific
 * part of the competition.
 */
public class Navigation {

  /** Constant which is the size of a tile of the game board in centimeters */
  public static final double SIDE_SQUARE = 30.48;
  /**
   * Constant which is the forward speed assigned to the motors throughout the run of the
   * competition
   */
  public static final int FORWARD_SPEED = 250;
  /**
   * Constant which is the rotating speed assigned to the motors throughout the run of the
   * competition
   */
  public static final int ROTATE_SPEED = 200;
  /**
   * Constant which is the number of block after which relocalization should be done to ensure
   * precision in our odometer
   */
  public static final int SECURE_BLOCK_LENGTH = 4;
  /** Variable which is the angle the robot must cover to aim to the intended destination */
  private double theta;
  /** EV3LargeRegulatedMotor which is the left motor of the robot */
  private EV3LargeRegulatedMotor leftMotor;
  /** EV3LargeRegulatedMotor which is the right motor of the robot */
  private EV3LargeRegulatedMotor rightMotor;
  /** Variable which is the odometer of the robot */
  private Odometer odometer;
  /** Sample provider of the color sensor used to fetch light sensor's readings. */
  private SampleProvider csValue;
  /** Array containing data obtained from light sensor. */
  private float[] csData;

  /** Constructor for the class Navigation which links parameters to class variables. */
  public Navigation(
      Odometer odometer,
      EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor,
      SampleProvider csValue,
      float[] csData) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.odometer = odometer;
    this.csValue = csValue;
    this.csData = csData;

    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.stop();
      motor.setAcceleration(3000);
    }
  }
  /**
   * Method responsible to make robot travel to a certain point. This method takes care of turning
   * towards the right angle to travel to the intended destination. The method even takes care of
   * taking the smallest angle needed.
   *
   * @param x x-coordinate to travel to.
   * @param y y-coordinate to travel to.
   */
  public void travelTo(double x, double y) {
    x = x * SIDE_SQUARE;
    y = y * SIDE_SQUARE;
    double currentX = odometer.getX();
    double currentY = odometer.getY();
    Point2D.Double currentPosition = new Point2D.Double(currentX, currentY);
    Point2D.Double desiredPosition = new Point2D.Double(x, y);
    double distanceToTravel =
        currentPosition.distance(desiredPosition); // calculates distance between the two points
    double differenceInTheta = turnToAngle(x, y);
    turnTo(differenceInTheta);

    // drive forward required distance
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed((int) (FORWARD_SPEED * CaptureFlagMain.balanceConstant));
    leftMotor.rotate(convertDistance(CaptureFlagMain.WHEEL_RADIUS, distanceToTravel), true);
    rightMotor.rotate(convertDistance(CaptureFlagMain.WHEEL_RADIUS, distanceToTravel), false);
  }
  /**
   * Method that converts distance to travel to required wheel rotations.
   *
   * @param radius wheel radius.
   * @param distance distance to travel.
   * @return wheel rotations in degrees.
   */
  public static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * Method that converts angle to rotate by to required wheel rotations.
   *
   * @param radius wheel radius.
   * @param width distance between the wheels.
   * @param angle angle to rotate by in degrees.
   * @return wheel rotations in degrees.
   */
  public static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  /**
   * Method responsible to rotate robot by minimal angle to face a certain orientation.
   *
   * @param differenceInTheta angle to rotate by in radians.
   */
  public void turnTo(double differenceInTheta) {
    // makes robot turn by the minimal angle (in radians)
    if ((differenceInTheta >= -(Math.PI)) && (differenceInTheta <= Math.PI)) {
      if (differenceInTheta < 0) {
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed((int) (ROTATE_SPEED * CaptureFlagMain.balanceConstant));
        leftMotor.rotate(
            -(convertAngle(
                CaptureFlagMain.WHEEL_RADIUS,
                CaptureFlagMain.TRACK,
                Math.toDegrees(-differenceInTheta))),
            true);
        rightMotor.rotate(
            convertAngle(
                CaptureFlagMain.WHEEL_RADIUS,
                CaptureFlagMain.TRACK,
                Math.toDegrees(-differenceInTheta)),
            false);
      } else {
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed((int) (ROTATE_SPEED * CaptureFlagMain.balanceConstant));
        rightMotor.rotate(
            -(convertAngle(
                CaptureFlagMain.WHEEL_RADIUS,
                CaptureFlagMain.TRACK,
                Math.toDegrees(differenceInTheta))),
            true);
        leftMotor.rotate(
            convertAngle(
                CaptureFlagMain.WHEEL_RADIUS,
                CaptureFlagMain.TRACK,
                Math.toDegrees(differenceInTheta)),
            false);
      }
    } else if (differenceInTheta < -(Math.PI)) {
      differenceInTheta = differenceInTheta + (2 * Math.PI);
      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed((int) (ROTATE_SPEED * CaptureFlagMain.balanceConstant));
      leftMotor.rotate(
          convertAngle(
              CaptureFlagMain.WHEEL_RADIUS,
              CaptureFlagMain.TRACK,
              Math.toDegrees(differenceInTheta)),
          true);
      rightMotor.rotate(
          -(convertAngle(
              CaptureFlagMain.WHEEL_RADIUS,
              CaptureFlagMain.TRACK,
              Math.toDegrees(differenceInTheta))),
          false);
    } else if (differenceInTheta > (Math.PI)) {
      differenceInTheta = (2 * Math.PI) - differenceInTheta;
      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed((int) (ROTATE_SPEED * CaptureFlagMain.balanceConstant));
      rightMotor.rotate(
          convertAngle(
              CaptureFlagMain.WHEEL_RADIUS,
              CaptureFlagMain.TRACK,
              Math.toDegrees(differenceInTheta)),
          true);
      leftMotor.rotate(
          -(convertAngle(
              CaptureFlagMain.WHEEL_RADIUS,
              CaptureFlagMain.TRACK,
              Math.toDegrees(differenceInTheta))),
          false);
    }
  }

  /**
   * Method that makes robot move by desired distance.
   *
   * @param distance distance to move by.
   * @param immediateReturn boolean that determines synchronization of motors.
   */
  public void advance(long distance, boolean immediateReturn) {
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed((int) (FORWARD_SPEED * CaptureFlagMain.balanceConstant));
    leftMotor.rotate(convertDistance(CaptureFlagMain.WHEEL_RADIUS, distance), true);
    rightMotor.rotate(convertDistance(CaptureFlagMain.WHEEL_RADIUS, distance), immediateReturn);
  }

  /**
   * Method responsible for making robot move to a premount point in right angles (imitates square
   * driver). <br>
   * It also handles whether to travel in x-direction or y-direction first to ensure no collision
   * with zipline.
   */
  public void travelToPremount() {
    // below, navigation to premount
    double premountpointX = CaptureFlagMain.ziplineOther_green_x;
    double premountpointY = CaptureFlagMain.ziplineOther_green_y;

    // takes care of avoiding home search zone
    if (CaptureFlagMain.LL_mysearch_x <= premountpointX
        && premountpointX <= CaptureFlagMain.UR_mysearch_x) {
      travelTo(odometer.getX() / SIDE_SQUARE, premountpointY);
      travelTo(premountpointX, premountpointY);
    } else {
      if (CaptureFlagMain.LL_mysearch_y <= odometer.getY()
          && odometer.getY() <= CaptureFlagMain.UR_mysearch_y) {
        travelTo(odometer.getX() / SIDE_SQUARE, premountpointY);
        travelTo(premountpointX, premountpointY);
      } else {
        travelTo(premountpointX, odometer.getY() / SIDE_SQUARE);
        travelTo(premountpointX, premountpointY);
      }
    }
  }

  /**
   * Method responsible to return robot to its starting corner after it has located enemy's flag.
   * This is the final piece in the puzzle to complete the challenge of capturing a flag while
   * crossing obstacles.
   */
  public void returnToOrigin() {

    if (CaptureFlagMain.startingCorner == 0) {
      travelTo(0.5, 0.5);
    } else if (CaptureFlagMain.startingCorner == 1) {
      travelTo(CaptureFlagMain.MAP_SIZE - 0.5, 0.5);

    } else if (CaptureFlagMain.startingCorner == 2) {
      travelTo(CaptureFlagMain.MAP_SIZE - 0.5, CaptureFlagMain.MAP_SIZE - 0.5);
    } else if (CaptureFlagMain.startingCorner == 3) {
      travelTo(0.5, CaptureFlagMain.MAP_SIZE - 0.5);
    }
  }

  /**
   * Calculates angle to turn to for when wanting to travel to a specific point
   *
   * @param x x-coordinate of desired point
   * @param y y-coordinate of desired point
   * @return angle to face to travel to desired point
   */
  public double turnToAngle(double x, double y) {
    double currentX = odometer.getX();
    double currentY = odometer.getY();

    // correct orientation
    double updatedTheta = Math.atan2(x - currentX, y - currentY);
    if (updatedTheta < 0) { // Make it follow the 0 - 2*pi convention (not -pi to +pi)
      updatedTheta = ((2.0 * Math.PI) + updatedTheta);
    }

    double differenceInTheta = (updatedTheta - odometer.getTheta());
    return differenceInTheta;
  }

  /**
   * Method which implements the travelling, but relocalizes every specified block length to ensure
   * that the odometer is correct throughout the competition. This method only works with X and Y
   * where either one is zero in relation to the current position of the robot. Thus, this method
   * only works when the robot is travelling along the grid lines as it needs the lines to localize.
   *
   * @param x the X position of the final destination
   * @param y the Y position of the final destination
   */
  public void travelToWLocalize(double x, double y) {
    double dx = x - Math.round(odometer.getX() / Navigation.SIDE_SQUARE);
    double dy = y - Math.round(odometer.getY() / Navigation.SIDE_SQUARE);

    LightLocalization lightLocalization =
        new LightLocalization(this, odometer, leftMotor, rightMotor, csValue, csData);

    if (dx > Navigation.SECURE_BLOCK_LENGTH || dy > Navigation.SECURE_BLOCK_LENGTH) {
      if (Math.abs(dx) > Math.abs(dy)) { // moves horizontally
        if (dx > 0) { // moves to the right
          for (int i = 0; i < Math.floor(dx / Navigation.SECURE_BLOCK_LENGTH); i++) {
            travelTo(
                Math.round(odometer.getX() / Navigation.SIDE_SQUARE)
                    + Navigation.SECURE_BLOCK_LENGTH,
                y);
            lightLocalization.localizeOnTheMove = true;
            turnTo(
                Math.toRadians(
                    45)); // turn to 45 to ensure we cross the correct lines during localization
            lightLocalization.doLocalization();
            lightLocalization.localizeOnTheMove = false;
          }
        } else { // moves to the left
          for (int i = 0; i < Math.floor(dx / Navigation.SECURE_BLOCK_LENGTH); i++) {
            travelTo(
                Math.round(odometer.getX() / Navigation.SIDE_SQUARE)
                    - Navigation.SECURE_BLOCK_LENGTH,
                y);
            lightLocalization.localizeOnTheMove = true;
            turnTo(
                Math.toRadians(
                    45)); // turn to 45 to ensure we cross the correct lines during localization
            lightLocalization.doLocalization();
            lightLocalization.localizeOnTheMove = false;
          }
        }
      } else { // moves vertically
        if (dy > 0) { // moves to the right
          for (int i = 0; i < Math.floor(dy / Navigation.SECURE_BLOCK_LENGTH); i++) {
            System.out.println(dy / Navigation.SECURE_BLOCK_LENGTH); // check going there
            travelTo(
                x,
                Math.round(odometer.getY() / Navigation.SIDE_SQUARE)
                    + Navigation.SECURE_BLOCK_LENGTH);
            lightLocalization.localizeOnTheMove = true;
            turnTo(
                Math.toRadians(
                    45)); // turn to 45 to ensure we cross the correct lines during localization
            lightLocalization.doLocalization();
            lightLocalization.localizeOnTheMove = false;
          }
        } else { // moves to the left
          for (int i = 0; i < Math.floor(dy / Navigation.SECURE_BLOCK_LENGTH); i++) {
            travelTo(
                x,
                Math.round(odometer.getY() / Navigation.SIDE_SQUARE)
                    - Navigation.SECURE_BLOCK_LENGTH);
            lightLocalization.localizeOnTheMove = true;
            turnTo(
                Math.toRadians(
                    45)); // turn to 45 to ensure we cross the correct lines during localization
            lightLocalization.doLocalization();
            lightLocalization.localizeOnTheMove = false;
          }
        }
      }
    }
    Sound.setVolume(30);
    Sound.beep();
    travelTo(x, y);
  }
}
