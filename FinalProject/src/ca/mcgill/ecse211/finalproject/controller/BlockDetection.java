package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Contains all methods necessary to detect the enemy's flag based on light intensity readings. <br>
 * It is is instantiated within the controller and its respective methods are called upon by the
 * controller as well. The algorithm used to detect blocks and find the right one is the edge
 * follower. The implemented algorithm makes the robot travel from corner to corner in an
 * anticlockwise manner. The anticlockwise is important as it always points the color sensor towards
 * the blocks. By having this method of only following the border, we maximize our chances of
 * detecting the flag without spending too much time searching.
 */
public class BlockDetection {
  /** Period for which sensor reading must be taken. */
  private static final long LOOP_TIME = 5;
  /** Navigation which contains basic methods of moving our robot. */
  private Navigation navigation;
  /** Odometer which calculates robot's position using odometry. */
  private Odometer odometer;
  /** Left motor of the robot. */
  private EV3LargeRegulatedMotor leftMotor;
  /** Right motor of the robot. */
  private EV3LargeRegulatedMotor rightMotor;
  /** Sample provider of the color sensor used to fetch light sensor's readings. */
  private SampleProvider csSensor;
  /** Array containing data obtained from light sensor. */
  private float[] csData;
  /** Lightlocalization which is needed to update position throughout run */
  private LightLocalization lightLocalization;
  /**
   * The corner zero is lower left corner of search region. This variable corresponds to the X
   * value.
   */
  private double cornerZero_x;
  /**
   * The corner zero is lower left corner of search region. This variable corresponds to the Y
   * value.
   */
  private double cornerZero_y;
  /**
   * The corner one is lower right corner of search region. This variable corresponds to the X
   * value.
   */
  private double cornerOne_x;
  /**
   * The corner one is lower right corner of search region. This variable corresponds to the Y
   * value.
   */
  private double cornerOne_y;
  /**
   * The corner two is upper right corner of search region. This variable corresponds to the X
   * value.
   */
  private double cornerTwo_x;
  /**
   * The corner two is upper right corner of search region. This variable corresponds to the Y
   * value.
   */
  private double cornerTwo_y;
  /**
   * The corner three is upper left corner of search region. This variable corresponds to the X
   * value.
   */
  private double cornerThree_x;
  /**
   * The corner three is upper left corner of search region. This variable corresponds to the Y
   * value.
   */
  private double cornerThree_y;
  /** This is a 2D array for the X and Y of the corners */
  private double[][] corners = new double[4][2];
  /**
   * Array of length three for the color data returned from the light sensor as there are three
   * channels: RGB
   */
  private double[] colorData = new double[3];
  /** Variable to keep track of the closest corner to the current position on the map */
  private int closestCorner = 0;
  /** Variable to determine which is the color of the opponent's flag */
  private String color;
  /** Variables that hold the values of the start and end of sensor polling. */
  private long correctionStart, correctionEnd;
  /** Variable which indicates if the flag has been found yet */
  private boolean hasflagfound = false;

  /** Constructor of the class BlockDetection, which links the parameters to the class variables. */
  public BlockDetection(
      Navigation navigation,
      Odometer odometer,
      EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor,
      SampleProvider csSensor,
      float[] csData,
      LightLocalization lightLocalization) {
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
   * Main method of this class which will contain logic to go about finding the opponents flag. It
   * navigates robot to the flag zone and then instructs it to looks for blocks and identify enemy
   * flag. This method also represents one of the states that controller will be in along the
   * competition.
   */
  public void findFlag() {
    gotoSearch();
    search();
  }

  /**
   * Method that is used to find distance to all four corners of the search region
   *
   * @param x1 x-coordinate of desired corner
   * @param y1 y-coordinate of desired corner
   * @return distance to desired corner
   */
  public static double distanceBetweenPoint(double x1, double y1, double x2, double y2) {
    for (double point : new double[] {x1, x2, y1, y2}) {
      point *= Navigation.SIDE_SQUARE;
    }

    java.awt.geom.Point2D.Double currentPosition = new java.awt.geom.Point2D.Double(x1, y1);
    java.awt.geom.Point2D.Double desiredPosition = new java.awt.geom.Point2D.Double(x2, y2);
    double distance =
        currentPosition.distance(desiredPosition); // calculates distance between the two point
    return distance;
  }

  /**
   * Determines color of detected block using intensityBuffer values. This method will therefore
   * help to determine if block found is indeed the flag of the opposing team.
   *
   * @return number representing color of detected block
   */
  private void determineColor(int number) {
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

  /**
   * This method polls the color sensor on the side of the robot, with a maximum frequency, to
   * determine if the block to the left of the robot is indeed the flag that we must find
   *
   * @return hasflagfound which tells if the flag has been found yet
   */
  public boolean foundFlag() {
    correctionStart = System.currentTimeMillis();

    csSensor.fetchSample(csData, 0); // get data from the sensor
    double r = colorData[0] * 100;
    double g = colorData[1] * 100;
    double b = colorData[2] * 100;

    // check to cap the frequency of the polling for color
    correctionEnd = System.currentTimeMillis();
    if (correctionEnd - correctionStart < LOOP_TIME) {
      try {
        Thread.sleep(LOOP_TIME - (correctionEnd - correctionStart));
      } catch (InterruptedException e) {
      }
    }

    // cases for the color that is being looked for with the RGB values which correspond to the
    // specific color,
    // which have been found through testing
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

  /**
   * This method brings the robot to the closest corner of the search zone relative to its current
   * position as to not waste time. The method also localizes at that corner to ensure accuracy with
   * our odometer.
   */
  public void gotoSearch() {
    // initialize corners of search zone (do it inside method in order to ensure LL_search_x etc
    // were initialsied by wifi for sure)
    cornerZero_x = CaptureFlagMain.LL_search_x;
    cornerZero_y = CaptureFlagMain.LL_search_y;
    cornerOne_x = CaptureFlagMain.UR_search_x;
    cornerOne_y = CaptureFlagMain.LL_search_y;
    cornerTwo_x = CaptureFlagMain.UR_search_x;
    cornerTwo_y = CaptureFlagMain.UR_search_y;
    cornerThree_x = CaptureFlagMain.LL_search_x;
    cornerThree_y = CaptureFlagMain.UR_search_y;

    corners =
        new double[][] {
          {cornerZero_x, cornerZero_y},
          {cornerOne_x, cornerOne_y},
          {cornerTwo_x, cornerTwo_y},
          {cornerThree_x, cornerThree_y}
        };

    // find corner closest to robot's current position and travel to it(after dismounting zipline)
    double shortestDistance =
        distanceBetweenPoint(
            CaptureFlagMain.ziplineOther_red_x,
            CaptureFlagMain.ziplineOther_red_y,
            cornerZero_x,
            cornerZero_y);
    if (distanceBetweenPoint(
            CaptureFlagMain.ziplineOther_red_x,
            CaptureFlagMain.ziplineOther_red_y,
            cornerOne_x,
            cornerOne_y)
        < shortestDistance) {
      shortestDistance =
          distanceBetweenPoint(
              CaptureFlagMain.ziplineOther_red_x,
              CaptureFlagMain.ziplineOther_red_y,
              cornerOne_x,
              cornerOne_y);
      closestCorner = 1;
    }
    if ((distanceBetweenPoint(
            CaptureFlagMain.ziplineOther_red_x,
            CaptureFlagMain.ziplineOther_red_y,
            cornerTwo_x,
            cornerTwo_y)
        < shortestDistance)) {
      shortestDistance =
          distanceBetweenPoint(
              CaptureFlagMain.ziplineOther_red_x,
              CaptureFlagMain.ziplineOther_red_y,
              cornerTwo_x,
              cornerTwo_y);
      closestCorner = 2;
    }
    if ((distanceBetweenPoint(
            CaptureFlagMain.ziplineOther_red_x,
            CaptureFlagMain.ziplineOther_red_y,
            cornerThree_x,
            cornerThree_y)
        < shortestDistance)) {
      closestCorner = 3;
    }
    // below, travels to nearest block detection point.
    if (closestCorner == 0) {
      navigation.travelTo(this.cornerZero_x, this.cornerZero_y);
    } else if (closestCorner == 1) {
      navigation.travelTo(this.cornerOne_x, this.cornerOne_y);
    } else if (closestCorner == 2) {
      navigation.travelTo(this.cornerTwo_x, this.cornerTwo_y);
    } else {
      navigation.travelTo(this.cornerThree_x, this.cornerThree_y);
    }

    // localise to correct our angle and position
    lightLocalization.localizeOnTheMove = true;
    // turn to 45 to ensure we cross the correct lines during localization
    navigation.turnTo(Math.toRadians(45));
    lightLocalization.doLocalization();
    lightLocalization.localizeOnTheMove = false;
  }

  /**
   * This is the method that actually implements the logic of the flag searching amongst the blocks
   * in the search zone. To accomplish this task, the robot drives from corner to corner while
   * looking if the block if the flag. This method is quite easy however, it covers quite a few
   * cases while only failing in a few cases. The whole search algorithm also has a timeout in case
   * the search takes too long.
   */
  public void search() {
    // inital time when the searching has started
    double starttime = System.currentTimeMillis();
    // calculating of the starting corner and the next corner
    double[] startPoint = corners[closestCorner];
    double[] nextPoint = corners[(closestCorner + 1) % 4];

    // vector denoting distance from current point to next point
    double[] vector = {
      nextPoint[0] - corners[closestCorner][0], nextPoint[1] - corners[closestCorner][1]
    };

    // a simple check to stop if the next point runs into the wall or if the next point was the
    // starting point
    while (nextPoint[0] == CaptureFlagMain.MAP_SIZE
        || nextPoint[0] == 0
        || nextPoint[1] == CaptureFlagMain.MAP_SIZE
        || nextPoint[1] == 0
        || nextPoint == startPoint) {

      // a travel method that does not block allowing for searching while the motors turn
      if (vector[0] == 0) {
        navigation.turnTo(navigation.turnToAngle(nextPoint[0], nextPoint[1]));
        navigation.advance((long) (vector[1] * Navigation.SIDE_SQUARE), false);
      } else if (vector[1] == 0) {
        System.out.println("turning to " + navigation.turnToAngle(nextPoint[0], nextPoint[1]));
        navigation.turnTo(navigation.turnToAngle(nextPoint[0], nextPoint[1]));
        navigation.advance((long) (vector[0] * Navigation.SIDE_SQUARE), false);
      }

      // make sure to keep checking while the motor are still in movement
      while (leftMotor.isMoving() || rightMotor.isMoving()) {
        // when the flag is found, we stop, beep, and localize
        if (foundFlag()) {
          hasflagfound = true;
          leftMotor.stop();
          rightMotor.stop();
          Sound.setVolume(30);
          Sound.beep();
          Sound.beep();
          Sound.beep();
          lightLocalization.localizeOnTheMove = true;
          navigation.turnTo(
              Math.toRadians(
                  45)); // turn to 45 to ensure we cross the correct lines during localization
          lightLocalization.doLocalization();
          lightLocalization.localizeOnTheMove = false;
          return;
        }
        // if the max time has been reached, we stop to keep going with the rest of the course
        else if (System.currentTimeMillis() - starttime > (60 * 1000)) {
          Sound.setVolume(30);
          Sound.beep();
          Sound.beep();
          Sound.beep();
          return;
        }
      }

      // calculating the next point to travel to and calculating the vector
      nextPoint = corners[(closestCorner + 1) % 4];

      vector[0] = nextPoint[0] - corners[closestCorner][0];
      vector[1] = nextPoint[1] - corners[closestCorner][1];
    }
  }
}
