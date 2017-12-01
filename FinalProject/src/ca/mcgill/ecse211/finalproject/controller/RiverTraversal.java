package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;

/**
 * Responsible for finding a path through shallow water and actual traversal of river using shallow
 * water crossing. This class mainly contains the doTrversal method which takes care of this river
 * traversal to go to the green side. The traversal implemented makes sure to cross the river in the
 * middle of the squares so as to not fall in the water.
 */
public class RiverTraversal {
  /** Odometer which is responsible for calculating robot's current position using odometry. */
  private Odometer odometer;
  /** Navigation which contains basic methods of moving our robot. */
  private Navigation navigation;
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
  /**
   * Variables which denote the X and Y of the entry point from the red zone to the shallow river to
   * cross it.
   */
  private double entryPoint_x, entryPoint_y;
  /**
   * Boolean which indicates the orientation of the part of the shallow river that touches the red
   * zone, this helps in getting to the river without traveling on the shoreline.
   */
  private boolean horizontalFirst = false;

  /** Constructor for the class RiverTraversal which links parameters to class variables. */
  public RiverTraversal(Navigation navigation, Odometer odometer) {
    this.navigation = navigation;
    this.odometer = odometer;
  }

  /**
   * Method responsible for seeing if point is within red zone boundaries. This is done by comparing
   * the X and Y of the point in question to the X and Y of the redzone.
   */
  private boolean isContainedInRed(int x, int y) {

    // 1st segment: (corner zero to corner one)
    if (y == cornerZero_y) {
      if (cornerZero_x <= x && x <= cornerOne_x) {
        return true;
      }
    }
    // 2nd segment: (corner one to corner two)
    else if (x == cornerOne_x) {
      if (cornerOne_y <= y && y <= cornerTwo_y) {
        return true;
      }
    }
    // 3rd segment: (corner two to corner three)
    else if (y == cornerTwo_y) {
      if (cornerThree_x <= x && x <= cornerTwo_x) {
        return true;
      }
    }
    // 4th segment: (corner three to corner zero)
    else if (x == cornerThree_x) {
      if (cornerZero_y <= y && y <= cornerThree_y) {
        return true;
      }
    }
    return false;
  }

  /**
   * Method which finds which point is farthest from the position
   *
   * @param position the current position of the robot
   * @param point1 a point
   * @param point2 a second point
   * @return the point which is the farthest from the position
   */
  private double farthestPoint(double position, double point1, double point2) {
    if (Math.abs(position - point1) > Math.abs(position - point2)) {
      return point1;
    } else {
      return point2;
    }
  }
  /**
   * Method responsible to determine entry point of river. This method goes through the X and Y of
   * the vertical and horizontal shallow rectangles. If a corner is along the border of the red
   * zone, then that is the starting shallow piece to cross.
   */
  private void entryPointRiver() {
    // initialize corners
    cornerZero_x = CaptureFlagMain.LL_redZone_x;
    cornerZero_y = CaptureFlagMain.LL_redZone_y;
    cornerOne_x = CaptureFlagMain.UR_redZone_x;
    cornerOne_y = CaptureFlagMain.LL_redZone_y;
    cornerTwo_x = CaptureFlagMain.UR_redZone_x;
    cornerTwo_y = CaptureFlagMain.UR_redZone_y;
    cornerThree_x = CaptureFlagMain.LL_redZone_x;
    cornerThree_y = CaptureFlagMain.UR_redZone_y;
    // start with vertical segments:
    if ((isContainedInRed(
        CaptureFlagMain.LL_verticalShallow_x, CaptureFlagMain.LL_verticalShallow_y))) {
      entryPoint_x =
          ((CaptureFlagMain.UR_verticalShallow_x + CaptureFlagMain.LL_verticalShallow_x) * 0.5);
      entryPoint_y = CaptureFlagMain.LL_verticalShallow_y;
    } else if ((isContainedInRed(
        CaptureFlagMain.UR_verticalShallow_x, CaptureFlagMain.UR_verticalShallow_y))) {
      entryPoint_x =
          ((CaptureFlagMain.UR_verticalShallow_x + CaptureFlagMain.LL_verticalShallow_x) * 0.5);
      entryPoint_y = CaptureFlagMain.UR_verticalShallow_y;
    }
    // now horizontal segments:
    else if ((isContainedInRed(
        CaptureFlagMain.LL_horizontalShallow_x, CaptureFlagMain.LL_horizontalShallow_y))) {
      entryPoint_x = CaptureFlagMain.LL_horizontalShallow_x;
      entryPoint_y =
          ((CaptureFlagMain.LL_horizontalShallow_y + CaptureFlagMain.UR_horizontalShallow_y) * 0.5);
      horizontalFirst = true;
    } else {
      entryPoint_x = CaptureFlagMain.UR_horizontalShallow_x;
      entryPoint_y =
          ((CaptureFlagMain.LL_horizontalShallow_y + CaptureFlagMain.UR_horizontalShallow_y) * 0.5);
      horizontalFirst = true;
    }
  }
  /**
   * Method responsible for performing river traversal by first entering through middle of shallow
   * water entry, <br>
   * traveling along this middle until one reaches midpoint of vertical segment's entry of shallow
   * water, <br>
   * then traveling to exit of vertical segment of shallow water.
   */
  public void doTraversal() {
    // find entry point
    entryPointRiver();

    if (horizontalFirst) {
      // travel to entry point of shallow water in square like fashion
      navigation.travelTo(odometer.getX() / 30.48, entryPoint_y);
      navigation.travelTo(entryPoint_x, entryPoint_y);
    } else {
      // travel to entry point of shallow water in square like fashion
      navigation.travelTo(entryPoint_x, odometer.getY() / 30.48);
      navigation.travelTo(entryPoint_x, entryPoint_y);
    }

    double[] bridgeMiddle = {
      ((CaptureFlagMain.LL_verticalShallow_x + CaptureFlagMain.UR_verticalShallow_x) * 0.5),
      ((CaptureFlagMain.LL_horizontalShallow_y + CaptureFlagMain.UR_horizontalShallow_y) * 0.5)
    };

    navigation.travelTo(bridgeMiddle[0], bridgeMiddle[1]);

    if (horizontalFirst) {
      navigation.travelTo(
          bridgeMiddle[0],
          farthestPoint(
              bridgeMiddle[1],
              CaptureFlagMain.UR_verticalShallow_y,
              CaptureFlagMain.LL_verticalShallow_y));
    } else {
      navigation.travelTo(
          farthestPoint(
              bridgeMiddle[0],
              CaptureFlagMain.UR_horizontalShallow_x,
              CaptureFlagMain.LL_horizontalShallow_y),
          bridgeMiddle[1]);
    }
    // end of traversal
  }
}
