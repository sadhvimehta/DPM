package ca.mcgill.ecse211.finalproject.sensor;

/**
 * <h1>OdometryController</h1>
 *
 * <p style="text-indent: 30px">
 */
public interface UltrasonicController {

	  public void processUSData(float usData);

	  public float readUSData();
	}
