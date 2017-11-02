package ca.mcgill.ecse211.finalproject.sensor;

/**
 * <h1>LightController</h1>
 *
 * <p style="text-indent: 30px">
 */
public interface LightController {
	
	  public void processLSData(float lsData);

	  public float readLSData();
}
