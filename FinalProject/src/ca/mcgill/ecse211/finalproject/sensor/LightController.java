package ca.mcgill.ecse211.finalproject.sensor;

public interface LightController {
	
	  public void processLSData(float lsData);

	  public float readLSData();
}
