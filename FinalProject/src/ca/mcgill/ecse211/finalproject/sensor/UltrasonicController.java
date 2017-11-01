package ca.mcgill.ecse211.finalproject.sensor;

public interface UltrasonicController {

	  public void processUSData(float usData);

	  public float readUSData();
	}
