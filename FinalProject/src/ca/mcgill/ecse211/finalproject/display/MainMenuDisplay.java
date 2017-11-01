package ca.mcgill.ecse211.finalproject.display;

import lejos.hardware.Button;
import lejos.hardware.lcd.TextLCD;

public class MainMenuDisplay {
	
	private final TextLCD t;
	
	public MainMenuDisplay(TextLCD t){
	    this.t = t;
	}
	

	public void displayCoord(int x, int y){ // method that allows display/input of premount/mount coordinates.
		
		// clear display
		t.clear();
		
		// displays default coordinates of (0,0)
        t.drawString("< X-0:    |  Y-0: >", 0, 0);
        t.drawString("          |        ", 0, 1);
        t.drawString("  " + x + "       |  " + y + "     ", 0, 2);
        t.drawString("          |        ", 0, 3);
	}
	
	public void displayCorner(int corner){
		
		// clear the display
        t.clear();

        // choose corner
        t.drawString("Choose corner", 0, 0);
        t.drawString(corner + " >", 0, 1);
	}
}
