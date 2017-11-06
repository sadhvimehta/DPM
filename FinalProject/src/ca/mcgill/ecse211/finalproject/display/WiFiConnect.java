package ca.mcgill.ecse211.finalproject.display;

import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.finalproject.main.Main;
import lejos.hardware.Button;

/**
 * Uses WifiConnection to communicate with server and receive map data.
 */
public class WiFiConnect {

  /**
   * IP address of computer running server application.
   */
  private final String SERVER_IP;
  /**
   * Team number of participating team
   */
  private final int TEAM_NUMBER;

  /**
   * Enable/disable printing of debug info
   */
  private final boolean ENABLE_DEBUG_WIFI_PRINT;
  

  public WiFiConnect(String SERVER_IP, int TEAM_NUMBER, boolean ENABLE_DEBUG_WIFI_PRINT){
	  this.SERVER_IP = SERVER_IP;
	  this.TEAM_NUMBER = TEAM_NUMBER;
	  this.ENABLE_DEBUG_WIFI_PRINT = ENABLE_DEBUG_WIFI_PRINT;
  }

  /**
   * Starts connection with wifi server.
   */
  @SuppressWarnings("rawtypes")
  public void startWifiInitialization(){

    System.out.println("Running..");

    // Initialize WifiConnection class
    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

    // Connect to server and get the data, catching any errors that might occur
    try {
      /*
       * getData() will connect to the server and wait until the user/TA presses the "Start" button
       * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
       * pressing the upper left hand corner button (back/escape) on the EV3. getData() will throw
       * exceptions if it can't connect to the server (e.g. wrong IP address, server not running on
       * laptop, not connected to WiFi router, etc.). It will also throw an exception if it connects
       * but receives corrupted data or a message from the server saying something went wrong. For
       * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot
       * will receive a message saying an invalid team number was specified and getData() will throw
       * an exception letting you know.
       */
      Map data = conn.getData();
      
      initialiseParameters(data);

      // Example 1: Print out all received data
      System.out.println("Map:\n" + data);
      
      int myCorner = ((Long) data.get("myCorner")).intValue();
      System.out.println("my corner: " + myCorner);

      // Example 2 : Print out specific values
      int redTeam = ((Long) data.get("RedTeam")).intValue();
      System.out.println("Red Team: " + redTeam);

      int og = ((Long) data.get("OG")).intValue();
      System.out.println("Green opponent flag: " + og);

      // Example 3: Compare value
      int sh_ll_x =  ((Long) data.get("SH_LL_x")).intValue();
      if (sh_ll_x < 5) {
        System.out.println("Shallow water LL zone X < 5");
      }
      else {
        System.out.println("Shallow water LL zone X >= 5");
      }

    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }
  }
  
  /**
   * Initializes fields within {@link Main}, supplied by the Wifi server.
   * @param data (required) is the object containing all parameters of the competition map.
   */
  private void initialiseParameters(Map data){ // initialization of required map parameters
	  int redTeam = ((Long) data.get("RedTeam")).intValue();
	  if(redTeam == 15){
		  Main.teamColor = "Red";
		  Main.startingCorner = ((Long) data.get("RedCorner")).intValue();
		  Main.flagColor = ((Long) data.get("OG")).intValue();
		  Main.LL_x = ((Long) data.get("Red_LL_x")).intValue();
		  Main.LL_y = ((Long) data.get("Red_LL_y")).intValue();
		  Main.UR_x = ((Long) data.get("Red_UR_x")).intValue();
		  Main.UR_y = ((Long) data.get("Red_UR_y")).intValue();
		  Main.LL_search_x = ((Long) data.get("SG_LL_x")).intValue();
		  Main.LL_search_y = ((Long) data.get("SG_LL_y")).intValue();
		  Main.UR_search_x = ((Long) data.get("SG_UR_x")).intValue();
		  Main.UR_search_y = ((Long) data.get("SG_UR_y")).intValue();
	  }
	  else{
		  Main.teamColor = "Green";
		  Main.startingCorner = ((Long) data.get("GreenCorner")).intValue();
		  Main.flagColor = ((Long) data.get("OR")).intValue();
		  Main.LL_x = ((Long) data.get("Green_LL_x")).intValue();
		  Main.LL_y = ((Long) data.get("Green_LL_y")).intValue();
		  Main.UR_x = ((Long) data.get("Green_UR_x")).intValue();
		  Main.UR_y = ((Long) data.get("Green_UR_y")).intValue();
		  Main.LL_search_x = ((Long) data.get("SR_LL_x")).intValue();
		  Main.LL_search_y = ((Long) data.get("SR_LL_y")).intValue();
		  Main.UR_search_x = ((Long) data.get("SR_UR_x")).intValue();
		  Main.UR_search_y = ((Long) data.get("SR_UR_y")).intValue();
	  }
	  // below, initialise common points
	  Main.ziplineEndPoint_x = ((Long) data.get("ZC_R_x")).intValue();
	  Main.ziplineEndPoint_y = ((Long) data.get("ZC_R_y")).intValue();
	  Main.ziplineOther_x = ((Long) data.get("ZC_G_x")).intValue();
	  Main.ziplineOther_y = ((Long) data.get("ZC_G_y")).intValue();
	  Main.LL_horizontalShallow_x = ((Long) data.get("SH_LL_x")).intValue();
	  Main.LL_horizontalShallow_y = ((Long) data.get("SH_LL_y")).intValue();
	  Main.UR_horizontalShallow_x = ((Long) data.get("SH_UR_x")).intValue();
	  Main.UR_horizontalShallow_y = ((Long) data.get("SH_UR_y")).intValue();
	  Main.LL_verticalShallow_x = ((Long) data.get("SV_LL_x")).intValue();
	  Main.LL_verticalShallow_y = ((Long) data.get("SV_LL_y")).intValue();
	  Main.UR_verticalShallow_x = ((Long) data.get("SV_UR_x")).intValue();
	  Main.UR_verticalShallow_y = ((Long) data.get("SV_UR_y")).intValue();
	  	
  }
}
