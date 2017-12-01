package ca.mcgill.ecse211.finalproject.display;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;

import java.util.Map;

/**
 * Uses WifiConnection to communicate with server and receive map data. This class allows the robot
 * to get an idea of the map that the competition will take place in.
 */
public class WiFiConnect {

  /** IP address of computer running server application. */
  private final String SERVER_IP;
  /** Team number of participating team */
  private final int TEAM_NUMBER;

  /** Enable/disable printing of debug info */
  private final boolean ENABLE_DEBUG_WIFI_PRINT;

  /** Constructor for the class WifiConnect which links parameters to class variables. */
  public WiFiConnect(String SERVER_IP, int TEAM_NUMBER, boolean ENABLE_DEBUG_WIFI_PRINT) {
    this.SERVER_IP = SERVER_IP;
    this.TEAM_NUMBER = TEAM_NUMBER;
    this.ENABLE_DEBUG_WIFI_PRINT = ENABLE_DEBUG_WIFI_PRINT;
  }

  /** Starts connection with wifi server. */
  @SuppressWarnings("rawtypes")
  public void startWifiInitialization() {

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

    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }
  }

  /**
   * Initializes fields within {@link CaptureFlagMain}, supplied by the Wifi server.
   *
   * @param data (required) is the object containing all parameters of the competition map.
   */
  private void initialiseParameters(Map data) { // initialization of required map parameters
    int redTeam = ((Long) data.get("RedTeam")).intValue();
    if (redTeam == 15) {
      CaptureFlagMain.teamColor = "Red";
      CaptureFlagMain.startingCorner = ((Long) data.get("RedCorner")).intValue();
      CaptureFlagMain.flagColor = ((Long) data.get("OG")).intValue();
      CaptureFlagMain.LL_x = ((Long) data.get("Red_LL_x")).intValue();
      CaptureFlagMain.LL_y = ((Long) data.get("Red_LL_y")).intValue();
      CaptureFlagMain.UR_x = ((Long) data.get("Red_UR_x")).intValue();
      CaptureFlagMain.UR_y = ((Long) data.get("Red_UR_y")).intValue();
      CaptureFlagMain.LL_search_x = ((Long) data.get("SG_LL_x")).intValue();
      CaptureFlagMain.LL_search_y = ((Long) data.get("SG_LL_y")).intValue();
      CaptureFlagMain.UR_search_x = ((Long) data.get("SG_UR_x")).intValue();
      CaptureFlagMain.UR_search_y = ((Long) data.get("SG_UR_y")).intValue();
      CaptureFlagMain.LL_mysearch_x = ((Long) data.get("SR_LL_x")).intValue();
      CaptureFlagMain.LL_mysearch_y = ((Long) data.get("SR_LL_y")).intValue();
      CaptureFlagMain.UR_mysearch_x = ((Long) data.get("SR_UR_x")).intValue();
      CaptureFlagMain.UR_mysearch_y = ((Long) data.get("SR_UR_y")).intValue();
    } else {
      CaptureFlagMain.teamColor = "Green";
      CaptureFlagMain.startingCorner = ((Long) data.get("GreenCorner")).intValue();
      CaptureFlagMain.flagColor = ((Long) data.get("OR")).intValue();
      CaptureFlagMain.LL_x = ((Long) data.get("Green_LL_x")).intValue();
      CaptureFlagMain.LL_y = ((Long) data.get("Green_LL_y")).intValue();
      CaptureFlagMain.UR_x = ((Long) data.get("Green_UR_x")).intValue();
      CaptureFlagMain.UR_y = ((Long) data.get("Green_UR_y")).intValue();
      CaptureFlagMain.LL_search_x = ((Long) data.get("SR_LL_x")).intValue();
      CaptureFlagMain.LL_search_y = ((Long) data.get("SR_LL_y")).intValue();
      CaptureFlagMain.UR_search_x = ((Long) data.get("SR_UR_x")).intValue();
      CaptureFlagMain.UR_search_y = ((Long) data.get("SR_UR_y")).intValue();
      CaptureFlagMain.LL_mysearch_x = ((Long) data.get("SG_LL_x")).intValue();
      CaptureFlagMain.LL_mysearch_y = ((Long) data.get("SG_LL_y")).intValue();
      CaptureFlagMain.UR_mysearch_x = ((Long) data.get("SG_UR_x")).intValue();
      CaptureFlagMain.UR_mysearch_y = ((Long) data.get("SG_UR_y")).intValue();
    }
    // below, initialise common points
    CaptureFlagMain.ziplineEndPoint_red_x = ((Long) data.get("ZC_R_x")).intValue();
    CaptureFlagMain.ziplineEndPoint_red_y = ((Long) data.get("ZC_R_y")).intValue();
    CaptureFlagMain.ziplineOther_red_x = ((Long) data.get("ZO_R_x")).intValue();
    CaptureFlagMain.ziplineOther_red_y = ((Long) data.get("ZO_R_y")).intValue();
    CaptureFlagMain.ziplineEndPoint_green_x = ((Long) data.get("ZC_G_x")).intValue();
    CaptureFlagMain.ziplineEndPoint_green_y = ((Long) data.get("ZC_G_y")).intValue();
    CaptureFlagMain.ziplineOther_green_x = ((Long) data.get("ZO_G_x")).intValue();
    CaptureFlagMain.ziplineOther_green_y = ((Long) data.get("ZO_G_y")).intValue();
    CaptureFlagMain.LL_horizontalShallow_x = ((Long) data.get("SH_LL_x")).intValue();
    CaptureFlagMain.LL_horizontalShallow_y = ((Long) data.get("SH_LL_y")).intValue();
    CaptureFlagMain.UR_horizontalShallow_x = ((Long) data.get("SH_UR_x")).intValue();
    CaptureFlagMain.UR_horizontalShallow_y = ((Long) data.get("SH_UR_y")).intValue();
    CaptureFlagMain.LL_verticalShallow_x = ((Long) data.get("SV_LL_x")).intValue();
    CaptureFlagMain.LL_verticalShallow_y = ((Long) data.get("SV_LL_y")).intValue();
    CaptureFlagMain.UR_verticalShallow_x = ((Long) data.get("SV_UR_x")).intValue();
    CaptureFlagMain.UR_verticalShallow_y = ((Long) data.get("SV_UR_y")).intValue();
    CaptureFlagMain.LL_redZone_x = ((Long) data.get("Red_LL_x")).intValue();
    CaptureFlagMain.LL_redZone_y = ((Long) data.get("Red_LL_y")).intValue();
    CaptureFlagMain.UR_redZone_x = ((Long) data.get("Red_UR_x")).intValue();
    CaptureFlagMain.UR_redZone_y = ((Long) data.get("Red_UR_y")).intValue();
    CaptureFlagMain.LL_greenZone_x = ((Long) data.get("Green_LL_x")).intValue();
    CaptureFlagMain.LL_greenZone_y = ((Long) data.get("Green_LL_y")).intValue();
    CaptureFlagMain.UR_greenZone_x = ((Long) data.get("Green_UR_x")).intValue();
    CaptureFlagMain.UR_greenZone_y = ((Long) data.get("Green_UR_y")).intValue();
  }
}
