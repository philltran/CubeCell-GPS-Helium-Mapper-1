// Version
#define APP_NAME    "Cube Mapper"
#define APP_VERSION "MaxP v2.2.1"

// -----------------------------------------------------------------------------
// CONFIGURATION
// Stuff you might reasonably want to change is here:
// TODO: Not all of this is implemented yet!
// -----------------------------------------------------------------------------

// The Mapper is always sending when it either moves some distance, or waits some time:
#define MIN_DIST_M              70.0       // Minimum distance in meters from the last sent location before we can send again. A hex is about 340m.
#define MAX_TIME_S              ( 2 * 60)  // If no minimum movement, uplink will still be sent every N seconds

// After a while, it still watches GPS, but turns the display off and sends slower:
#define REST_WAIT_S             (15 * 60)  // If we still haven't moved in this many seconds, start sending even slower
#define REST_TIME_S             (20 * 60)  // Slow resting uplink frequency in seconds
#define REST_LOW_VOLTAGE        3.6        // Below this voltage, send more slowly

// After a long while, shut down GPS & screen and go into deeper sleep, missing the initial movement.
#define SLEEP_WAIT_S            (2 * 60 * 60) // If we are not moving for this long, go into deeper sleep
#define SLEEP_TIME_S            (5 * 60)      // Wake up this often to power on the GPS and check for movement
#define SLEEP_GPS_TIMEOUT_S     60
#define SLEEP_LOW_VOLTAGE       3.4          // Below this voltage, stay in deep sleep

#define USB_POWER_VOLTAGE        4.1         // Assume we have USB power above this voltage
#define UNKNOWN_LOCATION_UPLINK  1           // If no GPS reception, send a non-Mapper Uplink packet anyway
#define GPS_LOST_WAIT_S         (10 * 60)    // How long after losing GPS do we send a lost packet?
#define GPS_LOST_TIME_S         (45 * 60)    // How often to send Lost GPS packets?


#define LORAWAN_PORT 2                      // FPort for Uplink messages -- must match Helium Console Decoder script!
#define LORAWAN_CONFIRMED_EVERY 0           // Send confirmed message for ACK every N messages (0 means never, 1 means always, 2 every-other-one..)
#define LORAWAN_SF DR_SF7                   // Spreading factor (recommended DR_SF7 for network map purposes, DR_SF10 is slower/more-reach)

// Deadzone defines a circular area where no map packets will originate.
// Set Radius to zero to disable, or leave it enabled to select center position from menu.
// (Thanks to @Woutch for the name)
#define DEADZONE_LAT              34.5678
#define DEADZONE_LON            -123.4567
#define DEADZONE_RADIUS_M       500  // meters

// -----------------------------------------------------------------------------
// Less-common Configuration items, but feel free to adjust..
// -----------------------------------------------------------------------------
#define VBAT_CORRECTION         1.004   // Edit this for calibrating your battery voltage
#define MENU_TIMEOUT_MS         3000    // Menu timeout milliseconds
#define LONG_PRESS_MS           500       // How long to hold the button to Select
#define BATTERY_UPDATE_RATE_MS  (15 * 1000) // How often to sample battery voltage
#define SCREEN_UPDATE_RATE_MS   500 // Refresh OLED screen this often
#define SCREEN_ON_TIME_MS       (MENU_TIMEOUT_MS + 5000) // Keep screen on this long after button
