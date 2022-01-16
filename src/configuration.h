// -----------------------------------------------------------------------------
// CONFIGURATION
// Stuff you might reasonably want to change is here:
// -----------------------------------------------------------------------------

#define MIN_DIST_M              68.0       // Minimum distance in meters from the last sent location before we can send again. A hex is about 340m.
#define MAX_TIME_S              ( 2 * 60)  // If no minimum movement, the LoRa frame will still be sent once every N seconds

#define REST_WAIT_S             (30 * 60)  // If we still haven't moved in this many seconds, start sending even slower
#define REST_TIME_S             (10 * 60)  // Slow resting ping frequency in seconds

#define BATTERY_LOW_VOLTAGE 3.4  // Below this voltage, power off until USB power allows charging

#define LORAWAN_PORT 2              // FPort for Uplink messages -- must match Helium Console Decoder script!
#define LORAWAN_CONFIRMED_EVERY 0  // Send confirmed message for ACK every N messages (0 means never, 1 means always, 2 every-other-one..)
#define LORAWAN_SF DR_SF7           // Spreading factor (recommended DR_SF7 for network map purposes, DR_SF10 is slower/more-reach)

// -----------------------------------------------------------------------------
// Version
// -----------------------------------------------------------------------------

#define APP_NAME "Cube Mapper"
#define APP_VERSION "1.6.2 MaxP"

// -----------------------------------------------------------------------------
// Less common Configuration iteams
// -----------------------------------------------------------------------------

#define LOGO_DELAY 2000  // Time to show logo on first boot (ms)

#define VBAT_CORRECTION 1.004  // Edit this for calibrating your battery voltage
