
#ifdef BOARD_VESC
void setup_wifi()
{}

#else

#include <IPAddress.h>
#include <IPv6Address.h>

#include <WiFiSTA.h>

WiFiSTAClass wifi;

void setup_wifi()
{
    wifi.begin("CAI_HOME", "64560266");


}

#endif
