# ADF4350
Mongoose native SPI driver for the ADF4350 wideband synthesizer device.

## Overview

The ADF4350 wideband synthesizer can be set to output a frequency between 137.5 MHz 
and 4.4 GHz. This library supports setting the ADF4350 RF ouptut from 137.5 MHz to 
4.4 GHz in 100 kHz steps.

## API

Call the mgos_adf4350_freq() function passing in the frequency required. 
The freq is in MHz.

## Connecting the ADF4350 to the ESP32 or ESP8266

The ADF4350 device is programmed via an SPI bus. The connections for this are 
defined in the mos.yml file.

##Example application

### mos.yml

```
#These are the defaults for the ADF4350 library but maybe redefined if required.
config_schema:
 - ["adf4350", "o",           {"title": "ADF4350 device settings"}]
 - ["adf4350.ce_pin", "i", 4, {title:   "ADF43509 CE pin"}]
 - ["adf4350.spi_cs", "i", 2, {title:   "The SPI CS connected to the ADF4350 LE pin (0,1 or 2)"}]

#SPI bus defaults
 - ["spi.enable",    "i", 1,  {title: "Enable SPI bus."}]
 - ["spi.cs0_gpio",  "i", -1, {title: "The SPI bus CS0 pin."}]
 - ["spi.cs1_gpio",  "i", -1, {title: "The SPI bus CS1 pin."}]
 - ["spi.cs2_gpio",  "i", 5,  {title: "The SPI bus CS2 pin."}]
 - ["spi.mosi_gpio", "i", 23, {title: "Connected to the ADF4350 DATA input."}]
 - ["spi.miso_gpio", "i", 19, {title: "Not used as the ADF4350 only supports write opperations."}]
 - ["spi.sclk_gpio", "i", 18, {title: "The SPI CLK line."}]
 - ["debug.level",   "i", 2,  {title: "Debug Level"}]
```

### application

This application sets the RF output frequency of the ADF4350 to 2.5 GHz initially.
After this (every 30 seconds) the RFoutput frequency is set to a random value between
137.5 MHz and 4.4 GHz.

```
#include <mgos.h>
#include <adf4350.h>

static void timer_cb(void *arg) {
  double      startTime = mgos_uptime();
  double      stopTime = 0;
  float freqMHz = mgos_rand_range(137.5, 4400);
  uint32_t value = freqMHz*10;
  
  freqMHz = value/10.0;
  LOG(LL_ERROR, ("%s: SET freqMHz %f", __FUNCTION__, freqMHz) );
  mgos_adf4350_freq(freqMHz);

  stopTime = mgos_uptime();
  LOG(LL_ERROR, ("%s: Execution took %f seconds", __FUNCTION__, stopTime-startTime) );

}

/**
 * @brief The MGOS program entry point.
 **/
enum mgos_app_init_result mgos_app_init(void) {

  mgos_set_timer(30000, MGOS_TIMER_REPEAT, timer_cb, NULL);
  mgos_adf4350_freq(2500);

  return MGOS_APP_INIT_SUCCESS;
}

```

## Testing

This library has been tested with an ESP32 device connected to an ADF4350 device.