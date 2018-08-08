# werma_816

A ROS driver for the [Werma 816.480.53 USB LED](https://www.werma.com/en/s_c1097i2061/LED_Beacon_USB_EM_5VDC_MC/81648053.html).

## Notes

The [official documentation](https://www.werma.com/gfx/file/report/brochure/2018/744.000.002-01_-.zip)
(zip file, open "Manual.pdf". Starts on page 12) *says* that you are able to

* get protocol version,
* get device name, and
* read current color

However, my device failed to respond correctly to any of those, and simply replied with `Command Error`. **As such, this
library will *not* error if writing fails in any way - just if opening the serial port fails.**

Additionally, the library takes in a `baud` parameter - this technically shouldn't be needed, because the IC used uses
native CDC serial, but YMMV.