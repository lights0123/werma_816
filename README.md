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

Additionally, the library takes in a `baud` parameter - this technically shouldn't be needed, because the IC uses
native CDC serial, but YMMV.

These LEDs **only support values of 1 to 100!** As such, instead of having `2 ^ (3 * 8)` (~16 million) possible
combinations, you only have `3 ^ 10` (1 million) possible combinations. The library automatically takes care of the
conversion between standard, 24-bit color and the percentage range used by the LED.

## Install

In the `src` directory of your `catkin_ws`, run:

```bash
git clone https://github.com/lights0123/werma_816
git clone https://github.com/wjwwood/serial
```

Then, just `catkin_make` as usual.

### Running

An example launch file is provided. After installing and running `catkin_make`, just run

    roslaunch werma_816_driver werma_816.launch

Then, in another terminal,

    rostopic pub led/color werma_816_msgs/Color '{r: 0, g: 128, b: 255}'

## Documentation

The `werma_816_msgs/Color` message accepts 3 `uint8_t` values for red (`r`), green (`g`), and blue (`b`).