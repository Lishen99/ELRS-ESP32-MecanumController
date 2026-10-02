# Troubleshooting

Start with `status` on the command line: it reports the radio link, who is in
charge, the wheel commands, whether the motor driver answers, the Bluetooth
state, data counters and the cause of the last restart. `ch` shows the live
value of every handset channel.

## Wheels and motor driver

| Symptom | Cause and fix |
|---|---|
| `motor board: NOT ANSWERING` | SDA and SCL swapped or loose, no common ground, or the driver has no power. Check `motor_addr` if the driver is not at `0x34`. |
| A wheel turns the wrong way | `set motorN.rev on` (or `off`) for that channel, then `save`. |
| The robot turns when it should go sideways | Two channels drive each other's wheels: swap their `motorN.wheel` values. If all channels are right, check the mecanum wheels themselves: seen from above, the rollers must form an X. |
| A motor races at full speed instead of holding its speed | The encoder polarity is wrong for these motors: `set motor_enc_pol 1`, `save`, `reboot`. |
| Armed, but the wheels do not turn | `status` shows `wheels live at 0 % speed`: the speed wheel is all the way down, or not mixed onto CH12. `ch` shows CH12 following the wheel once it is. |
| The robot is too fast or too slow | `drive_speed_top` sets the speed at the top of the speed wheel, `drive_max` the overall maximum. See [driving.md](driving.md#speed). |

## Radio link

| Symptom | Cause and fix |
|---|---|
| `status`: nothing received on GPIO16 | The receiver has no power, there is no link (a receiver sends nothing until it is linked), or its TX and RX wires are swapped. Swapped wires can be fixed in software: `set crsf_rx_pin 17`, `set crsf_tx_pin 16`, `save`, `reboot`. |
| `status`: bytes arrive but no valid CRSF frames | The receiver's serial output is not CRSF at 420000 baud. |
| Receiver LED blinks slowly | No link: the handset is off, or the receiver is not bound. See [radio-setup.md](radio-setup.md#binding). |
| Receiver LED blinks fast | The receiver is in WiFi mode because it heard no handset within 60 s of power-up. Switch the handset on, then power-cycle the receiver. |
| The receiver will not enter bind mode | It may still run ExpressLRS 1.x, which has no bind mode. Update it to 2.x and then 3.x over WiFi. |
| The `bind` command has no effect | `bind` needs ExpressLRS 3.4 or newer on the receiver and the ESP32-to-receiver RX wire. Power-cycling the receiver three times works on any 3.x version. |
| Receiver LED solid at power-up, never links | It may be in its bootloader, which some receivers enter when their RX pad is held low at power-up. A 10 kΩ pull-up from GPIO17 to 3.3 V should help. |
| Linked but never arms | The arm switch is not on CH5 (`ch` shows it), or it was already on when the link came up: switch it off for a second, then on. |
| No telemetry on the handset | Telem Ratio is off, the sensors have not been discovered yet, or the ESP32-to-receiver RX wire is missing. |

## PS4 controller

| Symptom | Cause and fix |
|---|---|
| Does nothing, light bar amber | Let go of the sticks once: a controller that takes over must first see them centred. |
| Does nothing, light bar red | The handset is linked and in charge. Switch the handset off, or drive with it. |
| Will not pair | Hold Share and PS until the light bar flashes quickly. `status` shows `bluetooth: controller connected` once it has paired. Check that `bt_enable` is on. |

## USB, command line and companion computer

| Symptom | Cause and fix |
|---|---|
| The command line or the Python tool gets no answer, although the robot works | Another program has the port open, or the USB serial chip has stopped passing data. Close serial monitors, then unplug and replug the USB cable. |
| Upload fails with "Failed to connect" | Close any program using the port and try again. If it persists, hold the board's BOOT button while the upload starts. |
| The companion computer receives nothing on the UART | TX and RX swapped, no common ground, or its baud rate differs from `host_baud` (115200). |
| Locked out after changing a setting | The command line on USB always works: `defaults`, `save`, `reboot`. Or erase the flash with `pio run -t erase` and upload again. |

## Power

| Symptom | Cause and fix |
|---|---|
| `status`: `last reset: brownout` | The 5 V supply sags under load: use shorter or thicker wires, or add a capacitor across 5 V and GND. |
| The wheels kept turning while the ESP32 was off or being flashed | The motor driver holds its last command while the ESP32 is not running. Power the two together, and disarm before flashing. |
