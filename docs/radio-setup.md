# Radio setup

The robot is driven by an ExpressLRS (ELRS) handset through a receiver wired
to the ESP32 (see [hardware.md](hardware.md#receiver)). This guide covers the
receiver firmware, binding, and the handset channels the controller expects.
The reference setup is a Happymodel EP1 receiver and a RadioMaster Zorro, both
on ExpressLRS 3.5.4.

## Versions

The receiver must run the same **major** ExpressLRS version as the handset's
transmitter module: a 3.x handset needs a 3.x receiver. Matching the exact
version, as the reference setup does, avoids surprises.

The receiver's installed version is shown on its WiFi page (below).

## Flashing the receiver

The EP1 and EP2 are flashed over WiFi, so no adapter is needed.

1. Install the [ExpressLRS Configurator](https://github.com/ExpressLRS/ExpressLRS-Configurator/releases).
   Select the version that matches the handset, your receiver's target (for
   the EP1/EP2, the Happymodel EP 2.4 GHz receiver) and the WiFi flashing
   method. A binding phrase is optional; this guide binds without one.
2. Power the receiver with the handset switched off. After 60 seconds without
   a handset it starts a WiFi access point, `ExpressLRS RX`, with the password
   `expresslrs`.
3. Connect the computer to that network, then flash from the Configurator, or
   build the firmware file and upload it on the receiver's page at
   <http://10.0.0.1>.

A receiver still on ExpressLRS 1.x has no bind mode and cannot be bound
without a binding phrase. Update it in two steps: to 2.x first, then to 3.x.

## Binding

Without a binding phrase, the receiver is bound to the handset once, by hand:

1. Put the receiver into bind mode. Either switch its power on and off three
   times in quick succession, leaving it on the third time, or, with the
   receiver wired to the ESP32 and running ExpressLRS 3.4 or newer, type
   `bind` on the ESP32's command line. In bind mode the receiver's LED
   double-blinks.
2. On the handset, open the ExpressLRS Lua script (**SYS → Tools →
   ExpressLRS**) and select **[Bind]**. The receiver's LED turns solid.

The receiver remembers the handset from then on. If a binding phrase was set
when flashing, the same phrase on both sides binds them automatically
instead.

## Handset channels

The firmware's defaults assume a mode 2 handset with the usual ExpressLRS
channel order (AETR):

| Channel | Control | Used for |
|---|---|---|
| CH1 | right stick left / right (aileron) | driving sideways |
| CH2 | right stick up / down (elevator) | driving forward / back |
| CH3 | left stick up / down (throttle) | not used |
| CH4 | left stick left / right (rudder) | turning on the spot |
| CH5 | two-position switch | arming |
| CH12 | S1 scroll wheel | speed |

Every assignment is a setting (`drive_fwd`, `drive_strafe`, `drive_turn`,
`drive_speed`, `arm_ch`), so a different layout needs no rebuild. See
[settings.md](settings.md#driving).

### Arm switch: CH5

ExpressLRS sends CH5 (AUX1) with every packet and treats it as the arm
channel. Mix a two-position switch onto CH5 in the model's **MIXES** page.
The robot counts as armed while the channel is above 1600 µs, that is with
the switch at +100 %. If the switch arms in the position you want to be off,
set the mix weight to −100. Type `ch` to check: CH5 reads about 1000 µs off
and 2000 µs on.

### Speed wheel: CH12

The speed is set with a scroll wheel. On the Zorro:

1. Press **MDL** and go to the **MIXES** page.
2. Select **CH12** and edit its mix, or add one.
3. Set **Source** to **S1** (or S2) and **Weight** to **100**.

Type `ch` on the ESP32's command line and roll the wheel: CH12's `raw_us`
should move between about 1000 and 2000. If it moves the wrong way, set the
weight to −100.

CH12 is used because ExpressLRS sends it in the finest steps in either of its
switch modes: 16 steps in **Hybrid**, and 64 to 128 in **Wide**, where CH6 to
CH11 get only 6 steps in Hybrid. For the smoothest speed control, select
**Wide** under **Switch Mode** in the ExpressLRS Lua script.

To use the throttle stick or a three-position switch for the speed instead,
see the recipes in [settings.md](settings.md#recipes).

### Telemetry

The ESP32 sends telemetry to the handset through the receiver:

1. In the ExpressLRS Lua script, set **Telem Ratio** to around 1:8, which
   updates the values about once a second.
2. On the model's **Telemetry** page, select **Discover new sensors**.

`FM` then shows the robot's state (`ARMED` / `DISARMED`, or a text sent by
the companion computer), and `RxBt` its battery voltage when a battery
divider is fitted (see [hardware.md](hardware.md#optional-extras)). Values
from the companion computer, such as GPS and attitude, appear as sensors too.
With ExpressLRS 3.5.4, battery, GPS, attitude, flight mode, vertical speed and
altitude reach the handset; airspeed, RPM and temperature need 3.5.5 or
newer.

## Good to know

- **Switch the handset on first.** A receiver that hears no handset within
  60 seconds of power-up switches to WiFi mode and stays there until it is
  power-cycled.
- In the standard packet rates ExpressLRS carries CH1 to CH12. CH13 and CH14
  are not transmitted, and the receiver fills CH15 and CH16 with the link
  quality and signal strength, so leave those four free.
- After the link has been lost, the robot stays disarmed until the arm switch
  has been switched off and on again. See [driving.md](driving.md#arming).
