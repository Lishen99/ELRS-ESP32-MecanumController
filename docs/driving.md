# Driving

How the robot responds to its controllers: the controls, the speed, which
controller is in charge, arming, and what happens when something goes wrong.
The numbers below are the defaults; each one is a setting (see
[settings.md](settings.md#driving)).

## Controls

### ExpressLRS handset

| Control | Action |
|---|---|
| CH5 switch | Arm. Nothing moves while it is off, and switching it off stops the wheels at once. |
| Right stick up / down (CH2) | Forward / back |
| Right stick left / right (CH1) | Sideways |
| Left stick left / right (CH4) | Turn on the spot |
| S1 scroll wheel (CH12) | Speed: stopped at the bottom, 80 % of the maximum at the top |

Moving both sticks combines the motions: forward and sideways together drive
diagonally, and forward with a turn drives in an arc. When a combination asks
a wheel for more than full speed, all four wheels are scaled down together so
the robot keeps its direction.

Setting up these channels on the handset is covered in
[radio-setup.md](radio-setup.md#handset-channels).

### PS4 controller

To pair a controller the first time, hold **Share** and **PS** until the light
bar flashes quickly. After that, pressing **PS** reconnects it.

| Control | Action |
|---|---|
| Left stick | Forward / back and sideways |
| Right stick left / right | Turn on the spot |
| Hold L2 | Slow |
| Hold R2 | Fast |

Stick movements under 10 % of full travel are ignored, so a resting thumb does
not creep the robot along. The light bar shows the controller's role:

| Light bar | Meaning |
|---|---|
| Green | The PS4 controller is driving |
| Amber | It is in charge but waits for its sticks to be centred before it drives |
| Red | The ExpressLRS handset is in charge |

## Speed

The motor driver accepts speed commands from −100 to 100. The largest command
the controller sends is `drive_max`, 60 by default.

### Speed wheel on the handset

The speed wheel works like the throttle on a drone: it sets how fast the robot
may go, and the sticks use as much of that as they are pushed. The wheel at
the bottom keeps the robot still; at the top it allows `drive_speed_top`, 80 %
of `drive_max`. For example:

| Speed wheel | Right stick | Wheel command |
|---|---|---|
| Top (80 %) | fully forward | 48 |
| Top (80 %) | half forward | 24 |
| Half way (40 %) | fully forward | 24 |
| Bottom (0 %) | fully forward | 0 |

While armed, `status` shows the speed the wheel currently allows, for example
`wheels live at 80 % speed`. To allow full speed at the top of the wheel, use
`set drive_speed_top 100`.

### Speed modes on the PS4 controller

| Buttons | Speed | Wheel command at full stick |
|---|---|---|
| none | normal, 50 % | 30 |
| L2 held | slow, 30 % | 18 |
| R2 held | fast, 100 % | 60 |

With both triggers held, slow wins.

### Acceleration and braking

Speed commands change gradually, every 10 ms, by 2 steps when speeding up
(`drive_accel`) and 1 step when slowing down (`drive_decel`). From standstill
to 30 takes 150 ms and back to standstill 300 ms. When the robot is asked to
reverse, it slows to a stop at the braking rate before speeding up the other
way.

## Who is in charge

With the default `drive_src auto`:

- **While the handset's radio link is up, the handset is in charge**, armed or
  not. Its arm switch is therefore a kill switch for the whole robot: while it
  is off, nothing the PS4 controller does moves the wheels.
- **When the handset is switched off or out of range**, a connected PS4
  controller takes over. It only starts driving once its sticks have been
  seen centred, so the robot never jumps away because a stick happened to be
  pushed at the moment of the handover.

`drive_src elrs` or `drive_src bt` restricts driving to one controller, and
`drive_src off` disables the wheels altogether.

## Arming

The handset arms the robot when all of these hold:

- the radio link is up;
- the arm switch (CH5) is above 1600 µs;
- since power-up or the last link loss, the switch has been seen off for at
  least half a second.

The last rule means a robot never arms by itself because the switch was left
on: after a restart or a lost link, switch it off and on again. `status` says
so when that is what holds it back. Changing the arm settings (`arm_ch`,
`arm_us`) also disarms the robot until the switch has been cycled under the
new rule. Details are in [settings.md](settings.md#arming).

## Stopping and failsafes

| Event | The wheels |
|---|---|
| Sticks released | Slow down gently, at the braking rate |
| Arm switch off | Stop at once |
| Handset switched on while the PS4 controller drives | Stop at once: the handset takes charge, disarmed until its switch is cycled |
| Handset switched off or out of range | Keep the last command for 0.5 s (`failsafe_ms`), then slow down gently |
| PS4 controller disconnects or falls silent for 0.5 s | Slow down gently |
| ESP32 restarts, crashes, browns out, or its watchdog fires | Stop at once |
| ESP32 loses power | **The motor driver keeps its last command.** Power the two together. |

`set drive_fs stop` makes a lost controller stop the wheels at once instead of
slowing them down.

**Restarts and crashes.** A restart of the ESP32 does not reset the motor
driver, which would otherwise carry on at its last speed. On its way down,
the firmware therefore sends the driver a stop, driving the I²C lines
directly so that this also works from a crash handler, and releases every
auxiliary output pin. At power-up it sends a stop again before anything else.
A watchdog restarts the ESP32 if its main loop stalls for one second; that
second is the only gap. `status` reports the cause of the last restart.

**Commands that interrupt control are refused while driving.** `save`,
`reboot`, `defaults` and `bind` each pause the controller for a moment, so
they answer `ERR disarm first` while the handset is armed and `ERR stop the
wheels first` while the wheels turn.

## Checking the state

`status` on the command line summarises everything:

```
> status
elrs-bridge 0.1.0 (esp32), up 41 s, last reset: power-on
link: UP   armed: yes   failsafes: 0
drive: ELRS handset in charge, wheels live at 80 % speed
  motors M1..M4: +48 -48 -48 +48
  motor board: answering, 0 of 4117 commands failed
bluetooth: no controller (to pair a PS4 controller, hold Share and PS)
rc: 246 Hz, last frame 3 ms ago, 9513 frames
uplink: LQ 100%  RSSI -44 dBm  SNR 14 dB  handset power 10 mW  rf mode 7
```

`ch` lists the live value of every channel, which is the quickest way to
check the handset's switches and wheels.

## First power-up

Work through these steps on the bench, **with the wheels off the ground**.

1. **Motor driver.** Flash the ESP32 and open the command line
   (`pio device monitor`). `status` should report `motor board: answering`.
   If it reports `NOT ANSWERING`, check SDA, SCL, ground and the driver's
   power.
2. **PS4 controller.** Pair it. The ESP32's LED turns steady and the light bar
   green. Push the left stick forward: all four wheels turn forward. Let go:
   they slow to a stop. Switch the controller off: they stop within about a
   second.
3. **Radio link.** Wire the receiver, switch the handset on, then power the
   robot. `status` shows `link: UP`, and the PS4 controller's light bar turns
   red because the handset is now in charge.
4. **Channels.** Type `ch` and move each stick, the arm switch and the speed
   wheel: their `raw_us` values follow.
5. **Driving.** Arm: switch CH5 off, wait a second, switch it on. The LED
   blinks fast. Roll the speed wheel a little way up and push the right stick
   forward: the wheels turn forward, faster as the wheel rolls further up.
   Switch CH5 off: they stop at once.
6. **Failsafe.** Armed and driving, switch the handset off. After half a
   second the wheels slow to a stop. Switch it back on: the robot stays
   disarmed until CH5 has been switched off and on again.

Only then put the robot on the floor.
