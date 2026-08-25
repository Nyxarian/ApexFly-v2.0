# ApexFly v2.0 Height-Control Flight Incident Record

## Status

- Recorded: 2026-08-24
- Current decision: pause all automatic height-control work and do not continue propeller-on testing with the current implementation.
- No firmware change, build, or flashing was performed as part of this investigation.
- A future option is to disable/remove the automatic height-control path temporarily and return to a simpler manually commanded throttle path. That decision has not yet been implemented.

## User-Observed Events

### Test 1

- The aircraft took off and climbed faster than expected.
- Returning the throttle stick to center did not produce a stable hover.
- Pulling the throttle stick down did not produce the expected slow descent.
- The aircraft descended much faster than the ascent and effectively fell.

### Test 2

- The aircraft took off again.
- Pulling the throttle stick down appeared not to command a descent.
- The aircraft remained powered and the user felt that control had been lost.
- The user selected OFF, which immediately removed motor thrust and caused a crash.

## Evidence Files

The investigation used these exported blackbox files:

- `BlackboxReader/bin/exports/blackbox_20260824_143949/session_1722.csv`
- `BlackboxReader/bin/exports/blackbox_20260824_143949/session_1722.pid.csv`
- `BlackboxReader/bin/exports/blackbox_20260824_143949/session_1723.csv`
- `BlackboxReader/bin/exports/blackbox_20260824_143949/session_1723.pid.csv`
- `BlackboxReader/bin/exports/blackbox_20260824_143949/INDEX.txt`
- `BlackboxReader/bin/exports/blackbox_20260824_143949/serial_raw.log`

Relevant implementation files:

- `ApexFly v2.0/Modules/Control/altitude_control.c`
- `ApexFly v2.0/Modules/Baro/altitude_est.c`
- `ApexFly v2.0/Modules/Baro/baro.c`
- `ApexFly v2.0/Modules/Motor/motor_test.c`
- `ApexFly v2.0/Modules/Flash/blackbox.c`

## Confirmed Findings

### 1. Neither flight entered the AIRBORNE state

The altitude-state values are:

- `0`: GROUND_IDLE
- `1`: TAKEOFF_RAMP
- `2`: AIRBORNE
- `3`: ESTIMATE_HOLD

Both sessions remained in states `0` and `1`. Neither session entered state `2`.

This is the central failure. Once the aircraft was physically airborne, the firmware still treated it as an unconfirmed ground takeoff attempt.

### 2. The takeoff detector used a drifting absolute height

The takeoff detector requires either:

- `z > 0.15 m` and `vz > 0.12 m/s` continuously for 0.3 seconds; or
- `z > 0.30 m` continuously for 0.3 seconds.

The estimator is initialized during the broader IMU initialization flow, but it is not re-zeroed or referenced to a new takeoff origin at each arm/takeoff request.

In session 1722, while the motors remained at 1000 us for approximately the first 10 seconds, the estimated height moved from about `-1.17 m` to about `-2.93 m`, with a ground-idle range reaching approximately `-3.12 m`.

A real climb of several metres could therefore leave the absolute estimate below zero and prevent the takeoff condition from ever becoming true.

### 3. The estimator health flag did not reject severe ground drift

`AltitudeEst` considers the estimate healthy when:

- the values are finite; and
- the barometer has not timed out.

It does not reject an estimate that drifts by metres while the aircraft is stationary on the ground. Therefore the takeoff state machine can consume numerically valid but operationally unusable height data.

### 4. Low/center stick in TAKEOFF_RAMP cancels takeoff instead of descending

When state `1` is active and the stick returns to `<= 600`, the controller:

- resets the vertical-speed PID;
- forces vertical-speed correction to zero;
- slews the base PWM toward 1000 us at 80 us/s; and
- returns to GROUND_IDLE when 1000 us is reached.

This path is intended to cancel a takeoff while the aircraft is still on the ground. Because physical liftoff was never confirmed, the same path ran while the aircraft was actually airborne.

### 5. Session 1722 directly records the unsafe cancellation

Session 1722 used:

- vertical-speed PID: `Kp=100`, `Ki=0`, `Kd=0`;
- hover PWM: `1250 us`.

At about 22.18 seconds, the stick target returned to the center/low region. The output then fell from approximately `1239 us` to `1000 us` in 2.98 seconds.

Measured output slope:

```text
(1239 - 1000) us / 2.98 s = 80.2 us/s
```

This matches the programmed takeoff-cancellation slew rate of 80 us/s. The aircraft was not commanded to descend at `-0.20 m/s`; thrust was progressively withdrawn until it reached the ESC minimum.

### 6. The 0.30 m/s climb target was followed approximately

During a useful rising segment in session 1722, estimated height increased by about 1.34 m in 4.72 seconds:

```text
1.34 m / 4.72 s = 0.284 m/s
```

This is close to the configured `+0.30 m/s` target. The climb was not shown to be an unbounded acceleration in that segment. However, `0.30 m/s` felt too fast for the low-altitude test environment and should be reduced before any future test.

The configured rate is a PID target, not an independent hard velocity limiter.

### 7. Session 1723 was truncated because the W25Q64 log area was full

Session 1723 used:

- vertical-speed PID: `Kp=100`, `Ki=0`, `Kd=0`;
- hover PWM: `1350 us`.

It remained in TAKEOFF_RAMP and reached a base PWM of approximately `1392 us`. The final stored samples still contained a positive vertical-speed command.

The log index marks session 1723 as incomplete. The flash write address had reached `0x007FF000`, the configured end of the log area. The later low-stick and OFF sequence was therefore not persisted.

The log cannot establish whether the final low-stick command reached the aircraft. It only proves that the stored portion never entered AIRBORNE and was still following TAKEOFF_RAMP logic.

If a low-stick command reached the controller while the output was approximately 1360 us, the existing 80 us/s cancellation path would require approximately:

```text
(1360 - 1000) us / 80 us/s = 4.5 s
```

This would feel like a delayed or ineffective descent command.

### 8. Attitude control and radio connection were not the primary failure

In the recorded portions:

- `conn` remained `1`;
- roll and pitch remained relatively small;
- the aircraft did not show a primary roll/pitch attitude divergence before the vertical-control problem.

The incident evidence points to height estimation and altitude-state handling rather than a primary attitude-loop or recorded radio-link failure.

### 9. OFF is a hard motor stop

The current OFF/disarm path immediately commands all motors to 1000 us. It is not an emergency controlled-landing mode.

Using OFF in the air necessarily removes lift. It remains useful only when an immediate motor stop is safer than continued powered flight, such as an imminent collision with a person.

## Additional Safety Risks Found in Static Review

These issues were not the direct recorded trigger in both sessions, but they must be addressed before automatic height control is considered safe:

1. If the height estimate becomes unhealthy in flight, state `ESTIMATE_HOLD` freezes the last PWM and ignores the pilot's descent-stick intent.
2. The vertical-speed loop currently uses P-only control. A hover-PWM error can leave a steady climb or descent error.
3. The blackbox stores final base PWM but not raw throttle-stick input, reducing incident reconstruction quality.
4. Blackbox arm/disarm events use the same finite queue as samples, and arm/disarm enqueue failure is not retried.
5. The log is append-only rather than circular; once full, future incident tails are lost.

## Required Changes Before Any Future Propeller-On Height-Control Test

1. Establish a takeoff reference height at arm or accepted takeoff request:

```text
relative_height = estimated_height - takeoff_reference_height
```

Use relative height, not the long-term absolute estimate, for liftoff confirmation.

2. Do not allow a physically airborne aircraft to return through the ground takeoff-cancellation path.

3. Once PWM has entered a possible-liftoff region, center/low stick must retain controlled flight authority. It must not automatically retract to 1000 us merely because the takeoff detector failed.

4. Add a real estimator-quality gate that rejects excessive stationary height/velocity drift before takeoff.

5. Reduce the initial test climb rate to approximately `0.10-0.15 m/s`.

6. Define an estimator-failure response that preserves pilot descent authority and has a bounded safe outcome.

7. Add a separate emergency controlled-descent mode. Keep hard OFF as a final collision-risk motor-kill action, not as the normal emergency landing action.

8. Record at least:

- raw throttle-stick command (`0..1000`);
- relative takeoff height;
- absolute estimated height;
- estimator-validity reason/state;
- airborne-confirmation timer/evidence;
- requested vertical-speed target;
- final base PWM;
- disarm reason.

9. Make blackbox storage circular or require an explicit erase/space check before flight.

10. Verify parameter persistence and the displayed value. The incident logs contain hover PWM values of 1250 us and 1350 us, not 1500 us.

## Safe Verification Order If Work Resumes

Do not resume directly with an unrestricted propeller-on flight.

1. Unit/simulation test the state machine with recorded and synthetic height data.
2. Verify each arm starts from a relative height of approximately zero.
3. Inject metres of absolute sensor offset and confirm liftoff detection still uses relative height.
4. Verify center and low stick never command 1000 us while the simulated aircraft is airborne.
5. Verify estimator failure still permits a bounded descent response.
6. Test without propellers and confirm raw stick, requested vertical speed, state and PWM in telemetry/blackbox.
7. Use a restrained test stand or tethered low-power setup.
8. Only then perform a low-altitude open-area test with climb rate limited to `0.10-0.15 m/s` and a second person acting as safety observer.

## Deferred Decision: Temporarily Disable Automatic Height Control

The user may choose to temporarily cancel/disable this automatic height-control feature instead of immediately repairing it.

If that direction is selected later, it must still be implemented as an explicit safe flight mode. Do not simply delete the controller and feed unvalidated values to the motors. The replacement path should include:

- clear manual throttle mapping;
- bounded output and slew behavior;
- retained roll/pitch/yaw stabilization;
- defined radio-loss behavior;
- clear OFF semantics;
- blackbox recording of raw stick and final PWM;
- a new propeller-off verification before flight.

No decision or implementation has been made yet.
