# Tesla CAN signals used by this project

> **Read [Observed on this vehicle](#observed-on-this-vehicle) first.** The
> connector tapped so far does not carry the messages described in the rest of
> this document. Everything below the "Observed" section is community
> reverse-engineering that has not been confirmed on this car.


Signal definitions are taken from the community-reverse-engineered
[joshwardell/model3dbc](https://github.com/joshwardell/model3dbc). They are **not**
published by Tesla, were mapped primarily on Model 3, and must be verified
signal by signal on the target car (2023 Model Y, US import) before being
trusted. See [Verification checklist](#verification-checklist).

Bus: 500 kbit/s classic CAN. All signals below are **little-endian (Intel)** —
DBC `@1` notation, where the quoted start bit is the position of the signal's
least significant bit.

Byte and bit positions below are derived as `byte = startBit / 8`,
`bit = startBit % 8`. Every byte of all three messages is fully accounted for
with no overlapping signals, which is the cross-check that confirms the
little-endian reading.

## Verified map - what the controller actually uses

Third twisted pair of the passenger A-pillar connector, 500 kbit/s. Every bit
below was established by toggling the control and watching which bit moved on
this car, not taken from a DBC.

### `0x3F5` VCFRONT_lighting, 10 Hz - byte 4 only

A bitmap of light switch states, **not** the pairs of 2-bit lamp fields the
community DBC describes.

| Bit | Meaning | Evidence |
| --- | ------- | -------- |
| `0x01` | position / park lamps | `80` -> `81` |
| `0x04` | dipped beam | `80` -> `84` |
| `0x20` | main beam | `84` -> `A4` |
| `0x40` | front fog | `84` -> `C4` |
| `0x80` | unknown, set in every sample | - |

R48 6.11.7.1 permit condition: `data[4] & 0x64`.

Bit 0 tracks the switch position rather than the lamps: with dipped beam
selected the value is `0x84`, not `0x85`, even though the position lamps are
physically lit. Use `0x3E3` for actual rear lamp state.

### `0x3E3` VCRIGHT_lightStatus, 5 Hz

| Signal | Location | Evidence |
| ------ | -------- | -------- |
| tail (rear position lamps) | byte 0, bits 2-3 | `00 00` -> `04 00` |
| rear fog lamp | byte 1, bits 0-1 | `04 00` -> `04 01` |

Matches the community DBC. Two-bit lamp encoding, `0=off 1=on 2=fault 3=SNA`.

### `0x273` UI_vehicleControl, 2 Hz

| Signal | Location | Evidence |
| ------ | -------- | -------- |
| `UI_rearFogSwitch` | byte 2, bit 7 | `81 E1 10 ...` <-> `81 E1 90 ...` |

Matches the community DBC. A **level**, held continuously by the touchscreen,
not an event. The body controller follows it within ~5 ms and accepts frames
from any source - no rolling counter, no checksum, no source validation.

## Observed on this vehicle

2023 Model Y, US import. Connector on the passenger A-pillar, unmated, one
twisted pair measuring 60 Ω. Measured with `src/scan_uno.cpp` and
`src/sniff_uno.cpp`, listen-only.

**Bus:** 500 kbit/s classic CAN, clean (`REC=0`, no error flags), ~766 frames/s,
**47 unique IDs**, no extended-ID frames.

47 IDs is small - a full Tesla vehicle bus carries well over a hundred - and the
set mixes messages the community DBC attributes to different segments (VCLEFT,
VCRIGHT and VCFRONT status alongside DAS and RCM chassis messages). That
combination on a spare connector is the signature of a **gateway-forwarded
subset**, not a raw bus segment.

IDs present:

```
102 103 104 11B 129 20A 20E 221 222 248 25D 263 274 27D 293 299 2A8 2B2 2B9
2E8 30A 311 31A 32B 34F 389 38E 38F 39B 39C 39D 3A2 3E1 3E9 3F1 40C 421 464
488 489 4E1 4F1 5D0 5D4 5E5 7FF
```

**Not present: `0x3F5`, `0x3E3`, `0x273`.** The messages the rest of this
document describes are not on this connector.

### Confirmed signal: headlamps

Toggling the headlights reliably changes `0x4F1`, symmetric in both directions:

```
off -> on:   4F1  b0 55>A6 ^C0   b5 00>50 ^50
on  -> off:  4F1  b0 B1>62 ^C0   b5 50>00 ^50
```

Byte 5 moves `0x00` <-> `0x50`: two adjacent 2-bit fields both going 0 <-> 1,
matching Tesla's lamp encoding and consistent with a left/right lamp pair.

| Signal | Location | Extract |
| ------ | -------- | ------- |
| headlamp left (provisional) | `0x4F1` byte 5, bits 4-5 | `(data[5] >> 4) & 3` |
| headlamp right (provisional) | `0x4F1` byte 5, bits 6-7 | `(data[5] >> 6) & 3` |
| unidentified, tracks headlamps | `0x4F1` byte 0, bits 6-7 | `(data[0] >> 6) & 3` |

`0x4F1` does not appear in the community DBC at all. Byte 0's low nibble is a
free-running counter.

`0x4F1` also carries HVAC state - byte 7 bit `0x80` sets when climate is
switched on, and byte 6 ramps as the blower spools up. Lighting plus climate in
one message fits a **VCFRONT** (front body controller) broadcast, which suggests
rear lamps live elsewhere.

### Confirmed: rear fog state is NOT on this bus

With headlights on, switching rear fog on lit the lamp and showed the amber
tell-tale - and **nothing on this bus reported it**. `0x3E1` (which the DBC calls
`VCLEFT_lightStatus`) never changed. The only `0x4F1` activity was a one-second
blip that reverted.

### Confirmed: commands are not visible on this bus

Switching HVAC on produced only the resulting state change (a physical blower
ramp over ~5 s), never a request frame from the touchscreen. No `UI_`-style
multi-field control message was observed, and `0x273` is absent entirely.
Consistent with a gateway forwarding status broadcasts only.

## Second bus: vehicle bus (same connector, second twisted pair)

500 kbit/s, ~1000 frames/s, **132 unique IDs**, no extended frames. Far richer
than the first pair and consistent with a real vehicle bus segment.

**`0x3F5` VCFRONT_lighting is present** at 10 Hz. Confirmed by toggling the
headlights:

```
3B3  b1 86>82 ^04      (5 ms earlier)
3F5  b4 80>84 ^04      full: 00 00 C8 38 84 0C 00 00
```

Byte 4 bit `0x04` is a 2-bit field going `00` -> `01`, Tesla's lamp encoding.

| Signal | Location | Extract |
| ------ | -------- | ------- |
| headlamps (confirmed) | `0x3F5` byte 4, bits 2-3 | `(data[4] >> 2) & 3` |

**The community DBC bit layout does not apply to this car.** Decoding the
payload above with the documented Model 3 layout yields low beam = SNA and front
fog = SNA, which is nonsense. `tesla::decodeFrontLighting()` must be re-derived
empirically before use.

`0x3B3` changed 5 ms before `0x3F5` on the headlight toggle, but did **not**
move when the mirrors were folded, so it is lighting-specific rather than a
general control message.

**Not present: `0x3E3`, `0x273`.** Counted outside the ID table over 2.5 minutes
with lamps toggling: zero frames each.

### Search for rear fog state and a command path: exhausted

Both connectors swept in full with the per-bit change detector
(`src/sniff_uno.cpp`), whose method is validated - it found `0x3F5` from a
headlight toggle on this same bus.

| Bus | Ranges swept | Rear fog toggle | Mirror fold |
| --- | ------------ | --------------- | ----------- |
| gateway feed (47 IDs) | all | nothing | status only (`0x4F1`) |
| vehicle bus (132 IDs) | `000-2FF`, `300-37F`, `380-3FF`, `400-7FF` | nothing | status only (`0x102`) |

Conclusions:

- **Rear fog lamp state is not broadcast on either reachable segment.**
- **No command or request message is visible on either segment.** Touchscreen
  actions produce only the resulting status change from the controller that
  performed them.

What this leaves: the trigger signal for the violation is available
(`0x3F5` byte 4 on the vehicle bus, `0x4F1` byte 5 on the gateway feed). The
means to act on it is not.

## Third bus: main vehicle bus (same connector, third twisted pair)

The A-pillar connector carries **three** twisted pairs. The third is the one
that matters.

500 kbit/s, ~2330 frames/s, **180+ unique IDs** (scanner table full at 180, so
the real count is higher).

All three target messages present:

| ID | Message | Rate |
| -- | ------- | ---- |
| `0x3F5` | VCFRONT_lighting | 10 Hz |
| `0x3E3` | VCRIGHT_lightStatus - rear fog state | 5 Hz |
| `0x273` | UI_vehicleControl - the command path | 2 Hz |

Transmit reachability was confirmed on the second pair (`src/txtest_uno.cpp`,
one-shot, ID `0x7FE`): frames sent from this controller are acknowledged by
other nodes. Not yet retested on this third pair.

### `0x273` is periodic, not event-driven

At 2 Hz with the car untouched, the touchscreen broadcasts its full control
state continuously rather than sending a request on change. This is the harder
of the two cases anticipated in [Injection risk](#injection-risk):

- A one-off injection is very unlikely to *collide* - the CID occupies the bus
  for roughly 250 us out of every 500 ms.
- But the CID will re-assert its own value within 500 ms, so a single injected
  frame is overwritten unless the receiving controller latches on an edge.

Which of those applies is the next thing to establish: capture `0x273` byte 2
bit 7 while toggling rear fog. If the bit stays set for as long as the lamp is
on, it is a level and the controller must be out-asserted. If it pulses, it is
an event and a single injection suffices.

## Vehicle behaviour vs R48 6.11.7

Established by observation, no CAN required:

Two separate breaches, both confirmed on the car:

- **6.11.7.1, partially breached.** With the lights fully off the car correctly
  refuses to light the rear fog lamp. But with the **position lamps alone** it
  allows it, and 6.11.7.1 lists only main beams, dipped beams and front fog
  lamps. Fixed by the permit gate: a switch-on press only counts when
  `0x3F5` byte 4 has `0x64` set.
- **6.11.7.3.1, breached.** Rear fog on, lights off (both extinguish, correctly),
  lights back on - the lamp returns by itself. The touchscreen holds
  `UI_rearFogSwitch` set the whole time and nothing ever clears it. Fixed by
  the suppression latch.

Note 6.11.7.3 offers a choice: 6.11.7.3.1 (the latch rule) **or** 6.11.7.3.2 (an
audible warning when the ignition is switched off or the key withdrawn and the
driver's door opened while the fog control is ON). The car implements neither.

6.11.8 requires a **circuit-closed** tell-tale - one indicating the lamp is
actually operating. Verified: the amber icon follows `0x3E3`, not the
touchscreen's switch, so suppressing the lamp clears the indicator too. This is
what makes the approach compliant rather than merely making the lamp dark.

## Lamp state encoding

Two-bit lamp status signals share one encoding:

| Value | Meaning |
| ----- | ------- |
| 0 | `LIGHT_OFF` |
| 1 | `LIGHT_ON` |
| 2 | `LIGHT_FAULT` |
| 3 | `LIGHT_SNA` (signal not available) |

Code treats only `1` as on. `FAULT` and `SNA` are deliberately **not** on, in
both directions: an unknown headlamp state must not grant permission for the
rear fog lamp, and an unknown rear fog state must not trigger an injection.

## `0x3F5` VCFRONT_lighting (8 bytes)

Front body controller. Source for the permit gate.

| Signal | Start | Len | Extract |
| ------ | ----- | --- | ------- |
| `VCFRONT_indicatorLeftRequest` | 0 | 2 | `data[0] & 3` |
| `VCFRONT_indicatorRightRequest` | 2 | 2 | `(data[0] >> 2) & 3` |
| `VCFRONT_hazardLightRequest` | 4 | 4 | `(data[0] >> 4) & 15` |
| `VCFRONT_ambientLightingBrightnes` | 8 | 8 | `data[1]` (×0.5 %) |
| `VCFRONT_switchLightingBrightness` | 16 | 8 | `data[2]` (×0.5 %) |
| `VCFRONT_courtesyLightingRequest` | 24 | 1 | `(data[3] >> 0) & 1` |
| `VCFRONT_approachLightingRequest` | 25 | 1 | `(data[3] >> 1) & 1` |
| `VCFRONT_seeYouHomeLightingReq` | 26 | 1 | `(data[3] >> 2) & 1` |
| `VCFRONT_hazardSwitchBacklight` | 27 | 1 | `(data[3] >> 3) & 1` |
| **`VCFRONT_lowBeamLeftStatus`** | 28 | 2 | `(data[3] >> 4) & 3` |
| **`VCFRONT_lowBeamRightStatus`** | 30 | 2 | `(data[3] >> 6) & 3` |
| **`VCFRONT_highBeamLeftStatus`** | 32 | 2 | `data[4] & 3` |
| **`VCFRONT_highBeamRightStatus`** | 34 | 2 | `(data[4] >> 2) & 3` |
| `VCFRONT_DRLLeftStatus` | 36 | 2 | `(data[4] >> 4) & 3` |
| `VCFRONT_DRLRightStatus` | 38 | 2 | `(data[4] >> 6) & 3` |
| **`VCFRONT_fogLeftStatus`** | 40 | 2 | `data[5] & 3` |
| **`VCFRONT_fogRightStatus`** | 42 | 2 | `(data[5] >> 2) & 3` |
| `VCFRONT_sideMarkersStatus` | 44 | 2 | `(data[5] >> 4) & 3` |
| `VCFRONT_sideRepeaterLeftStatus` | 46 | 2 | `(data[5] >> 6) & 3` |
| `VCFRONT_sideRepeaterRightStatus` | 48 | 2 | `data[6] & 3` |
| `VCFRONT_turnSignalLeftStatus` | 50 | 2 | `(data[6] >> 2) & 3` |
| `VCFRONT_turnSignalRightStatus` | 52 | 2 | `(data[6] >> 4) & 3` |
| **`VCFRONT_parkLeftStatus`** | 54 | 2 | `(data[6] >> 6) & 3` |
| **`VCFRONT_parkRightStatus`** | 56 | 2 | `data[7] & 3` |
| `VCFRONT_highBeamSwitchActive` | 58 | 1 | `(data[7] >> 2) & 1` |
| `VCFRONT_simLatchingStalk` | 59 | 2 | `(data[7] >> 3) & 3` |
| **`VCFRONT_lowBeamsOnForDRL`** | 61 | 1 | `(data[7] >> 5) & 1` |
| `VCFRONT_lowBeamsCalibrated` | 62 | 1 | `(data[7] >> 6) & 1` |

`VCFRONT_lowBeamsOnForDRL` matters for compliance: R48 6.11.7.1 permits the rear
fog lamp only when main-beam, dipped-beam or front fog lamps are switched on.
Low beams lit merely as daytime running lamps are read here as **not** dipped
beam, so they do not grant permission. This interpretation is an assumption
about the signal's meaning and needs confirming on the car — see the checklist.

## `0x3E3` VCRIGHT_lightStatus (2 bytes)

Right body controller. Source of rear fog feedback and rear position lamps.

| Signal | Start | Len | Extract |
| ------ | ----- | --- | ------- |
| `VCRIGHT_brakeLightStatus` | 0 | 2 | `data[0] & 3` |
| **`VCRIGHT_tailLightStatus`** | 2 | 2 | `(data[0] >> 2) & 3` |
| `VCRIGHT_turnSignalStatus` | 4 | 2 | `(data[0] >> 4) & 3` |
| `VCRIGHT_reverseLightStatus` | 6 | 2 | `(data[0] >> 6) & 3` |
| **`VCRIGHT_rearFogLightStatus`** | 8 | 2 | `data[1] & 3` |
| `VCRIGHT_interiorTrunkLightStatus` | 10 | 2 | `(data[1] >> 2) & 3` |

## `0x273` UI_vehicleControl (8 bytes)

Touchscreen (CID) control state. Carries the rear fog switch.

| Signal | Start | Len | Extract |
| ------ | ----- | --- | ------- |
| **`UI_frontFogSwitch`** | 3 | 1 | `(data[0] >> 3) & 1` |
| **`UI_rearFogSwitch`** | 23 | 1 | `(data[2] >> 7) & 1` |

This message also carries seat heater requests, mirror fold, lock requests,
display brightness, wiper mode and around thirty other fields. That shape —
a full control-state broadcast rather than a discrete event — drives the
injection risk described below.

## Injection risk

**Two nodes must never transmit the same CAN ID.** If this controller and the
CID both send `0x273` in the same arbitration window, both win arbitration on
the identifier, then diverge in the data field. That is a bit error: it produces
error frames on the vehicle bus and increments both nodes' transmit error
counters. Sustained, it can drive the CID toward bus-off.

Whether this matters depends on how the CID sends `0x273`, which is an
empirical question and the single most important measurement of the project:

> Log `0x273` for 60 seconds, car parked and untouched, listen-only.
> Count the frames.

- **Near zero, sent only on change** — injection is clean. Send one frame when
  the rule fires.
- **Steady periodic rate** — the controller is contending with the CID and the
  design needs to change.

Do this capture before writing any transmit code.

## Verification checklist

Nothing here should be trusted until confirmed on the car. All passive,
listen-only:

- [ ] The passenger-pillar tap carries these IDs at all (the DBC names the
      segment only as "VehicleBus")
- [ ] Bus is 500 kbit/s classic CAN, not CAN FD
- [ ] `0x3F5` and `0x3E3` appear, at what rate
- [ ] Toggling low beam changes `VCFRONT_lowBeamLeftStatus` / `RightStatus`
- [ ] Toggling high beam changes `VCFRONT_highBeamLeftStatus` / `RightStatus`
- [ ] Front fog, if fitted, changes `VCFRONT_fogLeftStatus` / `RightStatus`
- [ ] Position lamps change `VCFRONT_parkLeftStatus` and
      `VCRIGHT_tailLightStatus`
- [ ] Rear fog toggle changes `VCRIGHT_rearFogLightStatus`
- [ ] `VCFRONT_lowBeamsOnForDRL` is 1 in daylight auto mode and 0 with dipped
      beam genuinely selected
- [ ] `0x273` frame rate when idle (see above)
- [ ] `0x273` byte 2 bit 7 tracks the touchscreen rear fog switch
- [ ] No rolling counter or checksum in any of the three messages

## Non-compliant behaviour to characterise

Determines which rules the state machine must enforce. Needs no CAN at all:

- [ ] Does rear fog stay latched on across a park/lock/sleep/return cycle?
      Test both a short absence and a long one — deep sleep may clear state
      that a quick re-entry preserves.
- [ ] Can rear fog be switched on with the headlights off?
