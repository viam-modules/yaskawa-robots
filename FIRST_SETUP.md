# First-time setup: a new YRC1000 for the Viam Yaskawa module

Bring-up  a YRC1000 straight out of the crate, through to a working
`viam:yaskawa-robots` arm component.

---

## Phase 0 — Physical installation

1. **Disconnect power** and open the main door of the YRC1000.
2. Plug in the **two Ethernet connectors** (LAN2 and LAN3).
3. Plug in the **VGA cable** — optional, but useful during initial bring-up to watch
   the controller boot and confirm the MotoPlus application starts.
4. Close the main door.
5. Connect the **programming pendant**.
6. Power up: rotate the knob on the main door to **ON**.

---

## Phase 1 — Boot into Maintenance mode

Maintenance mode is required for the LAN/option-function setup and for initializing
the MotoPlus temporary file.

1. `MAIN MENU > SYSTEM INFO > SECURITY`
2. Select **MANAGEMENT MODE** and enter the management password (see
   [Passwords](#passwords)).
3. `SYSTEM INFO > CPU RESET`
4. Under **Stop Mode**, select **MAINTENANCE MODE**, then **Reset**.

The controller reboots into Maintenance mode.

---

## Phase 2 — Re-enter Management mode (in Maintenance mode)

1. `SYSTEM > SECURITY`
2. Select **MANAGEMENT MODE**, enter the management password, press **ENTER**.

---

## Phase 3 — Network configuration

### 3a. LAN interface

`SYSTEM > SETUP > OPTION FUNCTION > LAN INTERFACE SETTING`

| Setting | Value |
| --- | --- |
| HOST SETTING | **MANUAL SETTING** |
| HOST NAME | your choice (free-form) |
| IP ADDRESS SETTING (LAN2) | **DHCP** |
| IP ADDRESS SETTING (LAN3) | **DHCP** |
| DNS SETTING | source from DHCP (LAN2 / LAN3) |
| SMTP SETTING | source from DHCP |
| DEFAULT GATEWAY | from DHCP, or set manually for your network |

Press **ENTER** (bottom-right of the pendant). Answer **YES** to the "modify?"
confirmation.

### 3b. Network function setting

`SYSTEM > SETUP > OPTION FUNCTION > NETWORK FUNCTION SETTING`

| Setting | Value |
| --- | --- |
| FTP FUNCTION | **EXPANDED** |
| ETHERNET SERVER FUNCTION | **EXPANDED** |

### 3c. MotoPlus function

`SYSTEM > SETUP > OPTION FUNCTION`

| Setting | Value |
| --- | --- |
| MOTOPLUS FUNCTION | **USED** |

> Without this, no MotoPlus application will run, regardless of what you upload.

---

## Phase 4 — Initialize the MotoPlus temporary file

1. `FILE > INITIALIZE`
2. Select **SYSTEM DATA**
3. Navigate down to **MOTOPLUS TEMPORARY FILE**
4. **SELECT**, then **ENTER** (bottom-right)
5. Answer **YES** to initialize

---

## Phase 5 — Reboot to normal mode and verify the network

1. Restart the YRC1000 (power cycle, or `SYSTEM INFO > CPU RESET` back to normal mode).
2. Re-enter Management mode: `SYSTEM INFO > SECURITY > MANAGEMENT MODE` + password.
3. `SYSTEM INFO > NETWORK SERVICE` — confirm the LAN interfaces got IP addresses.

**Write down the LAN IP address.** It is the `host` attribute in the Viam arm config
and the address the firmware flasher connects to.

---

## Phase 6 — Send the robot to its second home position

1. `ROBOT > SECOND HOME POSITION`
2. Release the emergency-stop. And select teach mode (turn the top left key all the way tot the right)
3. **SERVO ON**.
4. Depress the enable (deadman) switch on the back of the pendant — the servos energize.
5. Press and hold **FWD**. The robot moves until it reaches the second home position.
6. When the robot is at zero, you are done.
7. Release the deadman switch and re-engage the emergency-stop.

---

## Phase 7 — Parameters

> **The key switch must be in TEACH** to edit parameters — rotate it fully clockwise,
> lined up with the TEACH marking.
>
> Parameter edits require **Management mode**. Neither `S4C` nor `S2C` changes take
> effect until the controller is restarted — see
> [Reboot after changing parameters](#7c-reboot-after-changing-parameters).

### Navigating to a parameter

`MAIN MENU > PARAMETER > S2C` (or `S4C`) → **SELECT** → in the **Jump To** box enter
the parameter number → **ENTER**.

### 7a. `S4C1084 = 1` — enable MotoPlus online download

`PARAMETER > S4C`, jump to **1084**, change the value from `0` to **`1`**, **ENTER**.

### 7b. S2C parameters — required by the VIAMMOTO firmware

The firmware validates all of these at startup
([`controller_status.c`](https://github.com/viam-modules/yaskawa-controller/blob/main/controller_status.c),
`controller_initialize()`). **A fatal mismatch halts the application** before it can
start — deliberately, because `GP_getInterpolationPeriod()` hangs indefinitely on a
misconfigured controller.

| Parameter | Required value | Severity |
| --- | --- | --- |
| `S2C541` | `0` | non-fatal (warning) |
| `S2C542` | `0` | non-fatal (warning) |
| `S2C1100` | `1` | **fatal** |
| `S2C1102` | `2` | **fatal** |
| `S2C1103` | `2` | **fatal** |
| `S2C1104` | `2` | **fatal** |
| `S2C1119` | `0` **or** `2` | non-fatal (warning) |
| `S2C1250` | `1` | **fatal** |
| `S2C1402` | `3` | **fatal** |

See: <https://github.com/Yaskawa-Global/motoros2#installing-motoros2-out-file> for more info

### How the firmware reports a bad parameter

- **Pendant alarm:** `VIAMMOTO: SET S2C<n> TO <v>`, or `VIAMMOTO: SET S2C<n> TO <v> +<k>`
  when `k` further fatal parameters are also wrong. The alarm text is length-capped, so
  **it names only the first one** — fix it, reboot, and expect the next one if `+k`
  was shown.
- **Logs:** one `CONFIGURATION ERROR: S2C<n> is <x>, must be <y>` line per mismatch,
  listing all of them at once. Non-fatal ones are tagged
  `(non-fatal, behavior may be unexpected)`.

### 7c. Reboot after changing parameters

**Restart the controller after any `S4C` or `S2C` edit.** Both parameter classes are
read at startup and a change is inert until then:

- **`S4C1084`** — the online-download server is brought up at boot. Setting the
  parameter without restarting leaves port 12000 closed, and the first flash still
  fails with `Connection refused`.
- **`S2C`** — the firmware validates them once at startup and
  **halts** on a fatal mismatch, with no in-place recovery path. It will not re-read
  the corrected value until it is restarted.

If you are setting both, do them in one sitting and reboot once.

---

## Phase 8 — Viam side: first firmware flash

1. Install and configure `viam-server` on the machine that will talk to the controller.
2. Add an **arm** component with the appropriate model
   (`viam:yaskawa-robots:gp12`, `:gp35l`, or `:gp180-120`) and the latest module version.
3. Set the `host` attribute to the controller IP from Phase 5.
4. Start `viam-server`.

### First flash

A brand-new controller has no application, so send the DoCommand:

```json
{ "flash_firmware": {} }
```

The module deletes any existing app, uploads the bundled `viammoto.out` over the
MotoPlus online-download protocol (TCP **12000**), and the controller **reboots**. The
module reconnects in the background.

On subsequent starts this is automatic: `flash_on_start` (default **true**) compares the
controller's running build id against the bundled firmware and flashes only on a
mismatch. An explicit `flash_firmware` always flashes, no version gate.

### Flash preconditions (all from the phases above)

- **Servo power OFF** and **HOLD released** on the pendant. Otherwise the flash is
  rejected with `rc=0x2010` and the module tells you to drop servo power.
- MotoPlus temporary file initialized (Phase 4).
- `S4C1084 = 1` (Phase 7a).
- `MOTOPLUS FUNCTION = USED` (Phase 3c).
- Controller firmware **YAS4.12.00-00 or newer**.

### Troubleshooting the first flash

| Symptom | Meaning |
| --- | --- |
| `download failed: connect failed: Connection refused` | The controller answered with a TCP RST — reachable, but **nothing is listening on port 12000**. The online-download server is not up: re-check Phase 4 (MotoPlus temporary file), Phase 7a (`S4C1084=1` **and rebooted since**) and Phase 3c (`MOTOPLUS FUNCTION = USED`). **Nothing was uploaded.** |
| `download failed: connect failed: Operation timed out` | No answer at all — wrong IP, wrong subnet, or a firewall dropping the SYN. |
| `rc=0x2010` in `delete_detail` / `download_detail` | Existing app still active: **turn servo power OFF and release HOLD**, then retry. |
| `rc=0x3902` in `delete_detail` | "Nothing to remove" — benign and expected on a fresh controller. The flash proceeds. |
| `send header failed:` / `send payload failed:` / `no 'DOWNLOAD Start' from controller` | A genuine upload failure, after a successful connect. |
| Pendant alarm `VIAMMOTO: SET S2C…` | The flash worked; the firmware is running and rejecting the parameter set. See Phase 7b. |

A read-only version check, no flashing:

```json
{ "firmware_status": {} }
```

Returns `expected_id` (bundled firmware), `running_id` (controller) and `in_sync`.

---

## Operating requirements (after setup)

- **Remote mode is required** for servo-on and alarm reset
  (`VIAM_ERROR_NOT_REMOTE`). Major alarms cannot be reset remotely — they need the
  pendant.
- **`INIT_ROS.JBI` is auto-deployed** by the firmware when absent, so no pendant job
  load is needed on a fresh controller. This applies only to the default `INIT_ROS`
  job name; a custom `inform_job_name` must be loaded manually.

### Ports

| Port | Protocol | Use |
| --- | --- | --- |
| 12000 | TCP | MotoPlus online download (firmware flashing) |
| 27654 | TCP | VIAMMOTO command/control |
| 27655 | UDP | VIAMMOTO state streaming |
| 21789 | UDP | Log broadcast (remote debugging) |

---

## Passwords

| Mode | Password |
| --- | --- |
| Management mode | a run of `9`s (the controller's default management password) |

Enter the exact password your controller uses — Yaskawa defaults vary by controller
and by how many digits the security screen accepts.

---
