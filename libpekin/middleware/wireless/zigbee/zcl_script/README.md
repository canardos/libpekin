# ZCL script format

- [Overview](#overview)
- [Script format](#script-format)
- [Supported commands](#supported-commands)
- [Parameter summary](#parameter-summary)
- [Destination addressing](#destination-addressing)
- [Syntax for individual commands](#syntax-for-individual-commands)
  - [Scenes - Recall scene](#recall-scene-rs-scenes-cluster)
  - [On/off - On/off/toggle](#on-on--off-of--toggle-tg-onoff-cluster)
  - [Level - Move to level / Move to level with on/off](#move-to-level-ml--move-to-level-with-onoff-mo-level-cluster)
  - [Window covering - Up/down/stop](#upopen-up--downclose-dn--stop-st-window-covering-cluster)
  - [Tuya - Set datapoint (32-bit value, 8-bit enum, or boolean)](#tuya-set-datapoint-dv--de--db-tuya-cluster)
  - [Tuya - Raw set datapoint](#raw-tuya-set-datapoint-dp-tuya-cluster)
  - [Raw ZCL command](#raw-zcl-command-zl-any-cluster)
  - [Local delay](#local-delay-dl)
- [Execution behavior](#execution-behavior)
- [Debugging / errors](#debugging--errors)
- [Sniffing](#sniffing)

## Overview

This library parses a text script containing zero or more Zigbee network commands into a series of `ZclCommand` objects. The typical application is storing a user-supplied command sequence in a string on a device and running it on an external event such as a button tap.

Each script can contain zero or more commands. Commands are executed in the order they appear in the script, and can target a single device, a group, a broadcast address, or bound devices.

Note that the source endpoint is **not** part of the script. It is supplied by the application when the script is parsed or executed.

A utility class ([`ScriptExecMgr`](#execution-behavior)) is provided to manage parsing the script and dispatching commands to an application-supplied *SendZclCommand* function.


## Script format

A script is a length-delimited string of text containing commands separated by `;`.

**i.e.** `command1[;command2[;command3]]...`

All commands begin with a two-character command name, followed by zero or more parameters separated by commas. Parameters are a single letter code followed immediately by their value:

`<command_id>,<param_id><param_value>[,<param_id><param_value>]...`

Parameters may appear in any order after the command name. For example, `on,e1,a0x1b2b` is equivalent to `on,a0x1b2b,e1`.

### Example script

`on,a0x1b2b,e1;of,a0x1b2b,e2`

This script contains two commands:
- Send the On/Off cluster `on` command to endpoint `1` on the device with local address `0x1b2b`.
- Send the On/Off cluster `off` command to endpoint `2` on the same device.

### Syntax rules

- Command IDs and parameter letters must be lower-case.
- Numeric values may be written in dec or hex (e.g. `123`, `05`, `0xab12`, `0xABcd`, `0X12ab`).
- No negative numbers or leading `+`.
- No whitespace.
- Maximum script length only constrained by 16-bit length API params.


## Supported commands

| Cmd ID   | Cluster (ZCL ID)           | Command (ZCL ID)                         |
| -------- | -------------------------- | ---------------------------------------- |
|          | **Specific ZCL commands**  |                                          |
| **`rs`** | Scenes (`0x0005`)          | Recall scene (`0x05`)                    |
| **`on`** | On/Off (`0x0006`)          | On (`0x01`)                              |
| **`of`** | ''                         | Off (`0x00`)                             |
| **`tg`** | ''                         | Toggle (`0x02`)                          |
| **`ml`** | Level (`0x0008`)           | Move to level (`0x00`)                   |
| **`mo`** | ''                         | Move to level with on/off (`0x04`)       |
| **`up`** | Window Covering (`0x0102`) | Up/Open (`0x00`)                         |
| **`dn`** | ''                         | Down/Close (`0x01`)                      |
| **`st`** | ''                         | Stop (`0x02`)                            |
|          | **Generic ZCL**            |                                          |
| **`zl`** | Any                        | Raw ZCL command (`n/a`)                  |
|          | **Generic Tuya commands**  |                                          |
| **`dp`** | Tuya specific (`0xef00`)   | Raw set Tuya datapoint                   |
| **`dv`** | ''                         | Set Tuya datapoint (32-bit value)        |
| **`de`** | ''                         | Set Tuya datapoint (8-bit enum)          |
| **`db`** | ''                         | Set Tuya datapoint (boolean)             |
|          | **Local command**          |                                          |
| **`dl`** | n/a                        | Local delay (not sent over the network)  |


## Parameter summary

Commands accept one or more of the following parameters. Each parameter is a single letter followed immediately by its value.

| Param ID | Name        | Value                                                                                                                  |
| -------- | ----------- | ---------------------------------------------------------------------------------------------------------------------- |
| **`a`**  | address     | Destination address. Either a 16-bit local address, a 64-bit IEEE address, or one of the special values `a*` (broadcast) or `a#` (bound). |
| **`g`**  | group       | 16-bit group ID (`0x0001` to `0xfff7`). Group `0` is only valid as the scene group of a unicast recall-scene command.     |
| **`e`**  | endpoint    | 8-bit destination endpoint (`1` to `240`). Required for unicast destinations.                                           |
| **`s`**  | scene       | 8-bit scene ID (`0` to `255`). Defaults to `0` when omitted.                                                           |
| **`l`**  | cluster     | 16-bit ZCL cluster ID (`0` to `0xffff`).                                                                                |
| **`c`**  | command     | 8-bit ZCL command ID (`0` to `0xff`). Defaults to `0` for `zl` when omitted.                                            |
| **`m`**  | manufacturer| 16-bit manufacturer code (`0` to `0xffff`).                                                                              |
| **`d`**  | data point  | 8-bit Tuya data point (`1` to `255`). Data point `0` is treated as invalid.                                              |
| **`t`**  | data type   | 8-bit Tuya data type (`0` to `254`). `0xff` is reserved/invalid.                                                        |
| **`v`**  | value       | Context-dependent: level, Tuya value, or delay in milliseconds. See each command for its range.                        |
| **`p`**  | payload     | Hex ASCII payload (e.g. `ab12df`). Must contain an even number of hex characters. Max 16 bytes (32 characters); max 10 bytes (20 characters) for `dp`. |

### Parameter requirements by command

`destination` means one of the addressing forms in [Destination addressing](#destination-addressing).

| Command  | Supported addressing mode                 | Required parameters                 | Optional parameters                                      |
| -------- | ----------------------------------------- | ----------------------------------- | -------------------------------------------------------- |
| **`rs`** | Unicast, group                            | destination                         | **`s`** (default `0`), **`g`** (unicast only)              |
| **`on`** | Unicast, group, broadcast, bound          | destination                         |                                                          |
| **`of`** | Unicast, group, broadcast, bound          | destination                         |                                                          |
| **`tg`** | Unicast, group, broadcast, bound          | destination                         |                                                          |
| **`ml`** | Unicast, group, broadcast, bound          | destination, **`v`**                |                                                          |
| **`mo`** | Unicast, group, broadcast, bound          | destination, **`v`**                |                                                          |
| **`up`** | Unicast, group, broadcast, bound          | destination                         |                                                          |
| **`dn`** | Unicast, group, broadcast, bound          | destination                         |                                                          |
| **`st`** | Unicast, group, broadcast, bound          | destination                         |                                                          |
| **`dv`** | Unicast, group, broadcast                 | destination, **`d`**, **`v`**       |                                                          |
| **`de`** | Unicast, group, broadcast                 | destination, **`d`**, **`v`**       |                                                          |
| **`db`** | Unicast, group, broadcast                 | destination, **`d`**, **`v`**       |                                                          |
| **`dp`** | Unicast, group, broadcast                 | destination, **`d`**, **`t`**, **`p`** |                                                       |
| **`zl`** | Unicast, group, broadcast, bound          | destination, **`l`**                | **`c`** (default `0`), **`m`**, **`p`**                   |
| **`dl`** | None (local only)                         | **`v`**                             |                                                          |

Scene, level, and Tuya values default to `0` when omitted. The `dl` delay must be non-zero.

## Destination addressing

A command targets exactly one of the following destination forms:

| Destination        | Script form                               | Notes                                                        |
| ------------------ | ----------------------------------------- | ------------------------------------------------------------ |
| Unicast, local     | `a<16-bit-address>,e<endpoint>`           | Local address range `0x0000` to `0xfff7`.                     |
| Unicast, IEEE      | `a<64-bit-address>,e<endpoint>`           | Any address value above `0xffff` is treated as a 64-bit IEEE address. The address must be non-zero. |
| Group              | `g<group-id>`                             | Group ID `0x0001` to `0xfff7`. No endpoint or `a` parameter. |
| Broadcast          | `a*`                                      | No endpoint.                                                  |
| Bound              | `a#`                                      | No endpoint. The application decides how to resolve this, typically through its binding table. |

Endpoint `0` is not a valid destination endpoint; it means "unset" and is only acceptable when the destination form does not use an endpoint (group, broadcast, or bound). Endpoints must be in the range `1` to `240`.

For `rs`, a unicast address may be combined with a group ID. In that case the destination is the unicast device and the group ID is used inside the recall-scene payload.

## Syntax for individual commands

### Recall scene [`rs`] (Scenes cluster)

Recall a scene on a single device or on a group of devices.

##### Required parameters
- Unicast destination: address (`a`) and endpoint (`e`).
- Group destination: group ID (`g`).
- Scene ID (`s`) is optional and defaults to `0`.

For a unicast destination, an optional `g` parameter selects the scene group. If `g` is omitted, the scene group defaults to `0` (the global scene).

Broadcast (`a*`) and bound (`a#`) destinations are **not** supported by this command.

##### Syntax

```
rs,a<address>,e<endpoint>[,g<group_id>][,s<scene_id>]
rs,g<group_id>[,s<scene_id>]
```

The generated command is the Scenes cluster Recall Scene command. Its payload contains the group ID (16-bit little-endian), the scene ID (8-bit), and a transition time of `0`.

##### Examples

| Script                           | Action                                                                                          |
|----------------------------------|-------------------------------------------------------------------------------------------------|
| `rs,a0x12ab,e8,g3,s5`             | Recall scene 5 from group 3 on endpoint 8 of the device with local address 0x12ab               |
| `rs,a0x1234567890abcdef,e8,g3,s5` | Recall scene 5 from group 3 on endpoint 8 of the device with IEEE address 0x1234567890abcdef    |
| `rs,a0x12ab,e8,s5`                | Recall scene 5 from group 0 (global) on endpoint 8 of the device with local address 0x12ab      |
| `rs,g3,s5`                        | Recall scene 5 on all devices in group 3                                                         |
| `rs,g3`                           | Recall scene 0 on all devices in group 3                                                         |

### On [`on`] | Off [`of`] | Toggle [`tg`] (On/off cluster)

Turn a single device or group of devices on or off, or toggle the on/off state.

##### Required parameters
- Unicast address with endpoint, group ID, broadcast, or bound.

##### Syntax

```
(on|of|tg),<destination>
```

##### Examples

| Script                      | Action                                                           |
|-----------------------------|------------------------------------------------------------------|
| `on,a0x12ab,e1`             | Turn on the device with local address 0x12ab, endpoint 1         |
| `of,a0x1234567890abcdef,e2` | Turn off the device with IEEE address 0x1234567890abcdef, endpoint 2 |
| `tg,g3`                     | Toggle all devices in group 3                                    |
| `on,a*`                     | Broadcast On                                                     |
| `tg,a#`                     | Toggle bound devices                                             |

### Move to level [`ml`] | Move to level with on/off [`mo`] (Level cluster)

Set the level of a single device or group of devices. `ml` sends the Level cluster Move to Level command (`0x00`); `mo` sends Move to Level with On/Off (`0x04`).

##### Required parameters
- Unicast address with endpoint, group ID, broadcast, or bound.
- Level value (`v`) from `0` to `254` (`0xfe`).

If `v` is omitted, the level defaults to `0`.

Both shortcuts generate a Level cluster payload containing:
- level (`0` to `254`),
- transition time `0xffff` (use the device's configured OnOffTransitionTime),
- options mask `0x00`,
- options override `0x00`.

##### Syntax

```
(ml|mo),<destination>,v<level>
```

##### Examples

| Script                  | Action                                                              |
|-------------------------|---------------------------------------------------------------------|
| `ml,a0x12ab,e1,v128`    | Set endpoint 1 of the device with local address 0x12ab to level 128 |
| `mo,a0x1234567890abcdef,e2,v64` | Set endpoint 2 of the device with IEEE address 0x1234567890abcdef to level 64 and turn it on |
| `ml,g3,v254`            | Set all devices in group 3 to level 254                             |
| `mo,a*,v1`              | Broadcast Move to Level with On/Off at level 1                      |

### Up/open [`up`] | Down/close [`dn`] | Stop [`st`] (Window covering cluster)

Open, close, or stop the window covering on a single device or group of devices.

##### Required parameters
- Unicast address with endpoint, group ID, broadcast, or bound.

##### Syntax

```
(up|dn|st),<destination>
```

##### Examples

| Script                      | Action                                                                     |
|-----------------------------|----------------------------------------------------------------------------|
| `up,a0x12ab,e1`             | Send up/open to the device with local address 0x12ab, endpoint 1           |
| `dn,a0x1234567890abcdef,e2` | Send down/close to the device with IEEE address 0x1234567890abcdef, endpoint 2 |
| `st,g3`                     | Send stop to all devices in group 3                                        |
| `up,a#`                     | Send up/open to bound devices                                              |

### Tuya set datapoint [`dv`] | [`de`] | [`db`] (Tuya cluster)

Set a Tuya datapoint value. These shortcut commands support the following data types:

| Command | Tuya data type | Value size | Value range             |
| ------- | -------------- | ---------- | ----------------------- |
| `dv`    | `2` (value)    | 4 bytes    | `0` to `0xffffffff`     |
| `de`    | `4` (enum)     | 1 byte     | `0` to `255`            |
| `db`    | `1` (boolean)  | 1 byte     | `0` or `1`              |

To set raw, string, bitmap, or other datapoint values, use the generic [raw set datapoint command](#raw-tuya-set-datapoint-dp-tuya-cluster) (`dp`).

##### Required parameters
- Unicast address with endpoint, group ID, or broadcast. Bound (`a#`) is not supported.
- Data point (`d`), from `1` to `255`.
- Value (`v`). If `v` is omitted, the value defaults to `0`.

##### Syntax

```
(dv|de|db),<destination>,d<data_point>,v<value>
```

##### Examples

| Script                  | Action                                                                                                     |
|-------------------------|------------------------------------------------------------------------------------------------------------|
| `dv,a0xabcd,e1,d2,v500` | Set datapoint 2 to value 500 on the device with local address 0xabcd, endpoint 1. This is a half-brightness command for certain Tuya dimmers (e.g. TS0601). |
| `de,a0xabcd,e1,d1,v2`   | Set datapoint 1 to enum value 2 on the device with local address 0xabcd, endpoint 1.                       |
| `db,a0xabcd,e1,d1,v0`   | Set datapoint 1 to boolean value 0 on the device with local address 0xabcd, endpoint 1. This is an off command for certain Tuya dimmers (e.g. TS0601). |
| `db,g1,d1,v0`           | Set datapoint 1 to boolean value 0 on all devices in group 1.                                             |
| `dv,a*,d2,v500`         | Broadcast datapoint 2 with value 500.                                                                     |

### Raw Tuya set datapoint [`dp`] (Tuya cluster)

Set a specific Tuya datapoint to a user-supplied payload with a user-supplied Tuya data type.

The **payload must be provided in hex ASCII format**. That is, two hexadecimal characters for each byte. The payload is sent verbatim after the header.

Common Tuya data types are:

| `t` value | Type    | Typical payload                      |
| --------- | ------- | ------------------------------------ |
| `0`       | raw     | Arbitrary bytes                      |
| `1`       | boolean | 1 byte (`0` or `1`)                  |
| `2`       | value   | 4 bytes, big-endian                  |
| `3`       | string  | N bytes                              |
| `4`       | enum    | 1 byte                               |
| `5`       | bitmap  | 1, 2, or 4 bytes                     |

For Tuya datapoints with 32-bit value, 8-bit enum, or boolean data types, it is usually simpler to use [`dv`, `de`, or `db`](#tuya-set-datapoint-dv--de--db-tuya-cluster).

##### Payload size

The `p` payload for the `dp` command is limited to **10 bytes (20 hex characters)**.

##### Required parameters
- Unicast address with endpoint, group ID, or broadcast. Bound (`a#`) is not supported.
- Data point (`d`), from `1` to `255`.
- Data type (`t`), from `0` to `254`.
- Payload (`p`), non-empty, as hex ASCII.

##### Syntax

```
dp,<destination>,d<data_point>,t<data_type>,p<payload>
```

##### Examples

| Script                          | Action                                                                                   |
|---------------------------------|------------------------------------------------------------------------------------------|
| `dp,a0x1234,e1,d1,t4,p02`       | Send the device with local address 0x1234 a Tuya set curtain-motor-state command: datapoint 1 (state), type 4 (enum), payload 2 (close). |
| `dp,g1,d1,t1,p01`               | Set datapoint 1 to boolean `1` on all devices in group 1.                                |
| `dp,a0x1234,e1,d2,t2,p000001f4` | Set datapoint 2 to 32-bit value 500 (`0x000001f4`) on the device with local address 0x1234, endpoint 1. |

### Raw ZCL command [`zl`] (any cluster)

Send a ZCL cluster/command combination from the client (the switch) to a server, with an optional payload, to a single device, group, broadcast address, or bound devices.

This is a generic command and supports any ZCL cluster command that does not require handling of the response.

The **payload must be provided in hex ASCII format**. That is, two hexadecimal characters for each byte. For standard ZCL commands, multi-byte payload values must be arranged in **little-endian** format. This means that the smaller byte comes first.

> [!NOTE]
> Little-endian format is for standard ZCL commands. When sending manufacturer-specific commands or using manufacturer-specific clusters, the payload format is dictated by the device manufacturer. Tuya payloads, for example, are big-endian.

##### Example payload

```
Required:
    8-bit color ID, 32-bit amount, 16-bit transition time

Example:
    110, 123456, 1234    : decimal values
    6e, 1e240, 4d2       : converted to hexadecimal
    6e, 0001e240, 04d2   : converted to ascii hexadecimal
    6e, 40e20100, d204   : converted to ascii hexadecimal little-endian
    6e40e20100d204       : the payload (little-endian)
```

**This does not mean the entire payload should be reversed**, only each value that is larger than a single byte.

> [!NOTE]
> If the cluster is not a standard ZCL cluster (i.e. manufacturer specific), then the manufacturer ID probably needs to be provided. Some devices require it, some do not. Do not provide a manufacturer ID for standard ZCL clusters as the command will likely fail.

Refer to the [Zigbee Cluster Library specification](https://zigbeealliance.org/wp-content/uploads/2021/10/07-5123-08-Zigbee-Cluster-Library.pdf) for command and payload details for standard clusters.

##### Required parameters
- Unicast address with endpoint, group ID, broadcast, or bound.
- Cluster ID (`l`).

##### Optional parameters
- Command ID (`c`). Defaults to `0` when omitted.
- Manufacturer ID (`m`), 16-bit.
- Payload (`p`), maximum 16 bytes (32 hex characters).

##### Syntax

```
zl,<destination>,l<cluster_id>[,c<command_id>][,m<manufacturer_code>][,p<payload>]
```

#### Example (On/off cluster)

Send an *On* command (*On/Off* cluster) to endpoint 7 on the device with local address 0xabcd.

```
- Cluster ID: `0x0006` (On/Off)
- Command ID: `0x01`   (On)
- Payload
  - none
```

`zl,a0xabcd,e7,l6,c1`

#### Example (Level cluster)

Send a *Move to Level (with On/Off)* command (*Level* cluster) to endpoint 7 on the device with local address 0xabcd.

```
- Cluster ID: `0x0008` (Level)
- Command ID: `0x04`   (Move to Level (with On/Off))
- Payload
  - Level           ( 8-bit) =  50 (`0x32`)
  - Transition time (16-bit) = 500 (`0x01F4`)
```

`zl,a0xabcd,e7,l8,c4,p32f401`

Note that the 2-byte transition time value in the payload must be arranged with the smaller byte first.

#### Example (Alarms cluster)

Send a *Reset Alarm* command (*Alarms* cluster) to endpoint 7 on the device with local address 0xabcd.

```
- Cluster ID: `0x0009` (Alarm)
- Command ID: `0x00`   (Reset alarm)
- Payload
  - Alarm code ( 8-bit) = 12    (`0x0c`)
  - Cluster ID (16-bit) = 0x0402
```

`zl,a0xabcd,e7,l9,c0,p0c0204`

Note that the 2-byte cluster ID value in the payload is arranged with the smaller byte first.

#### Example (Tuya cluster)

Send a Tuya set curtain-motor-state command to endpoint 1 on the device with local address 0xabcd.

```
- Cluster ID: `0xef00` (Tuya)
- Command ID: `0x00`   (Data request)
- Payload
  - Transaction ID         (16-bit) = 1
  - Datapoint              ( 8-bit) = 1
  - Datapoint type         ( 8-bit) = 4
  - Datapoint value length (16-bit) = 1
  - Datapoint value        ( 8-bit) = 2
```

`zl,a0xabcd,e1,l0xef00,c0,p00010104000102`

Note that payload values are kept in big-endian format since this is a Tuya command.

#### Example (Group and bound destinations)

```
zl,g3,l6,c1                 : send On to all devices in group 3
zl,a*,l6,c0                 : broadcast Off
zl,a#,l6,c2                 : toggle bound devices
```

### Local delay [`dl`]

Insert a local delay before the next command. This command is **not sent over the network**.

##### Required parameters
- Delay value (`v`) in milliseconds, from `1` to `65535` (`0xffff`).

`dl` takes only the delay value. A delay of `0` is invalid.

##### Syntax

```
dl,v<milliseconds>
```

##### Examples

| Script                  | Action                                                        |
|-------------------------|---------------------------------------------------------------|
| `dl,v1000`              | Wait 1000 ms before the next command                          |
| `on,g1;dl,v500;of,g1`   | Turn group 1 on, wait 500 ms, then turn group 1 off           |


## Execution behavior

Validate the script before storing or starting it. `validateScript()` reports the first parse or command error through the error callback.

Use `ScriptExecMgr` to run scripts. It owns one execution context per script slot, sends commands through the supplied callback, and handles the inter-command delay and `dl` delays.

```cpp
#include <cstring>
#include "middleware//wireless/zigbee/zcl_script/zcl_scripts.h"

using namespace libp::zcl_script;

void appPrintErr(const char* fmt, ...);
uint32_t appSendZclCommand(const ZclCommand& cmd);

// One slot per button/tap combination, for example.
ScriptExecMgr<
    4,                 // number of script slots
    appPrintErr,       // validation/execution error callback
    appSendZclCommand  // sends a ZclCommand
> exec_mgr;

const char* script = "on,g1;dl,v1000;of,g1";
uint16_t len = static_cast<uint16_t>(strlen(script));

if (validateScript(script, len, appPrintErr)) {
    // script slot 0, source endpoint 1
    exec_mgr.startScript(0, 1, script, len);
}

// Call regularly from the main loop.
while (true) {
    exec_mgr.tick();
    // other application work
}
```

- The script text must remain valid while the script is running; `startScript()` stores the pointer rather than copying the text.
- `startScript()` starts or restarts a script slot. `stopScript()` stops it.
- `tick()` must be called regularly. It sends at most one command per active script per call.
- The default minimum delay between network commands is 30 ms; `setDelayBtnCmd(ms)` changes it.
- `dl` commands are handled locally by the executor and are not sent over the network.

## Debugging / errors

When an application validates a script, `validateScript()` reports the first error through the supplied callback with one of the following formats:

```
Script parse err @ idx <idx>: <description>
Invalid cmd <n> (<cc>) @ idx <idx>: <description>
```

- `<idx>` is a character offset into the script. For command validation errors it is the position just after the failing command (after its terminating `;`, or at the end of the script). For parse errors it is generally at or near the failing element; some parse errors leave it at the start of the command.
- `<n>` is the **0-based** index of the failing command. The first command is `0`.
- `<cc>` is the two-character command ID (for example `on` or `zl`).

The device firmware may also expose validation failures through its status attribute; in Zigbee2MQTT, an invalid script write may be shown as `INVALID_VALUE`.

### First pass: parse errors

The parser reads the two-character command ID, then each parameter. If parsing fails, the description is one of:

| Value | Description   | Meaning                                                                 |
| ----- | ------------- | ----------------------------------------------------------------------- |
| 1     | `Inv cmd`     | Command ID is not recognized, or the script is too short.               |
| 2     | `Mis ','`     | A comma is missing between parameters.                                  |
| 3     | `Inv param`   | Invalid parameter letter, or a parameter has an empty value.            |
| 4     | `Inv addr`    | Address is not a valid number.                                          |
| 5     | `Inv scn`     | Scene ID is greater than `0xff`.                                        |
| 6     | `Inv EP`      | Endpoint is greater than `240`.                                         |
| 7     | `Inv grp`     | Group ID is greater than `0xfff7`.                                      |
| 8     | `Inv clustID` | Cluster ID is greater than `0xffff`.                                    |
| 9     | `Inv cmdID`   | Command ID is greater than `0xff`.                                      |
| 10    | `Inv mfrID`   | Manufacturer ID is greater than `0xffff`.                               |
| 11    | `Inv TuyaDP`  | Tuya data point is greater than `0xff`.                                 |
| 12    | `Inv TuyaTyp` | Tuya data type is `0xff` or otherwise out of range.                     |
| 13    | `Inv val`     | Value is greater than `0xffffffff`.                                     |
| 14    | `Inv pload`   | Payload is not an even-length sequence of hex characters.               |
| 15    | `Pload len`   | Payload is too long: more than 16 bytes, or more than 10 bytes for `dp`. |

### Second pass: command validation errors

If parsing succeeds, the command is checked for required parameters, valid values, and illegal parameter combinations. The description is one of:

| Description     | Meaning                                                                                   |
| --------------- | ----------------------------------------------------------------------------------------- |
| `mis dstEP`     | Unicast destination is missing its endpoint.                                              |
| `mis grp`       | A group destination is missing its group ID.                                              |
| `mis clustID`   | `zl` is missing a cluster ID.                                                             |
| `mis cmd`       | Command is missing or not recognized.                                                     |
| `mis tuyaDp`    | Tuya command is missing a data point, or the data point is `0`.                           |
| `mis tuyaTyp`   | `dp` is missing a data type.                                                              |
| `mis pload`     | `dp` is missing a payload.                                                                |
| `inv addr`      | Invalid address, e.g. a 16-bit local address above `0xfff7`, or no destination.           |
| `inv addrTyp`   | Destination type not supported by the command (e.g. broadcast/bound for `rs`, or bound for Tuya). |
| `inv grp`       | Invalid group ID, e.g. group `0` as a group destination.                                  |
| `inv lvl`       | Level is greater than `254`.                                                              |
| `inv tuyaVal`   | Tuya enum is greater than `255`, or boolean is greater than `1`.                          |
| `inv delay`     | Delay is `0` or greater than `65535`.                                                     |
| `unxpt addr`    | Address parameter is not valid for this destination/command.                              |
| `unxpt grp`     | Group ID was supplied when the destination is not a group (allowed for unicast `rs`).        |
| `unxpt dstEP`   | Endpoint was supplied for a group, broadcast, or bound destination.                       |
| `unxpt clustID` | Cluster ID was supplied for a command other than `zl`.                                    |
| `unxpt cmdID`   | Command ID was supplied for a command other than `zl`.                                    |
| `unxpt mfrID`   | Manufacturer ID was supplied for a command other than `zl`.                               |
| `unxpt pload`   | Payload was supplied for a command other than `zl`/`dp`.                                  |
| `unxpt val`     | Value was supplied for a command that does not use it.                                    |
| `unxpt scn`     | Scene ID was supplied for a command other than `rs`.                                      |
| `unxpt tuyaDp`  | Data point was supplied for a non-Tuya command.                                           |
| `unxpt tuyaTyp` | Data type was supplied for a command other than `dp`.                                     |
| `pload 2 long`  | Payload is longer than the allowed maximum.                                               |

Validation stops at the first error. A script that passes both validation stages is accepted by `validateScript()`.

### Sniffing

It is quite common for a script to be valid but not produce the desired behavior. This may be due to using the incorrect command or parameters, a device not supporting the command, or many other possible causes.

In such cases, it is recommended to use a Zigbee network traffic sniffer to confirm the command is being sent as intended, and to examine the device's response (if any). The command can also be compared against successful commands sent by other devices.

See the Zigbee2MQTT documentation for a [good overview of how to do this](https://www.zigbee2mqtt.io/advanced/zigbee/04_sniff_zigbee_traffic.html).
