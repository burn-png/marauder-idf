# Building External Apps for Marauder

This guide explains the external-app package format using the supplied
**Snake** app as the reference example.

The example is a Marauder app that declares a VM1 runtime and contains a
hexadecimal `CODE` payload. The manifest fields are readable text; the
`CODE` field is a serialized program payload consumed by the firmware's
app runtime.

> **Important:** This document is based on one working example. It
> documents the fields and conventions visible in that example, but it
> does not claim to define the complete VM1 instruction set, bytecode
> encoding, API surface, or runtime behavior. Those details must be
> confirmed from the firmware's loader, VM implementation, and existing
> app examples before publishing a full SDK.

## 1. App file structure

An app file consists of metadata fields followed by a program payload:

``` ini
MARAUDER_APP=1
API=1
RUNTIME=VM1
NAME=Snake
CATEGORY=Games
VERSION=1.0
DESCRIPTION=Joystick snake. Eat red, avoid walls and yourself. Center restarts.
CODE=<hexadecimal-program-payload>
```

Use one `KEY=VALUE` entry per line. Keep the metadata at the beginning
of the file and put `CODE` last, since the payload may be very long.

## 2. Manifest fields

  -----------------------------------------------------------------------------------------------------------------------
  Field                   Example                                                                 Meaning
  ----------------------- ----------------------------------------------------------------------- -----------------------
  `MARAUDER_APP`          `1`                                                                     Identifies the file as
                                                                                                  a Marauder app package.

  `API`                   `1`                                                                     Declares the app API
                                                                                                  version.

  `RUNTIME`               `VM1`                                                                   Selects the runtime
                                                                                                  expected to execute the
                                                                                                  app.

  `NAME`                  `Snake`                                                                 Human-readable app
                                                                                                  name.

  `CATEGORY`              `Games`                                                                 App category shown to
                                                                                                  users or the app
                                                                                                  manager.

  `VERSION`               `1.0`                                                                   App version string.

  `DESCRIPTION`           `Joystick snake. Eat red, avoid walls and yourself. Center restarts.`   Short description of
                                                                                                  the app and its
                                                                                                  controls.

  `CODE`                  Hexadecimal data                                                        Serialized program
                                                                                                  payload for the
                                                                                                  declared runtime.
  -----------------------------------------------------------------------------------------------------------------------

### Metadata recommendations

-   Keep `NAME` short enough to fit the device's app list and title
    area.
-   Use a concise `DESCRIPTION` that explains the app's purpose and
    important controls.
-   Increment `VERSION` when publishing a meaningful app update.
-   Use the API version supported by the target firmware. Do not assume
    that `API=1` will remain compatible with later API versions.
-   Use a runtime identifier supported by the target firmware. The
    supplied example uses `VM1`; do not invent other runtime names.
-   Keep category names consistent with the categories recognized by the
    firmware.

The example establishes these field names and sample values. It does
**not** establish which fields are optional, whether unknown fields are
rejected, or the complete list of supported categories.

## 3. The `CODE` payload

`CODE` contains a long hexadecimal string. In the Snake example, it is
the program data interpreted by the `VM1` runtime.

A few important distinctions:

-   `CODE` is not ordinary C++, Arduino code, Python, or source code.
-   The hexadecimal text is an encoded representation of program data;
    it should not be edited by hand.
-   The example's payload includes visible text fragments such as
    `SCORE`, `HOLD CENTER = EXIT`, `GAME OVER`, and `YOU WIN!`. These
    strings are embedded in the payload, but the example alone is not
    enough to derive the complete bytecode format.
-   A valid-looking hex string is not necessarily a valid program. The
    runtime may expect specific opcodes, operand formats, memory
    layouts, or API calls.

**Do not generate or modify VM1 bytecode by guessing.** Use the
project's actual compiler, assembler, app-authoring tool, or documented
bytecode format once identified.

## 4. App behavior and controls

The supplied Snake example describes the following behavior:

-   The joystick controls the snake.
-   The player eats red items to score or progress.
-   Hitting a wall or the snake itself ends the run.
-   Pressing the center control restarts the game.
-   The embedded text also indicates a center-button exit instruction, a
    game-over state, and a win state.

These are behaviors of the example app, not mandatory requirements for
every external app.

When designing an app, document its controls in `DESCRIPTION` where
practical. If the app has more controls than fit in one line, include a
separate README with the package.

## 5. Creating an app package

1.  **Inspect the firmware's app loader.** Confirm the expected file
    extension, package location, parser rules, and whether the loader
    accepts external files.
2.  **Confirm runtime and API compatibility.** Check which `RUNTIME` and
    `API` values the target firmware supports.
3.  **Build the program payload.** Use the project's supported VM1
    toolchain or authoring workflow. Do not hand-edit the hex payload.
4.  **Write the manifest.** Fill in the metadata fields using the format
    shown above.
5.  **Test on the target firmware.** Verify that the app is discovered,
    launches, responds to controls, exits cleanly, and handles errors
    without destabilizing the device.
6.  **Publish the package and documentation.** Include the app file, a
    short description, controls, version, and the firmware/API
    compatibility you tested.

## 6. Minimal manifest template

Replace the placeholder values with your app's metadata and a payload
produced by the supported toolchain.

``` ini
MARAUDER_APP=1
API=1
RUNTIME=VM1
NAME=My App
CATEGORY=Utilities
VERSION=1.0
DESCRIPTION=Briefly explain what the app does and how to control it.
CODE=<HEX_PAYLOAD_FROM_SUPPORTED_TOOLCHAIN>
```

`<HEX_PAYLOAD_FROM_SUPPORTED_TOOLCHAIN>` is a placeholder, not literal
package content. Replace it with the actual hexadecimal payload
generated for the selected runtime.

## 7. Validation checklist

Before sharing an app, check:

-   [ ] The package begins with the expected `MARAUDER_APP` marker.
-   [ ] The API and runtime identifiers are supported by the target
    firmware.
-   [ ] The name, category, version, and description are present and
    readable.
-   [ ] The `CODE` payload was generated by the correct toolchain.
-   [ ] The payload is complete and has not been truncated during
    copying or transfer.
-   [ ] The app launches and can be exited normally.
-   [ ] Inputs and controls behave as documented.
-   [ ] The app has been tested on the firmware version listed in its
    release notes.

## 8. Information still needed for a complete SDK

The supplied Snake package does not reveal enough to document these
areas accurately:

-   The complete VM1 instruction/opcode set and operand encoding.
-   The source language or compiler/assembler used to create `CODE`.
-   Available display, drawing, input, storage, timing, and system APIs.
-   App lifecycle rules, including initialization, update, and exit
    behavior.
-   Memory, execution-time, and payload-size limits.
-   Exact package extension, install path, and parser error behavior.
-   Whether apps can use firmware APIs beyond the VM1 runtime.

These should be documented from the firmware source or official
toolchain before third-party developers are told to rely on them.

------------------------------------------------------------------------

## Reference example

The Snake app supplied with this guide is the reference for the package
layout:

``` ini
MARAUDER_APP=1
API=1
RUNTIME=VM1
NAME=Snake
CATEGORY=Games
VERSION=1.0
DESCRIPTION=Joystick snake. Eat red, avoid walls and yourself. Center restarts.
CODE=<OMITTED: use the original complete payload>
```

The original `CODE` payload is intentionally not duplicated here. Keep
the original package as the byte-for-byte reference when testing parsers
or tooling.
