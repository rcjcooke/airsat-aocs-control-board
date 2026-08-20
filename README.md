# airsat-aocs-control-board
University of Surrey AirSat AOCS Control Board

# Todo
- Add voltage monitoring to all voltages
- Change SPI comms with OBC to CAN
  - Use CAN-SU to replicate space usage
  - Overlay with Space Packet for comms protocol (probably)

# Code Style
- We choose to use `#ifndef` with the `AIRSAT_` prefix instead of `#pragma once` for include guards
- All names (variable, function etc.) are defined in camel case
- Class member variables are prefixed with `m_`
- Class member constants are prefixed with `k`
- Global variables are prefixed with `g_`
- Methods called by interrupt service routines are suffixed with `ISR`
- Class member variables are initialised in constructor initialisation strings.