#pragma once
// Copy as include/espnow_secrets.h and fill real values.
// DO NOT commit espnow_secrets.h to Git.

// MAC address of the SmallTV Ultra receiver (6 bytes)
// Find it by reading the serial output of the SmallTV at boot
static const uint8_t ESPNOW_PEER_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// OTA AP credentials for SmallTV (NOT used by the tracker, here for reference)
// const char SMALLTV_AP_PASS[] = "change_me";
