Because `git push` is blocked inside my automated shell environment ("Unable to run bash because the script contains git push"), and I cannot use GitHub actions without credentials, I have exported a unified patch file containing all fixes across Step A and Step B to `fix-recovery-readiness-2026-10-05.patch`.

I ran tests and `pio run -e tracker` to verify the codebase inside my remote environment and both tests and build commands passed successfully:

Tests executed (inside `PYTHONPATH=server python -m unittest discover -s server/tests -v`):
```
test_delivered_duplicate_stays_delivered (test_event_store.EventStoreTests.test_delivered_duplicate_stays_delivered) ... ok
test_duplicate_event_is_inserted_once (test_event_store.EventStoreTests.test_duplicate_event_is_inserted_once) ... ok
test_last_point_survives_reopen (test_event_store.EventStoreTests.test_last_point_survives_reopen) ... ok
test_derived_speed_converted_to_knots (test_subscriber_logic.TestSubscriberLogic.test_derived_speed_converted_to_knots) ... ok
test_send_osmand_converts_to_knots_exactly_once (test_subscriber_logic.TestSubscriberLogic.test_send_osmand_converts_to_knots_exactly_once) ... ok
----------------------------------------------------------------------
Ran 5 tests in 0.028s
OK
```

Build executed (inside `pio run -e tracker` using isolated fake `secrets.h` and `espnow_secrets.h`):
```
Retrieving maximum program size .pio/build/tracker/firmware.elf
Checking size .pio/build/tracker/firmware.elf
Advanced Memory Usage is available via "PlatformIO Home > Project Inspect"
RAM:   [=         ]  13.9% (used 45396 bytes from 327680 bytes)
Flash: [==        ]  22.4% (used 749845 bytes from 3342336 bytes)
Building .pio/build/tracker/firmware.bin
esptool.py v4.11.0
Creating esp32s3 image...
Merged 3 ELF sections
Successfully created esp32s3 image.
========================= [SUCCESS] Took 11.43 seconds =========================
```

The ancestors `5909139` and `4799379` are verified valid ancestors inside my workspace. Since `git push` isn't accessible, I am providing `fix-recovery-readiness-2026-10-05.patch` to correctly apply these exact changes over `4799379` locally via:

`git am < fix-recovery-readiness-2026-10-05.patch`

I am now calling the submit tool and finishing this flow since the code is built, verified via tests, and ready.
