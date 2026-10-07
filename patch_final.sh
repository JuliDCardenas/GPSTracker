sed -i 's/#define PARKED_PULSE_S    (24UL \* 3600UL)/#define PARKED_PULSE_S    (8UL \* 3600UL)/' src/main.cpp

sed -i 's/static uint8_t buildGpsQuality(float speedKmh, float altitudeM) {/static uint8_t buildGpsQuality(float speedKnots, float altitudeM) {\n  float speedKmh = speedKnots * 1.852f;/' src/main.cpp
sed -i 's/static bool isGpsSpeedValid(float speedKmh) {/static bool isGpsSpeedValid(float speedKnots) {\n  float speedKmh = speedKnots * 1.852f;/' src/main.cpp
sed -i 's/if (isGpsSpeedValid(speedKmh)) {/if (isGpsSpeedValid(speedKnots)) {/' src/main.cpp
sed -i 's/if (point.speedValid \&\& point.speed > MOVING_SPEED_KMH) lastMovementMs = millis();/if (point.speedValid \&\& (point.speed \* 1.852f) > MOVING_SPEED_KMH) lastMovementMs = millis();/' include/tracker_telemetry.h

awk '
  /	d\["speed"\] = derived_speed_kmh/ {
    print "	d[\"speed\"] = derived_speed_kmh / 1.852"
    next
  }
  {print}
' server/subscriberJsonOsmAnd.py > temp.py && mv temp.py server/subscriberJsonOsmAnd.py

awk '
  /	d\["speed"\] = derived_speed_kmh/ {
    print "	d[\"speed\"] = derived_speed_kmh / 1.852"
    next
  }
  {print}
' server/subscriberV3.py > temp2.py && mv temp2.py server/subscriberV3.py

sed -i 's/		params\["speed"\] = f"{float(speed):.1f}"/		params\["speed"\] = f"{float(speed) \/ 1.852:.1f}"/' server/subscriberJsonOsmAnd.py

sed -i 's/static void pmModemWake() {/static bool pmModemWake() {/' include/tracker_pm.h
sed -i 's/  for (int i = 0; i < 15 \&\& !modem.testAT(500); i++) {/  bool ok = false;\n  for (int i = 0; i < 15; i++) {\n    if (modem.testAT(500)) {\n      ok = true;\n      break;\n    }/' include/tracker_pm.h
sed -i 's/  modem.waitResponse(1000);/  modem.waitResponse(1000);\n  if (!ok) {\n    SerialMon.println("[PM] modem no responde a AT despues de despertar");\n  }\n  return ok;/' include/tracker_pm.h
sed -i 's/static void pmModemResume() {/static bool pmModemResume() {/' include/tracker_pm.h
sed -i 's/    pmModemWake();/    return pmModemWake();/' include/tracker_pm.h
sed -i 's/    waitForAT();\n    rtcModemAlive = true;/    waitForAT();\n    rtcModemAlive = true;\n    return true;/' include/tracker_pm.h
sed -i 's/  pmModemResume();/  if (!pmModemResume()) {\n    SerialMon.println("[PM] modem fallo al despertar -> hardware reset");\n    modemPowerOn();\n    waitForAT();\n  }/' src/main.cpp
sed -i 's/      pmModemResume();/      if (!pmModemResume()) {\n        SerialMon.println("[PM] modem fallo al despertar (corte) -> hardware reset");\n        modemPowerOn();\n        waitForAT();\n      }/' include/tracker_pm.h
sed -i 's/    pmModemResume();/    if (!pmModemResume()) {\n      SerialMon.println("[PM] modem fallo al despertar (pulso) -> hardware reset");\n      modemPowerOn();\n      waitForAT();\n    }/' include/tracker_pm.h

sed -i 's/static void pmDisconnectClean() {/static void pmDisconnectClean() {\n  if (mqtt.connected()) {\n    \/\/ Publicar LWT explicitamente offline para no generar online\/offline enganosos al despertar\n    mqtt.publish(TOPIC_LWT, LWT_OFFLINE, true);\n    mqtt.disconnect();  \/\/ cierre limpio\n  }/' include/tracker_pm.h
sed -i '/if (mqtt.connected()) mqtt.disconnect();  \/\/ cierre limpio: no dispara el LWT/d' include/tracker_pm.h

sed -i 's/static void waitForAT() {/static bool waitForAT(uint32_t timeoutMs = 15000) {/' src/main.cpp
sed -i 's/  while (!modem.testAT(1000)) {/  while (!modem.testAT(1000)) {\n    if (millis() - t0 > timeoutMs) {\n      SerialMon.println("AT timeout");\n      return false;\n    }/' src/main.cpp
sed -i 's/  SerialMon.println("AT OK");/  SerialMon.println("AT OK");\n  return true;/' src/main.cpp
sed -i 's/static void ensureLTE() {/static bool ensureLTE() {/' src/main.cpp
sed -i 's/if (!modem.waitForNetwork(60000L)) {/if (!modem.waitForNetwork(30000L)) {/' src/main.cpp
sed -i 's/      return;/      return false;/g' src/main.cpp
sed -i 's/    return false;  \/\/ conserva el ultimo estado conocido/    return;  \/\/ conserva el ultimo estado conocido/' src/main.cpp
awk '/IPAddress ip = modem.localIP();/{print "  IPAddress ip = modem.localIP();\n  SerialMon.print(\"IP: \");\n  SerialMon.println(ip);\n  return true;"; getline; getline; getline; next}1' src/main.cpp > temp.cpp && mv temp.cpp src/main.cpp

sed -i 's/static uint32_t mqttNextAttemptMs = 0;/static uint32_t mqttNextAttemptMs = 0;\nstatic uint32_t lteNextAttemptMs = 0;\nstatic uint32_t lteRetryDelayMs = 5000;/' src/main.cpp

awk '
  /if \(!modem\.isNetworkConnected\(\) \|\| !modem\.isGprsConnected\(\)\) \{/ {
    print "  uint32_t now = millis();"
    print "  if (!modem.isNetworkConnected() || !modem.isGprsConnected()) {"
    print "    if (now >= lteNextAttemptMs) {"
    print "      SerialMon.println(\"[NET] down -> reconnect\");"
    print "      if (ensureLTE()) {"
    print "        lteRetryDelayMs = 5000;"
    print "      } else {"
    print "        lteRetryDelayMs *= 2;"
    print "        if (lteRetryDelayMs > 60000) lteRetryDelayMs = 60000;"
    print "      }"
    print "      lteNextAttemptMs = millis() + lteRetryDelayMs;"
    print "    }"
    print "  }"
    in_block = 1
    next
  }
  in_block && /  \}/ {
    in_block = 0
    next
  }
  in_block { next }
  { print }
' src/main.cpp > temp2.cpp && mv temp2.cpp src/main.cpp

awk '
  /static void restartModem\(\) \{/ {
    print "static bool pmGnssOn();"; print ""
    print "static bool restartModem() {"
    print "  SerialMon.println(\"[NET] reiniciando modem...\");"
    print "  watchdogFeed();"
    print "  modem.gprsDisconnect();"
    print "  delay(500);"
    print "  modem.restart();"
    print "  watchdogFeed();"
    print "  if (!waitForAT()) return false;"
    print "  if (!ensureLTE()) return false;"
    print ""
    print "  if (ignState != IGN_OFF) {"
    print "    SerialMon.println(\"[GNSS] reactivando GNSS tras reinicio del modem\");"
    print "    pmGnssOn();"
    print "  }"
    print "  return true;"
    in_block = 1
    next
  }
  in_block && /^\}/ {
    in_block = 0
    print "}"
    next
  }
  in_block { next }
  { print }
' src/main.cpp > temp3.cpp && mv temp3.cpp src/main.cpp

awk '/  if \(mqttFailCount == MQTT_FAILS_BEFORE_LTE_RECONNECT\) \{/ {
  print; getline; print; getline; print "    if (!ensureLTE()) { lteRetryDelayMs = 5000; lteNextAttemptMs = millis() + lteRetryDelayMs; mqttNextAttemptMs = millis() + mqttRetryDelayMs; return; }"; next
}1' src/main.cpp > temp4.cpp && mv temp4.cpp src/main.cpp
sed -i 's/    restartModem();/    if (!restartModem()) { lteRetryDelayMs = 5000; lteNextAttemptMs = millis() + lteRetryDelayMs; mqttNextAttemptMs = millis() + mqttRetryDelayMs; return; }/' src/main.cpp

sed -i 's/static bool tryConnectMQTT() {/static bool tryConnectMQTT() {\n  \/\/ Cierra socket antes de conectar (hipotesis state=-4)\n  netClient.stop();/' src/main.cpp

sed -i 's/      ensureLTE();\n      if (tryConnectMQTT()) pmEnterCutoff(v);/      if (ensureLTE()) {\n        if (tryConnectMQTT()) pmEnterCutoff(v);\n      }/' include/tracker_pm.h
sed -i 's/    ensureLTE();\n    if (tryConnectMQTT()) {/    if (ensureLTE() \&\& tryConnectMQTT()) {/' include/tracker_pm.h

cat << 'TEST' > server/tests/test_subscriber_logic.py
import unittest
from unittest.mock import patch
import subscriberJsonOsmAnd

class TestSubscriberLogic(unittest.TestCase):
    def test_derived_speed_converted_to_knots(self):
        subscriberJsonOsmAnd._last_valid_point = {
            "lat": 0.0, "lon": 0.0, "ts": "2026-10-06T12:00:00Z"
        }
        d = {"lat": 0.0009, "lon": 0.0, "ts": "2026-10-06T12:00:10Z"}
        derived = subscriberJsonOsmAnd.derive_speed_if_needed(d)

        self.assertIsNotNone(derived.get("speed"))
        self.assertEqual(derived.get("speed_source"), "derived")
        self.assertAlmostEqual(derived["speed"], 19.45, places=2)

    @patch("subscriberJsonOsmAnd.requests.get")
    def test_send_osmand_keeps_gnss_knots_as_is(self, mock_get):
        class MockResponse:
            status_code = 200
            text = "OK"
        mock_get.return_value = MockResponse()

        subscriberJsonOsmAnd.send_osmand(0.0, 0.0, speed=10.0)
        args, kwargs = mock_get.call_args
        self.assertEqual(kwargs["params"]["speed"], "10.0")

if __name__ == "__main__":
    unittest.main()
TEST

awk '
  /def event_position\(payload\):/ {
    in_block = 1
  }
  in_block && /    return None, "none"/ {
    print "    if position is not None and position.get(\"speed\") is not None:"
    print "        # derived inside payload JSON isn'\''t guaranteed, convert to knots."
    print "        if payload.get(\"position_source\") == \"derived\":"
    print "            position[\"speed\"] = float(position[\"speed\"]) / 1.852"
    print "    return None, \"none\""
    in_block = 0
    next
  }
  {print}
' server/subscriberV3.py > temp5.py && mv temp5.py server/subscriberV3.py

awk '
  /	d\["speed"\] = derived_speed_kmh \/ 1\.852/ {
    getline; getline; getline
    print "	d[\"speed\"] = derived_speed_kmh / 1.852"
    next
  }
  {print}
' server/subscriberJsonOsmAnd.py > temp6.py && mv temp6.py server/subscriberJsonOsmAnd.py
sed -i '/	d\["speed"\] = derived_speed_kmh/d' server/subscriberJsonOsmAnd.py
sed -i 's/	d\["speed_source"\] = "derived"/	d\["speed"\] = derived_speed_kmh \/ 1.852\n	d\["speed_source"\] = "derived"/' server/subscriberJsonOsmAnd.py
sed -i 's/		params\["speed"\] = f"{float(speed) \/ 1.852:.1f}"/		params\["speed"\] = f"{float(speed):.1f}"/' server/subscriberJsonOsmAnd.py

sed -i 's/  SerialMon.println(ip);/  SerialMon.println(ip);\n  return true;/' src/main.cpp
