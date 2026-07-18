/*
 * Local copy of micro_ros_arduino Wi-Fi UDP transport.
 * Needed because precompiled=full (or some IDE modes) skips compiling
 * the library's wifi_transport.cpp for ESP32-S3.
 */
#if defined(ESP32)

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <micro_ros_arduino.h>

extern "C" {

static WiFiUDP udp_client;

bool arduino_wifi_transport_open(struct uxrCustomTransport *transport) {
  struct micro_ros_agent_locator *locator =
      (struct micro_ros_agent_locator *)transport->args;
  udp_client.begin(locator->port);
  return true;
}

bool arduino_wifi_transport_close(struct uxrCustomTransport *transport) {
  (void)transport;
  udp_client.stop();
  return true;
}

size_t arduino_wifi_transport_write(struct uxrCustomTransport *transport,
                                    const uint8_t *buf, size_t len,
                                    uint8_t *errcode) {
  (void)errcode;
  struct micro_ros_agent_locator *locator =
      (struct micro_ros_agent_locator *)transport->args;

  udp_client.beginPacket(locator->address, locator->port);
  size_t sent = udp_client.write(buf, len);
  udp_client.endPacket();
  udp_client.flush();
  return sent;
}

size_t arduino_wifi_transport_read(struct uxrCustomTransport *transport,
                                   uint8_t *buf, size_t len, int timeout,
                                   uint8_t *errcode) {
  (void)errcode;
  (void)transport;

  uint32_t start_time = millis();
  while (millis() - start_time < (uint32_t)timeout &&
         udp_client.parsePacket() == 0) {
    delay(1);
  }

  int readed = udp_client.read(buf, len);
  return (readed < 0) ? 0 : (size_t)readed;
}

}  // extern "C"

#endif
