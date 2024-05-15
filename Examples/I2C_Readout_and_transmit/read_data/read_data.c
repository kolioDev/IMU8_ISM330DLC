#include "../c6dof_lib/c6dof_lib.h"
#include <WiFiClient.h>
#include <WiFiUdp.h>

const uint8_t SYNC_SEQUENCE[] = {0xAA, 0xBB, 0xCC}; // Define a SYNC sequence

const uint8_t NO_DATA_AVAILABLE = 0x45;

void transmit_raw_data_serial(u_int8_t *data, size_t size, uint32_t sample_index)
{
  Serial.write(SYNC_SEQUENCE, sizeof(SYNC_SEQUENCE));
  Serial.write((uint8_t *)&sample_index, sizeof(sample_index));
  Serial.write(data, size);
}

void transmit_raw_data_wifi_tcp(u_int8_t *data, size_t size, uint32_t sample_index, WiFiClient client)
{
  uint32_t currentTime = micros();

  size_t totalSize = sizeof(sample_index) + size + sizeof(currentTime);
  uint8_t buffer[totalSize];
  memcpy(buffer,                               &sample_index, sizeof(sample_index));
  memcpy(buffer + sizeof(sample_index),                 data,                size);
  memcpy(buffer + sizeof(sample_index) + size, &currentTime , sizeof(currentTime));

  client.write(buffer, totalSize);
}

void transmit_raw_data_wifi_udp(u_int8_t *data, size_t size, uint32_t sample_index, WiFiUDP udp, IPAddress receiverIp, unsigned int localPort)
{
  udp.beginPacket(receiverIp, localPort); // Send to Python client's IP
  udp.write(SYNC_SEQUENCE, sizeof(SYNC_SEQUENCE));
  udp.write((uint8_t *)&sample_index, sizeof(sample_index));
  udp.write(data, size);
  udp.endPacket();
}

u_int8_t read_row_data(c6dofimu8_t *board_ctx, u_int8_t *data)
{
  // Check if there is a i2c connection established

  // Check if the sensor has data ready
  uint8_t data_ready = c6dofimu8_get_drdy_status(board_ctx, C6DOFIMU8_TEMP_DRDY_MASK |
                                                                C6DOFIMU8_G_DRDY_MASK |
                                                                C6DOFIMU8_XL_DRDY_MASK);

  if (data_ready == C6DOFIMU8_EVENT_NOT_DETECTED)
  {
    return NO_DATA_AVAILABLE;
  }

  uint8_t err = c6dofimu8_get_data_raw(board_ctx, data);
  if (err == C6DOFIMU8_OK)
  {
    return 0;
  }
  else
  {
    return err;
  }
}

void read_and_transmit_data(c6dofimu8_t *board_ctx)
{
  // Check if the sensor has data ready
  uint8_t data_ready = c6dofimu8_get_drdy_status(board_ctx, C6DOFIMU8_TEMP_DRDY_MASK |
                                                                C6DOFIMU8_G_DRDY_MASK |
                                                                C6DOFIMU8_XL_DRDY_MASK);

  if (data_ready == C6DOFIMU8_EVENT_NOT_DETECTED)
  {
    // Serial.println("No new data ready");
    return;
  }

  t_c6dofimu8_axis accel_data, gyro_data;
  int8_t temperature;

  uint8_t err = c6dofimu8_get_data(board_ctx, &accel_data, &gyro_data, &temperature);
  if (err == C6DOFIMU8_OK)
  {
    // Serial.print("T:");
    // Serial.println(temperature);

    Serial.print("a_x:");
    Serial.println(accel_data.x);
    Serial.print("a_y:");
    Serial.println(accel_data.x);
    Serial.print("a_z:");
    Serial.println(accel_data.x);

    // Serial.print("a_y:");
    // Serial.println(accel_data.y);

    // Serial.print("a_z:");
    // Serial.println(accel_data.z);

    // Serial.print("G_x:");
    // Serial.println(gyro_data.x);

    // Serial.print("G_y:");
    // Serial.println(gyro_data.y);

    // Serial.print("G_z:");
    // Serial.println(gyro_data.z);
  }
  else
  {
    Serial.print("Error - ");
    Serial.println(err);
  }
}
