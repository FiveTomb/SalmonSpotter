#include "contiki.h"
#include "adc-zoul.h"
#include "dev/i2c.h"
#include "dev/gpio-hal.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "lib/sensors.h"
#include "net/routing/routing.h"
#include "random.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include "sys/log.h"

#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

#define WITH_SERVER_REPLY  1
#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

#define SEND_INTERVAL		  (5 * CLOCK_SECOND)

static struct simple_udp_connection udp_conn;
static uint32_t rx_count = 0;

// Scaling factor for fixed-point arithmetic
#define SCALE_FACTOR_PH 10000
#define SCALE_FACTOR_TDS 1000

// SHT31 Sensors (Temp)
#define SHT_31_DFLT_ADDR 0x44
#define SHT_31_ALT_ADDR 0x45
#define SHT31_READ_TEMP_HUM 0x2400

// Hold raw values from SHT31
static uint16_t temperature, outside_temperature;
static uint16_t pH_value;
static uint16_t tds_value_ppm;

// Hold converted values for SHT31
int16_t temperature_celsius_x100, outside_temperature_x100;

// Gravity pH Sensor
#define ADC_MAX_PH_SENSOR 2047
#define VREF_PH_SENSOR 3300    // Reference voltage in millivolts
#define ADC_RESOLUTION_PH_SENSOR (VREF_PH_SENSOR / ADC_MAX_PH_SENSOR)

// CQRobot TDS Sensor
#define VREF_TDS_SENSOR 3300 // Reference voltage in milliwatts
#define SCOUNT 30
#define TEMP_COEF 20
#define STANDARD_TEMP_TDS 25000

// LSM6DSO Addresses
#define LSM6DSO_DFLT_ADDR 0x6B
#define LSM6DSO_ALT_ADDR 0x6A

// LSM6DSO Registers
#define LSM6DSO_CTRL1_XL 0x10 // Config regoster for accelerometer
#define LSM6DSO_CTRL2_G 0x11  // Config register for gyroscope
#define LSM6DSO_OUTX_L_G 0x22 
#define LSM6DSO_OUTX_H_G 0x23
#define LSM6DSO_OUTY_L_G 0X24
#define LSM6DSO_OUTY_H_G 0X25
#define LSM6DSO_OUTZ_L_G 0X26
#define LSM6DSO_OUTZ_H_G 0X27
#define LSM6DSO_OUTX_L_A 0x28
#define LSM6DSO_OUTX_H_A 0x29
#define LSM6DSO_OUTY_L_A 0x2A
#define LSM6DSO_OUTY_H_A 0x2B
#define LSM6DSO_OUTZ_L_A 0x2C
#define LSM6DSO_OUTZ_H_A 0x2D
#define LSM6DSO_WHO_AM_I 0x0F // Register with pre defined value, good for testing signal to IMU

// Variables to hold information from axes
uint16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

// Initializing LSM6DSO IMU accelerometer and gyroscope
void initLSM6DSO()
{
  uint8_t config_byte = 0x48; // Example configuration byte for gyroscope and accelerometer
  uint8_t buffer[3];
  // Init LSM6DSO
  // Configure Accelerometer
  buffer[0] = LSM6DSO_CTRL1_XL;
  buffer[1] = config_byte;
  // Write to CTRL1_XL to configure the accelerometer
  i2c_burst_send(LSM6DSO_DFLT_ADDR, buffer, 2);

  // Configure Gyroscope
  buffer[0] = LSM6DSO_CTRL2_G;
  buffer[1] = config_byte;
  // Write to CTRL1_XL to configure the gyroscope
  i2c_burst_send(LSM6DSO_DFLT_ADDR, buffer, 2);
}

// Read a register from the LSM6DSO IMU
int16_t readSensorData(uint8_t sensorAddress, uint8_t registerAddress)
{
    uint8_t buffer[3];
    uint16_t combinedData;
    // Reading data
    buffer[0] = registerAddress; // Set the register address to start reading
    i2c_burst_send(sensorAddress, buffer, 1); // Send register address
    i2c_burst_receive(sensorAddress, buffer, 2); // Read 2 bytes

    combinedData = (int16_t)((buffer[1] << 8) | buffer[0]);

    return combinedData;
}

// Temperature conversion using fixed-point arithmetic
int16_t raw_to_celsius_x100(uint16_t raw_temp) {
    int32_t temp = (17500 * raw_temp) >> 16; // Multiplying by 100 to keep 2 decimal places
    return (int16_t)temp - 4500; // Subtracting 4500 (which is -45.00 in this fixed-point representation)
}

// Humidity conversion using fixed point arithmetic
int16_t raw_to_humidity_x100(uint16_t raw_humidity) {
    int32_t hum = (10000 * raw_humidity) >> 16; // Multiplying by 100 to keep 2 decimal places
    return (int16_t)hum;
}

// Assuming 'voltage' is already an integer representing the voltage in millivolts
int calculateTDS(int voltage, int temperature) {
    long long v = (long long)voltage; // Promote to long long to prevent overflow

    // Temperature compensation
    long long tempCompensationFactor = SCALE_FACTOR_TDS + TEMP_COEF * (temperature - STANDARD_TEMP_TDS);
    long long compensatedVoltage = (v * SCALE_FACTOR_TDS) / tempCompensationFactor;

    // Scale the constants
    long long const1 = 133420LL; // 133.42 * SCALE_FACTOR_TDS
    long long const2 = 255860LL; // 255.86 * SCALE_FACTOR_TDS
    long long const3 = 857390LL; // 857.39 * SCALE_FACTOR_TDS

    // Calculate TDS
    long long tds = (const1 * compensatedVoltage * compensatedVoltage * compensatedVoltage 
                - const2 * compensatedVoltage * compensatedVoltage 
                + const3 * compensatedVoltage) / (2 * SCALE_FACTOR_TDS * SCALE_FACTOR_TDS * SCALE_FACTOR_TDS);

    return (int)(tds / SCALE_FACTOR_TDS); // Adjust final scale and convert back to int
}

static void
udp_rx_callback(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{

  LOG_INFO("Received response '%.*s' from ", datalen, (char *) data);
  LOG_INFO_6ADDR(sender_addr);
#if LLSEC802154_CONF_ENABLED
  LOG_INFO_(" LLSEC LV:%d", uipbuf_get_attr(UIPBUF_ATTR_LLSEC_LEVEL));
#endif
  LOG_INFO_("\n");
  rx_count++;
}

// Networking Processes
PROCESS(udp_client_process, "UDP Client Process");
// Individual Sensor Processes
PROCESS(sht31_process, "Water Temperature readings");
PROCESS(sht31_alt_process, "Outdoor Temperature readings");
PROCESS(ph_sensor_process, "Grvaity pH Sensor");
PROCESS(tds_sensor_process, "TDS Sensor Readings");
PROCESS(lsm6dso_process, "IMU readings");
AUTOSTART_PROCESSES(&sht31_process, &sht31_alt_process, &ph_sensor_process, &tds_sensor_process, &lsm6dso_process, &udp_client_process);

// SHT31 Process (Water Temp)
PROCESS_THREAD(sht31_process, ev, data)
{ 
  static struct etimer sht31_timer; 
  PROCESS_BEGIN();

  // Setup a periodic sht31_timer that expires after 1 second. 
  etimer_set(&sht31_timer, CLOCK_SECOND * 1);

  // Initialize I2C (assuming default SDA and SCL pins for Zolertia RE-Mote)
  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_NORMAL_BUS_SPEED);

  while(1) {
    // Variables to hold readings from sensors
    temperature_celsius_x100 = raw_to_celsius_x100(temperature);
    //int16_t humidity_percent_x100 = raw_to_humidity_x100(humidity);
    uint8_t data[6];
    uint8_t command[2] = {SHT31_READ_TEMP_HUM >> 8, SHT31_READ_TEMP_HUM & 0xFF};

    // Send the read temperature and humidity command
    i2c_burst_send(SHT_31_DFLT_ADDR, command, 2);

    // Delay as per SHT31 datasheet for measurement completion
    clock_delay_usec(15000); // example delay, refer to the datasheet

    // Read the 6 bytes of data: Temp(MSB) Temp(LSB) Temp(CRC) Hum(MSB) Hum(LSB) Hum(CRC)
    i2c_burst_receive(SHT_31_DFLT_ADDR, data, 6);

    // Convert and store the temperature and humidity values (without CRC checking)
    temperature = (data[0] << 8) | data[1];
    //humidity = (data[3] << 8) | data[4];

    printf("Water Temperature: %d.%02d°C\n", 
            temperature_celsius_x100 / 100, temperature_celsius_x100 % 100);
    
    // Wait for the periodic sht31_timer to expire and then restart the sht31_timer. 
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sht31_timer));
    etimer_reset(&sht31_timer);
  }
  PROCESS_END();
}

// SHT31 Process (Outside Temp)
PROCESS_THREAD(sht31_alt_process, ev, data)
{ 
  static struct etimer sht31_timer; 
  PROCESS_BEGIN();
  
  // Variables to hold raw values from SHT31
  //static uint16_t temperature;//, humidity;

  // Setup a periodic sht31_timer that expires after 1 second.
  etimer_set(&sht31_timer, CLOCK_SECOND * 1);

  // Initialize I2C (assuming default SDA and SCL pins for Zolertia RE-Mote)
  i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_NORMAL_BUS_SPEED);

  while(1) {
    // Variables to hold readings from sensors
    outside_temperature_x100 = raw_to_celsius_x100(outside_temperature);
    //int16_t humidity_percent_x100 = raw_to_humidity_x100(humidity);
    uint8_t alt_data[6];
    uint8_t alt_command[2] = {SHT31_READ_TEMP_HUM >> 8, SHT31_READ_TEMP_HUM & 0xFF};

    // Send the read temperature and humidity command (Outdoor Sensor)
    i2c_burst_send(SHT_31_ALT_ADDR, alt_command, 2);

    // Delay as per SHT31 datasheet for measurement completion
    clock_delay_usec(15000); // example delay, refer to the datasheet

    // Read the 6 bytes of data: Temp(MSB) Temp(LSB) Temp(CRC) Hum(MSB) Hum(LSB) Hum(CRC)
    i2c_burst_receive(SHT_31_ALT_ADDR, alt_data, 6);

    // Convert and store the temperature and humidity values (without CRC checking)
    outside_temperature = (alt_data[0] << 8) | alt_data[1];
    //humidity = (alt_data[3] << 8) | alt_data[4];

    printf("Outside Temperature: %d.%02d°C\n", 
            outside_temperature_x100 / 100, outside_temperature_x100 % 100);
    
    // Wait for the periodic sht31_timer to expire and then restart the sht31_timer. 
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sht31_timer));
    etimer_reset(&sht31_timer);
  }

  PROCESS_END();
}

PROCESS_THREAD(ph_sensor_process, ev, data)
{
  static struct etimer timer;

  adc_zoul.configure(SENSORS_HW_INIT, ZOUL_SENSORS_ADC1);

  PROCESS_BEGIN();

  // Setup a periodic timer that expires after 1 seconds.
  etimer_set(&timer, CLOCK_SECOND * 1);

  while(1) {
    int value = adc_zoul.value(ZOUL_SENSORS_ADC1);
    if(value != 0) 
    {
      // Calculate voltage in millivolts
      int voltage_mV = value * ADC_RESOLUTION_PH_SENSOR;

      // Apply scaled equation for pH
      pH_value = (-56548 * voltage_mV + 155090) / SCALE_FACTOR_PH;

   
   #define SCALE_FACTOR_TDS 1000   // Print pH value
      printf("pH Value: %d\n", pH_value);
    } else {
      printf("ADC reading error\n");
    }

    // Wait for the periodic timer to expire and then restart the timer.
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    etimer_reset(&timer);
  }

  PROCESS_END();
}

PROCESS_THREAD(tds_sensor_process, ev, data)
{
  static struct etimer timer_sampling;
  PROCESS_BEGIN();

  // Initialize the ADC for the TDS sensor
  adc_zoul.configure(SENSORS_HW_INIT, ZOUL_SENSORS_ADC3);

  etimer_set(&timer_sampling, CLOCK_SECOND);

  while(1) 
  {
    // Wait for the timer event
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_sampling));

    // Read the raw sensor value from the ADC
    int raw_sensor_value = adc_zoul.value(ZOUL_SENSORS_ADC3);

    tds_value_ppm = calculateTDS(raw_sensor_value, 25000);

    // Print the raw sensor value
    printf("Raw TDS Sensor Value: %d\n", raw_sensor_value);
    printf("TDS value: %d ppm\n", tds_value_ppm);

    // Reset the timer for the next reading
    etimer_reset(&timer_sampling);
    
  }

  PROCESS_END();
}


// LSM6DSO Process
PROCESS_THREAD(lsm6dso_process, ev, data) 
{
    static struct etimer lsm6dso_timer;

    PROCESS_BEGIN();

    etimer_set(&lsm6dso_timer, CLOCK_SECOND / 10); // Set timer for 0.1 second intervals

    i2c_init(I2C_SDA_PORT, I2C_SDA_PIN, I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_NORMAL_BUS_SPEED);

    initLSM6DSO(); // Init gyro and accelerometer

    while (1) 
    {
        accel_x = readSensorData(LSM6DSO_DFLT_ADDR, LSM6DSO_OUTX_L_A);
        accel_y = readSensorData(LSM6DSO_DFLT_ADDR, LSM6DSO_OUTY_L_A);
        accel_z = readSensorData(LSM6DSO_DFLT_ADDR, LSM6DSO_OUTZ_L_A);

        printf("Accelerometer data: X = %d, Y = %d, Z = %d\n", accel_x, accel_y, accel_z);

        gyro_x = readSensorData(LSM6DSO_DFLT_ADDR, LSM6DSO_OUTX_L_G);
        gyro_y = readSensorData(LSM6DSO_DFLT_ADDR, LSM6DSO_OUTY_L_G);
        gyro_z = readSensorData(LSM6DSO_DFLT_ADDR, LSM6DSO_OUTZ_L_G);

        printf("Gyroscope data: X = %d, Y = %d, Z = %d\n", gyro_x, gyro_y, gyro_z);

        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&lsm6dso_timer));
        etimer_reset(&lsm6dso_timer);
    }

    PROCESS_END();
}

PROCESS_THREAD(udp_client_process, ev, data)
{
  // Add new variables to hold sensor data for UDP transmission
  char sensor_data_str[150];
  static struct etimer periodic_timer;
  uip_ipaddr_t dest_ipaddr;
  static uint32_t tx_count;
  static uint32_t missed_tx_count;


  PROCESS_BEGIN();

  /* Initialize UDP connection */
  simple_udp_register(&udp_conn, UDP_CLIENT_PORT, NULL,
                      UDP_SERVER_PORT, udp_rx_callback);

  etimer_set(&periodic_timer, random_rand() % SEND_INTERVAL);
  while(1) 
  {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
    
    // Prepare the sensor data for transmission
    snprintf(sensor_data_str, sizeof(sensor_data_str), 
                 "Water Temp: %d.%02d°C\nOutside Temp: %d.%02d°C, pH: %d, TDS: %d ppm\nAccel: X=%d, Y=%d, Z=%d, Gyro: X=%d, Y=%d, Z=%d",  
             temperature_celsius_x100 / 100, temperature_celsius_x100 % 100,
             outside_temperature_x100 / 100, outside_temperature_x100 % 100,
             pH_value, tds_value_ppm, 
             accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
    if(NETSTACK_ROUTING.node_is_reachable() &&
        NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr)) {

      /* Print statistics every 10th TX */
      if(tx_count % 10 == 0) {
        LOG_INFO("Tx/Rx/MissedTx: %" PRIu32 "/%" PRIu32 "/%" PRIu32 "\n",
                 tx_count, rx_count, missed_tx_count);
      }

      /* Send to DAG root */
      LOG_INFO("Sending request %"PRIu32" to ", tx_count);
      LOG_INFO_6ADDR(&dest_ipaddr);
      LOG_INFO_("\n");
      simple_udp_sendto(&udp_conn, sensor_data_str, strlen(sensor_data_str), &dest_ipaddr);
      tx_count++;
    } else {
      LOG_INFO("Not reachable yet\n");
      if(tx_count > 0) {
        missed_tx_count++;
      }
    }

    /* Add some jitter */
    etimer_set(&periodic_timer, SEND_INTERVAL
      - CLOCK_SECOND + (random_rand() % (2 * CLOCK_SECOND)));
  }

  PROCESS_END();
}