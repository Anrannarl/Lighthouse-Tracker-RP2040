// A simple program using PIO to read lighthouse sensor outputs
// It can support up to 22 sensors
// RP2040 Arduino core is used https://github.com/earlephilhower/arduino-pico
// The CPU clock is set to 250Mhz(overclock)
// I suggest using the official Raspberry Pi Pico board
// The system power supply must be a low noise linear regulator
// The SMPS on the Raspberry Pi Pico board is disabled

// Links you might find useful
// Pico C SDK document https://www.raspberrypi.com/documentation/pico-sdk/
// PIO examples https://github.com/GitJer/Some_RPI-Pico_stuff
// RP2040 logic analyzer https://github.com/gusmanb/logicanalyzer
// Raspberry Pi Pico datasheet https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf
// RP2040 datasheet https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf

// Some open-source projects
// libsurvive https://github.com/cntools/libsurvive
// LighthouseRedox https://github.com/nairol/LighthouseRedox
// HTC vive lighthouse custom tracking https://hackaday.io/project/19570-htc-vive-lighthouse-custom-tracking
// bitcraze Lighthouse https://www.bitcraze.io/documentation/lighthouse/
// HiveTracker https://hackaday.io/project/160182-hivetracker
// VIVE POSITION SENSOR https://hacks.esar.org.uk/vive-position-sensor/ https://github.com/esar/vivepos
// vive-tracker https://github.com/arenibProjects/vive-tracker
// Homebrew interface for the HTC v1 Lighthouses https://trmm.net/Lighthouse/
// Position Aware Devices https://blog.crashspace.org/2016/12/position-aware-device/
// DIY Position Tracking https://github.com/ashtuchkin/vive-diy-position-sensor
// EE267 6-DOF Pose Tracking with the VRduino https://stanford.edu/class/ee267/notes/ee267_notes_tracking.pdf
// EE267 https://github.com/kingofleaves/Lighthouse-Tracking https://github.com/mohism-ai-lab/EE267


#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "sensor_handler.pio.h"

// no need to change
#define SENSOR_COUNT 22
// it's possible to use lower buffer size to reduce latency
// try to find a value that doesn't crash the program or cause data corruption
// 16 is a safe value even for the worst scenario(22 coplanar sensors)
#define SM_BUFFER_SIZE 16
// use chained DMA to move data from SM to buffer automatically
// when the first DMA channel finishes, it triggers the second one right away and flags an interrupt
// CPU is used to reconfigure the finished DMA channel and move the data from the finished buffer to a queue
// five chained DMA for each SM
#define SM_DMA_COUNT 5
#define SM_QUEUE_LIMIT 200

PIO pio = pio0;
const uint32_t sm0 = 0, sm1 = 1; // sm0 and sm1 are used
const uint32_t sm0_clkdiv = 4, sm1_clkdiv = 4; // clkdiv = 4 to run SM at 1/4 CPU clock
// one count equals 3 * PIO tick (see the PIO program for more details)
// 48ns for the default setup
#define PIO_CYCLE_TIME 48
uint32_t sm0_offset, sm1_offset;
const uint32_t in_base = 0, sideset_base = 28, jmp_pin = 22;
// pin0 to pin 21 are used to read sensor outputs
// pin28 is assigned to sm0, pin22 is assigned to sm1
// pin28 is connected to pin22, it is used to trigger sm1 from sm0
// unless you design your onw hardware with RP2040, avoid using pin23(SMPS PWM mode) pin24(voltage monitor) pin25(LED)
// read Raspberry Pi Pico datasheet for more details
// to use more than 22 sensors, you need to modify the PIO program
// pin26(I2C1 SDA) and pin27(I2C1 SCL) are reserved for mpu6050

// two state machines
uint32_t sm_buffer[2][SM_DMA_COUNT][SM_BUFFER_SIZE];
int32_t dma_chan_sm[2][SM_DMA_COUNT];
dma_channel_config dma_conf_sm[2][SM_DMA_COUNT];
volatile bool sm_buffer_full[SM_DMA_COUNT];

queue_t sm0_queue;
queue_t sm1_queue;

queue_t output_queue;

struct sensor_info
{
  bool last_level;
  uint32_t time_elapsed;
};
sensor_info photosensor[SENSOR_COUNT];

struct sensor_output
{
  uint32_t sensor_id;
  bool logic_level;
  uint32_t pulse_width;
};

//core0
void setup()
{
  Serial.begin(115200);
}
void loop()
{
  while (!queue_is_empty(&sm0_queue) && !queue_is_empty(&sm1_queue))
  {
    uint32_t sm0_temp, sm1_temp, time_ns;
    queue_try_remove(&sm0_queue, &sm0_temp);
    queue_try_remove(&sm1_queue, &sm1_temp);
    time_ns = sm1_temp * PIO_CYCLE_TIME;
    for (uint32_t sensor_i = 0; sensor_i < SENSOR_COUNT; sensor_i++)
    {
      bool bit_temp = (sm0_temp << (SENSOR_COUNT - 1 - sensor_i)) >> 31;
      photosensor[sensor_i].time_elapsed += time_ns;
      if (bit_temp != photosensor[sensor_i].last_level)
      {
        sensor_output output_temp;
        output_temp.sensor_id = sensor_i;
        output_temp.logic_level = photosensor[sensor_i].last_level;
        output_temp.pulse_width = photosensor[sensor_i].time_elapsed;
        queue_try_add(&output_queue, &output_temp);
        photosensor[sensor_i].last_level = bit_temp;
        photosensor[sensor_i].time_elapsed = 0;
      }
    }
  }

  // print the sensor ID and time between rising/falling edges
  while (!queue_is_empty(&output_queue))
  {
    sensor_output output_temp;
    queue_try_remove(&output_queue, &output_temp);
    Serial.print(output_temp.sensor_id); Serial.print(" ");
    //Serial.print(output_temp.logic_level); Serial.print(" "); // use to verify sensor output, not necessary
    Serial.println(float(output_temp.pulse_width) / 1000); // output in microseconds
  }
}

//core1
void setup1()
{
  queue_init(&sm0_queue, sizeof(uint32_t), SM_QUEUE_LIMIT);
  queue_init(&sm1_queue, sizeof(uint32_t), SM_QUEUE_LIMIT);

  queue_init(&output_queue, sizeof(sensor_output), SM_QUEUE_LIMIT);

  sm0_offset = pio_add_program(pio, &level_monitor_program);
  sm1_offset = pio_add_program(pio, &time_stamp_program);

  pio_sm_config c_sm0 = level_monitor_program_get_default_config(sm0_offset);
  pio_sm_config c_sm1 = time_stamp_program_get_default_config(sm1_offset);

  sm_config_set_clkdiv(&c_sm0, sm0_clkdiv);
  sm_config_set_clkdiv(&c_sm1, sm1_clkdiv);

  sm_config_set_in_pins(&c_sm0, in_base);
  sm_config_set_jmp_pin(&c_sm1, jmp_pin);

  sm_config_set_sideset(&c_sm0, 1, false, false);
  sm_config_set_sideset_pins(&c_sm0, sideset_base);

  pio_gpio_init(pio, sideset_base);
  pio_sm_set_consecutive_pindirs(pio, sm0, sideset_base, 1, true);

  sm_config_set_fifo_join(&c_sm0, PIO_FIFO_JOIN_RX);
  sm_config_set_fifo_join(&c_sm1, PIO_FIFO_JOIN_RX);

  pio_sm_init(pio, sm0, sm0_offset, &c_sm0);
  pio_sm_init(pio, sm1, sm1_offset, &c_sm1);

  for (uint32_t sm_i = 0; sm_i < 2; sm_i++)
  {
    for (uint32_t dma_i = 0; dma_i < SM_DMA_COUNT; dma_i++)
    {
      dma_chan_sm[sm_i][dma_i] = dma_claim_unused_channel(true);
    }
  }

  for (uint32_t sm_i = 0; sm_i < 2; sm_i++)
  {
    for (uint32_t dma_i = 0; dma_i < SM_DMA_COUNT; dma_i++)
    {
      dma_conf_sm[sm_i][dma_i] = dma_channel_get_default_config(dma_chan_sm[sm_i][dma_i]);
    }
  }

  for (uint32_t sm_i = 0; sm_i < 2; sm_i++)
  {
    for (uint32_t dma_i = 0; dma_i < SM_DMA_COUNT; dma_i++)
    {
      channel_config_set_read_increment(&dma_conf_sm[sm_i][dma_i], false);
    }
  }

  for (uint32_t sm_i = 0; sm_i < 2; sm_i++)
  {
    for (uint32_t dma_i = 0; dma_i < SM_DMA_COUNT; dma_i++)
    {
      channel_config_set_write_increment(&dma_conf_sm[sm_i][dma_i], true);
    }
  }

  for (uint32_t dma_i = 0; dma_i < SM_DMA_COUNT; dma_i++)
  {
    channel_config_set_dreq(&dma_conf_sm[0][dma_i], pio_get_dreq(pio, sm0, false));
  }
  for (uint32_t dma_i = 0; dma_i < SM_DMA_COUNT; dma_i++)
  {
    channel_config_set_dreq(&dma_conf_sm[1][dma_i], pio_get_dreq(pio, sm1, false));
  }

  for (uint32_t sm_i = 0; sm_i < 2; sm_i++)
  {
    for (uint32_t dma_i = 0; dma_i < SM_DMA_COUNT; dma_i++)
    {
      if (dma_i == SM_DMA_COUNT - 1)
      {
        channel_config_set_chain_to(&dma_conf_sm[sm_i][dma_i], dma_chan_sm[sm_i][0]);
      }
      else
      {
        channel_config_set_chain_to(&dma_conf_sm[sm_i][dma_i], dma_chan_sm[sm_i][dma_i + 1]);
      }
    }
  }

  for (uint32_t sm_i = 0; sm_i < 2; sm_i++)
  {
    for (uint32_t dma_i = 0; dma_i < SM_DMA_COUNT; dma_i++)
    {
      dma_channel_set_irq0_enabled(dma_chan_sm[sm_i][dma_i], true);
    }
  }

  irq_set_exclusive_handler(DMA_IRQ_0, isr0);
  irq_set_enabled(DMA_IRQ_0, true);

  for (uint32_t dma_i = 0; dma_i < SM_DMA_COUNT; dma_i++)
  {
    if (dma_i == 0)
    {
      dma_channel_configure(dma_chan_sm[0][dma_i], &dma_conf_sm[0][dma_i], sm_buffer[0][dma_i], &pio->rxf[sm0], SM_BUFFER_SIZE, true);
    }
    else
    {
      dma_channel_configure(dma_chan_sm[0][dma_i], &dma_conf_sm[0][dma_i], sm_buffer[0][dma_i], &pio->rxf[sm0], SM_BUFFER_SIZE, false);
    }
  }
  for (uint32_t dma_i = 0; dma_i < SM_DMA_COUNT; dma_i++)
  {
    if (dma_i == 0)
    {
      dma_channel_configure(dma_chan_sm[1][dma_i], &dma_conf_sm[1][dma_i], sm_buffer[1][dma_i], &pio->rxf[sm1], SM_BUFFER_SIZE, true);
    }
    else
    {
      dma_channel_configure(dma_chan_sm[1][dma_i], &dma_conf_sm[1][dma_i], sm_buffer[1][dma_i], &pio->rxf[sm1], SM_BUFFER_SIZE, false);
    }
  }

  pio_sm_set_enabled(pio, sm0, true);
  pio_sm_set_enabled(pio, sm1, true);
}
//do not put extra stuff in core1
void loop1()
{
  for (uint32_t dma_i = 0; dma_i < SM_DMA_COUNT; dma_i++)
  {
    if (sm_buffer_full[dma_i] == true)
    {
      for (uint32_t data_i = 0; data_i < SM_BUFFER_SIZE; data_i++)
      {
        queue_try_add(&sm0_queue, &sm_buffer[0][dma_i][data_i]);
        queue_try_add(&sm1_queue, &sm_buffer[1][dma_i][data_i]);
      }
      sm_buffer_full[dma_i] = false;
    }
  }
}

void isr0()
{
  for (uint32_t dma_i = 0; dma_i < SM_DMA_COUNT; dma_i++)
  {
    if (dma_channel_get_irq0_status(dma_chan_sm[0][dma_i]) && dma_channel_get_irq0_status(dma_chan_sm[1][dma_i]))
    {
      sm_buffer_full[dma_i] = true;
      dma_channel_configure(dma_chan_sm[0][dma_i], &dma_conf_sm[0][dma_i], sm_buffer[0][dma_i], &pio->rxf[sm0], SM_BUFFER_SIZE, false);
      dma_channel_configure(dma_chan_sm[1][dma_i], &dma_conf_sm[1][dma_i], sm_buffer[1][dma_i], &pio->rxf[sm1], SM_BUFFER_SIZE, false);
      dma_channel_acknowledge_irq0(dma_chan_sm[0][dma_i]);
      dma_channel_acknowledge_irq0(dma_chan_sm[1][dma_i]);
    }
  }
}
