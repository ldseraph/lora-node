#include "adxl355.h"

#include <math.h>
#define DBG_SECTION_NAME "adxl355"
#include <ulog.h>

/* ADXL355 registers addresses */
#define DEVID_AD 0x00
#define DEVID_MST 0x01
#define PARTID 0x02
#define REVID 0x03
#define STATUS 0x04
#define FIFO_ENTRIES 0x05
#define TEMP2 0x06
#define TEMP1 0x07
#define XDATA3 0x08
#define XDATA2 0x09
#define XDATA1 0x0A
#define YDATA3 0x0B
#define YDATA2 0x0C
#define YDATA1 0x0D
#define ZDATA3 0x0E
#define ZDATA2 0x0F
#define ZDATA1 0x10
#define FIFO_DATA 0x11
#define OFFSET_X_H 0x1E
#define OFFSET_X_L 0x1F
#define OFFSET_Y_H 0x20
#define OFFSET_Y_L 0x21
#define OFFSET_Z_H 0x22
#define OFFSET_Z_L 0x23
#define ACT_EN 0x24
#define ACT_THRESH_H 0x25
#define ACT_THRESH_L 0x26
#define ACT_COUNT 0x27
#define FILTER 0x28
#define FIFO_SAMPLES 0x29
#define INT_MAP 0x2A
#define SYNC 0x2B
#define RANGE 0x2C
#define POWER_CTL 0x2D
#define SELF_TEST 0x2E

#define ADXl355_READ_THREE_REG 3

static rt_err_t sensor_adxl355_write_one_byte(
  sensor_adxl355_t *sensor_adxl355,
  uint8_t           reg,
  uint8_t           value) {
  uint16_t  addr    = sensor_adxl355->device->device_addr;
  uint8_t   write[] = { reg, value };
  rt_size_t size    = rt_device_write(sensor_adxl355->i2c_bus, addr, write, sizeof(write));
  if (size == 0) {
    LOG_E("write i2c reg(0x%x) value(0x%x) err", reg, value);
    return -RT_ERROR;
  }

  return RT_EOK;
}

static rt_err_t sensor_adxl355_read_byte(
  sensor_adxl355_t *sensor_adxl355,
  uint8_t           reg,
  uint8_t          *buffer,
  uint32_t          len) {
  uint16_t addr = sensor_adxl355->device->device_addr;

  rt_size_t size = rt_device_write(sensor_adxl355->i2c_bus, addr, &reg, sizeof(reg));
  if (size == 0) {
    LOG_E("start i2c reg(0x%x) write err", reg);
    return -RT_ERROR;
  }

  size = rt_device_read(sensor_adxl355->i2c_bus, addr, buffer, len);
  if (size == 0) {
    LOG_E("start i2c reg(0x%x) read err", reg);
    return -RT_ERROR;
  }

  return RT_EOK;
}

static rt_err_t sensor_adxl355_read_one_byte(
  sensor_adxl355_t *sensor_adxl355,
  uint8_t           reg,
  uint8_t          *buffer) {
  return sensor_adxl355_read_byte(sensor_adxl355, reg, buffer, 1);
}

static rt_err_t sensor_adxl355_power_up(sensor_adxl355_t *sensor_adxl355) {
  uint8_t  power_value;
  rt_err_t err = sensor_adxl355_read_one_byte(sensor_adxl355, POWER_CTL, &power_value);
  if (err != RT_EOK) {
    LOG_E("start read POWER_CTL err(%d)", err);
    return err;
  }
  power_value &= 0xFE;

  err = sensor_adxl355_write_one_byte(sensor_adxl355, POWER_CTL, power_value);
  if (err != RT_EOK) {
    LOG_E("start write POWER_CTL err(%d)", err);
    return err;
  }

  return RT_EOK;
}

static rt_err_t sensor_adxl355_power_down(sensor_adxl355_t *sensor_adxl355) {
  rt_err_t err = sensor_adxl355_write_one_byte(sensor_adxl355, POWER_CTL, 0x01);
  if (err != RT_EOK) {
    LOG_E("start write POWER_CTL err(%d)", err);
    return err;
  }

  return RT_EOK;
}

static float sensor_adxl355_data_conversion(uint8_t *raw, float scale) {
  int32_t sensor_data = raw[0] << 12 | raw[1] << 4 | (raw[2] >> 4);

  if (sensor_data & 0x00080000) {
    sensor_data |= 0xFFF00000;
  }
  
  return (float)(sensor_data / scale);
}

static rt_size_t sensor_adxl355_read(
  rt_device_t dev,
  rt_off_t    pos,
  void       *buffer,
  rt_size_t   max_size) {
  rt_err_t               err;
  sensor_adxl355_t      *sensor_adxl355 = (sensor_adxl355_t *)dev;
  sensor_adxl355_data_t *data           = buffer;

  uint8_t sensorX[ADXl355_READ_THREE_REG] = { 0 };
  uint8_t sensorY[ADXl355_READ_THREE_REG] = { 0 };
  uint8_t sensorZ[ADXl355_READ_THREE_REG] = { 0 };

  err = sensor_adxl355_power_up(sensor_adxl355);
  if (err != RT_EOK) {
    LOG_E("sensor read power up err(%d)", err);
    return err;
  }

  while ((sensorX[0] == 0) && (sensorX[1] == 0) && (sensorX[2] == 0)) {
    rt_thread_mdelay(50);
    err = sensor_adxl355_read_byte(sensor_adxl355, XDATA3, sensorX, ADXl355_READ_THREE_REG);
    if (err != RT_EOK) {
      LOG_E("start read XDATA3 err(%d)", err);
      return err;
    }

    err = sensor_adxl355_read_byte(sensor_adxl355, YDATA3, sensorY, ADXl355_READ_THREE_REG);
    if (err != RT_EOK) {
      LOG_E("start read YDATA3 err(%d)", err);
      return err;
    }

    err = sensor_adxl355_read_byte(sensor_adxl355, ZDATA3, sensorZ, ADXl355_READ_THREE_REG);
    if (err != RT_EOK) {
      LOG_E("start read ZDATA3 err(%d)", err);
      return err;
    }
  }

  err = sensor_adxl355_power_down(sensor_adxl355);
  if (err != RT_EOK) {
    LOG_E("sensor read power down err(%d)", err);
    return err;
  }

  float fSensorX = sensor_adxl355_data_conversion(sensorX,sensor_adxl355->scale);
  float fSensorY = sensor_adxl355_data_conversion(sensorY,sensor_adxl355->scale);
  float fSensorZ = sensor_adxl355_data_conversion(sensorZ,sensor_adxl355->scale);

  data->xangle = atan2(fSensorX, sqrt(fSensorY * fSensorY + fSensorZ * fSensorZ)) * (180 / 3.1415926);
  data->yangle = atan2(fSensorY, sqrt(fSensorX * fSensorX + fSensorZ * fSensorZ)) * (180 / 3.1415926);
  data->zangle = atan2(sqrt(fSensorX * fSensorX + fSensorY * fSensorY), fSensorZ) * (180 / 3.1415926);

  return sizeof(sensor_adxl355_data_t);
}

static rt_err_t sensor_adxl355_set_range(sensor_adxl355_t *sensor_adxl355, uint16_t range) {
  rt_err_t err = RT_EOK;
  uint8_t  reg_range;
  switch (range) {
  case 2:
    reg_range             = 0x81;
    sensor_adxl355->scale = 256000.0f;
    break;
  case 4:
    reg_range             = 0x82;
    sensor_adxl355->scale = 128000.0f;
    break;
  case 8:
  default:
    reg_range             = 0x83;
    sensor_adxl355->scale = 64000.0f;
    break;
  }
  err = sensor_adxl355_write_one_byte(sensor_adxl355, RANGE, reg_range);
  if (err != RT_EOK) {
    LOG_E("set range(%d) err(%d)", range, err);
    return err;
  }

  return RT_EOK;
}

static rt_err_t sensor_adxl355_init(rt_device_t dev) {
  rt_err_t          err            = RT_EOK;
  sensor_adxl355_t *sensor_adxl355 = (sensor_adxl355_t *)dev;

  err = rt_device_open(sensor_adxl355->i2c_bus, RT_DEVICE_OFLAG_RDONLY);
  if (err != RT_EOK) {
    LOG_E("%s: open err: %d.", RT_DEVICE_OFLAG_RDONLY, err);
    return err;
  }

  err = sensor_adxl355_write_one_byte(sensor_adxl355, FILTER, 0x08);
  if (err != RT_EOK) {
    LOG_E("set filter err(%d)", err);
    return err;
  }

  err = sensor_adxl355_set_range(sensor_adxl355, sensor_adxl355->device->range);
  if (err != RT_EOK) {
    LOG_E("set range err(%d)", err);
    return err;
  }

  return err;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops sensor_adxl355_ops = {
  sensor_adxl355_init,
  RT_NULL,
  RT_NULL,
  sensor_adxl355_read,
  RT_NULL,
  RT_NULL
};
#endif

static rt_err_t sensor_adxl355_probe(devices_t *devices) {
  RT_ASSERT(RT_NULL != devices);

  for (uint32_t i = 0; i < devices->num; i++) {
    device_t *d = &(devices->list[i]);

    sensor_adxl355_device_t *device = d->device;
    if (RT_NULL == device) {
      LOG_E("sensor_adxl355_device is null");
      return -RT_EINVAL;
    }

    sensor_adxl355_t *sensor_adxl355 = rt_malloc(sizeof(sensor_adxl355_t));
    if (RT_NULL == sensor_adxl355) {
      LOG_E("OOM sensor adxl355");
      return -RT_ENOMEM;
    }

    rt_device_t i2c_bus = rt_device_find(device->i2c_bus);
    if (i2c_bus == RT_NULL) {
      LOG_E("%s find err ", device->i2c_bus);
      return -RT_EINVAL;
    }
    sensor_adxl355->i2c_bus = i2c_bus;
    sensor_adxl355->device  = device;

    struct rt_device *parent = &(sensor_adxl355->parent);

    parent->type        = RT_Device_Class_SPIDevice;
    parent->rx_indicate = RT_NULL;
    parent->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    parent->ops = &sensor_adxl355_ops;
#else
    parent->init    = sensor_adxl355_init;
    parent->open    = RT_NULL;
    parent->close   = RT_NULL;
    parent->read    = sensor_adxl355_read;
    parent->write   = RT_NULL;
    parent->control = RT_NULL;
#endif

    rt_err_t err = rt_device_register(parent, d->name, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
    if (err != RT_EOK) {
      LOG_E("%s register err %d ", d->name, err);
      return err;
    }
  }

  return RT_EOK;
}

// TODO: dts
static device_t init_devices[] = {
  {
    .name   = "adxl355:0",
    .device = &(sensor_adxl355_device_t){
      .i2c_bus     = "i2c-bus0",
      .device_addr = 0x1D,
      .range       = 8,
    },
  }
};

static int sensor_adxl355_register() {
  devices_t devices = {
    .list = init_devices,
    .num  = (sizeof(init_devices) / sizeof(init_devices[0])),
  };

  rt_err_t err = sensor_adxl355_probe(&devices);
  if (err != RT_EOK) {
    return err;
  }
  return RT_EOK;
}

INIT_DEVICE_EXPORT(sensor_adxl355_register);