#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "sdkconfig.h"

//#define SDA_PIN 22 
//#define SCL_PIN 21 

#define SDA_PIN 18
#define SCL_PIN 19


#define ACK_VAL 0x0              
#define NACK_VAL 0x1             
#define LSM6DS3_SENSOR_ADDR 0x6b
#define ACK_CHECK_EN 0x1
#define WRITE_BIT I2C_MASTER_WRITE 
#define READ_BIT I2C_MASTER_READ
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0

#define WHO_AM_I 0xf
#define WHO_AM_I_LSM6DS3 0x69
#define FUNC_CFG_ACCESS 0x1
#define CTRL6_C 0x15
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL9_XL 0x18
#define FIFO_CTRL5 0x0A
#define ODR_XL_104HZ 0x40
#define CTRL2_G 0x11

#define OUT_TEMP_L 0x20
#define OUT_TEMP_H 0x21

#define OUTX_L_G 0x22
#define OUTX_H_G 0x23

#define OUTY_L_G 0x24
#define OUTY_H_G 0x25

#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27

#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29

#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B

#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D

int LSM6DS3_ADDR;

static char tag[] = "i2cscanner";

int InitCommunication(uint8_t func_addr) {
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, LSM6DS3_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, func_addr, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	
	return ret;
}



int ReadOneByteData(uint8_t func_addr, uint8_t *data) {
	int ret;
	/*Begin communication with LSM6DS3*/
	ret = InitCommunication(func_addr);
	if (ret != ESP_OK) {
		printf("WRITE ERROR\n");
		return ret;
	}

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, LSM6DS3_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, data, NACK_VAL);
	
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	if (ret != ESP_OK) {
		printf("WRITE ERROR read\n");
		return ret;
	}
	return ret;
}

int ReadMultiByteData(uint8_t func_addr, uint8_t *data,uint8_t sdata) {
	int ret,i;
	/*Begin communication with LSM6DS3*/
	ret = InitCommunication(func_addr);
	if (ret != ESP_OK) {
		printf("WRITE ERROR\n");
		return ret;
	}

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, LSM6DS3_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
	for(i=0;i<sdata-1;i++){
	i2c_master_read_byte(cmd, data + i, ACK_VAL);
	}
	i2c_master_read_byte(cmd, data + i, NACK_VAL);

	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	if (ret != ESP_OK) {
		printf("WRITE ERROR read\n");
		return ret;
	}
	return ret;
}


int WriteOneByteData(uint8_t func_addr, uint8_t data) {
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, LSM6DS3_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, func_addr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, data, NACK_VAL);

	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	if (ret != ESP_OK) {
		printf("WRITE ERROR in write\n");
		return ret;
	}

	return ret;
}

void task_i2cscanner(void *ignore) {
	ESP_LOGD(tag, ">> i2cScanner");
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = SDA_PIN;
	conf.scl_io_num = SCL_PIN;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	i2c_param_config(I2C_NUM_0, &conf);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

	int i;
	//int ret;
	LSM6DS3_ADDR = 0;
	esp_err_t espRc;
	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	printf("00:         ");
	for (i = 3; i< 0x78; i++) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
		i2c_master_stop(cmd);

		espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
		if (i % 16 == 0) {
			printf("\n%.2x:", i);
		}
		if (espRc == 0) {
			printf(" %.2x", i);
			LSM6DS3_ADDR = i;
		}
		else {
			printf(" --");
		}
		
		i2c_cmd_link_delete(cmd);
	}

	printf("END\n");
	printf(" %.2x\n", LSM6DS3_ADDR);

	uint8_t data,data1,buf[10];
	data = 0;
	ReadOneByteData(WHO_AM_I, &data);
	data = 0;
	ReadOneByteData(WHO_AM_I, &data);
	printf("WHO_AM_I: %.2x\n", data);
//====================================================================================
	// Disable access to embedded functions
	WriteOneByteData(FUNC_CFG_ACCESS, 0x00);

	WriteOneByteData(FIFO_CTRL5, 0x00);

	
	uint8_t CTRL9_val = (1 << 3) | (1 << 4) | (1 << 5);
	WriteOneByteData(CTRL9_XL, CTRL9_val);

	WriteOneByteData(CTRL6_C, 0x00);

	WriteOneByteData(CTRL1_XL, ODR_XL_104HZ);
	
	WriteOneByteData(CTRL2_G, ODR_XL_104HZ);
	
//====================================================================================

	ReadOneByteData(CTRL6_C, &data);
	printf("Power mode XL: %.2x\n", data); 

	ReadOneByteData(FIFO_CTRL5, &data);
	printf("FIFO XL: %.2x\n", data);

	//Check power status of accelerometer
	ReadOneByteData(CTRL1_XL, &data);
	printf("accelerometer power status: %.2x\n", data);

	//Check power status of gyroscope
	ReadOneByteData(CTRL2_G, &data);
	printf("gyroscope power status: %.2x\n", data);

	printf("TEMPERATURE + ACCELOMETER\n");
	// Disable access to embedded functions
	float temperature,ax,ay,az,gx,gy,gz;
	float x_fact = 0.00006103515625;
	float raddeg = 180 / 3.14159;
	float g_fact = 4.375e-3*raddeg;
	
	while (1) {
		data = 0;
		data1 = 0;
		ReadOneByteData(OUT_TEMP_L, &data);
		ReadOneByteData(OUT_TEMP_H, &data1);
	//Need to get in one block (multiByteData)	
		temperature = (float)(((int16_t)(data1 << 8) | data) * 0.0625) + 25.0;

		ReadMultiByteData(OUTX_L_G, buf, 12);

		gx = (float)(((int16_t)(buf[1] << 8) | buf[0])*g_fact);
		gy = (float)(((int16_t)(buf[3] << 8) | buf[2])*g_fact);
		gz = (float)(((int16_t)(buf[5] << 8) | buf[4])*g_fact);

		ax = (float)(((int16_t)(buf[7] << 8) | buf[6]) * x_fact);
		ay = (float)(((int16_t)(buf[9] << 8) | buf[8]) * x_fact);
		az = (float)(((int16_t)(buf[11] << 8) | buf[10]) * x_fact);

		printf("TEMPERATURE: %f  a: (%f, %f, %f) g: %.2f, %.2f, %.2f\r", temperature,ax,ay,az,gx,gy,gz);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	
}

void app_main()
{
	task_i2cscanner(NULL);
	for (int i = 5; i >= 0; i--) {
		printf("Restarting in %d seconds...\n", i);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	printf("Restarting now.\n");
	fflush(stdout);
	esp_restart();
}