/*
ESP32-BT2PS2
Software for Espressif ESP32 chipsets for PS/2 emulation and Bluetooth BLE/Classic HID keyboard interfacing.
Thanks to all the pioneers who worked on the code libraries that made this project possible, check them on the README!
Copyright Humberto M�ckel - Hamcode - 2023
hamberthm@gmail.com
Dedicated to all who love me and all who I love.
Never stop dreaming.
*/

#include "nvs_flash.h"
#include "driver/gpio.h"
#include <iostream>
#include <cmath>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "string.h"

#include "../include/globals.hpp"
#include "../include/bt_keyboard.hpp"

static constexpr char const *TAG = "BTKeyboard";

#define I2C_MASTER_SCL_IO 22      /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO 23      /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */
#define I2C_SLAVE_ADDR 0x27       /*!< I2C slave address for the slave device */

// BTKeyboard section
BTKeyboard bt_keyboard;

int NumDigits(int x) // Returns number of digits in keyboard code
{
    x = abs(x);
    return (x < 10 ? 1 : (x < 100 ? 2 : (x < 1000 ? 3 : (x < 10000 ? 4 : (x < 100000 ? 5 : (x < 1000000 ? 6 : (x < 10000000 ? 7 : (x < 100000000 ? 8 : (x < 1000000000 ? 9 : 10)))))))));
}

void pairing_handler(uint32_t pid)
{
    int x = (int)pid;
    std::cout << "Please enter the following pairing code, "
              << std::endl
              << "followed by ENTER on your keyboard: "
              << pid
              << std::endl;

    for (int i = 0; i < 10; i++) // Flash quickly many times to alert user of incoming code display
    {
        gpio_set_level(GPIO_NUM_2, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    int dig = NumDigits(x); // How many digits does our code have?

    for (int i = 1; i <= dig; i++)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGW(TAG, "PAIRING CODE (TYPE ON KEYBOARD AND PRESS ENTER): %d ", pid);
        int flash = ((int)((pid / pow(10, (dig - i)))) % 10); // This extracts one ditit at a time from our code
        ESP_LOGI(TAG, "Flashing %d times", flash);
        for (int n = 0; n < flash; n++) // Flash the LED as many times as the digit
        {
            gpio_set_level(GPIO_NUM_2, 1);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            gpio_set_level(GPIO_NUM_2, 0);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

        if (flash < 1) // If digit is 0, keep a steady light for 1.5sec
        {
            gpio_set_level(GPIO_NUM_2, 1);
            vTaskDelay(1500 / portTICK_PERIOD_MS);
            gpio_set_level(GPIO_NUM_2, 0);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    for (int i = 0; i < 10; i++) // Quick flashes indicate end of code display
    {

        gpio_set_level(GPIO_NUM_2, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

extern "C"
{

// HID report structure for keyboards
    typedef struct {
        uint8_t modifiers;
        uint8_t reserved;
        uint8_t keys[6];
    } hid_keyboard_report_t;
  
  
    void i2c_master_init() {
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = I2C_MASTER_SDA_IO;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_io_num = I2C_MASTER_SCL_IO;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
        conf.clk_flags = 0;
        i2c_param_config(I2C_MASTER_NUM, &conf);
        i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

        ESP_LOGI(TAG, "I2C Initialized");
    }

    void i2c_master_send_text(const hid_keyboard_report_t *report) {
        size_t size = sizeof(hid_keyboard_report_t);

        ESP_LOGI(TAG, "Sending report");
        ESP_LOG_BUFFER_HEX(TAG, report, size);

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (I2C_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write(cmd, (uint8_t *)report, size, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    }

    void reset_report(hid_keyboard_report_t *report) {
        report->modifiers = 0;  // Reset modifiers
        for (int i = 0; i < sizeof(report->keys); i++) {
        report->keys[i] = 0;
        }
    }
   
    void app_main(void)
    {
        gpio_reset_pin(GPIO_NUM_2);                       // using built-in LED for notifications
        gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT); // Set the GPIO as a push/pull output
        gpio_set_level(GPIO_NUM_2, 1);
        
        i2c_master_init();
        gpio_set_level(GPIO_NUM_2, 0);

        // init BTKeyboard
        esp_err_t ret;

        // To test the Pairing code entry, uncomment the following line as pairing info is
        // kept in the nvs. Pairing will then be required on every boot.
        // ESP_ERROR_CHECK(nvs_flash_erase());

        ret = nvs_flash_init();
        if ((ret == ESP_ERR_NVS_NO_FREE_PAGES) || (ret == ESP_ERR_NVS_NEW_VERSION_FOUND))
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        if (bt_keyboard.setup(pairing_handler))
        { // Must be called once
            while (!bt_keyboard.devices_scan())
            {
                if (BTKeyboard::btFound)
                {
                    BTKeyboard::btFound = false;
                    for (int i = 0; i < 60; i++)
                    {
                        if (BTKeyboard::isConnected)
                            break;
                        ESP_LOGI(TAG, "Waiting for BT manual code entry...");
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                }
                else
                {
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    ESP_LOGI(TAG, "Rescan...");
                }
            }                              // Required to discover new keyboards and for pairing
                                           // Default duration is 5 seconds
            gpio_set_level(GPIO_NUM_2, 1); // success, device found
        }

        uint8_t typematicRate = 20;    // characters per second in Typematic mode

        // fixed stuff
        uint8_t cycle = 1000 / typematicRate;            // keywait timeout in ms. Important so we can check connection and do Typematic
        TickType_t repeat_period = pdMS_TO_TICKS(cycle); // keywait timeout in ticks. Important so we can check connection and do Typematic
        BTKeyboard::KeyInfo info;                        // freshly received
        BTKeyboard::KeyInfo infoBuf;                     // currently pressed
        bool found = false;                              // just an innocent flasg I mean flag

        info.modifier = infoBuf.modifier = (BTKeyboard::KeyModifier)0;

        for (int j = 0; j < BTKeyboard::MAX_KEY_COUNT; j++)
        {
            infoBuf.keys[j] = 0;
            info.keys[j] = 0;
        }


        while (true)
        {
            if (bt_keyboard.wait_for_low_event(info, repeat_period))
            {
                hid_keyboard_report_t report;
                reset_report(&report);

                report.modifiers = (uint8_t)info.modifier;

                // release the keys that have been just released
                for (int i = 0; i < BTKeyboard::MAX_KEY_COUNT; i++)
                {
                    if (!infoBuf.keys[i]) // Detect END FLAG
                        break;
                    for (int j = 0; j < BTKeyboard::MAX_KEY_COUNT; j++)
                    {
                        if (infoBuf.keys[i] == info.keys[j])
                        {
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                    {
                        ESP_LOGI(TAG, "Up key: %x", infoBuf.keys[i]);
                        gpio_set_level(GPIO_NUM_2, 1);

                        report.keys[i] = info.keys[i];
                    }
                    else
                        found = false;
                }

                // press the keys that have been just pressed
                for (int i = 0; i < BTKeyboard::MAX_KEY_COUNT; i++)
                {
                    if (!info.keys[i]) // Detect END FLAG
                        break;
                    for (int j = 0; (j < BTKeyboard::MAX_KEY_COUNT); j++)
                    {
                        if (info.keys[i] == infoBuf.keys[j])
                        {
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                    {
                        ESP_LOGI(TAG, "Down key: %x", info.keys[i]);
                        gpio_set_level(GPIO_NUM_2, 0);

                        report.keys[i] = info.keys[i];
                    }
                    else
                        found = false;
                }
                
                i2c_master_send_text(&report);

                infoBuf = info;                 // Now all the keys are handled, we save the state
            }

            else
            {
                while (!BTKeyboard::isConnected)
                {                                  // check connection
                    gpio_set_level(GPIO_NUM_2, 0); // disconnected
                    bt_keyboard.quick_reconnect(); // try to reconnect
                    vTaskDelay(250 / portTICK_PERIOD_MS);
                }
                gpio_set_level(GPIO_NUM_2, 1);
            }
        }
    }
}
