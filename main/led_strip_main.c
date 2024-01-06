/* RMT example -- RGB LED Strip

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "led_strip.h"
#include "rotary_encoder.h"

#include "math.h"

static const char *TAG = "example";

#define RMT_TX_CHANNEL RMT_CHANNEL_0

#define EXAMPLE_CHASE_SPEED_MS (20)

#define ENCODER_CPR (32)


/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

int32_t encoder_value_to_deg(int32_t enc_value){
    return (enc_value*360/ENCODER_CPR)/4;
}

uint8_t encoder_to_wheelstate(int32_t enc_angle){
    int32_t step_deg = 60;
    int32_t span = 25;

    uint8_t state = 0xFF;

    int32_t wrapped_angle = enc_angle % 360;
    if (wrapped_angle < 0) {
        wrapped_angle += 360;
    }

    for (uint8_t i = 0; i < 6; i++)
    {
        //enc_angle is between span
        if((wrapped_angle >= ((-span)+i*step_deg)) && (wrapped_angle <= ((span)+i*step_deg))){
            state = i;
        }
    }
    
    return state;    
  
}

void set_pixels(uint32_t red, uint32_t green, uint32_t blue, led_strip_t *strip){
    for (int j = 0; j < CONFIG_EXAMPLE_STRIP_LED_NUMBER; j++) {
        ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
    }
    ESP_ERROR_CHECK(strip->refresh(strip, 100));
}

static void pattern_RunningLights(uint32_t red, uint32_t green, uint32_t blue, int WaveDelay, led_strip_t *strip, rotary_encoder_t *encoder_a, rotary_encoder_t *encoder_b) {
  
    uint16_t num_pixels = CONFIG_EXAMPLE_STRIP_LED_NUMBER;
    int Position=0;

    uint8_t state_a_old = encoder_to_wheelstate(encoder_value_to_deg(encoder_a->get_counter_value(encoder_a)));


    for(int i=0; i<num_pixels*2; i++)
    {
        if(encoder_to_wheelstate(encoder_value_to_deg(encoder_b->get_counter_value(encoder_b))) != 2){
            break;  
        }
        if(encoder_to_wheelstate(encoder_value_to_deg(encoder_a->get_counter_value(encoder_a))) != state_a_old){
            break;
        }

        Position++; // = 0; //Position + Rate;
        for(int i=0; i<num_pixels; i++) {
        // sine wave, 3 offset waves make a rainbow!
        //float level = sin(i+Position) * 127 + 128;
        //setPixel(i,level,0,0);
        //float level = sin(i+Position) * 127 + 128;

        ESP_ERROR_CHECK(strip->set_pixel(strip, i, 
            ((sin(i+Position) * 127 + 128)/255)*(red), 
            ((sin(i+Position) * 127 + 128)/255)*(green), 
            ((sin(i+Position) * 127 + 128)/255)*(blue)));

        ESP_ERROR_CHECK(strip->refresh(strip, 100));
        }
        vTaskDelay(pdMS_TO_TICKS(WaveDelay));
        }
}

static void pattern_fade(uint32_t red, uint32_t green, uint32_t blue, int WaveDelay, led_strip_t *strip, int fadedelay, rotary_encoder_t *encoder_a, rotary_encoder_t *encoder_b) {
    uint16_t num_pixels = CONFIG_EXAMPLE_STRIP_LED_NUMBER;

    float fade = 1.0f;
    int8_t sign = -1;
    uint8_t state_a_old = encoder_to_wheelstate(encoder_value_to_deg(encoder_a->get_counter_value(encoder_a)));

    while(1){
        if(encoder_to_wheelstate(encoder_value_to_deg(encoder_b->get_counter_value(encoder_b))) != 1){
            break;  
        }
        if(encoder_to_wheelstate(encoder_value_to_deg(encoder_a->get_counter_value(encoder_a))) != state_a_old){
            break;
        }

        set_pixels((uint32_t)(red*fade), (uint32_t)(green*fade), (uint32_t)(blue*fade), strip);
        fade = fade + (sign*0.01f);
        vTaskDelay(pdMS_TO_TICKS(fadedelay));
        if(fade < 0.0f){
            sign = sign*-1;
            fade = 0.0f;
        }
        if(fade > 1.0f){
            sign = sign*-1;
            fade = 1.0f;
        }
    }
}


void app_main(void)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;
    int8_t sign = 1;

    // Rotary encoder underlying device is represented by a PCNT unit in this example
    uint32_t pcnt_unit_a = 0;

    // Create rotary encoder instance
    rotary_encoder_config_t config_enc1 = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit_a, 10, 9);
    rotary_encoder_t *encoder_a = NULL;
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_enc1, &encoder_a));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder_a->set_glitch_filter(encoder_a, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder_a->start(encoder_a));

    // Rotary encoder underlying device is represented by a PCNT unit in this example
    uint32_t pcnt_unit_b = 1;

    // Create rotary encoder instance
    rotary_encoder_config_t config_enc2 = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit_b, 25, 26);
    rotary_encoder_t *encoder_b = NULL;
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_enc2, &encoder_b));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder_b->set_glitch_filter(encoder_b, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder_b->start(encoder_b));



    rmt_config_t config_led = RMT_DEFAULT_CONFIG_TX(CONFIG_EXAMPLE_RMT_TX_GPIO, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config_led.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config_led));
    ESP_ERROR_CHECK(rmt_driver_install(config_led.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(CONFIG_EXAMPLE_STRIP_LED_NUMBER, (led_strip_dev_t)config_led.channel);
    led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));
    red =  125;

    while (true) {

        // Flush RGB values to LEDs
        
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // strip->clear(strip, 50);
        // vTaskDelay(pdMS_TO_TICKS(1000));
        
        //ESP_LOGI(TAG, "Encoder a deg: %d", (encoder_value_to_deg(encoder_a->get_counter_value(encoder_a))));
        uint8_t state_a = encoder_to_wheelstate(encoder_value_to_deg(encoder_a->get_counter_value(encoder_a)));
        //ESP_LOGI(TAG, "wheel a state: %d", (state_a));

        //ESP_LOGI(TAG, "Encoder b deg: %d", (encoder_value_to_deg(encoder_b->get_counter_value(encoder_b))));
        uint8_t state_b = encoder_to_wheelstate(encoder_value_to_deg(encoder_b->get_counter_value(encoder_b)));


        if(state_a != 255){
            led_strip_hsv2rgb(60*state_a, 100, 60, &red, &green, &blue);        //Set v to low (20) when hooking up with pc usb port. Otherwise brownout could occur
            if(state_b == 0){   //only set pixels if state_b is 0
                set_pixels(red, green, blue, strip);
            }
        }

        switch (state_b)
        {
        case 0:
            //Do nothing
            vTaskDelay(pdMS_TO_TICKS(10));
            break;
        case 1:
            pattern_fade(red, green, blue, 10, strip, 10, encoder_a, encoder_b);
            break;
        case 2: 
            pattern_RunningLights(red, green, blue, 10, strip, encoder_a, encoder_b);
            vTaskDelay(pdMS_TO_TICKS(10));
            break;

        case 3: 
            vTaskDelay(pdMS_TO_TICKS(10));
            break;

        case 4:
            vTaskDelay(pdMS_TO_TICKS(10));
            break;
        
        case 5:
            vTaskDelay(pdMS_TO_TICKS(10));
            break;

        default:
            vTaskDelay(pdMS_TO_TICKS(10));
            break;
        }

        //ESP_LOGI(TAG, "wheel a state: %d", (state_a));
        //ESP_LOGI(TAG, "wheel b state: %d", (state_b));

        //vTaskDelay(pdMS_TO_TICKS(1000));

    }

}
