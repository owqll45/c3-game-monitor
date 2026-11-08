idf_component_register(SRCS "main.c" "u8g2font.c" "efont_data.c" "xbox_ble.c" "icons.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_driver_spi esp_driver_gpio esp_timer
                                     esp_wifi esp_netif esp_event nvs_flash bt esp_adc)
