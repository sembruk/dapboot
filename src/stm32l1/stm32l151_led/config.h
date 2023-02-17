#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#ifndef APP_BASE_ADDRESS
#   define APP_BASE_ADDRESS (0x08000000 + BOOTLOADER_OFFSET)
#endif
#ifndef FLASH_PAGE_SIZE
#   define FLASH_PAGE_SIZE  128
#endif
#ifndef DFU_UPLOAD_AVAILABLE
#   define DFU_UPLOAD_AVAILABLE 1
#endif
#ifndef DFU_DOWNLOAD_AVAILABLE
#   define DFU_DOWNLOAD_AVAILABLE 1
#endif
#define TARGET_DFU_WTRANSFERSIZE 128

#ifndef HAVE_LED
#   define HAVE_LED 1
#endif
#ifndef LED_OPEN_DRAIN
#   define LED_OPEN_DRAIN 1
#endif
#ifndef LED_GPIO_PORT
#   define LED_GPIO_PORT GPIOC
#endif
#ifndef LED_GPIO_PIN
#   define LED_GPIO_PIN GPIO13
#endif

#ifndef HAVE_BUTTON
#   define HAVE_BUTTON 1
#endif
#ifndef BUTTON_ACTIVE_HIGH
#   define BUTTON_ACTIVE_HIGH 1
#endif
#ifndef BUTTON_GPIO_PORT
#   define BUTTON_GPIO_PORT GPIOB
#endif
#ifndef BUTTON_GPIO_PIN
#   define BUTTON_GPIO_PIN GPIO2
#endif
// Blue-Pull has 100k resistors on PB2, so we can't use weak pulls to read it.
#ifndef BUTTON_USES_PULL
#   define BUTTON_USES_PULL 0
#endif

#ifndef BUTTON_SAMPLE_DELAY_CYCLES
#   define BUTTON_SAMPLE_DELAY_CYCLES 1440000
#endif


#define HAVE_USB_PULLUP_CONTROL 0

#ifndef USES_GPIOC
#   define USES_GPIOC 1
#endif

#endif
