This is a collection of programs for the STM32F103 microcontroller (Blue-Pill board), leveraging FreeRTOS as operating system.

<br>

## GPIO

### [Blinky](https://github.com/fabimass/stm32f103-freertos/tree/main/gpio/gpio_blinky)

The following program turns on and off the bluepill led periodically.

### [Button](https://github.com/fabimass/stm32f103-freertos/tree/main/gpio/gpio_button)

The following program turns on a led when you push a button, otherwise the led is off.

<br>

## ADC

### [Polling](https://github.com/fabimass/stm32f103-freertos/tree/main/adc/adc_poll)

The following code consists of 2 tasks:

- The first one periodically gets the value on the ADC input by using single conversion with polling method, then sends it to a queue.
- The second task reads the value from the queue and then turns on/off the led according to a threshold value.

### [Interrupt](https://github.com/fabimass/stm32f103-freertos/tree/main/adc/adc_isr)

The following code consists of 2 tasks:

- The first one periodically starts a new ADC conversion. The result will be handled via ISR, and the result will be written in a queue.
- The second task reads the value from the queue and then turns on/off the led according to a threshold value.

### [DMA](https://github.com/fabimass/stm32f103-freertos/tree/main/adc/adc_dma)

The following code consists of the following:

- The ADC is set to transfer conversions via DMA to a memory buffer. When the transfer is completed a semaphore will be released.
- A task that is waiting for the semaphore will then read all the values saved in memory, calculate the average, and finally will turn on/off the led according to a threshold value.

### [Reading multiple channels](https://github.com/fabimass/stm32f103-freertos/tree/main/adc/adc_mult)

The code consists of the following:

- The ADC is set to transfer the conversions of 2 channels to a memory buffer via DMA. When the transfer is completed a semaphore will be released.
- A task that is waiting for the semaphore will then read all the values saved in memory, calculate the average for each channel, and put the final values into a queue.
- Another task reads from that queue and then will turn on/off the leds according to the value from each channel (led 1 is tied to channel 1 and led 2 to channel 2).

<br>

## TIMER

### [Blinky](https://github.com/fabimass/stm32f103-freertos/tree/main/timer/timer_blinky)

The following program turns on and off the bluepill led periodically using a timer.

- The formula for the timer is: Tout= (Prescaler x Preload) / Clock Frequency
- When using 72 MHz Clock frequency and Prescaler value as 1000, for an output time of 500ms,
  the Preload value will come out to be (0.5 x 72 x 1000000) / 1000 = 36000

### [PWM](https://github.com/fabimass/stm32f103-freertos/tree/main/timer/timer_pwm)

The following program generates a pwm signal on one of the timer output pins.

- When the timer starts, the output is high, the timer will count until it reaches the Pulse value, then the output will go to low, and the timer will continue until it reaches the Preload value. Finally, the timer resets and the cycle starts all over.
- The pwm frequency is given by: freq= Clock Frequency / (Prescaler x Preload)
- When using 72 MHz Clock frequency and Preload value as 100, for a 1KHz pwm, the Prescaler value will come out to be (72 x 1000000)/(100 x 1000) = 720
- Then, duty cycle is given directly by the Pulse value

<br>

## SPI

### [TFT Display + Touchpad](https://github.com/fabimass/stm32f103-freertos/tree/main/spi/tft_display_tactile)

The following code was made to control a 240x320 TFT screen which uses the ILI9341 controller for the display, and the XPT2046 controller for the touch screen.

### [TFT Display + BMP image](https://github.com/fabimass/stm32f103-freertos/tree/main/spi/tft_display_image)

The following code was made to control a 240x320 TFT screen which uses the ILI9341 controller, it shows how to render an image on the screen.

### [SD Card](https://github.com/fabimass/stm32f103-freertos/tree/main/spi/sd_card)

The following program shows how to handle files within an SD card.

- First, it will create a binary file called datafile.dat; then, it will log the data in a text file; finally, it will read the binary file that was created before.
- FATFS configurations were done following this video: https://www.youtube.com/watch?v=aqSNz26Cuio
- SD Card filesystem must be FAT32.

<br>

## I2C

### [LCD Display](https://github.com/fabimass/stm32f103-freertos/tree/main/i2c/lcd_display)

The following program was made to control a 2x16 LCD through an I2C adapter. It shows a countdown timer on the display.

<br>

## UART

### [Echo](https://github.com/fabimass/stm32f103-freertos/tree/main/uart/uart_pc)

The following program shows how to establish a UART communication between the blue pill and a computer.

- The microcontroller waits for something to arrive from the computer, and then replies with the same data.
- On the computer side, I use [Termite](https://www.compuphase.com/software_termite.htm) as a RS232 terminal.

### [DMA](https://github.com/fabimass/stm32f103-freertos/tree/main/uart/uart_dma)

The following program receives data through UART and stores it in memory by using DMA.

### [WiFi](https://github.com/fabimass/stm32f103-freertos/tree/main/uart/wifi)

The following program uses the UART to communicate with an ESP8266 microcontroller, which will connect to a Firebase database over WiFi.

- The STM32 will send the `connect` command in order to connect to the WiFi network, and the ESP8266 will answer with `1` once the connection is successful.
- The STM32 will send the `get-led` command for the ESP8266 to query the database and return the led status.
- The STM32 will send the `set-count` command along with a value for the ESP8266 to update the database.
- The ESP8266 program was made in Arduino and is provided inside the ESP8266 folder.
