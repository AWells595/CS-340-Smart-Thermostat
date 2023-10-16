
/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Code created by Alex Wells
 *
 * This code is designed to function as a smart thermostat, taking a temperature reading from the board and comparing
 * it against an adjustable set temperature and turning on the heat if it's below that set temperature
 *
 * The heat is simulated by turning on the red LED on the board.
 * Button on right of board raises set temperature, button on the left lowers it.
 *
 * Task scheduler is implemented to handle the 3 different periods required for full functionality
 *
 * UART is used to simulate transferring data to cloud for IOT functionality
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* Preprocessor instructions */
#define DISPLAY(x) UART_write(uart, &output, x);

// Global control variables
int setPoint = 0; // target temp for heat, if reached heat turns off
int temp = 0; // temp in Celsius as captured by the I2C reading of the temperature sensor
int secondsPassed = 0; // number of seconds elapsed since start of program
char isHeatOn = 0; // flag to turn on heat
char shouldLowerSetPoint = 0; // flag to recognize button press to lower set temp
char shouldRaiseSetPoint = 0; // flag to recognize button press to raise set temp

// Global constants
const int NUM_TASKS = 3;
const int MILLISECONDS_IN_1_SECOND = 1000;

// Driver Handles - Global Variables
I2C_Handle i2c;
Timer_Handle timer0;
UART_Handle uart;


/**
 * Task type has 2 attributes, period which is the period in milliseconds that should elapse between the task being completed
 * and TickFct which is a pointer to the function that will be called after the period elapses
 */
typedef struct Task {
    unsigned long period;
    void (*TickFct)();
} Task;


/**
 * Checks the flags raised by the button inputs, if a flag is raised it either lowers or raises the setPoint value by 1
 * Design specification does not indicate that one takes precedence over the other so if both buttons are pressed in a 200ms period
 * the setPoint will both raise and lower itself by 1, negating the change
 */
void checkButtonFlags(){
    if(shouldLowerSetPoint) {
        setPoint--;
        shouldLowerSetPoint = 0;
    }
    if(shouldRaiseSetPoint){
        setPoint++;
        shouldRaiseSetPoint = 0;
    }
}


// UART Global Variables
char output[64];
int bytesToSend;
void initUART(void)
{
    UART_Params uartParams;
    // Init the driver
    UART_init();
    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;
    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}


// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
char *id;
} sensors[3] = {
                { 0x48, 0x0000, "11X" },
                { 0x49, 0x0000, "116" },
                { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;



// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))
    // Init the driver
    I2C_init();
    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"))
    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
            DISPLAY(snprintf(output, 64, "No\n\r"))
     }
     if(found)
     {
         DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
     }
     else
     {
         DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
     }
}

int16_t readTemp(void)
{
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;
}


volatile unsigned char TimerFlag = 0;
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}
void initTimer(void)
{
    Timer_Params params;
    // Init the driver
    Timer_init();
    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000; // 100ms greatest common factor between 200ms, 500ms, and 1000ms
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

/**
 * Sends the required outputs to the UART
 */
void sendInfoToUART(){
    DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temp, setPoint, isHeatOn, secondsPassed));
}


/**
 * Reads the temperature from the I2C's temperature sensor and determines if heat should be on or off based on that reading
 * If heat is on LED is turned on to indicate so else the LED is turneed off
 */
void checkTempAndIfHeatOn(){
    temp = readTemp();
    if(temp < setPoint) {
        isHeatOn = 1;
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    }
    else{
        isHeatOn = 0;
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    }
}



/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void lowerTempButton(uint_least8_t index)
{
    // lower temp
    shouldLowerSetPoint = 1;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void raiseTempButton(uint_least8_t index)
{
    // raise temp
    shouldRaiseSetPoint = 1;
}

/**
 * Initializes GPIO and configures buttons and button callbacks
 */
void initGPIO(){
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, lowerTempButton);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, raiseTempButton);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    initUART();
    initTimer();
    initI2C();
    initGPIO();

    /* local variables */
    int timeInMiliseconds = 0;
    Task tasks[NUM_TASKS];

    /* Configure tasks */
    // displayData
    tasks[0].period = 1000;
    tasks[0].TickFct = &sendInfoToUART;

    // tempHandler
    tasks[1].period = 500;
    tasks[1].TickFct = &checkTempAndIfHeatOn;

    // buttonChecker
    tasks[2].period = 200;
    tasks[2].TickFct = &checkButtonFlags;

    // main loop
    while(1){
        while(!TimerFlag){} // waits for callback
        /*
         * Schedule as follows:
         *
         * Every 200ms check button flags and adjust setPoint, task[2]
         * Every 500ms read temperature and determine if heat should be on or off, task[1]
         * Every Second send data to the UART for display, task[0]
         */
        timeInMiliseconds = timeInMiliseconds + 100;
        int i;
        // task scheduler
        for(i = 0; i < NUM_TASKS; i++) {
            if(timeInMiliseconds % tasks[i].period == 0) {
                if(timeInMiliseconds >= MILLISECONDS_IN_1_SECOND) {
                    timeInMiliseconds = 0; // reset to prevent possible but unlikely overflow
                    secondsPassed++;
                 }
                tasks[i].TickFct();
            }
        }
        // reset timer flag
        TimerFlag = 0;
    }
}
