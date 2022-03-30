

/**
 * main.c
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "inc/hw_sysctl.h"
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"



#include <math.h>

// Se agregan las nuevas librerias para i2c y para el mpu6050
#include "hw_mpu6050.h"
#include "i2cm_drv.h"
#include "mpu6050.h"


//#include "uart.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"

//*********
// Definiciones para configuración del SPI
//*********
#define NUM_SPI_DATA    1  // Número de palabras que se envían cada vez
#define SPI_FREC  4000000  // Frecuencia para el reloj del SPI
#define SPI_ANCHO      16  // Número de bits que se envían cada vez, entre 4 y 16
#define frecuenciamuestreo 1000

uint16_t dato;  // dato enviado por SPI
uint16_t i=0;
uint16_t contador = 0;

// DEFINICION DE VARIABLES

float ref=0, u_k=0;

float e_k_1 = 0, e_k = 0, E_k = 0, eD = 0, wf0, wf1, wf2;
float kP=1.0 , kI=0, kD=0 ;
uint16_t freq_timer = 1000;    // Velocidad con la que se envian datos en Hz

uint16_t u_kint;

int16_t AccelX= 0, AccelY = 0, AccelZ = 0, gyroX = 0, gyroY = 0, gyroZ = 0;

float x = 0.0, y = 0, z = 0;
//
// A boolean that is set when a MPU6050 command has completed.
//
volatile bool g_bMPU6050Done;

//
// I2C master instance
//
tI2CMInstance g_sI2CMSimpleInst;

//
//Device frequency
//
int clockFreq;


//*********
// The interrupt handler for the timer interrupt.
//*********
void
Timer0IntHandler(void)
{

    // Notar que ahora necesitamos dos espacios para las conversiones.
    uint32_t pui32ADC0Value[2];


    uint32_t pui32DataTx[NUM_SPI_DATA]; // la función put pide tipo uint32_t
    uint8_t ui32Index;

    // Clear the timer interrupt. Necesario para lanzar la próxima interrupción.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    // Trigger the ADC conversion.
    ADCProcessorTrigger(ADC0_BASE, 2);

      // Wait for conversion to be completed.
      while(!ADCIntStatus(ADC0_BASE, 2, false))
      {
      }

      // Clear the ADC interrupt flag.
      ADCIntClear(ADC0_BASE, 2);

      ADCSequenceDataGet(ADC0_BASE, 2, pui32ADC0Value);

      e_k = ref-x;
      eD = e_k-e_k_1;
      u_k = kP*e_k + kI*E_k/freq_timer + kD*eD*freq_timer;
      E_k = E_k + e_k;
      e_k_1 = e_k;


      if (u_k> 6) // Se acota la salida para que se encuentre entre -6 y 6
         u_k = 6;
      else if (u_k < -6)
          u_k = -6;

      u_k = u_k*4095/12+2047.5;

      u_kint=(uint16_t)(u_k);

    dato=0b0111000000000000|u_kint; // Enviamos el dato al DAC

    pui32DataTx[0] = (uint32_t)(dato);



////////////////////////////////////////////////

    if (contador < 100) {
        contador ++;

    }
    else{
             UARTprintf(" %d \n ", (int)x);
        contador = 0;
    }


////////////////////////////////////////////////

    // Send data
    for(ui32Index = 0; ui32Index < NUM_SPI_DATA; ui32Index++)
    {
        // Send the data using the "blocking" put function.  This function
        // will wait until there is room in the send FIFO before returning.
        // This allows you to assure that all the data you send makes it into
        // the send FIFO.
        SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
    }

    // Wait until SSI0 is done transferring all the data in the transmit FIFO.
    while(SSIBusy(SSI0_BASE))
    {
    }

}



void InitI2C0(void)
{
    //enable del modulo I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable de los puertos del I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configuracion de puertos B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Asigna comunicacion I2C a los pines.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Activa e inicializa modulo I2C.
    // Se usa el system clock
    // Transmision de datos a 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    // Limpieza de FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    // Inicia I2C master driver
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, SysCtlClockGet());

}


void ConfigureUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // pines pertenecientes al UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // UART0 comunica con la computadora
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000); // UART0 a 115200 baudrate, frecuencia de 16MHz
}

void delayMS(int ms) {
    //ROM_SysCtlDelay( (ROM_SysCtlClockGet()/(3*1000))*ms ) ;  // more accurate
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;  // less accurate
}

//
// The function that is provided by this example as a callback when MPU6050
// transactions have completed.
//
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // See if an error occurred.
    //
    if (ui8Status != I2CM_STATUS_SUCCESS)
    {
        //
        // An error occurred, so handle it here if required.
        //
    }
    //
    // Indicate that the MPU6050 transaction has completed.
    //
    g_bMPU6050Done = true;
}

//
// The interrupt handler for the I2C module.
//
void I2CMSimpleIntHandler(void)
{
    //
    // Call the I2C master driver interrupt handler.
    //
    I2CMIntHandler(&g_sI2CMSimpleInst);
}



//
// Configuracion del muestreo del MPU.
//
void MPU6050Example(void)
{
    uint_fast16_t pui16Accel[3], pui16Gyro[3];
    float fAccel[3], fGyro[3];
    tMPU6050 sMPU6050;
   // uint_fast16_t AccelX = 0, AccelY = 0, AccelZ = 0, gyroX = 0, gyroY = 0, gyroZ = 0;



    //
    // Initialize the MPU6050. This code assumes that the I2C master instance
    // has already been initialized.
    //
    g_bMPU6050Done = false;
    MPU6050Init(&sMPU6050, &g_sI2CMSimpleInst, 0x68, MPU6050Callback, &sMPU6050); // 0x68 slave adress
    while (!g_bMPU6050Done)
    {
    }

    //
    // Configura el rango del acelerometro del MPU a +/- 2 g.
    //
    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_ACCEL_CONFIG, ~MPU6050_ACCEL_CONFIG_AFS_SEL_M,
        MPU6050_ACCEL_CONFIG_AFS_SEL_2G, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }


    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_1, 0x00, 0b00000010 & MPU6050_PWR_MGMT_1_DEVICE_RESET, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_2, 0x00, 0x00, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }


    while (1)
    {
        //
        // Request another reading from the MPU6050.
        //
        g_bMPU6050Done = false;
        MPU6050DataRead(&sMPU6050, MPU6050Callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {
        }
        //
        // Get the new accelerometer and gyroscope readings.
        //
        MPU6050DataAccelGetFloat(&sMPU6050, &fAccel[0], &fAccel[1], &fAccel[2]);
        MPU6050DataGyroGetFloat(&sMPU6050, &fGyro[0], &fGyro[1], &fGyro[2]);
        //
        MPU6050DataAccelGetRaw(&sMPU6050, &pui16Accel[0], &pui16Accel[1], &pui16Accel[2]);
        MPU6050DataGyroGetRaw(&sMPU6050, &pui16Gyro[0], &pui16Gyro[1], &pui16Gyro[2]);
        // Do something with the new accelerometer and gyroscope readings.
        //

        // datos en bruto del MPU
        AccelX = pui16Accel[0];
        AccelY = pui16Accel[1];
        AccelZ = pui16Accel[2];

        gyroX = pui16Gyro[0];
        gyroY = pui16Gyro[1];
        gyroZ = pui16Gyro[2];



        x = 1.125*(atan2(fAccel[0], sqrt (fAccel[1] * fAccel[1] + fAccel[2] * fAccel[2]))*180.0)/3.2; //angulo a usar de referencia

        y = (atan2(fAccel[1], sqrt (fAccel[0] * fAccel[0] + fAccel[2] * fAccel[2]))*180.0)/3.14;
        z = fGyro[2];





        //delayMS(100);
    }
}





int main(void)
{
    //Habilitar Temporizador 0
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);


        // The ADC0 peripheral must be enabled for use.
        SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

        // For this example ADC0 is used with AIN0 and AIN1.
        // The actual port and pins used may be different on your part, consult
        // the data sheet for more information.  GPIO port E needs to be enabled
        // so these pins can be used.
        // TODO: change this to whichever GPIO port you are using.
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

        // Select the analog ADC function for these pins.
        // Consult the data sheet to see which functions are allocated per pin.
        // TODO: change this to select the port/pin you are using.
        GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);  // Configura el pin PE3
        GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);  // Configura el pin PE2

        // Se configura la secuencia 2, que permitiría hasta cuatro muestras (aunque
        // se usarán dos).
        ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);

        // Step 0 en la secuencia 2: Canal 0 (ADC_CTL_CH0) en modo single-ended (por defecto).
        ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH0);

        // Step 1 en la secuencia 2: Canal 1 (ADC_CTL_CH1) en modo single-ended (por defecto),
        // y configura la bandera de interrupción (ADC_CTL_IE) para "setearse"
        // cuando se tenga esta muestra. También se indica que esta es la última
        // conversión en la secuencia 2 (ADC_CTL_END).
        // Para más detalles del módulo ADC, consultar el datasheet.
        ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);

        // Since sample sequence 2 is now configured, it must be enabled.
        ADCSequenceEnable(ADC0_BASE, 2);  // Notar el cambio de "secuencia".

        // Clear the interrupt status flag.  This is done to make sure the
        // interrupt flag is cleared before we sample.
        ADCIntClear(ADC0_BASE, 2);  // Notar el cambio de "secuencia".

        // Enable the peripherals used by this example.
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

        // Enable processor interrupts.
        IntMasterEnable();

        // Configure the two 32-bit periodic timers.
        TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

        // El tercer argumento determina la frecuencia. El reloj se puede obtener
        // con SysCtlClockGet().
        // La frecuencia está dada por SysCtlClockGet()/(valor del 3er argumento).
        // Ejemplos: Si se pone SysCtlClockGet(), la frecuencia será de 1 Hz.
        //           Si se pone SysCtlClockGet()/1000, la frecuencia será de 1 kHz
        //TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
        TimerLoadSet(TIMER0_BASE, TIMER_A, (uint32_t)(SysCtlClockGet()/freq_timer));

        // Setup the interrupt for the timer timeout.
        IntEnable(INT_TIMER0A);
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

        // Enable the timers.
        TimerEnable(TIMER0_BASE, TIMER_A);

        // Las conversiones se hacen al darse la interrupción del timer, para que
        // el muestreo sea preciso. Luego de las configuraciones, el programa se
        // queda en un ciclo infinito haciendo nada.


       // The SSI0 peripheral must be enabled for use.
       SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

       // For this example SSI0 is used with PortA[5:2].  The actual port and pins
       // used may be different on your part, consult the data sheet for more
       // information.  GPIO port A needs to be enabled so these pins can be used.
       // TODO: change this to whichever GPIO port you are using.
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

       // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
       // This step is not necessary if your part does not support pin muxing.
       // TODO: change this to select the port/pin you are using.
       GPIOPinConfigure(GPIO_PA2_SSI0CLK);
       GPIOPinConfigure(GPIO_PA3_SSI0FSS);
       GPIOPinConfigure(GPIO_PA4_SSI0RX);
       GPIOPinConfigure(GPIO_PA5_SSI0TX);

       // Configure the GPIO settings for the SSI pins.  This function also gives
       // control of these pins to the SSI hardware.  Consult the data sheet to
       // see which functions are allocated per pin.
       // The pins are assigned as follows:
       //      PA5 - SSI0Tx
       //      PA4 - SSI0Rx
       //      PA3 - SSI0Fss
       //      PA2 - SSI0CLK
       // TODO: change this to select the port/pin you are using.
       GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                      GPIO_PIN_2);

       // Configure and enable the SSI port for SPI master mode.  Use SSI0,
       // system clock supply, idle clock level low and active low clock in
       // freescale SPI mode, master mode, 4MHz SSI frequency, and 16-bit data.
       // For SPI mode, you can set the polarity of the SSI clock when the SSI
       // unit is idle.  You can also configure what clock edge you want to
       // capture data on.  Please reference the datasheet for more information on
       // the different SPI modes.
       SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                          SSI_MODE_MASTER, SPI_FREC, SPI_ANCHO);

       // Enable the SSI0 module.
       SSIEnable(SSI0_BASE);


    InitI2C0();
    ConfigureUART();
    MPU6050Example();



    return(0);
}
