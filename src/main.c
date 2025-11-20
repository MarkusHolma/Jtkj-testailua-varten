/*
TIETOKONEJÄRJESTELMÄT - TIER 1

Tekijät:
Matti Damski
Markus Holma
Heikki Komonen

Generatiivista tekoälyä ei ole käytetty tämän projektin koodin tuotannossa.
Tekoälyä on käytetty vain projektin ohjeiden tiivistämiseen.

*/

#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <tkjhat/sdk.h>

#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX      1
// Erilaisten tiloje määrittelyitä
enum state { WAITING = 1, READY = 2, IDLE = 3};
enum state programState = WAITING;
//muuttujien määrittelyt
uint32_t ambientLight;
char gyro_result;
float ax, ay, az, gx, gy, gz, t;
bool print_gap = false;
bool print_end = false;

//Nappien 1 ja 2 toiminta eri tiloissa( Keskeytyskäsittelijä )
static void btn_fxn(uint gpio, uint32_t eventMask) {
    if(gpio == BUTTON1){

        if(programState != IDLE){
            //printf("Mennään tauolle\n");
            toggle_led();
            programState = IDLE;

        } else {
            //printf("Jatketaan\n");
            toggle_led();
            programState = WAITING;
        }
    }
    else if(gpio == BUTTON2) {
        if(programState != IDLE){
            print_gap = true;
        }

        else{
            print_end = true;
        }
    }
}
//Käytetään sensor_taskia anturidatan keräämiseen(Löytyy esimerkeistä, examples/hat_imu_ex)
static void sensor_task(void *arg){
(void)arg;
//Määrittelyt
init_ICM42670();
    if (init_ICM42670() == 0) {
        printf("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            printf("ICM-42670P could not initialize accelerometer or gyroscope");
        }
        int _enablegyro = ICM42670_enable_accel_gyro_ln_mode();
        printf ("Enable gyro: %d\n",_enablegyro);
        int _gyro = ICM42670_startGyro(ICM42670_GYRO_ODR_DEFAULT, ICM42670_GYRO_FSR_DEFAULT);
        printf ("Gyro return:  %d\n", _gyro);
        int _accel = ICM42670_startAccel(ICM42670_ACCEL_ODR_DEFAULT, ICM42670_ACCEL_FSR_DEFAULT);
        printf ("Accel return:  %d\n", _accel);
        printf("Inputtien välillä pitää olla vähintään 1 sekunti\n");
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }
    //luetaan data
for(;;) {
    if(programState == WAITING){
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {

            ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t);
            programState = READY;
        }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

}


//Käytetään print_taskia pisteiden ja pilkkujen ja välien ja lähetyksen tulostamiseen
static void print_task(void *arg){
    (void)arg;
    
    while(1){
        if(print_gap){
            printf(" ");
            sleep_ms(200);
            print_gap = false;

        }

        if(print_end){
            printf("  \n");
            sleep_ms(200);
            print_end = false;

        }

        if(programState == READY) {
            if(gy <= (-150)){
                printf("-");
                sleep_ms(1000);

            }
        
            else if(gy >= (150)) {
                printf(".");
                sleep_ms(1000);
            
            }
        programState = WAITING;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


int main() {
    stdio_init_all();
    // Uncomment this lines if you want to wait till the serial monitor is connected
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.
    //nappien määrittely
    init_button1();
    init_button2();
    init_led();

    gpio_set_dir(LED1, GPIO_OUT); 
    gpio_set_irq_enabled_with_callback(BUTTON2, GPIO_IRQ_EDGE_FALL, true, btn_fxn);
    gpio_set_irq_enabled(BUTTON1, GPIO_IRQ_EDGE_FALL, true);

    TaskHandle_t hsensorTask, hprintTask = NULL;

    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(sensor_task, // (en) Task function
                "sensor",                        // (en) Name of the task 
                DEFAULT_STACK_SIZE,              // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                            // (en) Arguments of the task 
                2,                               // (en) Priority of this task
                &hsensorTask);                   // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Sensor task creation failed\n");
        return 0;
    }
    BaseType_t result1 = xTaskCreate(print_task,  // (en) Task function
                "print",              // (en) Name of the task 
                DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                 // (en) Arguments of the task 
                1,                    // (en) Priority of this task
                &hprintTask);         // (en) A handle to control the execution of this task

    if(result1 != pdPASS) {
        printf("Print Task creation failed\n");
        return 0;
    }
    // Start the scheduler (never returns)
    vTaskStartScheduler();
    // Never reach this line.
    return 0;
}

