#include <math.h>
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <tkjhat/sdk.h>

#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX      1

enum state { WAITING = 1, READY = 2, IDLE = 3};
enum state programState = WAITING;

uint32_t ambientLight;
char gyro_result;
float ax, ay, az, gx, gy, gz, t;

/*
void imu_task(void *pvParameters) {
    (void)pvParameters;
    
    float ax, ay, az, gx, gy, gz, t;
    // Setting up the sensor. 
    if (init_ICM42670() == 0) {
        printf("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            printf("ICM-42670P could not initialize accelerometer or gyroscope");
        }
        /*int _enablegyro = ICM42670_enable_accel_gyro_ln_mode();
        printf ("Enable gyro: %d\n",_enablegyro);
        int _gyro = ICM42670_startGyro(ICM42670_GYRO_ODR_DEFAULT, ICM42670_GYRO_FSR_DEFAULT);
        printf ("Gyro return:  %d\n", _gyro);
        int _accel = ICM42670_startAccel(ICM42670_ACCEL_ODR_DEFAULT, ICM42670_ACCEL_FSR_DEFAULT);
        printf ("Accel return:  %d\n", _accel);
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }
    // Start collection data here. Infinite loop. 
    while (1)
    {
            
            printf("Gyro: X=%f, Y=%f, Z=%f\n", ax, ay, az, gx, gy, gz, t);
            /*printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);
        } else {
            printf("Failed to read imu data\n");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

}*/

static void btn_fxn(uint gpio, uint32_t eventMask) {
    toggle_led();
    /*if(programState == IDLE){
        printf("jatketaan\n");
        programState = WAITING;
    }*/
    if(programState == WAITING){
        printf("Mennään tauolle\n");
        programState = IDLE;

    }
        else if(programState == READY){
            printf("Mennään tauolle\n");
            programState = IDLE;
        }
            else {
            printf("Jatketaan\n");
            programState = WAITING;
            }

}


static void btn_fxn2(uint gpio, uint32_t eventMask) {
    printf("    ");
}


static void sensor_task(void *arg){
(void)arg;

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



static void print_task(void *arg){
    (void)arg;
    
    while(1){
        if(programState == READY) {
            if(gy <= (-150)){
                printf("-");
                sleep_ms(1000);

            }
        
            else if(gy >= (150)) {
                printf(".");
                sleep_ms(1000);
            
            }
        //printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);
        programState = WAITING;
        }


        // Do not remove this
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

    init_button1();
    init_button2();
    init_led();


    gpio_set_dir(LED1, GPIO_OUT);
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, btn_fxn);
    gpio_set_irq_enabled_with_callback(BUTTON2, GPIO_IRQ_EDGE_RISE, true, btn_fxn2);
    
    
    TaskHandle_t hsensorTask, hprintTask = NULL;

    /*
    xTaskCreate(usbTask, "usb", 2048, NULL, 3, &hUSB);
    #if (configNUMBER_OF_CORES > 1)
        vTaskCoreAffinitySet(hUSB, 1u << 0);
    #endif
    */
    //xTaskCreate(imu_task, "IMUTask", 1024, NULL, 2, &hIMUTask);


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

