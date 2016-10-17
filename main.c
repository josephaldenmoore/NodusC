/* ========================================
 *
 * Copyright Flex, 2016
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#define CLOCK_KHZ                   500
#define ADC_REF_MV                  3300
#define INPUT_BUFFER_MAX            100
#define ADC_BUFFER_SIZE             64
#define SCHEDULER_NUM_OF_TASKS      4
#define CAL_TABLE_NUM_OF_POINTS     5
#define ADC_CYCLES_PER_SECOND       8
#define BYTES_IN_BLE_PACKET         20
//#define ADC_SENSORS                 1 //WILL BE 16 IN FINAL
#define ADC_TEMPERATURE_CHANNEL     1
#define IIR_WEIGHT                  8 // Should be power of 2 y = ((IIR_weight - 1)y + x) / IIR_WEIGHT 

#define NUMBER_OF_SENSORS           8

#define LOW                         0
#define HIGH_Z                      1

#include <project.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <queue.h>
#include <nodus_api.h>

// Function declarations
void StackEventHandler( uint32 eventCode, void *eventParam );

// Type definitions

typedef struct fsr_params
{
    uint8 numberOfSensors;
    uint8 samplesToAvg; // For most efficient result this aught to be a power of 2
    
} fsr_params;

typedef struct cal_point
{
    float force;
    uint16 adc_val;
}cal_point;

typedef struct task
{
    void  (*task_cb)(struct task*, queue*);
    bool  is_active;
    uint8 times_per_second;
    uint8 counter;
}task;

typedef struct scheduler
{
    task tasks[SCHEDULER_NUM_OF_TASKS];
}scheduler;

typedef enum scheduler_task
{
    TASK_CALIBRATE,
    TASK_WRITE_UART,
    TASK_READ_ADC,
    TASK_READ_TEMPERATURE,
    MAX_SCHEDULER_TASK
} scheduler_task;

typedef struct pin
{
    int id;
    void (*pin_write)(uint8 value);
} pin;

// Global variable declarations
static cal_point cal_table[CAL_TABLE_NUM_OF_POINTS];
static uint16 filtered_adc[BYTES_IN_BLE_PACKET/2];
static pin pins[NUMBER_OF_SENSORS + 1]; // the +1 is for the resistor

static volatile uint32 temperature = 0;
static volatile bool temperature_convert = false;

static bool input_ready = false;
static bool calibrated = false;
static bool deviceConnected = false;
static volatile uint8 sampling_freq = 10;


// function declarations
void SendADCData(uint8 buffer[20]);
void CustomEventHandler(uint32 event, void * eventParam);


// converts a string into a float
float str2float(char* str)
{
    int whole = 0;
    int dec = 0;
    int dec_len = 0;
    float result = 0.0;
    char* str_cpy = str;
    char cur_c;
    bool in_dec = false;
    
    cur_c = *str_cpy;
    while (cur_c != '\0')
    {
        // if char is a letter
        if (cur_c >= '0' && cur_c <= '9')
        {
            if (in_dec == false)
            {
                whole *= 10;
                whole += (cur_c - '0');
            }
            else
            {
                dec *= 10;
                dec += (cur_c - '0');
                dec_len++;
            }
        }
        else if (cur_c == '.')
        {
            in_dec = true;
        }
        else if ( !(cur_c == '\n' || cur_c == '\r') )
        {
            UART_UartPutString("WARNING: Unrecognized char\r\n");
        }
        
        cur_c = *(++str_cpy);
    }
    result = (float)dec;
    while (dec_len != 0)
    {
        result /= 10.0;
        dec_len--;
    }
    result += (float)whole;
    UART_UartPutString("DEBUG: float conversion complete\r\n");                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
    return result;
}


// raise a base to a power
// fun with recursion, can prove it terminates
int powr(int b, int p)
{
    int r;
    int exp_h;
    
    if (p == 0)
    {
        return 1;
    }
    else 
    {
        exp_h = powr(b, p >> 1); // p >> 1 === p / 2
        if (p%2 == 0)
        {
            return exp_h * exp_h;
        }
        else
        {
            return b * exp_h * exp_h;
        }
    }
}


// converts an integer to a string
void int2string(uint16 n, char str[6])
{
    if (n < 10)
    {
        str[0] = (char)('0' + n);
        str[1] = '\0';
    }
    else if (n < 100)
    {
        str[1] = (char)('0' + (n % 10));
        str[0] = (char)('0' + ((n / 10) % 10));
        str[2] = '\0';
    }
    else if (n < 1000)
    {
        str[2] = (char)('0' + (n % 10));
        str[1] = (char)('0' + ((n / 10) % 10));
        str[0] = (char)('0' + ((n / 100) % 10));
        str[3] = '\0';
    }
    else if (n < 10000)
    {
        str[4] = '\0';
        str[3] = (char)('0' + (n % 10));
        str[2] = (char)('0' + ((n / 10) % 10));
        str[1] = (char)('0' + ((n / 100) % 10));
        str[0] = (char)('0' + ((n / 1000) % 10));
    }
    else
    {
        str[5] = '\0';
        str[4] = (char)('0' + (n % 10));
        str[3] = (char)('0' + ((n / 10) % 10));
        str[2] = (char)('0' + ((n / 100) % 10));
        str[1] = (char)('0' + ((n / 1000) % 10));
        str[0] = (char)('0' + ((n / 10000) % 10));
    }
}


// Float -> String
void float2str(float f, char str[64])
{
    int whole_part;
    int fraction;
    float temp;
    int i = 0;
    int dec_places = 2;  // Change this if you want more decimal places.
    char c;
    
    whole_part = (int) f;
    temp = f - whole_part;
    fraction = (int)(powr(10, dec_places) * temp);
    
    int2string(whole_part, str);
    for(i=0; i<64; i++)
    {
        c = str[i];
        if (c == '\0')
        {
            str[i] = '.';
            int2string(fraction, &str[i+1]);
            break;
        }
    }
}

// linear interpolation of adc_result
float linterp(uint16 adc_reading)
{
    // The whole configuration creates a strictly decreasing function of resistance / force
    
    float result = 0.0;
    bool lower = false;
    bool upper = false;
    bool exact = false;
    
    // Find the index, assume the calibration point table is in ascending order
    // ^^^ bad assumption
    // the data isn't in an easy format in the table
    // if a point is <adc_val, force>, then the adc_val goes from high to low as
    // points are iterated across the table.
    
    int i;
    i=0;
    
    if (adc_reading >= cal_table[0].adc_val)
    {
        lower = true;
    }
    else
    {
        for (i=1; i<CAL_TABLE_NUM_OF_POINTS; i++)
        {
            if (adc_reading > cal_table[i].adc_val) break;
            else if (adc_reading == cal_table[i].adc_val)
            {
                exact = true;
                break;
            }
        }
        
        if (adc_reading < cal_table[CAL_TABLE_NUM_OF_POINTS-1].adc_val) upper = true;
    }
    
    if (lower)
    {
        result = 0.0;
    }
    else if (upper)
    {
        // We can extrapolate linearly from the last two points
        int x1, x2;
        float fx1, fx2;
        
        x1 = cal_table[CAL_TABLE_NUM_OF_POINTS-2].adc_val;
        fx1 = cal_table[CAL_TABLE_NUM_OF_POINTS-2].force;
        
        x2 = cal_table[CAL_TABLE_NUM_OF_POINTS-1].adc_val;
        fx2 = cal_table[CAL_TABLE_NUM_OF_POINTS-1].force;
        
        
        result = ( (x2 - adc_reading)* fx1 + (adc_reading - x1) * fx2 ) / ((float)(x2 - x1)) ;
    }
    else if (exact)
    {
        result = cal_table[i].force;
    }
    else // do a linear interpolation
    {
        // Basic of linear interpolation: f(x) = (x2 - x)/lin_range * f(x1) + (x - x1)/lin_range * (fx2)
        int x1, x2;
        float fx1, fx2;
        
        x1 = cal_table[i - 1].adc_val;
        fx1 = cal_table[i - 1].force;
        
        x2 = cal_table[i].adc_val;
        fx2 = cal_table[i].force;
        
        result = ( (x2 - adc_reading)* fx1 + (adc_reading - x1) * fx2 ) / ((float)(x2 - x1)) ;
    }
    
    return result;
}

uint16 read_adc( volatile int index )
{
    uint16 result;
    
    
    pins[index].pin_write(LOW);
    
    // Wait for transient response
    //CyDelay(5u);
    
    ADC_StartConvert();
    ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);
    result = ADC_GetResult16(0);
    
    
    pins[index].pin_write(HIGH_Z);
    
    return result;
}

// This method has a few "waits" in here, that concerns me
void readADC_routine(task* self, queue* ipc_queue)
{
    
    uint32 buf[NUMBER_OF_SENSORS];
    int sensor;
    
    pins[0].pin_write(LOW);
    //CyDelay(5u);
    for (sensor=0; sensor < NUMBER_OF_SENSORS; sensor++)
    {
        // Sensors are indexed 1 after resistor pin
        buf[sensor] = read_adc(sensor+1);
    }
    pins[0].pin_write(HIGH_Z);
    
    
    
    if (deviceConnected)
    {
        SendADCData((uint8*)filtered_adc);
    }
    
    if (sampling_freq != self->times_per_second && sampling_freq != 0)
    {
        self->times_per_second = sampling_freq;
        UART_UartPutString("We changed the Sampling Frequency\r\n");
    }
    
}

// Reads the ADC every second
void writeUART_routine(task* self, queue* ipc_queue)
{    
    char numAsStr[6];
    char tempAsStr[6];
    char floatAsStr[64];
    uint8 InterruptState;
    // This function uses the global cooked_adc
    uint32 adc_accumulator = filtered_adc[0];
    
    if (calibrated)
    {
        float force = 0.0;
        
        force = linterp((uint16)adc_accumulator);
        
        int2string((uint16)adc_accumulator, numAsStr);
        UART_UartPutString("ADC Value: ");
        UART_UartPutString(numAsStr);
        UART_UartPutString("\tCalculated Force: ");
        
        float2str(force, floatAsStr);
        
        UART_UartPutString(floatAsStr);
        UART_UartPutString("\r\n");
    }
    
    else
    {
        uint32 cels;
        cels = DieTemp_CountsTo_Celsius(temperature);
        int2string((uint16)cels, tempAsStr);
        int2string((uint16)adc_accumulator, numAsStr);
        UART_UartPutString("ADC Value: ");
        int i;
        for (i=0; i<NUMBER_OF_SENSORS; i++)
        {
            UART_UartPutString("\t");
            adc_accumulator = filtered_adc[i];
            int2string((uint16)adc_accumulator, numAsStr);
            UART_UartPutString(numAsStr);
        }
        UART_UartPutString("\tTemperature: ");
        UART_UartPutString(tempAsStr);
        UART_UartPutString("\r\n");
    }
}

void readTemperature_routine(task* self, queue* ipc_queue)
{
    int corrected_temp;
    ADC_EnableInjection();
	ADC_StartConvert();
	ADC_IsEndConversion(ADC_WAIT_FOR_RESULT_INJ);
    corrected_temp = ADC_GetResult16(ADC_TEMPERATURE_CHANNEL) * ADC_REF_MV / 1024;
    temperature = (3 * temperature + corrected_temp) / 4;
}

// Print an initial message on UART
void print_user_message()
{
    UART_UartPutString("\r\n***********************************************************************************\r\n");
    UART_UartPutString("---Force Sensor Test---\r\n");
    UART_UartPutString("Press 'c' to calibrate\r\n");
    UART_UartPutString("Press 'r' to toggle continuous read\r\n");
    UART_UartPutString("\r\n***********************************************************************************\r\n");
    UART_UartPutString("\r\n");
}

// string -> string
// Changes all characters in an ASCII string to lower case equivalent
char* lower(char* str)
{
    char* str_cpy = str;
    while (*str != '\0')
    {
        if (*str >= 'A' && *str <= 'Z')
        {
            *str += 'a' - 'A';
        }
        str++;
    }
    return str_cpy;
}

// Returns true if both strings contain the same characters up til the null terminator
bool str_eq(char* s1, char* s2)
{
    bool ret = true;
    int i = 0;
    while ((s1[i] != '\0' || s2[i] != '\0') && ret == true)
    {
        if (s1[i] != s2[i])
        {
            ret = false;
        }

        i++;
    }
    
    return ret;
}

// Very basic input handler
//  Called 64 times per second (I think, adc may mess with it)
// Grabs a character from the UART, if \0 it ignores it, if a newline
//  the input_ready flag is set to true
void get_user_input(char buffer[INPUT_BUFFER_MAX])
{
    static int index = 0;
    char in_char;
    
    
    in_char = (char)UART_UartGetChar();
    
    if ( index < (INPUT_BUFFER_MAX -1) )
    {
        // local echo
        if (0u != in_char)
        {            
            UART_UartPutChar(in_char);
            buffer[index] = in_char;
            index++;
            
            // A newline char indicates a command is entered
            if (in_char == '\n' || in_char == '\r')
            {
                UART_UartPutString("DEBUG: Input Accepted\r\n");
                // index was already incremented, so this terminates the string
                buffer[index] = '\0';
                UART_UartPutString(buffer);
                UART_UartPutString("\r\n");
                input_ready = true;
                index = 0;
            }
        }
    }
}

// Main scheduler task
// It runs all the tasks flagged as active each clock tick (64 Hz)
// (I think, adc may mess with it)
void run_scheduler(scheduler* sched, queue* ipc_queue)
{
    int i;
    // Iterate through all the tasks
    for (i = 0; i < SCHEDULER_NUM_OF_TASKS; i++)
    {
        task* cur_task = &(sched->tasks)[i];
        
        if (cur_task->is_active)
        {
            if (cur_task->counter == 0 && cur_task->times_per_second != 0)
            {
                cur_task->counter = (uint8) (CLOCK_KHZ / cur_task->times_per_second);
                // if the task is active, call the task
                (cur_task->task_cb)(cur_task, ipc_queue);
            }
            else
            {
                cur_task->counter -= 1;
            }
        }
        
    }
}

// The function for the task of Calibrating the sensor
void calibrate_sensor(task* self, queue* ipc_queue)
{
    uint32 adc_accumulator = filtered_adc[0]; // KLUDGE: make this better for multisensor support
    // Static variable declarations
    static int cal_point_ind = 0;
    
    // Non-static (ephemeral?) variable declarations
    char numAsStr[6];
    
    // basically do one loop of a for loop on the points in cal_table
    if (cal_point_ind < CAL_TABLE_NUM_OF_POINTS)
    {
        // Wait for a message in the IPC queue
        if (is_empty(ipc_queue) == false)
        {
            UART_UartPutString("DEBUG: Message Received\r\n");
            ipc_message* m = peek(ipc_queue);
            // Just making sure the message is the one we want
            if (m->address == TASK_CALIBRATE)
            {
                UART_UartPutString("DEBUG: Dequeing Message\r\n");
                m = dequeue(ipc_queue);
                
                UART_UartPutString("DEBUG: Message Reads: ");
                UART_UartPutString(m->data);
                UART_UartPutString("\r\n");
                
                cal_table[cal_point_ind].force = str2float(m->data);
                cal_table[cal_point_ind].adc_val = (uint16)(adc_accumulator);
                
                int2string((uint16)(adc_accumulator), numAsStr);
                UART_UartPutString("INFO: Message Reads: ");
                UART_UartPutString(numAsStr);
                UART_UartPutString("\r\n");
                
                
                // Update/reset static variables at end of "loop"
                cal_point_ind++;
            }
        }
    }
    else
    {
        UART_UartPutString("DEBUG: Exiting calibration task\r\n");
        // Turn self off
        self->is_active = false;
        // Reset static vars for any future call
        cal_point_ind = 0;
        // Set global static vars
        calibrated = true;
    }
}

// ISR for Interrupt triggered by timer (64 HZ)
CY_ISR(timer_isr)
{    
    Timer_ClearInterrupt( Timer_INTR_MASK_CC_MATCH );
    Timer_ClearInterrupt(Timer_INTR_MASK_TC);
}

int main()
{
    CyBle_Start(CustomEventHandler);
    /* Enable interrupt component connected to interrupt */
    //TC_CC_ISR_StartEx(InterruptHandler);
    Timer_Int_StartEx(timer_isr);

    /* Start components */
    // Timer
    Timer_Start();
    
    /* Start SCB (UART mode) operation */
    UART_Start();
    
    // ADC
    //ADC_IRQ_StartEx(adc_isr);
    
    ADC_Start();
    
    // Delay 25us as per the specification
    CyDelay(25u);   
    
    // Get an initial reading for the temperature
    // this may have to move at some point outside init
    int corrected_temp;
    ADC_EnableInjection();
	ADC_StartConvert();
	ADC_IsEndConversion(ADC_WAIT_FOR_RESULT_INJ);
    corrected_temp = ADC_GetResult16(ADC_TEMPERATURE_CHANNEL) * ADC_REF_MV / 1024;
    temperature = corrected_temp;
    
    
    CyGlobalIntEnable;   /* Enable global interrupts */
    
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    char user_input_buf[INPUT_BUFFER_MAX];
    scheduler myScheduler;
    
    // Initialize ipc fifo
    queue ipc_fifo;
    init_queue(&ipc_fifo);
    
    // Initialize 'calibrate'
    task calibrate;
    calibrate.is_active = false;
    calibrate.task_cb = calibrate_sensor;
    calibrate.times_per_second = 64;
    calibrate.counter = 0;
    // Add Task to Scheduler
    myScheduler.tasks[TASK_CALIBRATE] = calibrate;
    
    task writeUART;
    writeUART.is_active = false;
    writeUART.task_cb = writeUART_routine;
    writeUART.times_per_second = 2;
    writeUART.counter = 0;
    // Add Task to Scheduler
    myScheduler.tasks[TASK_WRITE_UART] = writeUART;
    
    task readADC;
    readADC.is_active = true;
    readADC.task_cb = readADC_routine;
    readADC.times_per_second = 10;
    readADC.counter = 0;
    // Add Task to Scheduler
    myScheduler.tasks[TASK_READ_ADC] = readADC;
    
    task readTemperature;
    readTemperature.is_active = false;
    readTemperature.task_cb = readTemperature_routine;
    readTemperature.times_per_second = 1;
    readTemperature.counter = 0;
    // Add Task to Scheduler
    myScheduler.tasks[TASK_READ_TEMPERATURE] = readTemperature;
    
    // initialize pins
    pins[0].id = 0;
    pins[0].pin_write = PIN_RESISTOR_Write;
    pins[1].id = 1;
    pins[1].pin_write = PIN_SENSOR_1_Write;
    pins[2].id = 2;
    pins[2].pin_write = PIN_SENSOR_2_Write;
    pins[3].id = 3;
    pins[3].pin_write = PIN_SENSOR_3_Write;
    pins[4].id = 4;
    pins[4].pin_write = PIN_SENSOR_4_Write;
    pins[5].id = 5;
    pins[5].pin_write = PIN_SENSOR_5_Write;
    pins[6].id = 6;
    pins[6].pin_write = PIN_SENSOR_6_Write;
    pins[7].id = 7;
    pins[7].pin_write = PIN_SENSOR_7_Write;
    pins[8].id = 8;
    pins[8].pin_write = PIN_SENSOR_8_Write;
    
    PIN_RESISTOR_Write(HIGH_Z);
    PIN_SENSOR_1_Write(HIGH_Z);
    PIN_SENSOR_2_Write(HIGH_Z);
    
    print_user_message();
    for(;;)
    {
        get_user_input(user_input_buf);
        
        if (input_ready)
        {
            UART_UartPutString("Input Ready\r\n");
            UART_UartPutString("Input is: ");
            UART_UartPutString(user_input_buf);
            UART_UartPutString("\r\n");
            
            if (str_eq(user_input_buf, "c\r") && !myScheduler.tasks[TASK_CALIBRATE].is_active)
            {
                UART_UartPutString("Activating Calibration task\r\n");
                myScheduler.tasks[TASK_CALIBRATE].is_active = true;
                calibrated = false;
            } 
            
            // toggle read
            else if (str_eq(user_input_buf, "r\r"))
            {
                UART_UartPutString("Toggling WRITE_UART task\r\n");
                myScheduler.tasks[TASK_WRITE_UART].is_active = !myScheduler.tasks[TASK_WRITE_UART].is_active;
            }
            
            else if (myScheduler.tasks[TASK_CALIBRATE].is_active)
            {
                ipc_message m;
                m.address = TASK_CALIBRATE;
                m.data = user_input_buf;
                UART_UartPutString("DEBUG: Enqueing message\r\n");
                enqueue(&ipc_fifo, m);
            }
            
            input_ready = false;
        }
        
        run_scheduler(&myScheduler, &ipc_fifo);
        CyBle_ProcessEvents();
        
        CySysPmSleep();
    }
}

/*******************************************************************************
* Function Name: CustomEventHandler
********************************************************************************
* Summary:
* Call back event function to handle various events from BLE stack
*
* Parameters:
*  event:		event returned
*  eventParam:	link to value of the events returned
*
* Return:
*  void
*
*******************************************************************************/
void CustomEventHandler(uint32 event, void * eventParam)
{
	CYBLE_GATTS_WRITE_REQ_PARAM_T *wrReqParam;
    
    switch(event)
    {
        case CYBLE_EVT_STACK_ON:
        case CYBLE_EVT_GAP_DEVICE_DISCONNECTED:
			/* Start Advertisement and enter Discoverable mode*/
        
			CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_FAST);
			break;
			
		case CYBLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
			/* Set the BLE state variable to control LED status */
            if(CYBLE_STATE_DISCONNECTED == CyBle_GetState())
            {
                /* Start Advertisement and enter Discoverable mode*/
                CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_FAST);
            }
            
			break;
			
        case CYBLE_EVT_GATT_CONNECT_IND:
			/* This flag is used in application to check connection status */
			deviceConnected = true;
			break;
        
        case CYBLE_EVT_GATT_DISCONNECT_IND:
			/* Update deviceConnected flag*/
			deviceConnected = false;
			
			/* Reset CapSense notification flag to prevent further notifications
			 * being sent to Central device after next connection. */
			
			/* Reset the CCCD value to disable notifications */
			
			/* Reset the color coordinates */

            
            CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_FAST);
            //advertising = TRUE;
			break;
        
            
        case CYBLE_EVT_GATTS_WRITE_REQ: 							
            /* This event is received when Central device sends a Write command 
             * on an Attribute. 
             * We first get the attribute handle from the event parameter and 
             * then try to match that handle with an attribute in the database.
             */
            wrReqParam = (CYBLE_GATTS_WRITE_REQ_PARAM_T *) eventParam;
            

            /* This condition checks whether the RGB LED characteristic was
             * written to by matching the attribute handle.
             * If the attribute handle matches, then the value written to the 
             * attribute is extracted and used to drive RGB LED.
             */
            
            /* ADD_CODE to extract the attribute handle for the RGB LED 
             * characteristic from the custom service data structure.
             */
            if(wrReqParam->handleValPair.attrHandle == CYBLE_NODUSC_SERVICE_CONFIG_SAMPLE_FREQUENCY_CHAR_HANDLE)
            {
                /* ADD_CODE to extract the value of the attribute from 
                 * the handle-value pair database. */
                sampling_freq = *wrReqParam->handleValPair.value.val;
            }

			/* ADD_CODE to send the response to the write request received. */
			CyBle_GattsWriteRsp(cyBle_connHandle);
			
			break;
        default:

       	 	break;
    }
}

void SendADCData(uint8 buffer[BYTES_IN_BLE_PACKET])
{
    CYBLE_GATTS_HANDLE_VALUE_NTF_T NotificationHandle;
    
    NotificationHandle.attrHandle = CYBLE_NODUSC_SERVICE_NODUSC_CHARACTERISTIC_CHAR_HANDLE;
    NotificationHandle.value.val = buffer;
    NotificationHandle.value.len = BYTES_IN_BLE_PACKET;
    
    CyBle_GattsNotification(cyBle_connHandle, &NotificationHandle);
}
