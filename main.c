/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#define INPUT_BUFFER_MAX    100

#include <project.h>
#include <stdbool.h>
#include <string.h>
#include <nodus_api.h>

void StackEventHandler( uint32 eventCode, void *eventParam );

typedef enum game_state
{
    STATE_GAME_START,
    STATE_GAME_NORTH,
    STATE_GAME_SOUTH
}game_state;

typedef struct inventory
{
    bool has_sword;
    bool has_sheild;
}inventory;


typedef struct task
{
    void* task_cb;
    bool  is_active;
    int   times_to_run;
}task;

typedef struct scheduler
{
    task tasks[2];
}scheduler;

// Static declarations
static game_state state = STATE_GAME_START;
static uint16 adc_result = 0;
static bool timer_flag = false;
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


void printNumOnTimerInterrupt()
{
    char numAsStr[6];
    int2string(adc_result, numAsStr);

    UART_UartPutString(numAsStr);
    UART_UartPutString("\r\n");

}

void count_to_ten()
{
    char numAsStr[4];
    int i;
    
    for (i = 1; i < 11; i++)
    {
        int2string(i, numAsStr);
        UART_UartPutString(numAsStr);
        UART_UartPutString("\r\n");
    }
}


void print_user_message()
{
    switch (state){
        case STATE_GAME_START:
        UART_UartPutString("\r\n***********************************************************************************\r\n");
        UART_UartPutString("Welcome Hero!\r\n");
        UART_UartPutString("Civil War has broken out between the feuding lords of the realm\r\n");
        UART_UartPutString("After King Henry died without an heir, two claiments to the throne arose\r\n");
        UART_UartPutString("The charismatic Crusader Charles the Lionhearted in the north has rallied several banners to the cause\r\n");
        UART_UartPutString("He is opposed by the corpulent cousin of the late king, William the Strong, who controls the capital and surrounding area\r\n");
        UART_UartPutString("\r\n");

        UART_UartPutString("You stand, like most great adventurers at one point, at a crossroads\r\n");
        UART_UartPutString("Do you go (N)orth to Charles, or (S)outh to William\r\n");

        UART_UartPutString("\r\n");
        break;
        
        case STATE_GAME_NORTH:
        UART_UartPutString("You take the winding road north to Newcastle\r\n");
        UART_UartPutString("As your horse walks, and bobs up and down your mind wanders.\r\n");
        UART_UartPutString("You think about how quickly things have turned for the worse.\r\n");
        UART_UartPutString("\r\n");
        UART_UartPutString("Why, it was only three months prior that you had a family, and were a hard working,\r\n");
        UART_UartPutString("god fearing, member of the local community.\r\n");
        UART_UartPutString("\r\n");
        UART_UartPutString("\r\n");
        break;
        
        case STATE_GAME_SOUTH:
        break;
    }
}


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

void get_user_input(char buffer[INPUT_BUFFER_MAX])
{
    char in_char;
    int index = 0;
    
    in_char = (char)UART_UartGetChar();
    
    while ((index < (INPUT_BUFFER_MAX -1)) && !(in_char == '\n' || in_char == '\r'))
    {
        // local echo
        if (0u != in_char)
        {
            /* Transmit the data through UART.
            * This functions is blocking and waits until there is a place in
            * the buffer.
            */
            if (in_char == '\n' || in_char == '\r')
            {
                UART_UartPutString("newline");
            }
            UART_UartPutChar(in_char);
            buffer[index] = in_char;
            index++;
        }
        in_char = (char)UART_UartGetChar();
    }
    
    UART_UartPutString("DEBUG: Out of the collctor\r\n");
    
    index++;
    buffer[index] = '\0';
}

void respond(char input[INPUT_BUFFER_MAX])
{
    UART_UartPutString("DEBUG: Lowering input\r\n");
    lower(input);
    UART_UartPutString("DEBUG: ");
    UART_UartPutString(input);
    UART_UartPutString("\r\n");
    
    if (str_eq(input, "n") || str_eq(input, "north"))
    {
        UART_UartPutString("DEBUG: Setting state to North\r\n");
        state = STATE_GAME_NORTH;
    }
    else if (str_eq(input, "s") || str_eq(input, "south"))
    {
        UART_UartPutString("DEBUG: Setting state to South\r\n");
        state = STATE_GAME_SOUTH;
    }
}


#if 0
void run_scheduler(scheduler* sched)
{
    int i;
    // Iterate through all the tasks
    for (i = 0; i < 2; i++)
    {
        task cur_task = (sched->tasks)[i];
        
        if (cur_task.is_active)
        {
            // Run continuously
            if (cur_task.times_to_run == 0)
            {
                (*(cur_task.task_cb))();
            }
        }
        
    }
}
#endif

CY_ISR(timer_isr)
{
    timer_flag = true;
    
    Timer_ClearInterrupt( Timer_INTR_MASK_CC_MATCH );
    Timer_ClearInterrupt(Timer_INTR_MASK_TC);
}

CY_ISR(adc_isr)
{
    uint32 intr_status;
    
    /* Read interrupt status registers */
    intr_status = ADC_SAR_INTR_MASKED_REG;
    
    adc_result = ADC_GetResult16(0);
    
    //ADC_StopConvert();
    
    /* Clear handled interrupt */
    //ADC_SAR_INTR_REG = intr_status;
}

int main()
{
    
    
    /* Enable interrupt component connected to interrupt */
    //TC_CC_ISR_StartEx(InterruptHandler);
    Timer_Int_StartEx(timer_isr);

    /* Start components */
    // Timer
    Timer_Start();
    
    /* Start SCB (UART mode) operation */
    UART_Start();
    
    // ADC
    ADC_IRQ_StartEx(adc_isr);
    ADC_EOC_Int_StartEx(adc_isr);
    ADC_Start();
    ADC_StartConvert();
    ADC_EnableInjection();
    ADC_IRQ_Enable();
    
    
    CyGlobalIntEnable;   /* Enable global interrupts */
    
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */


    scheduler myScheduler;
    
    #if 0
    task countToTen;
    countToTen.is_active = false;
    countToTen.times_to_run = 1;
    countToTen.task_cb = &(count_to_ten);
    // Add Task to Scheduler
    myScheduler.tasks[0] = count_to_ten;
    
    task fizzBuzz;
    fizzBuzz.is_active = true;
    countToTen.times_to_run = 0;
    fizzBuzz.task_cb = &(printNumOnTimerInterrupt);
    // Add Task to Scheduler
    myScheduler.tasks[1] = count_to_ten;
    
    
    #endif
    
    
    //CyBle_Start( StackEventHandler );
    
    
    
    //char user_input_buf[100];
    print_user_message();
    for(;;)
    {
        
        if (timer_flag)
        {
        //get_user_input(user_input_buf);
        //respond(user_input_buf);
        
        
        //run_scheduler(&myScheduler);
        printNumOnTimerInterrupt();
        //ADC_StartConvert();
        //CyBle_ProcessEvents();
        timer_flag = false;
        }
        CySysPmSleep();
    }
}

#if 0
void StackEventHandler( uint32 eventCode, void *eventParam )
{
    switch( eventCode )
    {
        /* Generic events */

        case CYBLE_EVT_HOST_INVALID:
        break;

        case CYBLE_EVT_STACK_ON:
            /* CyBle_GappStartAdvertisement( CYBLE_ADVERTISING_FAST ); */
        break;

        case CYBLE_EVT_TIMEOUT:
        break;

        case CYBLE_EVT_HARDWARE_ERROR:
        break;

        case CYBLE_EVT_HCI_STATUS:
        break;

        case CYBLE_EVT_STACK_BUSY_STATUS:
        break;

        case CYBLE_EVT_PENDING_FLASH_WRITE:
        break;


        /* GAP events */

        case CYBLE_EVT_GAP_AUTH_REQ:
        break;

        case CYBLE_EVT_GAP_PASSKEY_ENTRY_REQUEST:
        break;

        case CYBLE_EVT_GAP_PASSKEY_DISPLAY_REQUEST:
        break;

        case CYBLE_EVT_GAP_AUTH_COMPLETE:
        break;

        case CYBLE_EVT_GAP_AUTH_FAILED:
        break;

        case CYBLE_EVT_GAP_DEVICE_CONNECTED:
        break;

        case CYBLE_EVT_GAP_DEVICE_DISCONNECTED:
            /* CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_FAST); */
        break;

        case CYBLE_EVT_GAP_ENCRYPT_CHANGE:
        break;

        case CYBLE_EVT_GAP_CONNECTION_UPDATE_COMPLETE:
        break;

        case CYBLE_EVT_GAP_KEYINFO_EXCHNGE_CMPLT:
        break;


        /* GAP Peripheral events */

        case CYBLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
        break;


        /* GAP Central events */

        case CYBLE_EVT_GAPC_SCAN_PROGRESS_RESULT:
        break;

        case CYBLE_EVT_GAPC_SCAN_START_STOP:
        break;


        /* GATT events */

        case CYBLE_EVT_GATT_CONNECT_IND:
        break;

        case CYBLE_EVT_GATT_DISCONNECT_IND:
        break;


        /* GATT Client events (CYBLE_EVENT_T) */

        case CYBLE_EVT_GATTC_ERROR_RSP:
        break;

        case CYBLE_EVT_GATTC_XCHNG_MTU_RSP:
        break;

        case CYBLE_EVT_GATTC_READ_BY_GROUP_TYPE_RSP:
        break;

        case CYBLE_EVT_GATTC_READ_BY_TYPE_RSP:
        break;

        case CYBLE_EVT_GATTC_FIND_INFO_RSP:
        break;

        case CYBLE_EVT_GATTC_FIND_BY_TYPE_VALUE_RSP:
        break;

        case CYBLE_EVT_GATTC_READ_RSP:
        break;

        case CYBLE_EVT_GATTC_READ_BLOB_RSP:
        break;

        case CYBLE_EVT_GATTC_READ_MULTI_RSP:
        break;

        case CYBLE_EVT_GATTC_WRITE_RSP:
        break;

        case CYBLE_EVT_GATTC_EXEC_WRITE_RSP:
        break;

        case CYBLE_EVT_GATTC_HANDLE_VALUE_NTF:
        break;

        case CYBLE_EVT_GATTC_HANDLE_VALUE_IND:
        break;


        /* GATT Client events (CYBLE_EVT_T) */

        case CYBLE_EVT_GATTC_INDICATION:
        break;

        case CYBLE_EVT_GATTC_SRVC_DISCOVERY_FAILED:
        break;

        case CYBLE_EVT_GATTC_INCL_DISCOVERY_FAILED:
        break;

        case CYBLE_EVT_GATTC_CHAR_DISCOVERY_FAILED:
        break;

        case CYBLE_EVT_GATTC_DESCR_DISCOVERY_FAILED:
        break;

        case CYBLE_EVT_GATTC_SRVC_DUPLICATION:
        break;

        case CYBLE_EVT_GATTC_CHAR_DUPLICATION:
        break;

        case CYBLE_EVT_GATTC_DESCR_DUPLICATION:
        break;

        case CYBLE_EVT_GATTC_SRVC_DISCOVERY_COMPLETE:
        break;

        case CYBLE_EVT_GATTC_INCL_DISCOVERY_COMPLETE:
        break;

        case CYBLE_EVT_GATTC_CHAR_DISCOVERY_COMPLETE:
        break;

        case CYBLE_EVT_GATTC_DISCOVERY_COMPLETE:
        break;


        /* GATT Server events (CYBLE_EVENT_T) */

        case CYBLE_EVT_GATTS_XCNHG_MTU_REQ:
        break;

        case CYBLE_EVT_GATTS_WRITE_REQ:
        break;

        case CYBLE_EVT_GATTS_WRITE_CMD_REQ:
        break;

        case CYBLE_EVT_GATTS_PREP_WRITE_REQ:
        break;

        case CYBLE_EVT_GATTS_EXEC_WRITE_REQ:
        break;

        case CYBLE_EVT_GATTS_HANDLE_VALUE_CNF:
        break;

        case CYBLE_EVT_GATTS_DATA_SIGNED_CMD_REQ:
        break;


        /* GATT Server events (CYBLE_EVT_T) */

        case CYBLE_EVT_GATTS_INDICATION_ENABLED:
        break;

        case CYBLE_EVT_GATTS_INDICATION_DISABLED:
        break;


        /* L2CAP events */

        case CYBLE_EVT_L2CAP_CONN_PARAM_UPDATE_REQ:
        break;

        case CYBLE_EVT_L2CAP_CONN_PARAM_UPDATE_RSP:
        break;

        case CYBLE_EVT_L2CAP_COMMAND_REJ:
        break;

        case CYBLE_EVT_L2CAP_CBFC_CONN_IND:
        break;

        case CYBLE_EVT_L2CAP_CBFC_CONN_CNF:
        break;

        case CYBLE_EVT_L2CAP_CBFC_DISCONN_IND:
        break;

        case CYBLE_EVT_L2CAP_CBFC_DISCONN_CNF:
        break;

        case CYBLE_EVT_L2CAP_CBFC_DATA_READ:
        break;

        case CYBLE_EVT_L2CAP_CBFC_RX_CREDIT_IND:
        break;

        case CYBLE_EVT_L2CAP_CBFC_TX_CREDIT_IND:
        break;

        case CYBLE_EVT_L2CAP_CBFC_DATA_WRITE_IND:
        break;


        /* default catch-all case */

        default:
        break;
    }
}

#endif
