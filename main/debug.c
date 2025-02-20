
#include "freertos/FreeRTOS.h"
#include "string.h"

/*
 * Macros used by vListTask to indicate which state a task is in.
 */
#define tskRUNNING_CHAR      ( 'X' )
#define tskBLOCKED_CHAR      ( 'B' )
#define tskREADY_CHAR        ( 'R' )
#define tskDELETED_CHAR      ( 'D' )
#define tskSUSPENDED_CHAR    ( 'S' )
#define TRACE_BUFSIZ 512

const char _buf[TRACE_BUFSIZ];
const char *_bufENDp = _buf + TRACE_BUFSIZ;
volatile char *_bufp = _buf;

volatile char * gather_task_states( volatile char * pcWriteBuffer, const char * pcWriteBufferENDp );

volatile TaskStatus_t * pxTaskStatusArray;
volatile UBaseType_t uxArraySize;

void allocate_memory_for_status()
{
        /* Allocate an array index for each task.  NOTE!  if
         * configSUPPORT_DYNAMIC_ALLOCATION is set to 0 then pvPortMalloc() will
         * equate to NULL. */
        pxTaskStatusArray = pvPortMalloc( 12 * sizeof( TaskStatus_t ) ); /*lint !e9079 All values returned by pvPortMalloc() have at least the alignment required by the MCU's stack and this allocation allocates a struct that has the alignment requirements of a pointer. */
        if(pxTaskStatusArray == NULL) {
            printf("pxTaskStatusArray is NULL\n");
        }

}
void print_task_list()
{

    char *buffer = (char *)pvPortMalloc(1024); // Allocate a buffer for the stats
    if (buffer != NULL) {
        vTaskList(buffer); // Get task list
        printf("Task List:\n%s\n", buffer);

        //vTaskGetRunTimeStats(buffer); // Get runtime stats
        //printf("Run Time Stats:\n%s\n", buffer);

        free(buffer);
    }
}

void print_trace_info() {
    printf("%s",_buf);
    _bufp = _buf;
    printf("%c",*(_buf));
    printf("%c",*(_buf+1));
    printf("%c",*(_buf+2));
    printf("%c\n",*(_buf+3));

}
void vApplicationIdleHook()
{
    //ESP_LOGI(TAG,"In IDLE");
}

/*
 * Task control block.  A task control block (TCB) is allocated for each task,
 * and stores task state information, including a pointer to the task's context
 * (the task's run time environment, including register values)
 */
typedef struct tskTaskControlBlock       /* The old naming convention is used to prevent breaking kernel aware debuggers. */
{
    volatile StackType_t * pxTopOfStack; /*< Points to the location of the last item placed on the tasks stack.  THIS MUST BE THE FIRST MEMBER OF THE TCB STRUCT. */

    #if ( portUSING_MPU_WRAPPERS == 1 )
        xMPU_SETTINGS xMPUSettings; /*< The MPU settings are defined as part of the port layer.  THIS MUST BE THE SECOND MEMBER OF THE TCB STRUCT. */
    #endif

    ListItem_t xStateListItem;                  /*< The list that the state list item of a task is reference from denotes the state of that task (Ready, Blocked, Suspended ). */
    ListItem_t xEventListItem;                  /*< Used to reference a task from an event list. */
    UBaseType_t uxPriority;                     /*< The priority of the task.  0 is the lowest priority. */
    StackType_t * pxStack;                      /*< Points to the start of the stack. */
    char pcTaskName[ configMAX_TASK_NAME_LEN ]; /*< Descriptive name given to the task when created.  Facilitates debugging only. */ /*lint !e971 Unqualified char types are allowed for strings and single characters only. */

    #if ( configNUMBER_OF_CORES > 1 )
        BaseType_t xCoreID; /*< The core that this task is pinned to */
    #endif /* configNUMBER_OF_CORES > 1 */

    #if ( ( portSTACK_GROWTH > 0 ) || ( configRECORD_STACK_HIGH_ADDRESS == 1 ) )
        StackType_t * pxEndOfStack; /*< Points to the highest valid address for the stack. */
    #endif

    #if ( portCRITICAL_NESTING_IN_TCB == 1 )
        UBaseType_t uxCriticalNesting; /*< Holds the critical section nesting depth for ports that do not maintain their own count in the port layer. */
    #endif

    #if ( configUSE_TRACE_FACILITY == 1 )
        UBaseType_t uxTCBNumber;  /*< Stores a number that increments each time a TCB is created.  It allows debuggers to determine when a task has been deleted and then recreated. */
        UBaseType_t uxTaskNumber; /*< Stores a number specifically for use by third party trace code. */
    #endif

    #if ( configUSE_MUTEXES == 1 )
        UBaseType_t uxBasePriority; /*< The priority last assigned to the task - used by the priority inheritance mechanism. */
        UBaseType_t uxMutexesHeld;
    #endif

    #if ( configUSE_APPLICATION_TASK_TAG == 1 )
        TaskHookFunction_t pxTaskTag;
    #endif

    #if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0 )
        void * pvThreadLocalStoragePointers[ configNUM_THREAD_LOCAL_STORAGE_POINTERS ];
    #endif

    #if ( configGENERATE_RUN_TIME_STATS == 1 )
        configRUN_TIME_COUNTER_TYPE ulRunTimeCounter; /*< Stores the amount of time the task has spent in the Running state. */
    #endif

    #if ( ( configUSE_NEWLIB_REENTRANT == 1 ) || ( configUSE_C_RUNTIME_TLS_SUPPORT == 1 ) )
        configTLS_BLOCK_TYPE xTLSBlock; /*< Memory block used as Thread Local Storage (TLS) Block for the task. */
    #endif

    #if ( configUSE_TASK_NOTIFICATIONS == 1 )
        volatile uint32_t ulNotifiedValue[ configTASK_NOTIFICATION_ARRAY_ENTRIES ];
        volatile uint8_t ucNotifyState[ configTASK_NOTIFICATION_ARRAY_ENTRIES ];
    #endif

    /* See the comments in FreeRTOS.h with the definition of
     * tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE. */
    #if ( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 ) /*lint !e731 !e9029 Macro has been consolidated for readability reasons. */
        uint8_t ucStaticallyAllocated;                     /*< Set to pdTRUE if the task is a statically allocated to ensure no attempt is made to free the memory. */
    #endif

    #if ( INCLUDE_xTaskAbortDelay == 1 )
        uint8_t ucDelayAborted;
    #endif

    #if ( configUSE_POSIX_ERRNO == 1 )
        int iTaskErrno;
    #endif
} tskTCB;

volatile char IRAM_ATTR * log_event(volatile char * s, const char * e) {
    //s = gather_task_states(s, e);
    if(s + 2 < e){
        *s++ = '\r';
        *s++ = '\n';
        *s   = (char)0x00;
    }
    return s;
}
void IRAM_ATTR log_event_on_enter( )
{
    /*
    int64_t 		_now = esp_timer_get_time();
    TaskHandle_t	_hTask = xTaskGetCurrentTaskHandle();
    sprintf(_tmpbuf,"%08llu -> %s\n",_now,_hTask->pcTaskName);
    int8_t _len = strlen(_tmpbuf);
    if(_buflen + _len < TRACE_BUFSIZ){
        strcpy(_bufp,_tmpbuf);
        _bufp += _len;
        _buflen += _len;
    }
    */
   if(_bufp +2 < _bufENDp){
    *_bufp++ = '-';
    *_bufp++ = '>';
    *_bufp   = (char) 0x00;
   }
   else {
    //printf("En\n");
   }
   _bufp = log_event(_bufp, _bufENDp);

}
void IRAM_ATTR log_event_on_exit( )
{
    /*
    int64_t 		_now = esp_timer_get_time();
    TaskHandle_t	_hTask = xTaskGetCurrentTaskHandle();
    sprintf(_tmpbuf,"%08llu <- %s\n",_now,_hTask->pcTaskName);
    int8_t _len = strlen(_tmpbuf);
    if(_buflen + _len < TRACE_BUFSIZ){
        strcpy(_bufp,_tmpbuf);
        _bufp += _len;
        _buflen += _len;
    }
    */
   if(_bufp +2 < _bufENDp){
    *_bufp++ = '<';
    *_bufp++ = '-';
    *_bufp   = (char) 0x00;
   }
   else {
    //printf("Ex\n");
   }
   _bufp = log_event(_bufp, _bufENDp);
}
volatile char IRAM_ATTR * gather_task_states( volatile char * pcWriteBuffer, const char * pcWriteBufferENDp )
    {
        //TaskStatus_t * pxTaskStatusArray;
        UBaseType_t uxArraySize, x;
        char cStatus;

        /* Make sure the write buffer does not contain a string. */
        *pcWriteBuffer = ( char ) 0x00;

        /* Take a snapshot of the number of tasks in case it changes while this
         * function is executing. */
        uxArraySize = uxTaskGetNumberOfTasks() ;


        if( pxTaskStatusArray != NULL )
        {
            /* Write the task name to the string, padding with spaces so it
            * can be printed in tabular form more easily. */
            //pcWriteBuffer = prvWriteNameToBuffer( pcWriteBuffer, pxTaskStatusArray[ x ].pcTaskName );
            if(pcWriteBuffer +9 < pcWriteBufferENDp) {
                sprintf(pcWriteBuffer, "%08llu ",esp_timer_get_time());
                pcWriteBuffer += 9;
            } 
            else {
                //vPortFree( pxTaskStatusArray );
                return pcWriteBuffer;
            }
            /* Generate the (binary) data. */
            uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, NULL );

            /* Create a human readable table from the binary data. */
            for( x = 0; x < uxArraySize; x++ )
            {
                switch( pxTaskStatusArray[ x ].eCurrentState )
                {
                    case eRunning:
                        cStatus = tskRUNNING_CHAR;
                        break;

                    case eReady:
                        cStatus = tskREADY_CHAR;
                        break;

                    case eBlocked:
                        cStatus = tskBLOCKED_CHAR;
                        break;

                    case eSuspended:
                        cStatus = tskSUSPENDED_CHAR;
                        break;

                    case eDeleted:
                        cStatus = tskDELETED_CHAR;
                        break;

                    case eInvalid: /* Fall through. */
                    default:       /* Should not get here, but it is included
                                    * to prevent static checking errors. */
                        cStatus = ( char ) 0x00;
                        break;
                }


                /* Write the task number and their states */
                if(pcWriteBuffer +5< pcWriteBufferENDp) {
                    sprintf( pcWriteBuffer, "\t%u[%c]", ( unsigned int ) pxTaskStatusArray[ x ].xTaskNumber, cStatus );
                    pcWriteBuffer += 5;
                } 
                else {
                    //vPortFree( pxTaskStatusArray );
                    return pcWriteBuffer;
                }
            }

            /* Free the array again.  NOTE!  If configSUPPORT_DYNAMIC_ALLOCATION
             * is 0 then vPortFree() will be #defined to nothing. */
            //vPortFree( pxTaskStatusArray );
        }
        else {
                /* Write the task number and their states */
                if(pcWriteBuffer +4< pcWriteBufferENDp) {
                    *pcWriteBuffer++ = 'N';
                    *pcWriteBuffer++ = 'U';
                    *pcWriteBuffer++ = 'L';
                    *pcWriteBuffer++ = 'L';
                    *pcWriteBuffer   = '\0';
                } 

        }
        return pcWriteBuffer;
    }