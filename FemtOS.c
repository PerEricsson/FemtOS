/*
 * File:   TinyRTOS.c
 * Author: Per Ericsson
 *
 * Created on den 21 juli 2014, 13:26
 */


#include <xc.h>

// CONFIG
#pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select bit (MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)

#include <stdint.h>

#define TMR1HIGH    (0xFF - (100-1)/0x100)
#define TMR1LOW     (0xFF - (100-1)%0x100)

/**
 * All events in the system are defined here
 */
typedef enum event {
    NOP = 0, INIT, PERIDIC_TIMER, ONESHOT_TIMER, TASK0_EVT, BUTTON_DOWN, BUTTON_UP, TMR1_EVT
} Event;


// TODO: Integrate the event queues into the task control block TCB. Initialize
// queues during task initialization.

/**
 * Define the event queue structure and set up the queues for the application
 */
#define EventQueueSize 2

typedef struct {
    uint8_t nMessages;
    uint8_t head;
    uint8_t tail;
    Event queue[EventQueueSize];
} EventQueue;

/**
 * Declare and define event queues
 */
EventQueue queue0 = {0, 0, 0};
EventQueue queue1 = {0, 0, 0};
EventQueue queue2 = {0, 0, 0};

/**
 * postEvent takes an event and posts it in the queue pointed to by q.
 * Use this function outside of ISR since it disables (and re-enables) interrupts
 * to guarantee atomic execution. This function is used for direct posting to a
 * specific queue. Also see publishEvent which announces an event to all tasks
 * that have signed up as listeners.
 * The function blocks indefinitely if the queue is already full. No recovery!
 * @param e The event
 * @param q Pointer to the queue
 */
void postEvent(Event e, EventQueue *q) {
    di();
    if (q->nMessages < EventQueueSize) {
        q->nMessages++;
        q->queue[q->head] = e;
        q->head++;
        if (q->head == EventQueueSize)
            q->head = 0;
    } else
        while (1);
    ei();
}

/**
 * postEvent_ISR should be used inside interrupt service routines where interrupts
 * are already disabled.
 * @param e
 * @param q
 */
void postEvent_ISR(Event e, EventQueue *q) {
    if (q->nMessages < EventQueueSize) {
        q->nMessages++;
        q->queue[q->head] = e;
        q->head++;
        if (q->head == EventQueueSize)
            q->head = 0;
    } else
        while (1);
}

/**
 * getEvent returns the next event from queue pointed to by q. If queue is empty,
 * a NOP is returned.
 * @param q pointer to queue
 * @return Event
 */
Event getEvent(EventQueue *q) {
    Event e = NOP;
    di();
    if (q->nMessages) {
        q->nMessages--;
        e = q->queue[q->tail];
        q->tail++;
        if (q->tail == EventQueueSize)
            q->tail = 0;
    }
    ei();
    return e;
}

/**
 * getEvent_ISR should be used in interrupt service routines since interrupts are
 * already disabled.
 * @param q Pointer to queue
 * @return Event
 */
Event getEvent_ISR(EventQueue *q) {
    Event e = NOP;

    if (q->nMessages) {
        q->nMessages--;
        e = q->queue[q->tail];
        q->tail++;
        if (q->tail == EventQueueSize)
            q->tail = 0;
    }
    return e;
}

/**
 * getQueueLength returns the number of items in the queue.
 * @param q Pointer to queue
 * @return number of elements in queue
 */
uint8_t getQueueLength(EventQueue *q) {
    return q->nMessages;
}

/**
 * Functions relating to tasks
 */

// The maximum number of listeners to each task. All tasks get the same size.
#define ListenerListSize 3

// The tasks in the system.

typedef enum taskID {
    TASK0 = 0, TASK1, TASK2, TaskListSize
} taskID;

/**
 * TCB (Task Control Block) is a structure to keep all essential information about a task
 * f is a pointer to a function that process the events sent to the task
 * q is a pointer to the queue structure that contain events posted to the task
 * nListeners counts the number of listeners subscribing to events posted by the task
 * listeners is the collection of task ID's of listening tasks. Task ID is the same
 * as is the index for the listener in the allTasks array.
 */
typedef struct {
    void (*f)(Event, taskID);
    EventQueue *q;
    uint8_t nListeners;
    taskID listeners[ListenerListSize];
    // EventQueue q // Use an instance of event queue rather than a pointer to a queue (saves a little space)
} TCB;

TCB allTasks[TaskListSize]; // Array containg the TCB of all tasks. TaskID is the
// index into the array for a particular task.

/**
 * createTask makes an entry in the allTasks array for a task defined by a
 * function f and a queue q. No error handling if task array is full.
 * @param ID TaskID of the task. Set manually by user
 * @param f function that processes the events sent to the task
 * @param q pointer to event queue of the task. Queue must be set up separately.
 */
void createTask(taskID ID, void (*f)(Event, taskID), EventQueue *q) {
    allTasks[ID].f = f;
    allTasks[ID].q = q;
    allTasks[ID].nListeners = 0;
}

/**
 * addListener connects a listener task ID to a publishing task ID. No error handling
 * of a full listener list. Set size by adjusting ListenerListSize
 * @param subscriber the Task ID of the listening task
 * @param publisher the Task ID of the publishing task
 */
void addListener(taskID subscriber, taskID publisher) {
    if (allTasks[publisher].nListeners < ListenerListSize) {
        allTasks[publisher].listeners[allTasks[publisher].nListeners] = subscriber;
        allTasks[publisher].nListeners++;
    }
}

/**
 * publishEvent distributes an event e to all listeners of a particular task ID.
 * @param e Event
 * @param publisher task ID of publishing task. Same as index in allTasks array.
 */
void publishEvent(Event e, taskID publisher) {
    taskID i = 0;
    for (i = 0; i < allTasks[publisher].nListeners; i++) {
        postEvent(e, allTasks[allTasks[publisher].listeners[i]].q);
    }
}

/**
 * Periodic timers
 */

/**
 * struct to encapsulate inforamtion about a periodic timer. reloadValue is the
 * period time, currentValue holds the current count, task is the TaskID for the
 * receiving task. ID is the index of the task in the allTasks array.
 */
typedef struct {
    uint16_t reloadValue;
    uint16_t currentValue;
    taskID task;
} PeriodicTimer;

#define PeriodicTimerListSize 3 // Number of periodic timers.
PeriodicTimer allPeriodicTimers[PeriodicTimerListSize]; // Array of all periodic timers
uint8_t nPeriodicTimers = 0; // counter of periodic timers in use.

/**
 * createPeriodicTimer creates a timer that triggers a task periodically.
 * The period is measured in ticks. An ISR is responsible for managing the timers
 * and posting timer events to receiving tasks.
 * @param task Task ID is the index in the allTasks array
 * @param timeout number of ticks between activation of task
 */
void createPeriodicTimer(taskID task, uint16_t timeout) {
    if (nPeriodicTimers < PeriodicTimerListSize) {
        allPeriodicTimers[nPeriodicTimers].currentValue = timeout;
        allPeriodicTimers[nPeriodicTimers].reloadValue = timeout;
        allPeriodicTimers[nPeriodicTimers].task = task;
        nPeriodicTimers++;
    }
}

/**
 * One-shot timers
 * The timers measure time in units of system ticks
 */

typedef struct {
    uint16_t currentValue; // Current time
    uint8_t armed; // Controls whether the timer is active or not
    taskID task; // Task associated with timer
    uint8_t busy; // Is the timer free to use, or busy
} OneShotTimer;

#define OneShotTimerListSize 4
OneShotTimer allOneShotTimers[OneShotTimerListSize];

/**
 * Prepare array of timers
 */

void initializeOneShotTimers(void) {
    uint8_t i;

    for (i = 0; i < OneShotTimerListSize; i++) {
        allOneShotTimers[i].armed = 0;
        allOneShotTimers[i].busy = 0;
    }
}

/**
 * createOneShotTimer attaches one timer to the calling task.
 * @param taskID
 * @return timerID
 */
uint8_t createOneShotTimer(taskID ID) {
    uint8_t i;

    for (i = 0; i < OneShotTimerListSize; i++) {
        if (!allOneShotTimers[i].busy) {
            allOneShotTimers[i].task = ID;
            allOneShotTimers[i].busy = 1;
            allOneShotTimers[i].armed = 0;
            break;
        }
    }
    return i;
}

/**
 * Free the timer for someone else to use
 * @param timer
 */
void freeOneShotTimer(uint8_t timer) {
    allOneShotTimers[timer].busy = 0;
    allOneShotTimers[timer].armed = 0;
}

/**
 * armOneShotTimer sets the timeout value and activates the timer.
 * @param timer
 * @param timeout
 */
void armOneShotTimer(uint8_t timer, uint16_t timeout) {
    allOneShotTimers[timer].currentValue = timeout;
    allOneShotTimers[timer].armed = 1;
}

/**
 * disarmOneShotTimer disarms the timer and prevents it from tripping
 * @param timer
 */
void disarmOneShotTimer(uint8_t timer) {
    allOneShotTimers[timer].armed = 0;
}

/**
 * systemTick should be called by an interrupt driven timer once per system tick.
 * The function decrements all periodic timers. On timeout, the timer is rearmed
 * with its reload value and a timer event is posted to the affected task.
 * Oneshot timers are decremented and an event is posted to the relevant task upon
 * timeout.
 * It is important that the systemTick routine is called with interrupts disabled.
 */
void systemTick(void) {
    uint8_t i;

    for (i = 0; i < nPeriodicTimers; i++) {
        if (!--allPeriodicTimers[i].currentValue) {
            postEvent_ISR(PERIDIC_TIMER, allTasks[allPeriodicTimers[i].task].q);
            allPeriodicTimers[i].currentValue = allPeriodicTimers[i].reloadValue;
        }
    }

    for (i = 0; i < OneShotTimerListSize; i++) {
        if (allOneShotTimers[i].armed) {
            if (!--allOneShotTimers[i].currentValue) {
                postEvent_ISR(ONESHOT_TIMER, allTasks[allOneShotTimers[i].task].q);
                allOneShotTimers[i].armed = 0;
            }
        }
    }
}

/**
 * Timer 1 functions
 */
taskID TMR1receiver;

void armTimer1(uint16_t time, taskID task){
    /**
     * Setup and arm timer 1. Attach task to receive message.
     * The unit of time is fosc/4 devided by the prescaler factor (8) set
     * during hardware initialization.
     */

    TMR1ON = 0;

    TMR1receiver = task;

    TMR1H = 0xFF - (time - 1)/0x100;
    TMR1L = 0xFF - (time - 1)%0x100;
    TMR1ON = 1;
    TMR1IE = 1;
}

void Timer1Timeout(void){
    /**
     * Switch off timer 1 and send message to receiving task. This function
     * is called from the ISR, so interrupts are disabled.
     */
    
    TMR1IE = 0;
    TMR1ON = 0;

    postEvent_ISR(TMR1_EVT, allTasks[TMR1receiver].q);
}

/**
 * Set up hardware. Of specific importance is to set up a periodic timer
 * to generate the system tick.
 */
void initHardware(void) {
    /*
     Initialize ports and setup interrupt timer
     */

    ANSEL = 0; // Digital IO
    ANSELH = 0;

    TRISC = 0; // Outputs
    PORTC = 0; // Clear portc, RC<0:3> connects to DB<4:7>

    TRISB = 0b01111111; // RB7 output
    PORTB = 0; // LED on RB7

    TRISA = 0b00001001; // RA0 is contrast pot, RA3 always input
    PORTA = 0;

    /**
     * Initialize timer 2 to generate a system tick each 1 ms (1000 fosc/4),
     * where default fosc = 4 MHz is assumed.
     */
    PR2 = 124; // Timer 2 counter (125-1), 1 ms
    T2CON = 0b00001101; // Timer 2 on, 4 prescaler, 2 postscaler
    TMR2IF = 0; // Clear interrupt flag
    TMR2IE = 1; // Enable timer 2 interrupts

    /**
     * Initialize timer 1 to use internal oscillator
     */
    T1CON = 0b00110000; // Use internal oscillator, 8 prescaler, osc off
    TMR1IF = 0;     // Clear interrupt flag

    PEIE = 1; // Enable peripheral interrupts
}

/**
 * Interrupt service routine
 */
void interrupt isr(void) {
    /*
     Interrupt service routine
     */

    if (TMR2IF && TMR2IE) {
        systemTick(); // Call systemTick periodically
        TMR2IF = 0;
    }

    if (TMR1IF && TMR1IE) {
        // TMR1 interrupt
        Timer1Timeout();
        TMR1IF = 0;
    }

}

/**
 * startScheduler contains an infinite loop that watches all task event queues
 * and calls the event handler function for each task.
 */
void startScheduler(void) {
    taskID i = 0;

    // Post INIT events in all event queues.
    for (i = 0; i < TaskListSize; i++)
        postEvent_ISR(INIT, allTasks[i].q);

    ei();
    while (1) {
        for (i = 0; i < TaskListSize; i++)
            if (getQueueLength(allTasks[i].q))
                (*allTasks[i].f)(getEvent(allTasks[i].q), i);
    };
}

/**
 * User tasks. A task is a function taking an event as a parameter, but returns
 * nothing. The task often contain a state machine to handle the events.
 * void task(Event e){...}
 * Tasks must NOT block execution or the system stalls.
 * @param e event to be processed.
 */

void t0(Event e, taskID thisTask) {
    switch (e) {
        case PERIDIC_TIMER:
            publishEvent(TASK0_EVT, thisTask);
            break;
    }
}

void t1(Event e, taskID thisTask) {
    static uint8_t oneshot;

    switch (e) {
        case INIT:
            break;
        case TASK0_EVT:
            RC0 ^= 1;
            break;
        case BUTTON_UP:
            RC1 = 1;
            break;
        case BUTTON_DOWN:
            RC1 = 0;
            oneshot = createOneShotTimer(thisTask);
            armOneShotTimer(oneshot, 650);
            break;
        case ONESHOT_TIMER:
            RC3 ^= 1;
            freeOneShotTimer(oneshot);
            break;
    }
}

uint8_t getButton(void) {
    return RA3;
}

void t2(Event e, taskID thisTask) {

    typedef enum {
        UP, DOWN
    } States;
    static States theState = UP;

#define timerReload 20
    static uint8_t timer = timerReload;

    switch (e) {
        case PERIDIC_TIMER:
            RC2 = getButton();
            switch (theState) {
                case UP:
                    if (!getButton()) {
                        if (!--timer) {
                            theState = DOWN;
                            publishEvent(BUTTON_DOWN, thisTask);
                            timer = timerReload;
                        }
                    } else
                        timer = timerReload;
                    break;
                case DOWN:
                    if (getButton()) {
                        if (!--timer) {
                            theState = UP;
                            publishEvent(BUTTON_UP, thisTask);
                            timer = timerReload;
                        }
                    } else
                        timer = timerReload;
                    break;
            }
            break;
    }
}

/**
 * The main function calls hardware initialization, sets up tasks, hooks up
 * listeners with publishers, arms timers and finally starts the scheduler.
 */
void main(void) {
    /**
     * Init hardware to setup IOs and timers
     */
    initHardware();

    /**
     * Create all tasks in the system and connect a queue to each task. TaskID's
     * are assigned in order of creation.
     */
    createTask(TASK0, t0, &queue0); // TaskID = 0
    createTask(TASK1, t1, &queue1); // TaskID = 1
    createTask(TASK2, t2, &queue2); // TaskID = 2

    /**
     * Establish a connection between publishing and subscribing tasks. TaskID's
     * identify the tasks.
     */
    addListener(TASK1, TASK0); // Task 1 listens to task 0
    addListener(TASK1, TASK2); // Task 1 listens to task 2

    /**
     * Creates periodic timers and connects them to tasks.
     */
    createPeriodicTimer(TASK0, 100);
    createPeriodicTimer(TASK2, 10);

    /**
     * Initialize oneshot timers
     */
    initializeOneShotTimers();

    /**
     * Everything is set up. Start the scheduler. We don't expect a return from
     * this function.
     */
    startScheduler();

    return;
}
