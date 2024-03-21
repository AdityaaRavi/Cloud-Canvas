#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_nvic.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "prcm.h"
#include "utils.h"
#include "systick.h"
#include "rom_map.h"
#include "interrupt.h"
#include "gpio.h"
#include "utils.h"

// Common interface includes
#include "uart_if.h"
#include "uart.h"
#include "common.h"
// Simplelink includes
#include "simplelink.h"

// Pin configurations
#include "pin_mux_config.h"

// oled includes
#include "spi.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"

#include <stdio.h>

// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"

//Common interface includes
#include "pinmux.h"
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"
#include "i2c_if.h"

#include "cJSON.h"

#define optionalDelay 1000000

#define B0 0b01100000110111111001001001101101
#define B1 0b01100000110111111100100000110111
#define B2 0b01100000110111110000100011110111
#define B3 0b01100000110111111000100001110111
#define B4 0b01100000110111111111000000001111
#define B5 0b01100000110111110011000011001111
#define B6 0b01100000110111111011000001001111
#define B7 0b01100000110111111101000000101111
#define B8 0b01100000110111110001000011101111
#define B9 0b01100000110111111001000001101111
#define ENTER 0b01100000110111110011101011000101 //ENTER
#define LAST 0b01100000110111111110000000011111


#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1


#define APPLICATION_NAME        "SSL"
#define APPLICATION_VERSION     "1.1.1.EEC.Spring2018"
#define SERVER_NAME             "a23h4lxy81wbkr-ats.iot.us-east-1.amazonaws.com"
#define GOOGLE_DST_PORT         8443

#define SL_SSL_CA_CERT "/cert/rootCA.der" //starfield class2 rootca (from firefox) // <-- this one works
#define SL_SSL_PRIVATE "/cert/private.der"
#define SL_SSL_CLIENT  "/cert/client.der"


//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                7    /* Current Date */
#define MONTH               3     /* Month 1-12 */
#define YEAR                2024  /* Current year */
#define HOUR                9    /* Time - hours */
#define MINUTE              54    /* Time - minutes */
#define SECOND              30    /* Time - seconds */

#define POSTHEADER "POST /things/new_thing_cc3200/shadow HTTP/1.1\r\n"
#define GETHEADER "GET /things/new_thing_cc3200/shadow HTTP/1.1\r\n"
#define HOSTHEADER "Host: a23h4lxy81wbkr-ats.iot.us-east-1.amazonaws.com\r\n"
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

#define DATA1 "{\"state\": {\r\n\"desired\" : {\r\n\"var\" : \"test\"\r\n}}}\r\n\r\n"

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

typedef struct
{
   /* time */
   unsigned long tm_sec;
   unsigned long tm_min;
   unsigned long tm_hour;
   /* date */
   unsigned long tm_day;
   unsigned long tm_mon;
   unsigned long tm_year;
   unsigned long tm_week_day; //not required
   unsigned long tm_year_day; //not required
   unsigned long reserved[3];
}SlDateTime;



//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

#define SPI_IF_BIT_RATE  100000

// some helpful macros for systick

// the cc3200's fixed clock frequency of 80 MHz
// note the use of ULL to indicate an unsigned long long constant
#define SYSCLKFREQ 80000000ULL

// macro to convert ticks to microseconds
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

// macro to convert microseconds to ticks
#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))

// systick reload value set to 40ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 3200000UL
#define ZERO_INT 100

// track systick counter periods elapsed
// if it is not 0, we know the transmission ended
volatile int systick_cnt = 0;

extern void (* const g_pfnVectors[])(void);

volatile unsigned long SW_intcount = 0;
volatile unsigned SW_intflag;
volatile int first_edge = 1;
int start  = 0;
char  prevLetter = '/';
char buffer[64];
int bufIndex = 0;

volatile long currButton;
volatile long prevButton;
volatile long prevData;
int sameButton = 0;
int EnterMessage = 0;

volatile unsigned long data = 0;
int track = 0;
int color = 1;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

// an example of how you can use structs to organize your pin settings for easier maintenance
typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

static PinSetting button = { .port = GPIOA0_BASE, .pin = 0x1};

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************

// Standard Letter of Each Button
char firstLetter(unsigned long value)
{
    char letter;
    switch(value)
    {
        case B0:
            letter = ' ';
            Report("letter: %c \n\r", letter);
            break;
        case B1:
            letter = '*';
            DisplayColor();
            break;
        case B2:
            letter = 'a';
            setCursor(6, 120);
            Outstr("Restoring Picture...");
            Report("letter: %c \n\r", letter);
            break;
        case B3:
            letter = 'd';
            Report("letter: %c \n\r", letter);
            break;
        case B4:
            letter = 'g';
            Report("letter: %c \n\r", letter);
            break;
        case B5:
            letter = 'j';
            Report("letter: %c \n\r", letter);
            break;
        case B6:
            letter = 'm';
            Report("letter: %c \n\r", letter);
            break;
        case B7:
            letter = 'p';
            Report("letter: %c \n\r", letter);
            break;
        case B8:
            letter = 't';
            Report("letter: %c \n\r", letter);
            break;
        case B9:
            letter = 'w';
            Report("letter: %c \n\r", letter);
            break;
        case ENTER:
            letter = '-';
            Report("letter: enter %c \n\r", letter);
            break;
        case LAST:
            letter = '+';
            Report("letter: %c \n\r", letter);
            break;
        default:
            Report("error: not valid. \n\r", letter);
            break;
    }
    return letter;
}




/**
 * Reset SysTick Counter
 */
static inline void SysTickReset(void) {
    // any write to the ST_CURRENT register clears it
    // after clearing it automatically gets reset without
    // triggering exception logic
    // see reference manual section 3.2.1
    HWREG(NVIC_ST_CURRENT) = 1;

    // clear the global count variable
    systick_cnt = 0;
}

/**
 * SysTick Interrupt Handler
 *
 * Keep track of whether the systick counter wrapped
 */
static void SysTickHandler(void) {
    // increment every time the systick handler fires
    systick_cnt++;
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void) {
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);

    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

/**
 * Initializes SysTick Module
 */
static void SysTickInit(void) {

    // configure the reset value for the systick countdown register
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);

    // register interrupts on the systick module
    MAP_SysTickIntRegister(SysTickHandler);

    // enable interrupts on systick
    // (trigger SysTickHandler when countdown reaches 0)
    MAP_SysTickIntEnable();

    // enable the systick module itself
    MAP_SysTickEnable();
}

static void GPIOA2IntHandler(void) {    // IR interrupt handler

    if (first_edge) {
        SysTickReset();
        first_edge = 0;
    }
    else {
        // read the countdown register and compute elapsed cycles
        uint64_t delta = SYSTICK_RELOAD_VAL - SysTickValueGet();

        // convert elapsed cycles to microseconds
        uint64_t delta_us = TICKS_TO_US(delta);

        SysTickReset();

        if (delta_us >= 35000){ // Timeout
            SW_intcount = 0;
        }
        else if (delta_us >= 2500 && delta_us < 35000){ // Finds Start Time
            data = 0;
            SW_intcount = 1;
        }
        
        if(delta_us > 1300 && delta_us < 2500){ // Determines 1 bit
            data = data << 1;
            data = data + 1;
        }
        else if(delta_us < 1300){ // Determines 0 bit
            data = data << 1;
        }
        else if(delta_us > 2500){
            data = 0;
        }
    }

    SW_intcount++;

    // Resets interrupt handle for next button pressed
    if (SW_intcount == 34) {
        SW_intcount = 0;
        first_edge = 1;
        SW_intflag = 1;
    }

    unsigned long ulStatus;
    ulStatus = MAP_GPIOIntStatus (button.port, true);
    MAP_GPIOIntClear(button.port, ulStatus);       // clear interrupts on GPIOA2

}
//****************************************************************************
//
//! Main function
//!
//! \param none
//!
//!
//! \return None.
//
//****************************************************************************

void enableSPI(){

    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    MAP_UtilsDelay(8000000);
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    // Initialize the message
    //

    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                   SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                   (SPI_SW_CTRL_CS |
                   SPI_4PIN_MODE |
                   SPI_TURBO_OFF |
                   SPI_CS_ACTIVEHIGH |
                   SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);
}

volatile char receivedChar;
volatile int receivedCharCount = 0;
volatile char receivedCharBuffer[256] = {};
volatile int sameButtonPressReceived = 0;

int charCount = 0;
//////////////////////////////////////////////////////////////////////////////// Connecting to AWS
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulPingPacketsRecv = 0; //Number of Ping Packets received
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
signed char    *g_Host = SERVER_NAME;
SlDateTime g_time;
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End: df
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static long WlanConnect();
static int set_time();
static void BoardInit(void);
static long InitializeAppVariables();
static int tls_connect();
static int connectToAccessPoint();
static int http_post(int ignore, char* xPos, char* yPos, char* color, char* radius, int numPoints);
static int http_get(int* xPosArray, int* yPosArray, int* radiusArray, int* colorArray, int* size);

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
    if(!pWlanEvent) {
        return;
    }

    switch(pWlanEvent->Event) {
        case SL_WLAN_CONNECT_EVENT: {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'.
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                       g_ucConnectionSSID,g_ucConnectionBSSID[0],
                       g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                       g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                       g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT: {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code) {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                    "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default: {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    if(!pNetAppEvent) {
        return;
    }

    switch(pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT: {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                       "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
        }
        break;

        default: {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse) {
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {
    if(!pDevEvent) {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
    if(!pSock) {
        return;
    }

    switch( pSock->Event ) {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status) {
                case SL_ECLOSE:
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n",
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default:
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End breadcrumb: s18_df
//*****************************************************************************


//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    0 on success else error code
//!
//! \return None
//!
//*****************************************************************************
static long InitializeAppVariables() {
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    g_Host = SERVER_NAME;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    return SUCCESS;
}


//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState() {
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode
    if (ROLE_STA != lMode) {
        if (ROLE_AP == lMode) {
            // If the device is in AP mode, we need to wait for this event
            // before doing anything
            while(!IS_IP_ACQUIRED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
            }
        }

        // Switch to STA role and restart
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again
        if (ROLE_STA != lRetVal) {
            // We don't want to proceed if the device is not coming up in STA-mode
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }

    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);



    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal) {
        // Wait
        while(IS_CONNECTED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();

    return lRetVal; // Success
}

//****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  None
//!
//! \return  0 on success else error code
//!
//! \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect() {
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    UART_PRINT("Attempting connection to access point: ");
    UART_PRINT(SSID_NAME);
    UART_PRINT("... ...");
    lRetVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT(" Connected!!!\n\r");


    // Wait for WLAN Event
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
        // Toggle LEDs to Indicate Connection Progress
        _SlNonOsMainLoopTask();
//        GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
        _SlNonOsMainLoopTask();
//        GPIO_IF_LedOn(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
    }

    return SUCCESS;

}




long printErrConvenience(char * msg, long retVal) {
    UART_PRINT(msg);
//    GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    return retVal;
}


//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! This function demonstrates how certificate can be used with SSL.
//! The procedure includes the following steps:
//! 1) connect to an open AP
//! 2) get the server name via a DNS request
//! 3) define all socket options and point to the CA certificate
//! 4) connect to the server via TCP
//!
//! \param None
//!
//! \return  0 on success else error code
//! \return  LED1 is turned solid in case of success
//!    LED2 is turned solid in case of failure
//!
//*****************************************************************************
static int tls_connect() {
    SlSockAddrIn_t    Addr;
    int    iAddrSize;
    unsigned char    ucMethod = SL_SO_SEC_METHOD_TLSV1_2;
    unsigned int uiIP;
//    unsigned int uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA;
    unsigned int uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256;
// SL_SEC_MASK_SSL_RSA_WITH_RC4_128_SHA
// SL_SEC_MASK_SSL_RSA_WITH_RC4_128_MD5
// SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_DHE_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_RC4_128_SHA
// SL_SEC_MASK_TLS_RSA_WITH_AES_128_CBC_SHA256
// SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA256
// SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256
// SL_SEC_MASK_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256 // does not work (-340, handshake fails)
    long lRetVal = -1;
    int iSockID;

    lRetVal = sl_NetAppDnsGetHostByName(g_Host, strlen((const char *)g_Host),
                                    (unsigned long*)&uiIP, SL_AF_INET);

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't retrieve the host name \n\r", lRetVal);
    }

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons(GOOGLE_DST_PORT);
    Addr.sin_addr.s_addr = sl_Htonl(uiIP);
    iAddrSize = sizeof(SlSockAddrIn_t);
    //
    // opens a secure socket
    //
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
    if( iSockID < 0 ) {
        return printErrConvenience("Device unable to create secure socket \n\r", lRetVal);
    }

    //
    // configure the socket as TLS1.2
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECMETHOD, &ucMethod,\
                               sizeof(ucMethod));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
    //
    //configure the socket as ECDHE RSA WITH AES256 CBC SHA
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &uiCipher,\
                           sizeof(uiCipher));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }



/////////////////////////////////
// START: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION
    //
    //configure the socket with CA certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                           SL_SO_SECURE_FILES_CA_FILE_NAME, \
                           SL_SSL_CA_CERT, \
                           strlen(SL_SSL_CA_CERT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
// END: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION
/////////////////////////////////


    //configure the socket with Client Certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                SL_SO_SECURE_FILES_CERTIFICATE_FILE_NAME, \
                                    SL_SSL_CLIENT, \
                           strlen(SL_SSL_CLIENT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //configure the socket with Private Key - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
            SL_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME, \
            SL_SSL_PRIVATE, \
                           strlen(SL_SSL_PRIVATE));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }


    /* connect to the peer device - Google server */
    lRetVal = sl_Connect(iSockID, ( SlSockAddr_t *)&Addr, iAddrSize);

    if(lRetVal >= 0) {
        UART_PRINT("Device has connected to the website:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }
    else if(lRetVal == SL_ESECSNOVERIFY) {
        UART_PRINT("Device has connected to the website (UNVERIFIED):");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }
    else if(lRetVal < 0) {
        UART_PRINT("Device couldn't connect to server:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
        return printErrConvenience("Device couldn't connect to server \n\r", lRetVal);
    }

//    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
//    GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    return iSockID;
}



int connectToAccessPoint() {
    long lRetVal = -1;
//    GPIO_IF_LedConfigure(LED1|LED3);

//    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
//    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    lRetVal = InitializeAppVariables();
    ASSERT_ON_ERROR(lRetVal);

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0) {
      if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
          UART_PRINT("Failed to configure the device in its default state \n\r");

      return lRetVal;
    }

    UART_PRINT("Device is configured in default state \n\r");

    CLR_STATUS_BIT_ALL(g_ulStatus);

    ///
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    UART_PRINT("Opening sl_start\n\r");
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal) {
        UART_PRINT("Failed to start the device \n\r");
        return lRetVal;
    }

    UART_PRINT("Device started as STATION \n\r");

    //
    //Connecting to WLAN AP
    //
    lRetVal = WlanConnect();
    if(lRetVal < 0) {
        UART_PRINT("Failed to establish connection w/ an AP \n\r");
//        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }

    UART_PRINT("Connection established w/ AP and IP is aquired \n\r");
    return 0;
}


//char buffer[256] = {};
int buffIndex = -1;


// Function to convert a C array to JSON representation
char *array_to_json(int *array, int size) {
    // Calculate the maximum size needed for the JSON string
    int max_size = size * 5 + 3; // Assuming each number takes at most 5 characters and additional 3 characters for brackets and commas
    char *json_str = malloc(max_size * sizeof(char));
    if (!json_str) {
        fprintf(stderr, "Memory allocation failed.\n");
        return NULL;
    }


    int offset = 0;
    offset += snprintf(json_str + offset, max_size - offset, "[");

    int i = 0;
    for (; i < size; i++) {
        offset += snprintf(json_str + offset, max_size - offset, "%d", array[i]);
        if (i < size - 1) {
            offset += snprintf(json_str + offset, max_size - offset, ",");
        }
    }

    offset += snprintf(json_str + offset, max_size - offset, "]");

    return json_str;
}

char *color_array_to_json(int *array, int size){
    // Calculate the maximum size needed for the JSON string
    int max_size = size * 5 + 3; // Assuming each number takes at most 5 characters and additional 3 characters for brackets and commas
    char *json_str = malloc(max_size * sizeof(char));
    if (!json_str) {
        fprintf(stderr, "Memory allocation failed.\n");
        return NULL;
    }


    int offset = 0;
    offset += snprintf(json_str + offset, max_size - offset, "[");

    int i = 0;
    for (; i < size; i++) {
        int colorCode = 0;
        if(array[i] == 25) colorCode = 1;
        else if (array[i] == 3385) colorCode = 2;
        else if (array[i] == 16825) colorCode = 3;

        offset += snprintf(json_str + offset, max_size - offset, "%d", colorCode);
        if (i < size - 1) {
            offset += snprintf(json_str + offset, max_size - offset, ",");
        }
    }

    offset += snprintf(json_str + offset, max_size - offset, "]");

    return json_str;
}

int main() {
    long lRetVal = -1;
    int onStart = 1;

    BoardInit();

    PinMuxConfig();

    UART_Communication();


    // Enable SysTick
    SysTickInit();

    // Initialize UART Terminal
    InitTerm();

    // Clear UART Terminal
    ClearTerm();

    //
    // Register the interrupt handlers
    //
    MAP_GPIOIntRegister(button.port, GPIOA2IntHandler);

    //
    // Configure rising edge interrupts on SW2 and SW3
    //

    MAP_GPIOIntTypeSet(button.port, button.pin, GPIO_FALLING_EDGE);    // SW2

    unsigned long ulStatus;
    ulStatus = MAP_GPIOIntStatus(button.port, false);
    MAP_GPIOIntClear(button.port, ulStatus);           // clear interrupts on GPIOA2

    // clear global variables
    SW_intcount=0;
    SW_intflag=0;

    //int col = 0;

    // Enable SW2 and SW3 interrupts
    MAP_GPIOIntEnable(button.port, button.pin);

    Message("\t\t****************************************************\n\r");
    Message("\t\t\t\tSystick Example\n\r\n\r");
    Message("\t\t****************************************************\n\r");
    Message("\n\n\n\r");

    prevData = 1;
    currButton = -2;
    prevButton = -1;
    char letter;

    enableSPI();
    MAP_UtilsDelay(8000000);
    Adafruit_Init();
    MAP_UtilsDelay(4000000);

    fillScreen(0x0000);
    fillCircle(65, 64, 4, 5);


    int myTextY = 100;
    int rY = 10;

    //establishes the initial position of the ball to be printed
    int x=64;
    int y=64;
    int prevX = -1;
    int prevY = -1;
    int xPosCursor[4000] = {};
    int yPosCursor[4000] = {};
    int colorCursor[4000] = {};
    int cursorRadius[4000] = {};
    int cursorBuffIndex = 0;
    int currentColor = 25;

    // assigns the x and y offsets to get the acceleration values; stores the data in xtilt and ytilt
    unsigned char xtilt = 0;
    unsigned char ytilt = 0;
    unsigned char xoff=(unsigned char)0x5;
    unsigned char yoff=(unsigned char)0x3;

    setCursor(0, myTextY);

    const int draw_limit = 6;

    unsigned int num_cycles = 0;

    //Draw homescreen
    fillCircle(30, 50, 30, 0xFF00FF);
    fillCircle(55, 45, 30, 0xFF00FF);
    fillCircle(75, 50, 30, 0xFF00FF);
    setCursor(20, 40);
    Outstr("Welcome to");
    setCursor(20, 50);
    Outstr("Cloud Canvas!");
    setCursor(0, myTextY+10);
    Outstr("Press 4 to continue");

    while (1) {
        // stop drawing after 12 loop.
        if(num_cycles > draw_limit) UART_PRINT("Drawing paused!\n");;

        while(SW_intflag == 0)
        {
           if(num_cycles++ < draw_limit && onStart == 0){
           I2C_IF_Open(I2C_MASTER_MODE_FST);
           I2C_IF_Write(0x18,&xoff,1,0);
           I2C_IF_Read(0x18,&xtilt,1);
           I2C_IF_Write(0x18,&yoff,1,0);
           I2C_IF_Read(0x18,&ytilt,1);
           I2C_IF_Close();

           //up
           if (ytilt > 0 && ytilt <= 30) {
               if (y >= 10) {
               y = y - (int)ytilt/7;
               }
           }
           else if (ytilt > 30 && ytilt <= 65) { // up
               if (y >= 10) {
               y = y - (int)ytilt/5;
               }
           }

           // down
           else if (ytilt > 220 && ytilt <= 247) {
               if (y <= 118) {
                   y = y + (255 - (int)ytilt)/7;
               }
           }
           else if (ytilt > 190 && ytilt <= 220) {
               if (y <= 118) {
               y = y + (255 - (int)ytilt)/5;
               }
           }

           // left
           if (xtilt > 0 && xtilt <= 30) {
               if (x <= 118) { // Instead of checking x >= 10
                   x = x + (int)xtilt/7; // Instead of x - (int)xtilt/7
               }
           }
           else if (xtilt > 30 && xtilt <= 65) { // left
               if (x <= 118) { // Instead of checking x >= 10
                   x = x + (int)xtilt/5; // Instead of x - (int)xtilt/5
               }
           }

           // right
           else if (xtilt > 220 && xtilt <= 247) {
               if (x >= 10) { // Instead of checking x <= 118
                   x = x - (255 - (int)xtilt)/7; // Instead of x + (255 - (int)xtilt)/7
               }
           }
           else if (xtilt > 190 && xtilt <= 220) { // right
               if (x >= 10) { // Instead of checking x <= 118
                   x = x - (255 - (int)xtilt)/5; // Instead of x + (255 - (int)xtilt)/5
               }
           }

           //draws the circle at the updated position
           fillCircle(x, y, 4, currentColor);
           if(x != prevX || y != prevY){
               prevX = x;
               prevY = y;
               xPosCursor[cursorBuffIndex] = x;
               yPosCursor[cursorBuffIndex] = y;
               colorCursor[cursorBuffIndex] = currentColor;
               cursorRadius[cursorBuffIndex] = 4;
               cursorBuffIndex++;
           }

           MAP_UtilsDelay(3*optionalDelay);
           printf("x: %d, y: %d \n", x, y);

            if(uart_intflag) {
                   uart_intflag = 0;
                   setCursor(0, rY);
//                   fillRect(0, 0, 128, 64, 0);
                   int i = 0;
                   for(; i < receivedCharCount; i++) printLetterOnOled(receivedCharBuffer[i]);
                   receivedCharCount = 0;
                   receivedCharBuffer[receivedCharCount] = '\0';
               }
        }
        }

        if(SW_intflag){
            //DisplayButtonPressed(data);
            prevData = data;
            SW_intflag = 0;

            letter = firstLetter(prevData);
            buffer[++buffIndex] = letter;

              printf("Button pressed: %c\n", letter);

              if(letter == 'a'){
                  http_get(&xPosCursor, &yPosCursor, &cursorRadius, &colorCursor, &cursorBuffIndex);
                  fillScreen(0x0000);
                  int hello = 0;
                  for(; hello < cursorBuffIndex; hello++){
                      fillCircle(xPosCursor[hello], yPosCursor[hello], cursorRadius[hello], colorCursor[hello]);
                      MAP_UtilsDelay(optionalDelay); // CHANGE
                  }
                  fillCircle(0,128, 5, 0);
                  x = xPosCursor[hello - 1];
                  y = yPosCursor[hello - 1];
                  currentColor = colorCursor[hello - 1];


              }

              if(letter == 'd'){
                  if(currentColor == 25) currentColor = 3385;
                  else if (currentColor == 3385) currentColor = 16825;
                  else currentColor = 25;

                  printf("current color: %d\n", currentColor);
                  fillCircle(x, y, 4, currentColor);
              }

              // starting drawing again
              if(letter == 'g'){
                  UART_PRINT("Drawing resumed!\n");
                  num_cycles = 0;
              }

              if (onStart == 1 && letter == 'g') {
                    onStart = 0;
                    fillScreen(0x0000);
                    currentColor = 25;
                    x = 64;
                    y = 64;
                    fillCircle(x, y, 4, currentColor); //redraw circle
               }

              if(letter == 'j'){
                  setCursor(6, 120);
                  Outstr("Erasing Drawing.");
                  cursorBuffIndex = 0;
                  fillScreen(0x0000);
                  currentColor = 25;
                  x = 64;
                  y = 64;
                  fillCircle(x, y, 4, currentColor);

              }

             if(letter == '+'){ // DO: CHANGE BACK TO '-'
                 Report("Now trying to send stuff\n\r");
                 //int i = 0;
                 setCursor(6, 0);
                 Outstr("Sending Now... ");
//
                 // getting the json representation of the C array
                 char* xPos = array_to_json(xPosCursor, cursorBuffIndex);
                 char* yPos = array_to_json(yPosCursor, cursorBuffIndex);
                 char* color = color_array_to_json(colorCursor, cursorBuffIndex);
                 char* radius = array_to_json(cursorRadius, cursorBuffIndex);


                 //buffer[buffIndex] = '\0';
                 http_post(lRetVal, xPos, yPos, color, radius, cursorBuffIndex);

                 free(xPos);
                 free(yPos);
                 free(color);
                 free(radius);

                 printf("\n");
                 setCursor(6, 120);
                 Outstr("Sent!");

                 //    http_get(&xPosCursor, &yPosCursor, &cursorRadius, &colorCursor, &cursorBuffIndex);
                 fillScreen(0x0000);
                 int hello = 0;
                 for(; hello < cursorBuffIndex; hello++){
                   fillCircle(xPosCursor[hello], yPosCursor[hello], cursorRadius[hello], colorCursor[hello]);
                   MAP_UtilsDelay(optionalDelay);
                 }

//                 MAP_UARTCharPut(UARTA1_BASE, '-');
             }

        }

    }

}

#define DATA1_TEMPLATE "{\"state\": {\r\n\"desired\" : {\r\n\"update\":\"%s sent a new drawing!\", \"xpos\" : \"%s\", \"ypos\" : \"%s\", \"color\" : \"%s\", \"radius\":\"1\",\"numpoints\":\"%d\"\r\n}}}\r\n\r\n"

static int http_post(int ignore, char* xPos, char* yPos, char* color, char* radius, int numPoints){
    //Connect the CC3200 to the local access point
   int iTLSSockID = connectToAccessPoint();
   //Set time so that encryption can be used
   iTLSSockID = set_time();
   if(iTLSSockID < 0) {
       UART_PRINT("Unable to set time in the device");
       LOOP_FOREVER();
   }
   //Connect to the website with TLS encryption
   iTLSSockID = tls_connect();
   if(iTLSSockID < 0) {
       ERR_PRINT(iTLSSockID);
   }

//    http_get(iTLSSockID);

    char acSendBuff[8000];
    char acRecvbuff[8000];
    char cCLLength[200];
    char body[7200];
    char* pcBufHeaders;
    int lRetVal = 0;

//    sprintf(body, DATA1_TEMPLATE, "Divya", xPos, yPos, color, numPoints);
    sprintf(body, DATA1_TEMPLATE, "Adityaa", xPos, yPos, radius, color, numPoints);

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(body);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

//#define DATA1 "{\"state\": {\r\n\"desired\" : {\r\n\"var\" : \"Vanakam Chennai!\"\r\n}}}\r\n\r\n"

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

      strcpy(pcBufHeaders, body);
      pcBufHeaders += strlen(body);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
//        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r", lRetVal);
        //sl_Close(iSSLSockID);
//        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }
    sl_Stop(SL_STOP_TIMEOUT);
    return 0;
}

void parseJson(char* result, int* xPosArray, int* yPosArray, int* radiusArray, int* colorArray, int* size){
    // {"welcome":"aws-iot","var":"","xpos":"[64,66,68,71,73]","ypos":"[64,64,64,65,65]","color":"[5,5,5,5,5]","numpoints":"5"}
    char * xpos = strstr(result, "xpos") + 7;
    char * ypos = strstr(result, "ypos") + 7;
    //char * radius = strstr(result, "radius") + 9;
    char * color = strstr(result, "color") + 8;
    char * numpoints = strstr(result, "numpoints") + 12;
    *size = (int) strtol(numpoints, &numpoints, 10);
    int i = 0;
    int colorCode = 0;
    for(; i < *size; i++){
        xPosArray[i] = strtol(xpos, &xpos, 10);
        xpos++; // skip ,

        yPosArray[i] = strtol(ypos, &ypos, 10);
        ypos++; // skip ,

        radiusArray[i] = 4;//strtol(radius, &radius, 10);
        // radius++; // skip ,

        colorCode = strtol(color, &color, 10);
        if(colorCode == 1) colorArray[i] = 25;
        else if (colorCode == 2) colorArray[i] = 3385;
        else if (colorCode == 3) colorArray[i] = 16825;
        else colorArray[i] = 25;

        color++; // skip ,
    }

}


static int http_get(int* xPosArray, int* yPosArray, int* radiusArray, int* colorArray, int* size){
    int iTLSSockID = connectToAccessPoint();
    //Set time so that encryption can be used
    iTLSSockID = set_time();
    if(iTLSSockID < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }
    //Connect to the website with TLS encryption
    iTLSSockID = tls_connect();
    if(iTLSSockID < 0) {
        ERR_PRINT(iTLSSockID);
    }


    char acSendBuff[8000];
    char acRecvbuff[8000];
    char cCLLength[200];
//    char body[3000];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, GETHEADER);
    pcBufHeaders += strlen(GETHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(DATA1);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, DATA1);
    pcBufHeaders += strlen(DATA1);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
//        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
//        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");

        char *result = strstr(acRecvbuff, "{\"welcome\"");
        char *ptr = strstr(acRecvbuff, ",\"reported\":");
        int num_char = ptr - result;
        result[num_char] = '\0';
        printf("%s\n", result);

        parseJson(result, xPosArray, yPosArray, radiusArray, colorArray, size);


    }

    sl_Stop(SL_STOP_TIMEOUT);
    return 0;
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


