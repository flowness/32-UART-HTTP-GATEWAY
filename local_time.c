/*
 * Copyright (c) 2016, Texas Instruments Incorporated
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

//*****************************************************************************
// includes
//*****************************************************************************
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>


/* TI-DRIVERS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
//#include <ti/drivers/UART.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/net/utils/clock_sync.h>
#include <ti/drivers/net/wifi/slnetifwifi.h>
#include <ti/net/http/httpclient.h>

#include "uart_term.h"
#include "pthread.h"

//*****************************************************************************
// defines
//*****************************************************************************
#define LOCALTIME_APPLICATION_NAME                      "Local Time"
#define LOCALTIME_APPLICATION_VERSION                   "1.0.0"
/*
#define LOCALTIME_SSID_NAME                             "225_732a6c"                // AP SSID
#define LOCALTIME_SECURITY_TYPE                         SL_WLAN_SEC_TYPE_WPA_WPA2   // Security type could be SL_WLAN_SEC_TYPE_WPA_WPA2
#define LOCALTIME_SECURITY_KEY                          "2a6c1234"                  // Password of the secured AP
*/
/*
#define LOCALTIME_SSID_NAME                             "Paradox-rnd_2.4"                // AP SSID
#define LOCALTIME_SECURITY_TYPE                         SL_WLAN_SEC_TYPE_WPA_WPA2   // Security type could be SL_WLAN_SEC_TYPE_WPA_WPA2
#define LOCALTIME_SECURITY_KEY                          "P@r@d0xx"                  // Password of the secured AP
*/

#define LOCALTIME_SSID_NAME                             "BEZEQINT-2599"                // AP SSID
#define LOCALTIME_SECURITY_TYPE                         SL_WLAN_SEC_TYPE_WPA_WPA2   // Security type could be SL_WLAN_SEC_TYPE_WPA_WPA2
#define LOCALTIME_SECURITY_KEY                          "9692217671"                  // Password of the secured AP


//#define HOSTNAME "https://yg8rvhiiq0.execute-api.eu-west-1.amazonaws.com"
//#define REQUEST_URI "/poc/measurement"
#define USER_AGENT            "HTTPClient (ARM; TI-RTOS)"
#define HTTP_MIN_RECV         (256)

//#define HOSTNAME "https://httpbin.org"
//#define REQUEST_URI "/post"

//#define ABS_URI "https://httpbin.org/post"
#define ABS_URI "https://yg8rvhiiq0.execute-api.eu-west-1.amazonaws.com/poc/measurement"

#define LOCALTIME_SLNET_IF_WIFI_PRIO                    (5)

#define LOCALTIME_SPAWN_STACK_SIZE                      (4096)
#define LOCALTIME_STOP_TIMEOUT                          (200)
#define LOCALTIME_IP_ACQUIRED_WAIT_SEC                  (6)
#define LOCALTIME_SPAWN_TASK_PRIORITY                   (9)
#define LOCALTIME_CLR_STATUS_BIT_ALL(status_variable)   (status_variable = 0)
#define LOCALTIME_SET_STATUS_BIT(status_variable, bit)  (status_variable |= (1<<(bit)))
#define LOCALTIME_CLR_STATUS_BIT(status_variable, bit)  (status_variable &= ~(1<<(bit)))
#define LOCALTIME_GET_STATUS_BIT(status_variable, bit)  (0 != (status_variable & (1<<(bit))))

#define LOCALTIME_IS_CONNECTED(status_variable)         LOCALTIME_GET_STATUS_BIT(status_variable,STATUS_BIT_CONNECTION)
#define LOCALTIME_IS_IP_ACQUIRED(status_variable)       LOCALTIME_GET_STATUS_BIT(status_variable,STATUS_BIT_IP_ACQUIRED)

#define LOCALTIME_ASSERT_ON_ERROR(error_code)\
{\
     if(error_code < 0) \
     {\
         UART_PRINT("error %d\r\n",error_code);\
         return error_code;\
     }\
}

//*****************************************************************************
// typedefs
//*****************************************************************************
// Status bits - These are used to set/reset the corresponding bits in
// given variable

typedef enum _LocalTime_Status_e_
{
    STATUS_BIT_NWP_INIT = 0,        // If this bit is set: Network Processor is
                                    // powered up
    STATUS_BIT_CONNECTION,          // If this bit is set: the device is connected to
                                    // the AP or client is connected to device (AP)
    STATUS_BIT_IP_LEASED,           // If this bit is set: the device has leased IP to
                                    // any connected client
    STATUS_BIT_IP_ACQUIRED,         // If this bit is set: the device has acquired an IP
    STATUS_BIT_SMARTCONFIG_START,   // If this bit is set: the SmartConfiguration
                                    // process is started from SmartConfig app
    STATUS_BIT_P2P_DEV_FOUND,       // If this bit is set: the device (P2P mode)
                                    // found any p2p-device in scan
    STATUS_BIT_P2P_REQ_RECEIVED,    // If this bit is set: the device (P2P mode)
                                    // found any p2p-negotiation request
    STATUS_BIT_CONNECTION_FAILED,   // If this bit is set: the device(P2P mode)
                                    // connection to client(or reverse way) is failed
    STATUS_BIT_PING_DONE,           // If this bit is set: the device has completed
                                    // the ping operation
    STATUS_BIT_IPV6L_ACQUIRED,      // If this bit is set: the device has acquired an IPv6 address
    STATUS_BIT_IPV6G_ACQUIRED,      // If this bit is set: the device has acquired an IPv6 address
    STATUS_BIT_AUTHENTICATION_FAILED,
    STATUS_BIT_RESET_REQUIRED,
}LocalTime_Status_e;

typedef enum
{
    LocalTime_GetTime,
    LocalTime_UpdateTime,
    LocalTime_SetTimezone,
    LocalTime_GetTimezone,
    LocalTime_SwitchAP,
    LocalTime_SwitchStation
}LocalTime_UseCases;

/* Control block definition */
typedef struct _LocalTime_ControlBlock_t_
{
    uint32_t                status;    //SimpleLink Status
    LocalTime_UseCases      useCase;
}LocalTime_ControlBlock;


//****************************************************************************
// GLOBAL VARIABLES
//****************************************************************************
LocalTime_ControlBlock   LocalTime_CB;


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************

//*****************************************************************************
// SimpleLink Callback Functions
//*****************************************************************************

void SimpleLinkNetAppRequestMemFreeEventHandler (uint8_t *buffer)
{
}

void SimpleLinkNetAppRequestEventHandler (SlNetAppRequest_t *pNetAppRequest, SlNetAppResponse_t *pNetAppResponse)
{
}

//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
    {
        return;
    }

     switch(pWlanEvent->Id)
     {
        case SL_WLAN_EVENT_CONNECT:
            LOCALTIME_SET_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_CONNECTION);

            UART_PRINT("STA Connected to AP: %s , BSSID: %x:%x:%x:%x:%x:%x\n\r", pWlanEvent->Data.Connect.SsidName, pWlanEvent->Data.Connect.Bssid[0], pWlanEvent->Data.Connect.Bssid[1],
                       pWlanEvent->Data.Connect.Bssid[2], pWlanEvent->Data.Connect.Bssid[3], pWlanEvent->Data.Connect.Bssid[4], pWlanEvent->Data.Connect.Bssid[5]);
            break;

        case SL_WLAN_EVENT_DISCONNECT:
            LOCALTIME_CLR_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_CONNECTION);
            LOCALTIME_CLR_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_IP_ACQUIRED);

            break;

        case SL_WLAN_EVENT_STA_ADDED:
            /* when device is in AP mode and any client connects to it.       */
            LOCALTIME_SET_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_CONNECTION);
            break;

        case SL_WLAN_EVENT_STA_REMOVED:
            /* when device is in AP mode and any client disconnects from it.  */
            LOCALTIME_CLR_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_CONNECTION);
            LOCALTIME_CLR_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_IP_LEASED);
            break;

        default:
            break;
    }
}

//*****************************************************************************
//
//! \brief The Function Handles the Fatal errors
//!
//! \param[in]  slFatalErrorEvent - Pointer to Fatal Error Event info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent)
{
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
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
    {
        return;
    }

    switch(pNetAppEvent->Id)
    {
        case SL_NETAPP_EVENT_IPV4_ACQUIRED:
            LOCALTIME_SET_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_IP_ACQUIRED);
            UART_PRINT("IPv4 acquired: IP = %d.%d.%d.%d\n\r",
                (uint8_t)SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,3),
                (uint8_t)SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,2),
                (uint8_t)SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,1),
                (uint8_t)SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,0));
            break;

        case SL_NETAPP_EVENT_DHCPV4_LEASED:
            LOCALTIME_SET_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_IP_LEASED);
            break;

        case SL_NETAPP_EVENT_DHCPV4_RELEASED:
            LOCALTIME_CLR_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_IP_LEASED);
            break;

        default:
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
void SimpleLinkHttpServerEventHandler(SlNetAppHttpServerEvent_t *pHttpEvent,
                                    SlNetAppHttpServerResponse_t *pHttpResponse)
{
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
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
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
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
}


//*****************************************************************************
//
//! LocalTime_initDevice
//!
//! @param  none
//!
//! @return 0 on success else error code
//!
//! @brief  Initialise the device to default state without deleting the stored
//!            profiles.
//
//*****************************************************************************

int32_t LocalTime_initDevice(void)
{
    uint8_t val = 1;
    uint8_t configOpt = 0;
    uint8_t power = 0;

    int32_t retVal = -1;
    int32_t mode = -1;


    mode = sl_Start(0, 0, 0);
    LOCALTIME_ASSERT_ON_ERROR(mode);
    sl_WlanSetMode(ROLE_STA);
    sl_Stop(0);
    mode = sl_Start(NULL,NULL,NULL);
    LOCALTIME_ASSERT_ON_ERROR(mode);

    if (mode == 0)
    {
        UART_PRINT("set device to STA mode: %d \n\r",mode);
    }
    else
    {
        UART_PRINT("set device to STA mode error: %d\n\r",mode);
    }


    // Check if the device is not in station-mode
    if (ROLE_STA != mode)
    {
        if (ROLE_AP == mode)
        {
            // If the device is in AP mode, we need to wait for this
            // event before doing anything
            while(!LOCALTIME_IS_IP_ACQUIRED(LocalTime_CB.status))
            {
            }
        }
    }
    else
    {
        // Set connection policy to Auto + Auto Provisisoning
        // (Device's default connection policy)
        retVal = sl_WlanPolicySet(SL_WLAN_POLICY_CONNECTION,SL_WLAN_CONNECTION_POLICY(1, 0, 0, 1), NULL, 0);
        LOCALTIME_ASSERT_ON_ERROR(retVal);

        // Device in station-mode. Disconnect previous connection if any
        // The function returns 0 if 'Disconnected done', negative number if already disconnected
        // Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
        //
        retVal = sl_WlanDisconnect();
        if(0 == retVal)
        {
            // Wait
            while(LOCALTIME_IS_CONNECTED(LocalTime_CB.status))
            {
            }
        }

        // Enable DHCP client
        retVal = sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE,1,1,&val);
        LOCALTIME_ASSERT_ON_ERROR(retVal);

        // Disable scan
        configOpt = SL_WLAN_SCAN_POLICY(0, 0);
        retVal = sl_WlanPolicySet(SL_WLAN_POLICY_SCAN , configOpt, NULL, 0);
        LOCALTIME_ASSERT_ON_ERROR(retVal);

        // Set Tx power level for station mode
        // Number between 0-15, as dB offset from max power - 0 will set maximum power
        power = 0;
        retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, SL_WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1,(uint8_t *)&power);
        LOCALTIME_ASSERT_ON_ERROR(retVal);

        // Set PM policy to normal
        retVal = sl_WlanPolicySet(SL_WLAN_POLICY_PM , SL_WLAN_NORMAL_POLICY, NULL, 0);
        LOCALTIME_ASSERT_ON_ERROR(retVal);

        retVal = sl_Stop(LOCALTIME_STOP_TIMEOUT);
        LOCALTIME_ASSERT_ON_ERROR(retVal);

        LOCALTIME_CLR_STATUS_BIT_ALL(LocalTime_CB.status);

        //
        // Assumption is that the device is configured in station mode already
        // and it is in its default state
        //
        mode = sl_Start(0,0,0);

        if (mode < 0 || mode != ROLE_STA)
        {
            UART_PRINT("Failed to start the device \n\r");
            LOCALTIME_ASSERT_ON_ERROR(mode);
        }
    }
    return mode;
}

//*****************************************************************************
//
//! \brief Display Application Banner
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
int32_t LocalTime_displayBanner(void)
{
    int32_t     status = -1;
    uint8_t     macAddress[SL_MAC_ADDR_LEN];
    uint16_t    macAddressLen = SL_MAC_ADDR_LEN;
    uint16_t    configSize = 0;
    uint8_t     configOpt = SL_DEVICE_GENERAL_VERSION;
    SlDeviceVersion_t ver = {0};

    configSize = sizeof(SlDeviceVersion_t);
    status = sl_Start(0, 0, 0);
    if (status < 0)
    {
        return -1;
    }

    /* Print device version info. */
    status = sl_DeviceGet(SL_DEVICE_GENERAL, &configOpt, &configSize, (uint8_t*)(&ver));
    if (status < 0)
    {
        return -1;
    }

    /* Print device Mac address */
    status = sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen, &macAddress[0]);
    if (status < 0)
    {
        return -1;
    }

    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\t    %s Example Ver: %s\n\r",LOCALTIME_APPLICATION_NAME, LOCALTIME_APPLICATION_VERSION);
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("\t CHIP: 0x%x",ver.ChipId);
    UART_PRINT("\n\r");
    UART_PRINT("\t MAC:  %d.%d.%d.%d",ver.FwVersion[0],ver.FwVersion[1],ver.FwVersion[2],ver.FwVersion[3]);
    UART_PRINT("\n\r");
    UART_PRINT("\t PHY:  %d.%d.%d.%d",ver.PhyVersion[0],ver.PhyVersion[1],ver.PhyVersion[2],ver.PhyVersion[3]);
    UART_PRINT("\n\r");
    UART_PRINT("\t NWP:  %d.%d.%d.%d",ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3]);
    UART_PRINT("\n\r");
    UART_PRINT("\t ROM:  %d",ver.RomVersion);
    UART_PRINT("\n\r");
    UART_PRINT("\t HOST: %s", SL_DRIVER_VERSION);
    UART_PRINT("\n\r");
    UART_PRINT("\t MAC address: %02x:%02x:%02x:%02x:%02x:%02x", macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
    UART_PRINT("\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("\t ==============================================\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("\n\r");
    status = sl_Stop(LOCALTIME_STOP_TIMEOUT);
    if (status < 0)
    {
        return -1;
    }

    return status;
}

//*****************************************************************************
//
//! \brief    Get from the user the selected option
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
void LocalTime_setUseCase(void)
{
    char sel[2];

    UART_PRINT("==============================================\n\r");
    UART_PRINT("   Options :\n\r");
    UART_PRINT("   1) Get time.  \n\r");
    UART_PRINT("   2) Update time.  \n\r");
    UART_PRINT("   3) Set time zone.  \n\r");
    UART_PRINT("   4) Get time zone.  \n\r");
    UART_PRINT("   5) Switch to AP mode.  \n\r");
    UART_PRINT("   6) Switch to Station mode.  \n\r");
    UART_PRINT("==============================================\n\r");
    UART_PRINT("Please enter your selection:  ");
    GetCmd(sel,sizeof(sel));
    UART_PRINT("\n\r");
    UART_PRINT("\n\r");
    LocalTime_CB.useCase = (LocalTime_UseCases)(atoi((const char*)sel) - 1);
}


//*****************************************************************************
//
//! \brief    Connect to WLAN AP
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
void LocalTime_connect(void)
{
    SlWlanSecParams_t   secParams = {0};
    char                sel[2];
    uint8_t             i = 0;
    
    UART_PRINT("Please wait...trying to connect to the AP\n\r");
    UART_PRINT("\n\r");
    secParams.Key = (signed char*)LOCALTIME_SECURITY_KEY;
    secParams.KeyLen = strlen(LOCALTIME_SECURITY_KEY);
    secParams.Type = LOCALTIME_SECURITY_TYPE;
    sl_WlanConnect((signed char*)LOCALTIME_SSID_NAME, strlen(LOCALTIME_SSID_NAME), 0, &secParams, 0);
    i = 0;
    while ((!LOCALTIME_IS_IP_ACQUIRED(LocalTime_CB.status)) && (i < LOCALTIME_IP_ACQUIRED_WAIT_SEC))
    {
        sleep(1);
        i++;
    }

    if (!LOCALTIME_IS_IP_ACQUIRED(LocalTime_CB.status))
    {
        UART_PRINT("Could not connect to AP %s\n\r",LOCALTIME_SSID_NAME);
        UART_PRINT("Please press enter to continue or reset the device after configure your network parameters.\n\r");
        GetCmd(sel,sizeof(sel));
    }    
}

/*
void* uartRxThread(void *pvParameters)
{

    //MsgObj msg;
    int len=0;
    char message[512]={0};
    // Print Application name
    HTTPClient_extSecParams httpClientSecParams;

    httpClientSecParams.rootCa = "dst-root-ca-x3.der"; //"dummy-ca-cert.der";
    httpClientSecParams.clientCert = NULL;
    httpClientSecParams.privateKey = NULL;

    HTTPClient_Handle httpClientHandle;
    int16_t statusCode;
    httpClientHandle = HTTPClient_create(&statusCode,0);
    if (statusCode < 0)
    {
        UART_PRINT("httpTask(%d): creation of http client handle failed", statusCode);
    }

    statusCode = HTTPClient_setHeader(httpClientHandle, HTTPClient_HFIELD_REQ_USER_AGENT,USER_AGENT,strlen(USER_AGENT),HTTPClient_HFIELD_PERSISTENT);
    if (statusCode < 0) {
        UART_PRINT("httpTask(%d): setting request header failed", statusCode);
    }

    //msg.id=0;
    while(1)
    {
        UART_PRINT("uartRxThread ready to recive \n\r");

        len=GetString(message,sizeof(message));
        if(len>0)
        {
            UART_PRINT("uartRxThread(%d) recived: %s\n\r",len,message);


            statusCode = HTTPClient_connect(httpClientHandle,HOSTNAME,&httpClientSecParams,0);
            if (statusCode < 0) {
                UART_PRINT("httpTask(%d): connect failed\n\r", statusCode);
            }
            statusCode = HTTPClient_sendRequest(httpClientHandle,HTTP_METHOD_POST,REQUEST_URI,message,strlen(message),0);
            if (statusCode < 0) {
                UART_PRINT("httpTask(%d): send failed\n\r", statusCode);
            }

            if (statusCode != HTTP_SC_OK) {
                UART_PRINT("httpTask(%d): cannot get status\n\r", statusCode);
            }

            UART_PRINT("HTTP Response Status Code: %d\n\r", statusCode);

            bool moreDataFlag = false;

            do {
                statusCode = HTTPClient_readResponseBody(httpClientHandle, message, sizeof(message), &moreDataFlag);
                if (statusCode < 0) {
                    UART_PRINT("httpTask(%d): response body processing failed\n\r", statusCode);
                }
                len += statusCode;
            }while (moreDataFlag);

            UART_PRINT(message);
            UART_PRINT("\n\r");


            //msg.len=len;
            //if(Mailbox_post((Mailbox_Handle)pvParameters, &msg, BIOS_NO_WAIT))
            //{
            //    UART_PRINT("uartRxThread(%d) mailbox success: %d- %s\n\r",msg.len,msg.id,msg.val);
            //}
            //else
            //{
            //    UART_PRINT("uartRxThread(%d) mailbox failed: %d- %s\n\r",msg.len,msg.id,msg.val);
            //}
            //msg.id++;
            //

        }
    }

    statusCode = HTTPClient_disconnect(httpClientHandle);
    if (statusCode < 0)
    {
        UART_PRINT("httpTask(%d): disconnect failed", statusCode);
    }

    HTTPClient_destroy(httpClientHandle);

    return (0);
}
*/
//*****************************************************************************
//
//! \brief Task Created by main function.
//!
//! \param pvParameters is a general void pointer (not used here).
//!
//! \return none
//
//*****************************************************************************
void mainThread(void *pvParameters)
{
    int32_t             status = 0;
    pthread_t           spawn_thread = (pthread_t)NULL;
    pthread_attr_t      pAttrs_spawn;
    int32_t             mode;
    struct sched_param  priParam;
    struct tm           netTime;

    int len=0;
    char message[512]={0};
    /* Print Application name */
    HTTPClient_extSecParams httpClientSecParams;
/*
    pthread_t           uart_rx_thread = (pthread_t)NULL;
    //pthread_t           cloud_tx_thread = (pthread_t)NULL;
    pthread_attr_t      pAttrs;
    int                 detachState;
    uint8_t             ssid[33];
    uint16_t            len = 33;
    uint16_t            config_opt = SL_WLAN_AP_OPT_SSID;
    char                tzsel[5];
*/

    // Initialize SlNetSock layer with CC3x20 interface
    SlNetIf_init(0);
    SlNetIf_add(SLNETIF_ID_1, "CC3220", (const SlNetIf_Config_t *)&SlNetIfConfigWifi, LOCALTIME_SLNET_IF_WIFI_PRIO);

    SlNetSock_init(0);
    SlNetUtil_init(0);

    GPIO_init();
    SPI_init();

    /* Configure the UART */
    InitTerm();
    
    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
    
    /* clear SimpleLink Status */
    LocalTime_CB.status = 0;
    
    /* Start the SimpleLink Host */
    pthread_attr_init(&pAttrs_spawn);
    priParam.sched_priority = LOCALTIME_SPAWN_TASK_PRIORITY;
    status = pthread_attr_setschedparam(&pAttrs_spawn, &priParam);
    status |= pthread_attr_setstacksize(&pAttrs_spawn, LOCALTIME_SPAWN_STACK_SIZE);

    status = pthread_create(&spawn_thread, &pAttrs_spawn, sl_Task, NULL);

    if(status != 0)
    {
        UART_PRINT("could not create simplelink task\n\r");
        return;
    }


    httpClientSecParams.rootCa = "dst-root-ca-x3.der"; //"dummy-ca-cert.der";
    httpClientSecParams.clientCert = NULL;
    httpClientSecParams.privateKey = NULL;

    HTTPClient_Handle httpClientHandle;
    int16_t statusCode;
    httpClientHandle = HTTPClient_create(&statusCode,0);
    if (statusCode < 0)
    {
        UART_PRINT("httpTask(%d): creation of http client handle failed", statusCode);
    }

    statusCode = HTTPClient_setHeader(httpClientHandle, HTTPClient_HFIELD_REQ_USER_AGENT,USER_AGENT,strlen(USER_AGENT),HTTPClient_HFIELD_PERSISTENT);
    if (statusCode < 0) {
        UART_PRINT("httpTask(%d): setting request header failed", statusCode);
    }
/*
    // Set priority and stack size attributes
    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 8;

    detachState = PTHREAD_CREATE_DETACHED;
    status = pthread_attr_setdetachstate(&pAttrs, detachState);
    if (status != 0) {
        // pthread_attr_setdetachstate() failed
        while (1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    status |= pthread_attr_setstacksize(&pAttrs, LOCALTIME_SPAWN_STACK_SIZE);
    if (status != 0) {
        // pthread_attr_setstacksize() failed
        while (1);
    }

    status = pthread_create(&uart_rx_thread, &pAttrs, uartRxThread, NULL);
    if(status != 0)
    {
        UART_PRINT("could not create uart_rx_thread task\n\r");
        return;
    }

    status = pthread_create(&cloud_tx_thread, &pAttrs, cloudTxThread, NULL);
    if(status != 0)
    {
        UART_PRINT("could not create cloud_tx_thread task\n\r");
        return;
    }

    */

    /* Displays the Application Banner */
    LocalTime_displayBanner();

    /* initialize the device */
    mode = LocalTime_initDevice();
/*
    if (mode == ROLE_STA)
    {
    */
        LocalTime_connect();
/*
    }
    else if (mode == ROLE_AP)
    {
        sl_Memset(ssid,0,33);
        sl_WlanGet(SL_WLAN_CFG_AP_ID, &config_opt , &len, ssid);
        UART_PRINT("AP mode SSID %s\n\r",ssid);
    }
*/

     while (1)
     {
         UART_PRINT("uartRxThread ready to recive \n\r");
         sl_Memset(message,0,512);
         len=GetString(message,sizeof(message));
         //UART_PRINT("uartRxThread(%d) recived: %s\n\r",len,message);

         if(len>0)
         {
             //UART_PRINT("\r\n\r\n\r\n");
             UART_PRINT("uartRxThread(%d) correct string: %s\n\r",len,message);

            /* statusCode = HTTPClient_connect(httpClientHandle,HOSTNAME,&httpClientSecParams,0);
             if (statusCode < 0) {
                 UART_PRINT("httpTask(%d): connect failed\n\r", statusCode);
             }*/

             statusCode = HTTPClient_sendRequest(httpClientHandle,HTTP_METHOD_POST,ABS_URI,message,strlen(message),0);

//             statusCode = HTTPClient_sendRequest(httpClientHandle,HTTP_METHOD_POST,REQUEST_URI,message,strlen(message),0);
             if (statusCode < 0) {
                 UART_PRINT("httpTask(%d): send failed\n\r", statusCode);
             }

             if (statusCode != HTTP_SC_OK) {
                 UART_PRINT("httpTask(%d): cannot get status\n\r", statusCode);
             }

             UART_PRINT("HTTP Response Status Code: %d\n\r", statusCode);

             bool moreDataFlag = false;

             do {
                 statusCode = HTTPClient_readResponseBody(httpClientHandle, message, sizeof(message), &moreDataFlag);
                 if (statusCode < 0) {
                     UART_PRINT("httpTask(%d): response body processing failed\n\r", statusCode);
                 }
                 len += statusCode;
             }while (moreDataFlag);

             UART_PRINT(message);
             //UART_PRINT("\r\n\r\n\r\n");


             /*msg.len=len;
             if(Mailbox_post((Mailbox_Handle)pvParameters, &msg, BIOS_NO_WAIT))
             {
                 UART_PRINT("uartRxThread(%d) mailbox success: %d- %s\n\r",msg.len,msg.id,msg.val);
             }
             else
             {
                 UART_PRINT("uartRxThread(%d) mailbox failed: %d- %s\n\r",msg.len,msg.id,msg.val);
             }
             msg.id++;
             */

         }
     }
     /*
         if ((mode == ROLE_STA) && (!LOCALTIME_IS_CONNECTED(LocalTime_CB.status)))
         {
             LocalTime_connect();
         }
         else
         {
             // Set Desired use case
             LocalTime_setUseCase();
             switch (LocalTime_CB.useCase)
             {
                 case LocalTime_GetTime:
                     status = ClockSync_get(&netTime);
                     if ((status == 0) || (status == CLOCKSYNC_ERROR_INTERVAL))
                     {
                         UART_PRINT("Local time = %s\n\r",asctime(&netTime));
                     }
                     else
                     {
                         UART_PRINT("Error = %d\n\r",status);
                     }

                 break;
                 case LocalTime_UpdateTime:
                     status = ClockSync_update();
                     if (status == 0)
                     {
                         UART_PRINT("Update successful\n\r");
                     }
                     else if (status == CLOCKSYNC_ERROR_INTERVAL)
                     {
                         UART_PRINT("Min time between updates did not elapse\n\r");
                     }
                     else
                     {
                         UART_PRINT("Update failed = %d\n\r",status);
                     }
                 break;

                 case LocalTime_SetTimezone:
                     UART_PRINT("Please enter offset from GMT in minutes:  ");
                     GetCmd(tzsel,sizeof(tzsel));
                     UART_PRINT("\n\r");
                     // update time zone
                     ClockSync_setTimeZone(atoi(tzsel));

                 break;

                 case LocalTime_GetTimezone:
                     UART_PRINT("Timezone = %d.\n\r",ClockSync_getTimeZone());
                 break;

                 case LocalTime_SwitchAP:
                     if (mode == ROLE_AP)
                     {
                         UART_PRINT("Device already in AP mode.\n\r");
                     }
                     else
                     {
                        status = sl_WlanSetMode(ROLE_AP);
                        if (status >= 0)
                        {
                            UART_PRINT("Please reset the device....\n\r");
                            return;
                        }
                     }

                 break;
                 case LocalTime_SwitchStation:
                     if (mode == ROLE_STA)
                     {
                         UART_PRINT("Device already in Station mode.\n\r");
                     }
                     else
                     {
                        status = sl_WlanSetMode(ROLE_STA);
                        if (status >= 0)
                        {
                            UART_PRINT("Please reset the device....\n\r");
                            return;
                        }
                     }
                 break;
             }
         }
    // }

     */
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
