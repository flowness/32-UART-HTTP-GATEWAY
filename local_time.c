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
#include <stdio.h>

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
#include <ti/drivers/Watchdog.h>
#include <ti/utils/json/json.h>


Watchdog_Handle watchdogHandle;
Json_Handle templateHandle;
Json_Handle templateDebugHandle;

Json_Handle jsonObjHandle;
Json_Handle jsonDebugObjHandle;


#include "uart_term.h"
#include "pthread.h"


  char *templatestr =   "{"
                            "\"SN\":string,"
                            "\"TimeStamp\":uint32,"
                            "\"Count\":uint32,"
                            "\"FlowRaw\":int32,"
                            "\"Flow\":int32,"
                            "\"Volume\":int32,"
                            "\"CRC\":uint32,"
                            "\"Debug\": raw"
/*                            "{"
                                "\"MSPCount\":uint32"
                            "}"
 */                       "}";

  char *templateDebugstr =  "{"
                                "\"MSPCount\":uint32,"
                                "\"CCCount\":uint32,"
                                "\"CCParseError\":uint32,"
                                "\"CCPostError\":raw"
                            "}";

#define MAX_ERROR_ARRAY 10

unsigned long TransmitionCount=0;
unsigned long CCParseError=0;


  int errorArray[MAX_ERROR_ARRAY][2];

//*****************************************************************************
// defines
//*****************************************************************************
#define LOCALTIME_APPLICATION_NAME                      "Local Time"
#define LOCALTIME_APPLICATION_VERSION                   "1.0.2"

#define LOCALTIME_SSID_NAME                             "AquaSafe"                // AP SSID
#define LOCALTIME_SECURITY_TYPE                         SL_WLAN_SEC_TYPE_WPA_WPA2   // Security type could be SL_WLAN_SEC_TYPE_WPA_WPA2
#define LOCALTIME_SECURITY_KEY                          "Paradox1"                  // Password of the secured AP

/*
#define LOCALTIME_SSID_NAME                             "Paradox-rnd_2.4"                // AP SSID
#define LOCALTIME_SECURITY_TYPE                         SL_WLAN_SEC_TYPE_WPA_WPA2   // Security type could be SL_WLAN_SEC_TYPE_WPA_WPA2
#define LOCALTIME_SECURITY_KEY                          "P@r@d0xx"                  // Password of the secured AP

#define LOCALTIME_SSID_NAME                             "BEZEQINT-2599"                // AP SSID
#define LOCALTIME_SECURITY_TYPE                         SL_WLAN_SEC_TYPE_WPA_WPA2   // Security type could be SL_WLAN_SEC_TYPE_WPA_WPA2
#define LOCALTIME_SECURITY_KEY                          "9692217671"                  // Password of the secured AP
*/

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
         UART_PRINT("error(%d) %d\r\n",__LINE__,error_code);\
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
    UART_PRINT("SimpleLinkNetAppRequestEventHandler called\r\n");

    if(pNetAppRequest == NULL)
    {
        return;
    }

    UART_PRINT("SimpleLinkNetAppRequestEventHandler called with ID: %d\r\n",pNetAppRequest->AppId);

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

    UART_PRINT("SimpleLinkWlanEventHandler called\r\n");

    if(pWlanEvent == NULL)
    {
        return;
    }

    UART_PRINT("SimpleLinkWlanEventHandler called with ID: %d\r\n",pWlanEvent->Id);

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
            UART_PRINT("STA Disconnected from AP\r\n");

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
    UART_PRINT("SimpleLinkFatalErrorEventHandler called\r\n");

    if(slFatalErrorEvent == NULL)
    {
        return;
    }
    UART_PRINT("SimpleLinkFatalErrorEventHandler called with ID: %d\r\n",slFatalErrorEvent->Id);

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
    UART_PRINT("SimpleLinkNetAppEventHandler called\r\n");

    if(pNetAppEvent == NULL)
    {
        return;
    }
    UART_PRINT("SimpleLinkNetAppEventHandler called with ID: %d\r\n",pNetAppEvent->Id);

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
            UART_PRINT("IPv4 LEASED\n\r");
            LOCALTIME_SET_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_IP_LEASED);
            break;

        case SL_NETAPP_EVENT_DHCPV4_RELEASED:
            UART_PRINT("IPv4 RELEASED\n\r");
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
    UART_PRINT("SimpleLinkHttpServerEventHandler called\r\n");

    if(pHttpEvent == NULL)
    {
        return;
    }
    UART_PRINT("SimpleLinkHttpServerEventHandler called with ID: %d\r\n",pHttpEvent->Event);

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
    UART_PRINT("SimpleLinkGeneralEventHandler called\r\n");

    if(pDevEvent == NULL)
    {
        return;
    }
    UART_PRINT("SimpleLinkGeneralEventHandler called with ID: %d\r\n",pDevEvent->Id);

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
    UART_PRINT("SimpleLinkSockEventHandler called\r\n");

    if(pSock == NULL)
    {
        return;
    }
    UART_PRINT("SimpleLinkSockEventHandler called with ID: %d\r\n",pSock->Event);

}


int16_t createTemplate(void)
{
    int16_t retVal;

    retVal = Json_createTemplate(&templateHandle, templatestr, strlen(templatestr));

    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't create template \n\r",retVal);
    }
    else
    {
        UART_PRINT("Template object created successfully. \n\r");
    }


    retVal = Json_createTemplate(&templateDebugHandle, templateDebugstr, strlen(templateDebugstr));

    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't create template \n\r",retVal);
    }
    else
    {
        UART_PRINT("Template object created successfully. \n\r");
    }
    return retVal;




}

int16_t createObject(void)
{
    int16_t retVal;

    retVal = Json_createObject(&jsonObjHandle,templateHandle,512);
    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't create json object \n\r", retVal);
    }
    else
    {
        UART_PRINT("Json object created successfully. \n\n\r");
    }

    retVal = Json_createObject(&jsonDebugObjHandle,templateDebugHandle,128);
    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't create json object \n\r", retVal);
    }
    else
    {
        UART_PRINT("Json object created successfully. \n\n\r");
    }
    return retVal;

}

int16_t parse(char* jsonBuffer)
{
    int16_t retVal;

    retVal =
        Json_parse(jsonObjHandle,jsonBuffer, strlen(jsonBuffer));

    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't parse the Json file \n\r", retVal);
    }
    else
    {
        UART_PRINT("Json was parsed successfully \n\r");
    }
    return retVal;
}

char *TimeStamp_k =  "\"TimeStamp\"";
char *Count_k =  "\"Count\"";
char *FlowRaw_k =  "\"FlowRaw\"";
char *Flow_k =  "\"Flow\"";
char *Volume_k =  "\"Volume\"";
char *CRC_k =  "\"CRC\"";
char *Debug_k =  "\"Debug\"";
char *Debug_CCCount_k =  "\"CCCount\"";
char *Debug_CCPostError_k =  "\"CCPostError\"";
char *Debug_CCParseError_k =  "\"CCParseError\"";



char errorArrayString[MAX_ERROR_ARRAY*16] = {0};//"[[0],[0]],[[1],[2]]";

int16_t AdjustTransmitionCount(char* jsonBuffer)
{
    int16_t retVal;
    uint16_t size=128;
    char Debug_v[128]={0};

    retVal = Json_getValue(jsonObjHandle,Debug_k,&Debug_v,&size);
    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't get the Json Debug_k \n\r", retVal);
        return retVal;
    }
    else
    {
        UART_PRINT("got Json Debug successfully\n\r");
        retVal = Json_parse(jsonDebugObjHandle,Debug_v, strlen(Debug_v));
        if(retVal < 0)
        {
            UART_PRINT("Error: %d  , Couldn't parse the Debug Json file \n\r", retVal);
        }
        else
        {
            UART_PRINT("Debug Json was parsed successfully \n\r");
            size=sizeof(TransmitionCount);
            retVal = Json_setValue(jsonDebugObjHandle, Debug_CCCount_k, &TransmitionCount, size);
            if(retVal < 0)
            {
                UART_PRINT("Error: %d  , Couldn't set the Debug Json CCCount\n\r", retVal);
            }
            else
            {
                UART_PRINT("Debug Json CCCount was set successfully with %d\n\r",TransmitionCount);

                retVal = Json_setValue(jsonDebugObjHandle, Debug_CCParseError_k, &CCParseError, size);
                if(retVal < 0)
                {
                    UART_PRINT("Error: %d  , Couldn't set the Debug Json CCCount\n\r", retVal);
                }
                else
                {
                    UART_PRINT("Debug Json CCParseError was set successfully with %d\n\r",CCParseError);
                    char *temperrorArrayString=errorArrayString;
                    temperrorArrayString+=sprintf(temperrorArrayString,"[");
                    int i;
                    for(i=0;i<MAX_ERROR_ARRAY;i++)
                    {

                        if(errorArray[i][0] == 0)
                        {
                            if(i!=0)
                            {
                                temperrorArrayString--;
                                *temperrorArrayString=0;
                            }
                            break;
                        }
                        temperrorArrayString+=sprintf(temperrorArrayString,"[%i,%i],",errorArray[i][0],errorArray[i][1]);
                    }
                    temperrorArrayString+=sprintf(temperrorArrayString,"]");
                    retVal = Json_setValue(jsonDebugObjHandle, Debug_CCPostError_k, errorArrayString, strlen(errorArrayString));
                    if(retVal < 0)
                    {
                        UART_PRINT("Error: %d  , Couldn't set the Debug Json CCPostError\n\r", retVal);
                    }
                    else
                    {
                        UART_PRINT("Debug Json CCPostError was set successfully: %s \n\r",errorArrayString);
                        size=128;
                        retVal = Json_build(jsonDebugObjHandle, Debug_v, &size);
                        if(retVal < 0)
                        {
                            UART_PRINT("Error: %d  , Couldn't build Debug Json\n\r", retVal);
                        }
                        else
                        {
                            UART_PRINT("Debug Json was build successfully \n\r");
                            retVal = Json_setValue(jsonObjHandle, Debug_k, Debug_v, size);
                            if(retVal < 0)
                            {
                                UART_PRINT("Error: %d  , Couldn't set the Debug Json with CCCount\n\r", retVal);
                            }
                            else
                            {
                                UART_PRINT("Debug Json was set successfully in JSON\n\r");
                                size=512;
                                retVal = Json_build(jsonObjHandle, jsonBuffer, &size);
                                if(retVal < 0)
                                {
                                    UART_PRINT("Error: %d  , Couldn't build Debug Json\n\r", retVal);
                                }
                                else
                                {
                                    UART_PRINT("Json was build successfully\n\r");
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return retVal;
}

int16_t CheckCRC()
{
    int16_t retVal;
    uint32_t TimeStamp_v=0;
    uint32_t Count_v=0;
    int32_t FlowRaw_v=0;
    int32_t Flow_v=0;
    int32_t Volume_v=0;

    uint16_t size64=sizeof(TimeStamp_v);
    uint32_t CRC_v=0;

    retVal = Json_getValue(jsonObjHandle,TimeStamp_k,&TimeStamp_v,&size64);
    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't get the Json TimeStamp_k \n\r", retVal);
        return retVal;
    }
    else
    {
        //UART_PRINT("got Json TimeStamp successfully %lu \n\n\r",TimeStamp_v);
    }

    retVal = Json_getValue(jsonObjHandle,CRC_k,&CRC_v,&size64);
    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't get the Json CRC_k \n\r", retVal);
        return retVal;
    }
    else
    {
        //UART_PRINT("got Json CRC successfully %lu \n\n\r",CRC_v);
    }

    retVal = Json_getValue(jsonObjHandle,Count_k,&Count_v,&size64);
    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't get the Json Count_k \n\r", retVal);
        return retVal;
    }
    else
    {
        //UART_PRINT("got Json count successfully %d \n\n\r",Count_v);
    }

    retVal = Json_getValue(jsonObjHandle,FlowRaw_k,&FlowRaw_v,&size64);
    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't get the Json FlowRaw_k \n\r", retVal);
        return retVal;
    }
    else
    {
        //UART_PRINT("got Json FlowRaw successfully %d \n\n\r",FlowRaw_v);
    }

    retVal = Json_getValue(jsonObjHandle,Flow_k,&Flow_v,&size64);
    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't get the Json Flow_k \n\r", retVal);
        return retVal;
    }
    else
    {
       // UART_PRINT("got Json Flow successfully %d \n\n\r",Flow_v);
    }

    retVal = Json_getValue(jsonObjHandle,Volume_k,&Volume_v,&size64);
    if(retVal < 0)
    {
        UART_PRINT("Error: %d  , Couldn't get the Json Volume_k \n\r", retVal);
        return retVal;
    }
    else
    {
        //UART_PRINT("got Json volume successfully %d \n\n\r",Volume_v);
    }

    uint32_t CRC_calc=TimeStamp_v^Count_v^(int32_t)FlowRaw_v^(int32_t)Flow_v^(int32_t)Volume_v;
    if(CRC_v==CRC_calc)
    {
        UART_PRINT("got CRC successfully \n\r");
        return 0;
    }
    else
    {
        UART_PRINT("Error CRC isn't correct expected: %lu but got %lu \n\r", CRC_v,CRC_calc);
        return -1;
    }


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
    if (mode!=0)
    {
        sl_WlanSetMode(ROLE_STA);
        sl_Stop(0);
        mode = sl_Start(NULL,NULL,NULL);
        LOCALTIME_ASSERT_ON_ERROR(mode);
    }

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
    if (LOCALTIME_IS_IP_ACQUIRED(LocalTime_CB.status)&&LOCALTIME_IS_CONNECTED(LocalTime_CB.status)) return;


    UART_PRINT("Please wait...trying to connect to the AP\n\r");
    UART_PRINT("\n\r");
    secParams.Key = (signed char*)LOCALTIME_SECURITY_KEY;
    secParams.KeyLen = strlen(LOCALTIME_SECURITY_KEY);
    secParams.Type = LOCALTIME_SECURITY_TYPE;
    sl_WlanConnect((signed char*)LOCALTIME_SSID_NAME, strlen(LOCALTIME_SSID_NAME), 0, &secParams, 0);
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
    struct sched_param  priParam;
    Watchdog_Params params;

    int len=0;
    char message[512]={0};
    char Debugmessage[1024]={0};

    /* Print Application name */
/*
 *     HTTPClient_extSecParams httpClientSecParams;
 *     int32_t             mode;
 *     struct tm           netTime;
 *
*/

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


  Watchdog_init();

    Watchdog_Params_init(&params);
    params.resetMode=Watchdog_RESET_ON;

    //remove comment block to enable watchdog

    watchdogHandle = Watchdog_open(Board_WATCHDOG0, &params);
    uint32_t tickValue = Watchdog_convertMsToTicks(watchdogHandle, 20000);
    Watchdog_setReload(watchdogHandle, tickValue);


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

/*
    httpClientSecParams.rootCa = "dst-root-ca-x3.der"; //"dummy-ca-cert.der";
    httpClientSecParams.clientCert = NULL;
    httpClientSecParams.privateKey = NULL;
*/
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

    /* Displays the Application Banner */
    LocalTime_displayBanner();

    /* initialize the device */
    LocalTime_initDevice();


     createTemplate();
     createObject();
     int i;
     for(i=0;i<MAX_ERROR_ARRAY;i++)
     {
         errorArray[i][0]=0;
         errorArray[i][1]=0;
     }

     while (1)
     {
         LocalTime_connect();

         UART_PRINT("\n\ruartRxThread ready to recive \n\r");
         sl_Memset(message,0,512);
         len=GetString(message,sizeof(message));

         if(len>0)
         {
             UART_PRINT("uartRxThread(%d) string recived \n\r",len);

             int16_t retVal=parse(message);
             if (retVal>=0)
             {

                 retVal=CheckCRC();
                 if (retVal>=0)
                 {
                     AdjustTransmitionCount(message);

                     statusCode = HTTPClient_sendRequest(httpClientHandle,HTTP_METHOD_POST,ABS_URI,message,strlen(message),0);

                     UART_PRINT("HTTP Response Status Code: %d\n\r\n\r", statusCode);
                     Watchdog_clear(watchdogHandle);
                     //UART_PRINT(message);
                     bool moreDataFlag = false;

                     HTTPClient_readResponseBody(httpClientHandle, Debugmessage, 1024, &moreDataFlag);



                     if(statusCode == HTTP_SC_OK)
                     {
                         TransmitionCount++;
                         for(i=0;i<MAX_ERROR_ARRAY;i++)
                         {
                             if(errorArray[i][0]==0) break;

                             errorArray[i][0]=0;
                             errorArray[i][1]=0;
                         }
                     }
                     else
                     {
                         for(i=0;i<MAX_ERROR_ARRAY;i++)
                         {
                             if(errorArray[i][0] == statusCode)
                             {
                                 errorArray[i][1]++;
                                 break;
                             }
                             else if(errorArray[i][0] == 0)
                             {
                                 errorArray[i][0]=statusCode;
                                 errorArray[i][1]++;
                                 break;
                             }
                         }
                     }
                 }
                 else
                 {
                     UART_PRINT("CRC error with message: %s",message);
                 }
             }
             else
             {
                 UART_PRINT("Parse error with message: %s",message);
                 CCParseError++;
             }
         }
     }
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
