// ****************************************************************************
// *                               INCLUDES                                   *
// ****************************************************************************
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/can.h"

#include "laukan/laukan_can.h"
#include "obd2.h"
#include "OBDII.h"

// ****************************************************************************
// *                     CONSTANTS AND TYPE DEFINITIONS                       *
// ****************************************************************************
#define NUM_OF_RECEIVE_MESSAGES_TO_BUFFER 10
#define TRANSMIT_TIMEOUT_MS 50
#define RESPONSE_TIMEOUT_MS 100

// ****************************************************************************
// *                                GLOBALS                                   *
// ****************************************************************************
static bool gbInitialized = false;
static laukan_canProtocolHandle gProtocolHandle;
// Use the broadcast address as default
static laukan_canProtocolHandle gTransmitCANID = 0x7DF;
static laukan_canProtocolHandle gResponseCANID = 0x7E8;
static can_message_t gMultiPartQueryMessage;

// ****************************************************************************
// *                     INTERNAL FUNCTIONS PROTOTYPES                        *
// ****************************************************************************

// ****************************************************************************
// *                         FUNCTION DEFINITIONS                             *
// ****************************************************************************
OBD2Ret_e OBD2Init(void)
{
    OBD2Ret_e ret = OBD2_RET_OK;
    if (gbInitialized)
    {
        ret = OBD2_RET_ALREADY_INITIALIZED;
    }

    if (ret == OBD2_RET_OK)
    {
        gMultiPartQueryMessage.identifier = gTransmitCANID;
        gMultiPartQueryMessage.data_length_code = 8;
        gMultiPartQueryMessage.data[0] = 0x30;
        memset(&gMultiPartQueryMessage.data[1], 0, sizeof(gMultiPartQueryMessage) - 1);
        gMultiPartQueryMessage.flags = CAN_MSG_FLAG_NONE;

        const uint32_t CANIDMask = 0x0700;
        const uint32_t CANIDValue = CANIDMask;
        laukan_CANRet_e canRet = laukan_canRegisterProtocol(&gProtocolHandle,
                                                            NUM_OF_RECEIVE_MESSAGES_TO_BUFFER,
                                                            CANIDMask,
                                                            CANIDValue);
        if (canRet != LAUKAN_CAN_RET_OK)
        {
            Serial.printf("Failed to register OBD2 protocol (%d)!\n");
        }
    }

    return ret;
}

OBDIIResponse OBDIIPerformQuery(OBDIICommand *command)
{
    OBDIIResponse response = {0};
    response.command = command;
    bool bStatusOK = true;

    if (gbInitialized)
    {
        can_message_t queryMessage;
        queryMessage.identifier = gTransmitCANID;
        queryMessage.flags = CAN_MSG_FLAG_NONE;
        queryMessage.data_length_code = 8;

        uint8_t commandLength = sizeof(command->payload);
        queryMessage.data[0] = commandLength;
        memcpy(&queryMessage.data[1], command->payload, commandLength);
        memset(&queryMessage.data[1 + commandLength], 0, 8 - commandLength - 1);

        if (laukan_canSend(&queryMessage, TRANSMIT_TIMEOUT_MS) == LAUKAN_CAN_RET_OK)
        {
            bStatusOK = false;
        }
    }
    else
    {
        bStatusOK = false;
    }

    if (bStatusOK)
    {
        can_message_t rxMessage;
        bool multiPart = false;
        uint8_t payloadLen = 0;

        if (laukan_canReceive(gProtocolHandle, &rxMessage, RESPONSE_TIMEOUT_MS) == LAUKAN_CAN_RET_OK)
        {
            // We expect all frames to contain 8 bytes
            if (rxMessage.data_length_code == 8)
            {
                uint8_t index = 0;

                if (rxMessage.data[index] == 0x10)
                {
                    // Multipart frame
                    multiPart = true;
                    index++;
                }
                payloadLen = rxMessage.data[index++];
                if (!multiPart)
                {
                    response = OBDIIDecodeResponseForCommand(command, &rxMessage.data[index], payloadLen);
                }
                else
                {
                    uint8_t payloadIndex = 0;
                    uint8_t payload[payloadLen];

                    const uint8_t dataLenInFirstFrame = rxMessage.data_length_code - index;
                    memcpy(&payload[payloadIndex], &rxMessage.data[index], dataLenInFirstFrame);
                    payloadIndex += dataLenInFirstFrame;
                    if (laukan_canSend(&gMultiPartQueryMessage, TRANSMIT_TIMEOUT_MS) == LAUKAN_CAN_RET_OK)
                    {
                        bStatusOK = false;
                    }
                    else
                    {
                        uint32_t frameCounter = 1;
                        while ((payloadIndex < payloadLen) &&
                               (gProtocolHandle, &rxMessage, RESPONSE_TIMEOUT_MS) == LAUKAN_CAN_RET_OK)
                        {
                            index = 0;
                            if ((rxMessage.data_length_code == 8) &&
                                (rxMessage.data[index++] == (0x20 + frameCounter)))
                            {
                                uint32_t copyLen = rxMessage.data_length_code - index;
                                if (payloadIndex + copyLen > payloadLen)
                                {
                                    // Last frame with extra bytes in the end
                                    copyLen = payloadLen - payloadIndex;
                                }
                                memcpy(&payload[payloadIndex], &rxMessage.data[index], copyLen);
                                payloadIndex += copyLen;
                            }
                            else
                            {
                                // We expect all frames to contain 8 bytes
                                bStatusOK = false;
                                break;
                            }

                            frameCounter++;
                        }

                        if (bStatusOK)
                        {
                            response = OBDIIDecodeResponseForCommand(command, payload, payloadLen);
                        }
                    }
                }
            }
        }
    }

    return response;
}
