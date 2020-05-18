// ****************************************************************************
// *                               INCLUDES                                   *
// ****************************************************************************
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/can.h"

#include "laukan_can.h"
#include "OBDII.h"
#include "OBDIICommunication.h"

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
static uint32_t gTransmitCANID = 0x7DF;
static uint32_t gResponseCANID = 0x7E8;
static can_message_t gMultiPartQueryMessage;

// ****************************************************************************
// *                     INTERNAL FUNCTIONS PROTOTYPES                        *
// ****************************************************************************

// ****************************************************************************
// *                         FUNCTION DEFINITIONS                             *
// ****************************************************************************
OBDIIRet_e OBDIIInit(void)
{
    OBDIIRet_e ret = OBD2_RET_OK;
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
            Serial.printf("Failed to register OBD2 protocol (%d)!\n", canRet);
        }
    }

    return ret;
}

// Counts # bits set in the argument
// Code from Kernighan
static inline unsigned int _BitsSet(unsigned int word)
{
    unsigned int c; // c accumulates the total bits set in v
    for (c = 0; word; c++)
    {
        word &= word - 1; // clear the least significant bit set
    }
    return c;
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
            if ((rxMessage.data_length_code) == 8 && (rxMessage.identifier == gResponseCANID))
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
                               laukan_canReceive(gProtocolHandle, &rxMessage, RESPONSE_TIMEOUT_MS) == LAUKAN_CAN_RET_OK)
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

OBDIICommandSet OBDIIGetSupportedCommands()
{
    OBDIICommandSet supportedCommands = {0};
    unsigned int numCommands;

    // Mode 1
    OBDIIResponse response = OBDIIPerformQuery(OBDIICommands.mode1SupportedPIDs_1_to_20);

    supportedCommands._mode1SupportedPIDs._1_to_20 = response.bitfieldValue;

    // If PID 0x20 is supported, we can query the next set of PIDs
    if (!(response.bitfieldValue & 0x01))
    {
        goto mode9;
    }

    response = OBDIIPerformQuery(OBDIICommands.mode1SupportedPIDs_21_to_40);

    supportedCommands._mode1SupportedPIDs._21_to_40 = response.bitfieldValue;

    // If PID 0x40 is supported, we can query the next set of PIDs
    if (!(response.bitfieldValue & 0x01))
    {
        goto mode9;
    }

    response = OBDIIPerformQuery(OBDIICommands.mode1SupportedPIDs_41_to_60);

    // Mask out the rest of the PIDs, because they're not yet implemented
    response.bitfieldValue &= 0xFFFC0000;

    supportedCommands._mode1SupportedPIDs._41_to_60 = response.bitfieldValue;

    //// If PID 0x60 is supported, we can query the next set of commands
    //if (!(response.bitfieldValue & 0x01)) {
    //	goto mode9;
    //}

    //response = OBDIIPerformQuery(socket, OBDIICommands.mode1SupportedPIDs_61_to_80);

    //supportedCommands._mode1SupportedPIDs._61_to_80 = response.bitfieldValue;

mode9:
    // Mode 9
    response = OBDIIPerformQuery(OBDIICommands.mode9SupportedPIDs);

    // Mask out the PIDs that are not yet implemented
    response.bitfieldValue &= 0xE0000000;

    supportedCommands._mode9SupportedPIDs = response.bitfieldValue;

    numCommands = _BitsSet(supportedCommands._mode1SupportedPIDs._1_to_20) + _BitsSet(supportedCommands._mode1SupportedPIDs._21_to_40) + _BitsSet(supportedCommands._mode1SupportedPIDs._41_to_60) + _BitsSet(supportedCommands._mode1SupportedPIDs._61_to_80) + _BitsSet(supportedCommands._mode9SupportedPIDs);

    numCommands += 2; // mode 1, pid 0 and mode 9, pid 0

    numCommands++; // mode 3

    OBDIICommand **commands = (OBDIICommand **)malloc(sizeof(OBDIICommand *) * numCommands);
    if (commands != NULL)
    {
        supportedCommands.commands = commands;
        supportedCommands.numCommands = numCommands;

        // Mode 1
        unsigned int pid;
        for (pid = 0; pid < sizeof(OBDIIMode1Commands) / sizeof(OBDIIMode1Commands[0]); ++pid)
        {
            OBDIICommand *command = &OBDIIMode1Commands[pid];
            if (OBDIICommandSetContainsCommand(&supportedCommands, command))
            {
                *commands = command;
                ++commands;
            }
        }

        // Mode 3
        *commands = OBDIICommands.DTCs;
        ++commands;

        // Mode 9
        for (pid = 0; pid < sizeof(OBDIIMode9Commands) / sizeof(OBDIIMode9Commands[0]); ++pid)
        {
            OBDIICommand *command = &OBDIIMode9Commands[pid];
            if (OBDIICommandSetContainsCommand(&supportedCommands, command))
            {
                *commands = command;
                ++commands;
            }
        }
    }

    return supportedCommands;
}
