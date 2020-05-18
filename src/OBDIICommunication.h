#ifndef __OBDII_COMMUNICATION_H
#define __OBDII_COMMUNICATION_H

#include "OBDII.h"

typedef enum
{
	OBD2_RET_OK = 0,
	OBD2_RET_ALREADY_INITIALIZED,
} OBDIIRet_e;

OBDIIRet_e OBDIIInit(void);

/** Query the car for a particular command and return the response.
 *
 * This is the main API that clients will interact with. It writes the command's request payload into the socket
 * and reads the response payload, decoding it as an `OBDIIResponse` object.
 *
 *     OBDIIResponse response = OBDIIPerformQuery(OBDIICommands.engineRPMs);
 *     if (response.success) {
 *         printf("%.2f", response.numericValue);
 *     } else {
 *         // Handle error
 *     }
 *     OBDIIResponseFree(&response);
 *
 * \param s The socket used to communicate with the vehicle
 * \param command The command to query the vehicle for
 *
 * \returns An `OBDIIResponse` object containing the decoded diagnostic data
 */
OBDIIResponse OBDIIPerformQuery(OBDIICommand *command);

/** Queries the car for the commands it supports.
 *
 *     OBDIICommandSet commands = OBDIIGetSupportedCommands();
 *     if (OBDIICommandSetContainsCommand(&commands, OBDIICommands.engineRPMs)) {
 *         // Query the vehicle
 *     }
 *     OBDIICommandSetFree(&commands);
 */
OBDIICommandSet OBDIIGetSupportedCommands();

#endif /* OBDIICommunication.h */
