/*
 * master_example.c
 */


#include "iec60870_master.h"
#include "iec60870_common.h"
#include "hal_time.h"
#include "hal_thread.h"
#include "hal_serial.h"
#include "cs101_master.h"

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

static bool running = false;

void
sigint_handler(int signalId)
{
    running = false;
}

/* Callback handler to log sent or received messages (optional) */
static void
rawMessageHandler (void* parameter, uint8_t* msg, int msgSize, bool sent)   //log of sent or received messages
{
    if (sent)
        printf("SEND: ");
    else
        printf("RCVD: ");

    int i;
    for (i = 0; i < msgSize; i++) {
        printf("%02x ", msg[i]);
    }

    printf("\n");
}

static bool
asduReceivedHandler (void* parameter, int address, CS101_ASDU asdu)  // to request message from slave
{
    printf("RECVD ASDU type: %s(%i) elements: %i\n",
            TypeID_toString(CS101_ASDU_getTypeID(asdu)),
            CS101_ASDU_getTypeID(asdu),
            CS101_ASDU_getNumberOfElements(asdu));

    if (CS101_ASDU_getTypeID(asdu) == M_ME_TE_1) {                            //printing the message based on the data type

        printf("  measured scaled values with CP56Time2a timestamp:\n");

        int i;

        for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {

            MeasuredValueScaledWithCP56Time2a io =
                    (MeasuredValueScaledWithCP56Time2a) CS101_ASDU_getElement(asdu, i);

            printf("    IOA: %i value: %i\n",
                    InformationObject_getObjectAddress((InformationObject) io),
                    MeasuredValueScaled_getValue((MeasuredValueScaled) io)
            );

            MeasuredValueScaledWithCP56Time2a_destroy(io);
        }
    }
    else if (CS101_ASDU_getTypeID(asdu) == M_SP_NA_1) {                      //printing the message based on the data type
        printf("  single point information:\n");

        int i;

        for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {

            SinglePointInformation io =
                    (SinglePointInformation) CS101_ASDU_getElement(asdu, i);

            printf("    IOA: %i value: %i\n",
                    InformationObject_getObjectAddress((InformationObject) io),
                    SinglePointInformation_getValue((SinglePointInformation) io)
            );

            SinglePointInformation_destroy(io);
        }
    }

    return true;
}

static void
linkLayerStateChanged(void* parameter, int address, LinkLayerState state)      // to change the state of the Link layer....
{
    printf("Link layer state: ");

    switch (state) {
    case LL_STATE_IDLE:
        printf("IDLE\n");
        break;
    case LL_STATE_ERROR:
        printf("ERROR\n");
        break;
    case LL_STATE_BUSY:
        printf("BUSY\n");
        break;
    case LL_STATE_AVAILABLE:
        printf("AVAILABLE\n");
        break;
    }
}

int
main(int argc, char** argv)                     //main or driver function
{
    signal(SIGINT, sigint_handler);             // indicates interruption

    const char* serialPort = "/dev/ttyUSB0";    // defining serial port

    if (argc > 1)
        serialPort = argv[1];

    SerialPort port = SerialPort_create(serialPort, 9600, 8, 'E', 1);     //configuring the serial port to use

    CS101_Master master = CS101_Master_create(port, NULL, NULL, IEC60870_LINK_LAYER_BALANCED);   //creates a new balanced master instance using the serial port defined


    CS101_Master_setOwnAddress(master, 3);

    /* Set the address of the slave (optional for balanced master */
    CS101_Master_useSlaveAddress(master, 3);      //sets the slave address    ____ it is done only once here__ cuz only one slave can exists

    /* set handler for received ASDUs (application layer data) */
    CS101_Master_setASDUReceivedHandler(master, asduReceivedHandler, NULL);     //provides a callback handler to receive ASDUs

    /* modify some of the default parameters */
    LinkLayerParameters llParams = CS101_Master_getLinkLayerParameters(master);   //this is an optional step
    llParams->useSingleCharACK = false;

    /* set handler for link layer state changes */
    CS101_Master_setLinkLayerStateChanged(master, linkLayerStateChanged, NULL);

    /* uncomment to log messages */
    //CS101_Master_setRawMessageHandler(master, rawMessageHandler, NULL);

    SerialPort_open(port);   //open the port

    running = true;

    int cycleCounter = 0;

    while (running) {

        CS101_Master_run(master);

        if (cycleCounter == 10)
            CS101_Master_sendInterrogationCommand(master, CS101_COT_ACTIVATION, 1, IEC60870_QOI_STATION);  //request a group of data items from a slave with a single request

        if (cycleCounter == 50) {

            InformationObject sc = (InformationObject)
                    SingleCommand_create(NULL, 5000, true, false, 0);    //  to control binary data (switch…​)

            printf("Send control command C_SC_NA_1\n");
            CS101_Master_sendProcessCommand(master, CS101_COT_ACTIVATION, 1, sc);     //To send a command for the direct operate command procedure

            InformationObject_destroy(sc);
        }

        if (cycleCounter == 80) {
            /* Send clock synchronization command */
            struct sCP56Time2a newTime;

            CP56Time2a_createFromMsTimestamp(&newTime, Hal_getTimeInMs());     //master sends a C_CS_NA_1 ACT message to slave containing the current valid time information

            printf("Send time sync command\n");
            CS101_Master_sendClockSyncCommand(master, 1, &newTime);
        }

        Thread_sleep(1);

        cycleCounter++;
    }

    CS101_Master_destroy(master);

    SerialPort_close(port);
    SerialPort_destroy(port);
}

