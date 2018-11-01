
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

static bool
asduReceivedHandler (void* parameter, int address, CS101_ASDU asdu)     // to request message from slave
{
    printf("SLAVE %i: RECVD ASDU type: %s(%i) elements: %i\n",
            address,
            TypeID_toString(CS101_ASDU_getTypeID(asdu)),
            CS101_ASDU_getTypeID(asdu),
            CS101_ASDU_getNumberOfElements(asdu));

    if (CS101_ASDU_getTypeID(asdu) == M_ME_TE_1) {

        printf("  measured scaled values with CP56Time2a timestamp (M_ME_TE_1):\n");    //  //printing the message based on the data type

        int i;

        for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {

            MeasuredValueScaledWithCP56Time2a io =
                    (MeasuredValueScaledWithCP56Time2a) CS101_ASDU_getElement(asdu, i);

            if (io != NULL) {

                printf("    IOA: %i value: %i\n",
                        InformationObject_getObjectAddress((InformationObject) io),
                        MeasuredValueScaled_getValue((MeasuredValueScaled) io)
                );

                MeasuredValueScaledWithCP56Time2a_destroy(io);
            }
            else {
                printf("     invalid object!\n");
            }
        }
    }
    else if (CS101_ASDU_getTypeID(asdu) == M_SP_NA_1) {
        printf("  single point information (M_SP_NA_1):\n");

        int i;

        for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {

            SinglePointInformation io =
                    (SinglePointInformation) CS101_ASDU_getElement(asdu, i);


            if (io != NULL) {

                printf("    IOA: %i value: %i\n",
                        InformationObject_getObjectAddress((InformationObject) io),
                        SinglePointInformation_getValue((SinglePointInformation) io)
                );

                SinglePointInformation_destroy(io);
            }
            else {
                printf("     invalid object!\n");
            }
        }
    }
    else if (CS101_ASDU_getTypeID(asdu) == M_EP_TD_1) {
        printf("   event of protection equipment (M_EP_TD_1):\n");

        int i;

        for (i = 0; i < CS101_ASDU_getNumberOfElements(asdu); i++) {

            EventOfProtectionEquipmentWithCP56Time2a epe = (EventOfProtectionEquipmentWithCP56Time2a)
                    CS101_ASDU_getElement(asdu, i);

            if (epe != NULL) {

                SingleEvent singleEvent = EventOfProtectionEquipmentWithCP56Time2a_getEvent(epe);

                printf("    IOA: %i state: %i  QDQ: %i\n",
                                   InformationObject_getObjectAddress((InformationObject) epe),
                                   SingleEvent_getEventState(singleEvent),
                                   SingleEvent_getQDP(singleEvent)
                           );

                EventOfProtectionEquipmentWithCP56Time2a_destroy(epe);
            }
            else {
                printf("     invalid object!\n");
            }
        }
    }

    return true;
}

static void
linkLayerStateChanged(void* parameter, int address, LinkLayerState state)   //// to change the state of the Link layer....
{
{
    printf("Link layer state changed for slave %i: ", address);

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
main(int argc, char** argv)    //main or driver function
{
    signal(SIGINT, sigint_handler);     // indicates interruption

    const char* serialPort = "/dev/ttyUSB0";   // defining serial port

    if (argc > 1)
        serialPort = argv[1];

    SerialPort port = SerialPort_create(serialPort, 9600, 8, 'E', 1);    //configuring the serial port to use

    CS101_Master master = CS101_Master_create(port, NULL, NULL, IEC60870_LINK_LAYER_UNBALANCED);   //creates a new unbalanced master instance using the serial port defined

    /* Setting the callback handler for generic ASDUs */
    CS101_Master_setASDUReceivedHandler(master, asduReceivedHandler, NULL);     //function provides a callback handler for received ASDUs.

    /* set callback handler for link layer state changes */
    CS101_Master_setLinkLayerStateChanged(master, linkLayerStateChanged, NULL);  //function provides a callback handler to change link layer state

    CS101_Master_addSlave(master, 1);
                                                //will create a new slave specific state machine to handle all communication with the slave
    CS101_Master_addSlave(master, 2);

    SerialPort_open(port);   //open the port

    running = true;

    int cycleCounter = 0;

    while (running) {

        CS101_Master_pollSingleSlave(master, 1);
        CS101_Master_run(master);

        CS101_Master_pollSingleSlave(master, 2);
        CS101_Master_run(master);

        if (cycleCounter == 10) {

            /* Send a general interrogation to a specific slave */
            CS101_Master_useSlaveAddress(master, 1);
                                                                                                                 // sends general interrogation to specific slave using address
            CS101_Master_sendInterrogationCommand(master, CS101_COT_ACTIVATION, 1, IEC60870_QOI_STATION);
        }

        if (cycleCounter == 30) {

            /* Send a read request */
            CS101_Master_useSlaveAddress(master, 1);
                                                                    //send read request to slave
            CS101_Master_sendReadCommand(master, 1, 102);
        }

        if (cycleCounter == 50) {

            printf("Send control command C_SC_NA_1\n");

            InformationObject sc = (InformationObject)
                    SingleCommand_create(NULL, 5000, true, false, 0);    //  to control binary data (switch…​)


            CS101_Master_useSlaveAddress(master, 1);
            CS101_Master_sendProcessCommand(master, CS101_COT_ACTIVATION, 1, sc);   //Before sending any command or other request to a specific slave the slave address has to be set with the CS101_Master_useSlaveAddress function

            InformationObject_destroy(sc);
        }

        if (cycleCounter == 80) {
            /* Send clock synchronization command */
            struct sCP56Time2a newTime;

            CP56Time2a_createFromMsTimestamp(&newTime, Hal_getTimeInMs());   ////master sends a C_CS_NA_1 ACT message to slave containing the current valid time information

            printf("Send time sync command\n");
            CS101_Master_useSlaveAddress(master, 1);
            CS101_Master_sendClockSyncCommand(master, 1, &newTime);
        }

        Thread_sleep(100);

        cycleCounter++;
    }

    CS101_Master_destroy(master);

    SerialPort_close(port);
    SerialPort_destroy(port);
}
