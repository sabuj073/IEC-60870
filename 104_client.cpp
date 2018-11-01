#include "cs104_connection.h"
#include "hal_time.h"
#include "hal_thread.h"

#include <stdio.h>
#include <stdlib.h>

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

/* Connection event handler */
static void
connectionHandler (void* parameter, CS104_Connection connection, CS104_ConnectionEvent event)   //Connection event handler to check the state of the connection
{
    switch (event) {
    case CS104_CONNECTION_OPENED:
        printf("Connection established\n");
        break;
    case CS104_CONNECTION_CLOSED:
        printf("Connection closed\n");
        break;
    case CS104_CONNECTION_STARTDT_CON_RECEIVED:
        printf("Received STARTDT_CON\n");
        break;
    case CS104_CONNECTION_STOPDT_CON_RECEIVED:
        printf("Received STOPDT_CON\n");
        break;
    }
}

/*
 * CS101_ASDUReceivedHandler implementation
 *
 * For CS104 the address parameter has to be ignored
 */
static bool
asduReceivedHandler (void* parameter, int address, CS101_ASDU asdu)   //// to request message from slave
{
    printf("RECVD ASDU type: %s(%i) elements: %i\n",
            TypeID_toString(CS101_ASDU_getTypeID(asdu)),
            CS101_ASDU_getTypeID(asdu),
            CS101_ASDU_getNumberOfElements(asdu));

    if (CS101_ASDU_getTypeID(asdu) == M_ME_TE_1) {

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
    else if (CS101_ASDU_getTypeID(asdu) == M_SP_NA_1) {           //  //printing the message based on the data type
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

int
main(int argc, char** argv)      // main or driver function
{
    const char* ip = "localhost";
    uint16_t port = IEC_60870_5_104_DEFAULT_PORT;

    if (argc > 1)
        ip = argv[1];            //ip is defined from a predefined array

    if (argc > 2)
        port = atoi(argv[2]);       //port is defined from a predefined array

    printf("Connecting to: %s:%i\n", ip, port);
    CS104_Connection con = CS104_Connection_create(ip, port);       //A new connection is simple created using predefined ip and port

    CS104_Connection_setConnectionHandler(con, connectionHandler, NULL);

    CS104_Connection_setASDUReceivedHandler(con, asduReceivedHandler, NULL);

    /* uncomment to log messages */
    //CS104_Connection_setRawMessageHandler(con, rawMessageHandler, NULL);

    if (CS104_Connection_connect(con)) {     //check for connection
        printf("Connected!\n");

        CS104_Connection_sendStartDT(con);

        Thread_sleep(5000);                   //delay for 5 second

        CS104_Connection_sendInterrogationCommand(con, CS101_COT_ACTIVATION, 1, IEC60870_QOI_STATION);    //request a group of data items with a single request

        Thread_sleep(5000);

        InformationObject sc = (InformationObject)
                SingleCommand_create(NULL, 5000, true, false, 0);    //  to control binary data (switch…​)

        printf("Send control command C_SC_NA_1\n");
        CS104_Connection_sendProcessCommandEx(con, CS101_COT_ACTIVATION, 1, sc);   //To send a command for the direct operate command procedure

        InformationObject_destroy(sc);

        /* Send clock synchronization command */
        struct sCP56Time2a newTime;

        CP56Time2a_createFromMsTimestamp(&newTime, Hal_getTimeInMs());

        printf("Send time sync command\n");
        CS104_Connection_sendClockSyncCommand(con, 1, &newTime);   //master sends a C_CS_NA_1 ACT message to slave containing the current valid time information

        Thread_sleep(1000);
    }
    else
        printf("Connect failed!\n");    //if connection not established it is failed

    Thread_sleep(1000);

    CS104_Connection_destroy(con);

    printf("exit\n");
}
