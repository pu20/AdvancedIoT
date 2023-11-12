#include "radio.h"
#include <cstdint>
#include "mbed.h"
#include <cstdio>

void rx_data();
void accept_input();

Timer t;
Timeout t2;

mbed::Serial pc(USBTX, USBRX);  // tx, rx


#if defined(SX128x_H)
    #define BW_KHZ              200
    #define SPREADING_FACTOR    7
    #define CF_HZ               2487000000
    #define TX_DBM              6
#else
    #if defined(SX126x_H)
        #define TX_DBM              (Radio::chipType == CHIP_TYPE_SX1262 ? 20 : 14)
    #else
        #define TX_DBM              20
    #endif
    #define BW_KHZ              125
    #define SPREADING_FACTOR    7
    #define CF_HZ               868300000
#endif
static uint8_t setSF=7;
static uint8_t i=0;
static uint8_t new_SF_setting = 0;
static uint8_t cycle_number = 0;
int output_value;
float time_per_cycle;
DigitalOut myled(LED1);

/**********************************************************************/

void txDoneCB()
{
}

void rxDoneCB(uint8_t size, float rssi, float snr, uint8_t crc_err)
{
    
    unsigned j;
    printf("rssi:%.1fdBm  snr:%.1fdB  crc_error:%d  \t", rssi, snr, crc_err);

    myled.write(!myled.read()); // toggle LED
    int change_SF_message[100];
    for (j = 0; j < size; j++) {
        printf("%02x ", Radio::radio.rx_buf[j]);
    }
    printf("\r\n");
}

void reset_board(){
    HAL_NVIC_SystemReset();
}

void accept_input()
{
    bool accept_input = true;
    int index = 0;
    t2.attach(&reset_board, 30.0);
    while(accept_input == true)
    {
        // pc.printf("Initiate Rx Next Cycle? Enter 'Y' to proceed: ");
        int ip_rec = 0;
        pc.printf("Set_SF:");
        char buf[200];
        while(!pc.readable())
        {

        }
        char c = pc.getc();
        switch (c) {
            case '7':
                output_value = 7;
                ip_rec = 1;
                break;
            case '8':
                output_value = 8;
                ip_rec = 1;
                break;
            case '9':
                output_value = 9;
                ip_rec = 1;
                break;
            case 'a':
                output_value = 10;
                ip_rec = 1;
                break;
            case 'b':
                output_value = 11;
                ip_rec = 1;
                break;
            case 'c':
                output_value = 12;
                ip_rec = 1;
                break;
            case 'r':
                HAL_NVIC_SystemReset();
            default:
                output_value = -1;  // Indicate an unsupported character
                break;
        }
        accept_input = false;
        pc.printf("\nYou entered: %c\n", c);
        
        if(output_value == -1)
        {
            accept_input = true;
        }
        else {
            setSF = output_value;
        }
        if(ip_rec == 1)
        {
            accept_input = false;
            t2.detach();
            break;
        }

        // --------------------TO ACCEPT A STRING INPUT--------------------
        // if(c == '\n' || c == '\r')
        // {
        //     buf[index] = '\0';
        //     printf("\nReceived String: %s\n", buf);
        //     index = 0;
        //     accept_input = false;
        // }
        // else 
        // {
        //     if(index < sizeof(buf) - 1)
        //     {
        //         buf[index++] = c;
        //     }
        // }  
    }
}


const RadioEvents_t rev = {
    /* Dio0_top_half */     NULL,
    /* TxDone_topHalf */    NULL,
    /* TxDone_botHalf */    txDoneCB,
    /* TxTimeout  */        NULL,
    /* RxDone  */           rxDoneCB,
    /* RxTimeout  */        NULL,
    /* RxError  */          NULL,
    /* FhssChangeChannel  */NULL,
    /* CadDone  */          NULL
};

void rx_data(){
    
    Radio::Init(&rev);
    Radio::Standby();
    Radio::LoRaModemConfig(BW_KHZ, setSF, 1);
    Radio::SetChannel(CF_HZ);
    Radio::LoRaPacketConfig(8, false, true, false);
    Radio::Rx(0);
    // printf("\nIN RX_DATA, SF IS: %d\n", setSF);

    i = 1;
    while(i == 1)
    {
        Radio::service();
        if(i == 0)
            break;
    }
}

int main()
{    
    printf("\r\nreset-rx\r\n");
    // printf("\nEnter SF:");   // To accept the SF value from the user
    // accept_input();  // To accept the SF value from the user
    setSF = 7;
    Radio::Init(&rev);
    Radio::Standby();
    Radio::LoRaModemConfig(BW_KHZ, setSF, 1);
    Radio::SetChannel(CF_HZ);
    Radio::LoRaPacketConfig(8, false, true, false);
    Radio::Rx(0);
    rx_data();

    // while(true)
    // {
    //     Radio::Rx(0);
    //     Radio::service();
    // }
    
}
