#include "radio.h"
#include <cstdint>
#include <cstdio>
#include "mbed.h"

Timer t;
Timeout t1;
mbed::Serial pc(USBTX, USBRX);

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
static uint8_t x = 1;
static uint8_t set_SF=7;
static uint8_t cycle_number = 0;
static uint8_t tx_power = 14;
static uint8_t configuration = 0;
int output_value = 0;
void accept_input();
/**********************************************************************/
EventQueue queue(4 * EVENTS_EVENT_SIZE);

void tx_test()
{
    static uint8_t seq = 0;
    
    Radio::radio.tx_buf[0] = x;  /* set payload */
    Radio::radio.tx_buf[1] = x;
    Radio::radio.tx_buf[2] = configuration;
    Radio::radio.tx_buf[3] = configuration;
    Radio::Send(4, 0, 0, 0);   /* begin transmission */
    // printf("sent\r\n");
    x++;
    if(x==101)
        x=1;
/*    {
        mbed_stats_cpu_t stats;
        mbed_stats_cpu_get(&stats);
        printf("canDeep:%u ", sleep_manager_can_deep_sleep());
        printf("Uptime: %llu ", stats.uptime / 1000);
        printf("Sleep time: %llu ", stats.sleep_time / 1000);
        printf("Deep Sleep: %llu\r\n", stats.deep_sleep_time / 1000);
    }*/
}

void txDoneCB()
{
    int time_interval=500;
    // set_SF = output_value;
    while ((set_SF <= 12) && (set_SF >=7)) {
        int i = 1;
        Radio::LoRaModemConfig(BW_KHZ, set_SF, 1);
        if(i<=100)
            tx_power = 14;
        // if((i > 100) && (i <= 200))  // To check at different Tx powers
            // tx_power = 20;
        Radio::set_tx_dbm(tx_power);
        // ThisThread::sleep_for(2*10000);   // Sleep for 20 sec

        if(set_SF==12)
            time_interval = 1000;
        else
            time_interval = 500;

        /*
        configuration is for Depth-Distance configuration value
        There are 8 different combinations of depths and distances, hence 8
        */
        while(configuration <= 8 && i <= 100)
        {
            if(i == 1)
            {
                tx_power = 14;
                configuration++;
                Radio::set_tx_dbm(tx_power);
                ThisThread::sleep_for(30*1000);
            }
            // if(i == 101) // To send at increased Tx Power
            // {
            //     tx_power = 20;
            //     Radio::set_tx_dbm(tx_power);
            //     configuration++;
            //     ThisThread::sleep_for(10000);
            // }
            // if(i%10 == 1)
            if (configuration == 9) {
                printf("CycleCompleted\n");
                break;
            }  
            printf("got-tx-done:%d, %d, %d, %d, %d\r\n", i, set_SF, configuration, time_interval, tx_power);
            tx_test();
            ThisThread::sleep_for(time_interval);
            i++;
        }
        if(configuration == 9)
        {
            configuration = 0;
            printf("CycleCompleted\n");
            HAL_NVIC_SystemReset(); 
        }
        t.stop();
        printf("SF: %d, Configuration: %d, Time Taken: %f\n", set_SF, configuration, t.read());
        t.reset();
        t.start();  
        /* 
        Check time taken for each configuration to transmit 100 frames, considering the added delays in the code
        Based on these values, one needs to physically change the Depth/Distance in a specific order as mentioned in the report
        */
    }
}

void reset_board(){
    HAL_NVIC_SystemReset();
}

void accept_input()
{
    bool accept_input = true;
    int index = 0;
    
    while(accept_input == true)
    {   
        char buf[200];
        int ip_rec = 0;
        // pc.printf("Y/N?");
        pc.printf("Set_SF:");
        t1.attach(&reset_board, 20.0);
        while(!pc.readable())
        {

        }
        // ThisThread::sleep_for(1000);
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
                HAL_NVIC_SystemReset();
                break;
        }
        set_SF = output_value;
        pc.printf("\nYou entered: %c\n", c);
        if(ip_rec == 1)
        {
            accept_input = false;
            t1.detach();
            break;
        }
        // if (output_value == 0) {
        //     HAL_NVIC_SystemReset();
        // }
        // else {
        //     // ThisThread::sleep_for(5000);
        //     HAL_NVIC_SystemReset();
        // }
        // pc.printf("\nYou entered: %c\n", c);
        // if(c == 'y' || c == 'Y')
        // {
        //     // while (1) {
        //     //     char newline = pc.getc();
        //     //     if (newline == '\n' || newline == '\r') {
        //     //         break;
        //     //     }
        //     // }
        //     accept_input = false;
        // }

        // --------------------TO ACCEPT A STRING INPUT--------------------
        // if(c == '\n' || c == '\r')
        // {
        //     buf[index] = '\0';
        //     printf("\nEntered SF: %s\n", buf);
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

void rxDoneCB(uint8_t size, float rssi, float snr)
{
}


void radio_irq_callback()
{
    queue.call(Radio::service);
}


const RadioEvents_t rev = {
    /* DioPin_top_half */   radio_irq_callback,
    /* TxDone_topHalf */    NULL,
    /* TxDone_botHalf */    txDoneCB,
    /* TxTimeout  */        NULL,
    /* RxDone  */           rxDoneCB,
    /* RxTimeout  */        NULL,
    /* RxError  */          NULL,
    /* FhssChangeChannel  */NULL,
    /* CadDone  */          NULL
};

int main()
{
    printf("\r\nreset-tx\n");
    // printf("Initiate Tx? Enter 'Y' to proceed: \n");
    // printf("Enter SF: \n");
    // accept_input();
    t.start();
    // set_SF = output_value;   // To set SF by accepting input from user
    set_SF = 7; // Vary from 7 to 12
    Radio::Init(&rev);

    Radio::Standby();
    Radio::LoRaModemConfig(BW_KHZ, set_SF, 1);
    Radio::SetChannel(CF_HZ);
    Radio::SetFixedPayloadLength(4);
    Radio::set_tx_dbm(tx_power);

               // preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(8, false, true, false);
    ThisThread::sleep_for(270*1000);
    // A total delay of 5 minutes before the transmission starts
    // This is done so that meanwhile the waterproof boxes can be sealed and the set-up is ready to be used
    txDoneCB();
}
