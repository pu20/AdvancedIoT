#include "sx12xx.h"

Callback<void()> SX126x::dio1_topHalf;    // low latency ISR context

void SX126x::dio1isr()
{
    if (dio1_topHalf)
        dio1_topHalf.call();
}

SX126x::SX126x(SPI& _spi, PinName _nss, PinName _busy, PinName _dio1)
    : spi(_spi), nss(_nss), busy(_busy), dio1(_dio1)
{
    uint8_t buf[8];
    IrqFlags_t irqEnable;

    nss = 1;

    dio1.rise(dio1isr);


    irqEnable.word = 0;
    irqEnable.bits.TxDone = 1;
    irqEnable.bits.RxDone = 1;
    irqEnable.bits.Timeout = 1;

    buf[0] = irqEnable.word >> 8;    // enable bits
    buf[1] = irqEnable.word; // enable bits
    buf[2] = irqEnable.word >> 8;     // dio1
    buf[3] = irqEnable.word;  // dio1
    buf[4] = 0; // dio2
    buf[5] = 0; // dio2
    buf[6] = 0; // dio3
    buf[7] = 0; // dio3
    xfer(OPCODE_SET_DIO_IRQ_PARAMS, 8, 0, buf);

}

void SX126x::PrintChipStatus(status_t status)
{
    printf("%02x cmdStatus:", status.octet);
    switch (status.bits.cmdStatus) {
        case 0: printf("Reserved"); break;
        case 1: printf("RFU"); break;
        case 2: printf("dataAvail"); break;
        case 3: printf("cmdTimeout"); break;
        case 4: printf("cmdError"); break;
        case 5: printf("execFail"); break;
        case 6: printf("cmdTxDone"); break;
    }
    printf(" chipMode:");
    switch (status.bits.chipMode) {
        case 0: printf("Unused"); break;
        case 1: printf("RFU"); break;
        case 2: printf("STBY_RC"); break;
        case 3: printf("STBY_XOSC"); break;
        case 4: printf("FS"); break;
        case 5: printf("RX"); break;
        case 6: printf("TX"); break;
    }
    printf("\r\n");
}

void SX126x::service()
{
    IrqFlags_t irqFlags, clearIrqFlags;
    uint8_t buf[4];

    if (busy) {
        return;
    }

    while (dio1) {
        xfer(OPCODE_GET_IRQ_STATUS, 0, 3, buf);
        irqFlags.word = buf[1] << 8;
        irqFlags.word |= buf[2];
        clearIrqFlags.word = 0;
        if (irqFlags.bits.TxDone) {
            chipMode = CHIPMODE_NONE;
            if (chipModeChange)
                chipModeChange.call();  // might change to Rx
            if (txDone)
                txDone.call();
            clearIrqFlags.bits.TxDone = 1;
        }
        if (irqFlags.bits.RxDone) {
            if (rxDone) {
                uint8_t len;
                float snr, rssi;
                int8_t s;
                xfer(OPCODE_GET_RX_BUFFER_STATUS, 0, 3, buf);
                len = buf[1];
                ReadBuffer(len, buf[2]);
                xfer(OPCODE_GET_PACKET_STATUS, 0, 4, buf);
                rssi = -buf[1] / 2.0;   // TODO FSK
                s = buf[2];
                snr = s / 4.0;
                rxDone(len, rssi, snr);
            }
            clearIrqFlags.bits.RxDone = 1;
        }
        if (irqFlags.bits.Timeout) {
            if (chipMode != CHIPMODE_NONE) {
                if (timeout)
                    timeout(chipMode == CHIPMODE_TX);
            }
            chipMode = CHIPMODE_NONE;
            if (chipModeChange)
                chipModeChange.call();
            clearIrqFlags.bits.Timeout = 1;
        }
        if (irqFlags.bits.CadDone) {
            if (cadDone)
                cadDone(irqFlags.bits.CadDetected);

            clearIrqFlags.bits.CadDone = 1;
            clearIrqFlags.bits.CadDetected = irqFlags.bits.CadDetected;
        }
        if (irqFlags.bits.PreambleDetected) {
            clearIrqFlags.bits.PreambleDetected = 1;
            if (preambleDetected)
                preambleDetected();
        }

        if (clearIrqFlags.word != 0) {
            buf[0] = clearIrqFlags.word >> 8;
            buf[1] = (uint8_t)clearIrqFlags.word;
            xfer(OPCODE_CLEAR_IRQ_STATUS, 2, 0, buf);
        }

    } // ...while (dio1)

} // ..service()

void SX126x::xfer(uint8_t opcode, uint8_t wlen, uint8_t rlen, uint8_t* ptr)
{
    const uint8_t* stopPtr;
    const uint8_t* wstop;
    const uint8_t* rstop;
    uint8_t nop = 0;

    if (sleeping) {
        nss = 0;
        while (busy)
            ;
        sleeping = false;
    } else {
        while (busy)
            ;
        nss = 0;
    }

    spi.write(opcode);

    wstop = ptr + wlen;
    rstop = ptr + rlen;
    if (rlen > wlen)
        stopPtr = rstop;
    else
        stopPtr = wstop;

    for (; ptr < stopPtr; ptr++) {
        if (ptr < wstop && ptr < rstop)
            *ptr = spi.write(*ptr);
        else if (ptr < wstop)
            spi.write(*ptr);
        else
            *ptr = spi.write(nop);    // n >= write length: send NOP
    }

    nss = 1;

    if (opcode == OPCODE_SET_SLEEP)
        sleeping = true;
}

void SX126x::start_tx(uint8_t pktLen)
{
    uint8_t buf[8];

    {
        uint8_t i;

        while (busy)
            ;

        nss = 0;
        spi.write(OPCODE_WRITE_BUFFER);
        spi.write(0);   // offset
        i = 0;
        for (i = 0; i < pktLen; i++) {
            spi.write(tx_buf[i]);
        }
        nss = 1;
    }

    buf[0] = 0x40;
    buf[1] = 0x00;
    buf[2] = 0x00;
    xfer(OPCODE_SET_TX, 3, 0, buf);

    chipMode = CHIPMODE_TX;
    if (chipModeChange)
        chipModeChange.call();
}

void SX126x::start_rx(unsigned timeout)
{
    uint8_t buf[8];

    buf[0] = timeout >> 16;
    buf[1] = timeout >> 8;
    buf[2] = timeout;
    xfer(OPCODE_SET_RX, 3, 0, buf);

    chipMode = CHIPMODE_RX;
    if (chipModeChange)
        chipModeChange.call();
}

uint8_t SX126x::setMHz(float MHz)
{
    unsigned frf = MHz * MHZ_TO_FRF;
    uint8_t buf[4];

    buf[0] = frf >> 24;
    buf[1] = frf >> 16;
    buf[2] = frf >> 8;
    buf[3] = frf;
    xfer(OPCODE_SET_RF_FREQUENCY, 4, 0, buf);
    return buf[3];
}

float SX126x::getMHz()
{
    uint32_t frf = readReg(REG_ADDR_RFFREQ, 4);
    return frf / (float)MHZ_TO_FRF;
}

void SX126x::setPacketType(uint8_t type)
{
    xfer(OPCODE_SET_PACKET_TYPE, 1, 0, &type);
}

uint8_t SX126x::getPacketType()
{
    uint8_t buf[2];
    xfer(OPCODE_GET_PACKET_TYPE, 0, 2, buf);
    return buf[1];
}

void SX126x::SetDIO2AsRfSwitchCtrl(uint8_t en)
{
    xfer(OPCODE_SET_DIO2_AS_RFSWITCH, 1, 0, &en);
}

void SX126x::ReadBuffer(uint8_t size, uint8_t offset)
{
    unsigned i;
    while (busy)
        ;

    nss = 0;

    spi.write(OPCODE_READ_BUFFER);
    spi.write(offset);
    spi.write(0);   // NOP
    i = 0;
    for (i = 0; i < size; i++) {
        rx_buf[i] = spi.write(0);
    }

    nss = 1;
}

void SX126x::set_tx_dbm(bool is1262, int8_t dbm)
{
    uint8_t buf[4];
    // use OCP default

    buf[3] = 1;
    if (is1262) {
        buf[0] = 4;
        buf[1] = 7;
        buf[2] = 0;

        if (dbm > 22)
            dbm = 22;
        else if (dbm < -3)
            dbm = -3;
    } else {
        if (dbm == 15)
            buf[0] = 6;
        else
            buf[0] = 4;
        buf[1] = 0;
        buf[2] = 1;

        if (dbm > 14)
            dbm = 14;
        else if (dbm < -3)
            dbm = -3;
    }
    xfer(OPCODE_SET_PA_CONFIG, 4, 0, buf);

    if (is1262 && dbm > 18) {
        /* OCP is set by chip whenever SetPaConfig() is called */
        writeReg(REG_ADDR_OCP, 0x38, 1);
    }

    // SetTxParams
    buf[0] = dbm;
    //if (opt == 0) txco
    buf[1] = SET_RAMP_200U;
    xfer(OPCODE_SET_TX_PARAMS, 2, 0, buf);
}

void SX126x::writeReg(uint16_t addr, uint32_t data, uint8_t len)
{
    uint8_t buf[6];
    uint8_t n;
    buf[0] = addr >> 8;
    buf[1] = (uint8_t)addr;
    for (n = len; n > 0; n--) {
        buf[n+1] = (uint8_t)data;
        data >>= 8;
    }
    xfer(OPCODE_WRITE_REGISTER, 2+len, 2+len, buf);
}

void SX126x::setStandby(stby_t stby)
{
    uint8_t octet = stby;
    xfer(OPCODE_SET_STANDBY, 1, 0, &octet);

    chipMode = CHIPMODE_NONE;
    if (chipModeChange)
        chipModeChange.call();
}

void SX126x::setFS()
{
    xfer(OPCODE_SET_FS, 0, 0, NULL);

    chipMode = CHIPMODE_NONE;
    if (chipModeChange)
        chipModeChange.call();
}

void SX126x::setCAD()
{
    xfer(OPCODE_SET_CAD, 0, 0, NULL);

    chipMode = CHIPMODE_RX;
    if (chipModeChange)
        chipModeChange.call();
}

void SX126x::setSleep(bool warmStart, bool rtcWakeup)
{
    sleepConfig_t sc;

    sc.octet = 0;
    sc.bits.rtcWakeup = rtcWakeup;
    sc.bits.warmStart = warmStart;
    xfer(OPCODE_SET_SLEEP, 1, 0, &sc.octet);

    chipMode = CHIPMODE_NONE;
    if (chipModeChange)
        chipModeChange.call();
}

void SX126x::hw_reset(PinName pin)
{
    DigitalInOut nrst(pin);
    nrst.output();
    nrst = 0;
#if (MBED_MAJOR_VERSION < 6)
    ThisThread::sleep_for(1);
#else
    ThisThread::sleep_for(1ms);
#endif
    nrst = 1;
    nrst.mode(PullUp);
    nrst.input();

    while (busy)
        ;
}

uint32_t SX126x::readReg(uint16_t addr, uint8_t len)
{
    uint32_t ret = 0;
    unsigned i;

    uint8_t buf[7];
    buf[0] = addr >> 8;
    buf[1] = (uint8_t)addr;
    xfer(OPCODE_READ_REGISTER, 2, 3+len, buf);
    for (i = 0; i < len; i++) {
        ret <<= 8;
        ret |= buf[i+3];
    }
    return ret;
}

void SX126x::setBufferBase(uint8_t txAddr, uint8_t rxAddr)
{
    uint8_t buf[2];

    buf[0] = 0; // TX base address
    buf[1] = 0; // RX base address
    xfer(OPCODE_SET_BUFFER_BASE_ADDR, 2, 0, buf);
}
