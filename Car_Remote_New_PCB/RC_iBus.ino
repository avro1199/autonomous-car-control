static inline uint16_t ibusChecksum(const uint8_t *buf)
{
    uint16_t sum = 0;
    for (int i = 0; i < 30; i++)
        sum += buf[i];
    return 0xFFFF - sum;
}

static bool readIbusFrame(uint16_t ch[14])
{
    while (IBusSerial.available() > 0)
    {
        uint8_t b = (uint8_t)IBusSerial.read();

        // Typical servo frame header: 0x20 0x40
        if (idx == 0)
        {
            if (b != 0x20)
                continue;
            frame[idx++] = b;
            continue;
        }

        frame[idx++] = b;

        if (idx == 2)
        {
            if (frame[1] != 0x40)
            { // not a servo frame
                idx = 0;
                continue;
            }
        }

        if (idx >= IBUS_FRAME_LEN)
        {
            idx = 0;

            uint16_t rxChk = (uint16_t)frame[30] | ((uint16_t)frame[31] << 8);
            uint16_t calcChk = ibusChecksum(frame);
            if (rxChk != calcChk)
                return false;

            for (int i = 0; i < 14; i++)
            {
                int off = 2 + i * 2;
                ch[i] = (uint16_t)frame[off] | ((uint16_t)frame[off + 1] << 8);
            }
            return true;
        }
    }
    return false;
}