#include <SoftwareSerial.h>

SoftwareSerial serialRig2(3, 4); // RX, TX - Second rig

bool wasLocked = false;

int lockEnablePin = 2; // Turns on lock, otherwise idle
int invertModePin = 9; // Pin 9, pull high for inverting transponder

long lastFreqRig1 = 0;  // Saving VFO data
long lastFreqRig2 = 0;

unsigned char *converted;

typedef enum{
    RIG_1_PORT = 0,
    RIG_2_PORT
} RigPort;

void setup()
{
    pinMode(lockEnablePin, INPUT); // Setup lock input
    pinMode(invertModePin, INPUT); // Mode select input
    
    Serial.begin(9600); // Setup serial ports
    serialRig2.begin(9600);

    wasLocked = false;
}

void loop()
{
    if (digitalRead(lockEnablePin)) // Are we locking?
    {
        if(wasLocked) // Check if we were locked on the last loop, if not get a good scan of VFOs
        {
            bool invertMode = digitalRead(invertModePin); // Are we on inverting transponder?
            long rig1CurrentFreq = getFreqMode(RIG_1_PORT); // The control rig
            long offset = lastFreqRig1 - rig1CurrentFreq; // How much have we moved from the lock position
            long tempFreq = 0; // Where to move the other rig to
            
            if(invertMode) // Are we in inverting transpoder mode?
            {
                tempFreq = lastFreqRig2 + offset; // Figure out where to go and set it
                if(getFreqMode(RIG_2_PORT) != tempFreq)
                {
                    setFreq(tempFreq, RIG_2_PORT);
                }
            }
            else // Non inverting, move the same direction
            {
                tempFreq = lastFreqRig2 - offset;
                if(getFreqMode(RIG_2_PORT) != tempFreq)
                {
                    setFreq(tempFreq, RIG_2_PORT);
                }
            }
            
        }
        else // Wasn't locked last loop, grab current VFOs
        {
            lastFreqRig1 = getFreqMode(RIG_1_PORT);
            lastFreqRig2 = getFreqMode(RIG_2_PORT);
        }

        wasLocked = true; // Prevent rescan
    }
    else // Not locked
    {
        wasLocked = false;
    }
}

// Code below borrowed from FT857 lib, modified for two ports

unsigned long getFreqMode(RigPort rigPort)
{
    byte rigGetFreq[5] = {0x00,0x00,0x00,0x00,0x03};
    byte chars[4];
    long timeout = millis();
    long elapsed = 0;
    unsigned long freq = 0;
    
    switch (rigPort)
    {
        case RIG_1_PORT:
        {
            Serial.flush(); // clear the RX buffer which helps prevent
            // any crap data from making it through
            
            sendCmd(rigGetFreq, 5, rigPort);
            
            while (Serial.available() < 5 && elapsed < 2000)
            {
                elapsed = millis() - timeout;
            }
            
            for (int j = 0; j < 4; j++) {
                chars[j] = Serial.read();
            }
            byte  mode = Serial.read();
            freq = from_bcd_be(chars, 8);
        }
            break;
            
        case RIG_2_PORT:
        {
            serialRig2.flush(); // clear the RX buffer which helps prevent
            // any crap data from making it through
            
            sendCmd(rigGetFreq, 5, rigPort);
            
            while (serialRig2.available() < 5 && elapsed < 2000)
            {
                elapsed = millis() - timeout;
            }
            
            for (int j = 0; j < 4; j++)
            {
                chars[j] = serialRig2.read();
            }
            byte  mode = serialRig2.read();
            freq = from_bcd_be(chars, 8);
        }
            break;

        default:
            break;
    }

    return freq;
}

void sendCmd(byte cmd[], byte len, RigPort rigPort)
{
    for (byte i=0; i<len; i++)
    {
        switch (rigPort)
        {
            case RIG_1_PORT:
                Serial.write(cmd[i]);
                break;
                
            case RIG_2_PORT:
                serialRig2.write(cmd[i]);
                break;
                
            default:
                break;
        }
    }
}

void setFreq(long freq, RigPort rigPort)
{
    byte rigFreq[5] = {0x00,0x00,0x00,0x00,0x01};
    
    unsigned char tempWord[4];
    converted = to_bcd_be(tempWord, freq, 8);
    
    for (byte i=0; i<4; i++)
    {
        rigFreq[i] = converted[i];
    }
    
    sendCmd(rigFreq, 5, rigPort);
    getByte(rigPort);
}


byte getByte(RigPort rigPort) {
    
    byte radioReply;
    unsigned long startTime = millis();
    
    switch (rigPort) {
        case RIG_1_PORT:
        {
            while (Serial.available() < 1 && millis() < startTime + 2000)
            {
                ;
            }
            radioReply = Serial.read();
        }
            break;
            
        case RIG_2_PORT:
        {
            while (serialRig2.available() < 1 && millis() < startTime + 2000)
            {
                ;
            }
            radioReply = serialRig2.read();
        }
            break;
            
        default:
            break;
    }
    
    return radioReply ;
}

// GPL
// taken from hamlib work

unsigned long from_bcd_be(const  byte bcd_data[], unsigned bcd_len)
{
    int i;
    long f = 0;
    
    for (i=0; i < bcd_len/2; i++) {
        f *= 10;
        f += bcd_data[i]>>4;
        f *= 10;
        f += bcd_data[i] & 0x0f;
    }
    if (bcd_len&1) {
        f *= 10;
        f += bcd_data[bcd_len/2]>>4;
    }
    return f;
}

unsigned char * to_bcd_be( unsigned char bcd_data[], unsigned long  freq, unsigned bcd_len)
{
    int i;
    unsigned char a;
    
    if (bcd_len&1) {
        bcd_data[bcd_len/2] &= 0x0f;
        bcd_data[bcd_len/2] |= (freq%10)<<4;
        /* NB: low nibble is left uncleared */
        freq /= 10;
    }
    for (i=(bcd_len/2)-1; i >= 0; i--) {
        a = freq%10;
        freq /= 10;
        a |= (freq%10)<<4;
        freq /= 10;
        bcd_data[i] = a;
    }
    return bcd_data;
}

