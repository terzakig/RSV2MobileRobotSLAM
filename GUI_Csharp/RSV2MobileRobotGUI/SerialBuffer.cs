using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class SerialBuffer
    {
        // constants
        public int BUFFER_SIZE = 10000;

        // memebrs
        private byte[] buffer;
        private int AvailableBytes;
        private int NextByteIndex;



        // constructor
        public SerialBuffer()
        {
            buffer = new byte[BUFFER_SIZE];
            AvailableBytes = 0;
            NextByteIndex = 0;
        }

        
        // add byte to the buffer
        public void addByte(byte ch)
        {
            AvailableBytes++;

            buffer[NextByteIndex+AvailableBytes-1] = ch;
            
        }

        // read a number of Bytes from the buffer starting at current index position
        public int readBytes(byte[] abuffer, int numBytes)
        {
            int bytestoread;
            if (AvailableBytes >= numBytes) bytestoread = numBytes;
            else bytestoread = AvailableBytes;

            int i;
            for (i = 0; i < bytestoread; i++)
                abuffer[i] = buffer[NextByteIndex + i];
            AvailableBytes -= bytestoread;
            NextByteIndex = (AvailableBytes == 0) ? 0 : NextByteIndex + bytestoread;

            return bytestoread;
        }


        public int bytesAvailable()
        {
            return AvailableBytes;
        }

        public void Clear()
        {
            AvailableBytes = 0;
            NextByteIndex = 0;
        }


    }
}
