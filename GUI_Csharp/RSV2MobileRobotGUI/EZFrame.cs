using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    struct EZFrame
    {
        public byte SenderID;               // 1-255. 
        public byte ReceiverID;             // 1-255. 0  broadcast
        public byte Ftype;
        public ushort PayloadLength;        // length of the payload
        public byte[] Payload;               // payload 
    }
}
