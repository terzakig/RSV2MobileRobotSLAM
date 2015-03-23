using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    struct EZPacket
    {
        public byte SenderID;     // 1-255. 
        public byte ReceiverID;   // 1-255. 0 broadcast
        public byte SenderNodeType;    // type of sender node (robot or station) 
        public ushort MessageLength;
        public byte[] Message;     // maximum 1024 bytes  
    }
}
