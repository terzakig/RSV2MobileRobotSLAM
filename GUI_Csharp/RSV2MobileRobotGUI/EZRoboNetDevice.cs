using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;



namespace RobosapienRFControl
{
    class EZRoboNetDevice
    {
        // frame starter and terminator characters 
        // (CRC checksum follows terminator)
        public const byte framestarter = (byte)'*';
        public const byte frameterminator = 0xA;
        // maximum size for a frame payload
        public const ushort MAX_FRAME_PAYLOAD_LENGTH = 50;
        // maximum size of a message in a packet
        public const ushort MAX_PACKET_MESSAGE_LENGTH = 1024;

        //maximum packet Number in the outbound queue
        public const byte MAX_OUTBOUND_PACKETS_NUM = 4;

        // maximum received packets array length
        public const ushort MAX_RECEIVED_PACKETS_NUM = 100;
        // ** Transmission Time out Constants **
        // frame acknowledge timeout interval (in ms)
        public const int FRAME_ACKNOWLEDGE_TIMEOUT = 500; // in milliseconds
        // packet transmission/reception timeout
        public const int PACKET_SENDING_TIMEOUT = 5;   // in  seconds 

        // maximum length of incomplete packets
        EZStationPacketFSM[] IncompletePackets;
        // number of incomplete packets
        public int IncompletePacketsNum; 

        // types of nodes
        public const byte t_Node_Station = 0;
        public const byte t_Node_Robot =1;

        // types of eZroboNet frame type
        public const byte t_Ping = 0;
        public const byte t_Data = 1;
        public const byte t_Acknowledge = 2;


        // state constants
        public const byte stIdle = 0;
        public const byte stSending = 1;
        public const byte stReceiving = 2;
        public const byte stWaitingAcknowledge = 3; 
        public const byte stPacketReady = 4;



        // Class Members
        public byte NodeID; 
        public byte DevNodeType; // type of node Robot or Station
        public System.IO.Ports.SerialPort CommPort;
        public SerialBuffer CommPortBuffer;

        public System.Windows.Forms.Timer SyncTimer; // a timer to clock the station EZRoboNet Device
      
        public byte state;
        public DateTime TransmissionStartTime, ReceptionStartTime;
        public DateTime FrameTransmissionStartTime, FrameReceptionStartTime; 
        public int FramesReceived, FramesSent, TotalFrames;
        public int CurrentPacketLength;
        public byte[] CurrentPacket;
        public byte PacketReceiverID;
        public byte PacketSenderID;
        public int remainingBytes;
        public byte[] FrameHead;
        // some flags...
        public Boolean PacketSendTimeout;
        public Boolean PacketReceiveTimeout;
        public byte PacketReceiveTimeoutSenderID;
        public Boolean PacketQueueOverflow;
        // a list of packets
        EZPacket[] ReceivedPackets;
        int ReceivedPacketsNum;

        // outbound packets queue
        EZPacket[] OutboundPackets;
        int OutboundPacketsNum;

        // List of Entities registered for communications through the NetDevice
        RoboEntity[] EntityList;
        int EntityNum; // number of entities in the list
        public const int MAX_ENTITY_NUM = 1000;
        // CRC calculation
        public static ushort calculateCRC(byte[] pData, int numBytes)
        {
            int index = 0;
            ushort crc = 0;

            while (index < numBytes)
            {
                crc = (ushort)((crc >> 8) | (crc << 8));
                crc ^= pData[index++];
                crc ^= (ushort)((crc & 0xFF) >> 4);
                crc ^= (ushort)((crc << 8) << 4);
                crc ^= (ushort)(((crc & 0xFF) << 4) << 1);
            }

            return crc;
        }

        // crc check
        public static Boolean checkCRC(byte[] pData, int numBytes, ushort acrc)
        {
            ushort ccrc = calculateCRC(pData, numBytes);

            return acrc == ccrc;
        }


        // Constructor
        public EZRoboNetDevice(byte node_id, byte node_type)
        {
            NodeID = node_id;
            DevNodeType = node_type;
            // initializing the CommPort and CommPortBuffer
            CommPortBuffer = new SerialBuffer();
            CommPort = new System.IO.Ports.SerialPort("COM1");
            CommPort.BaudRate = 19200;
            CommPort.WriteBufferSize = 10000;
            CommPort.ReadBufferSize = 10000;
            CommPort.Parity = System.IO.Ports.Parity.None;
            CommPort.Handshake = System.IO.Ports.Handshake.None;
            CommPort.StopBits = System.IO.Ports.StopBits.One;
            CommPort.ReceivedBytesThreshold = 2;
            CommPort.DataReceived += this.SerialDataReceived;
            

            // Initializing the Timer
            SyncTimer = new System.Windows.Forms.Timer(); // 10 msec interval
            SyncTimer.Interval = 30; // 10 msec interval
            SyncTimer.Tick += SyncTimerTick;
            // initializing state
            state = stIdle;
            // flags
            PacketSendTimeout = false;
            PacketReceiveTimeout = false;
            PacketQueueOverflow = false;

            // incoming data processing globals
            remainingBytes = 0;
            FrameHead = new byte[6];

            // initializing array of incomplete packets
            IncompletePackets = new EZStationPacketFSM[MAX_RECEIVED_PACKETS_NUM];
            // null-ing all entries
            int i;
            for (i = 0; i < MAX_RECEIVED_PACKETS_NUM; i++)
                IncompletePackets[i] = null;
            IncompletePacketsNum = 0;

            // initializing array of received packets
            ReceivedPackets = new EZPacket[MAX_RECEIVED_PACKETS_NUM];
            ReceivedPacketsNum = 0;

            // initializing array of outbound packets
            OutboundPackets = new EZPacket[MAX_OUTBOUND_PACKETS_NUM];
            OutboundPacketsNum = 0;

            // initializing entity list. No entities registered yet
            EntityList = new RoboEntity[MAX_ENTITY_NUM];
            
            for (i = 0; i < MAX_ENTITY_NUM; i++)
                EntityList[i] = null;
            EntityNum = 0;

            // starting the NetDevice (opening serial port and starting timer)
            CommPort.Open();
            SyncTimer.Start();

        }


        // **************** handling the Incomplete Packets Queue **********************
        // insert new packet reception FSM
        public void InsertNewIncompletePacket(EZFrame frame)
        {
            int i;
            // shifting the queue to the right
            for (i = IncompletePacketsNum; i > 0; i--)
                IncompletePackets[i] = IncompletePackets[i - 1];
            // increasing number of Packets
            IncompletePacketsNum++;
            // adding new packet reception FSM
            IncompletePackets[0] = new EZStationPacketFSM(this, frame);
        }

        // find a Packet Reception FSM using Sender node ID
        public EZStationPacketFSM findIncompletePacket(byte nodeID)
        {
            EZStationPacketFSM thepack = null;
            Boolean found = false;
            int i = 0;
            while ((!found) && (i < IncompletePacketsNum))
            {
                if (IncompletePackets[i].PacketSenderID == nodeID)
                {
                    found = true;
                    thepack = IncompletePackets[i];
                }
                i++;
            }

            return thepack;
        }

        // remoce a packet reception FSM from queue
        public EZStationPacketFSM removeIncompletePacket()
        {
            EZStationPacketFSM temp = null;
            if (IncompletePacketsNum > 0)
            {
                temp = IncompletePackets[IncompletePacketsNum - 1];
                IncompletePacketsNum--;
            }

            return temp;
        }

        // ******************** Packet Reception FSM list handling routines ends ****************


        // restart the NetDevice
        public void Restart()
        {
            CommPort.Open();
            SyncTimer.Start();
        }

        // stop the net device
        public void Stop()
        {
            SyncTimer.Stop();
            CommPort.Close();
        }


        // entity registration method
        public int registerEntity(RoboEntity roboentity)
        {
            EntityNum++;
            EntityList[EntityNum - 1] = roboentity;
            return EntityNum - 1;
        }

        // Event handler for the SerialPort (CommPort) member
        public void SerialDataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            byte ch;

            while (CommPort.BytesToRead > 0)
            {
                ch = (byte)CommPort.ReadByte();
                CommPortBuffer.addByte(ch);
            }
        }

        // Event Handler for the Timer (SyncTimer) member
        public void SyncTimerTick(object sender, EventArgs e)
        {
            processData(CommPortBuffer);
            processEvents();
            

            int packetsInQueue = packetsAvailable();
            do
            {
                if (packetsInQueue > 0)
                {
                    int i;
                    for (i = 0; i < packetsInQueue; i++)
                    {
                        EZPacket pack = getNextAvailablePacket();
                        int senderID = pack.SenderID;
                        int j = 0;
                        if (EntityNum > 0)
                        {
                            Boolean found = false;
                            while ((!found) & (j < EntityNum))
                            {
                                if (EntityList[j].AvatarNodeID == senderID)
                                    found = true;
                                else
                                    j++;
                            }
                            if (j < EntityNum)
                                EntityList[j].addIncomingPacket(pack);
                        }
                    }
                }
                packetsInQueue = packetsAvailable();
            } while (packetsInQueue > 0);

        }

        // returns an euivalent array of bytes to a packet
        byte[] packet2Bytes(EZPacket pack)
        {
            int packlen = 5 + pack.MessageLength; // 5 bytes in the header plus the message length
            byte[] rawmsg = new byte[packlen];
            rawmsg[0] = pack.SenderID;
            rawmsg[1] = pack.ReceiverID;
            rawmsg[2] = pack.SenderNodeType;
            rawmsg[3] = (byte)(pack.MessageLength & 0x0FF);
            rawmsg[4] = (byte)((pack.MessageLength >> 8) & 0x0FF);
            int i;
            for (i = 0; i < pack.MessageLength; i++)
                rawmsg[5 + i] = pack.Message[i];

            return rawmsg;
        }


        // returns the eqiv. array of bytes (with CRC checksum added) of a frame structure
        public byte[] frame2Bytes(EZFrame frame)
        {
            int framelen = 5 + frame.PayloadLength + 4; // 5 header bytes plus payload, plus starter, terminator and CRC checksum
            byte[] rawframe = new byte[framelen];

            rawframe[0] = framestarter;
            rawframe[1] = frame.SenderID;
            rawframe[2] = frame.ReceiverID;
            rawframe[3] = frame.Ftype;
            rawframe[4] = (byte)(frame.PayloadLength & 0x0FF);
            rawframe[5] = (byte)((frame.PayloadLength >> 8) & 0x0FF);
            int i;
            for (i = 0; i < frame.PayloadLength; i++)
                rawframe[6 + i] = frame.Payload[i];
            rawframe[framelen - 3] = frameterminator;
            // now adding CRC
            ushort crc = calculateCRC(rawframe, framelen - 2);
            rawframe[framelen - 2] = (byte)(crc & 0x0FF);
            rawframe[framelen - 1] = (byte)((crc >> 8) & 0x0FF);

            return rawframe;
        }

        // length of a raw frame
        public ushort getRawFrameLength(byte[] rawframe)
        {

            return (ushort)(9 + rawframe[4] + 256 * rawframe[5]);
        }

        // length of a raw packet
        public ushort getRawPacketLength(byte[] rawpacket)
        {

            return (ushort)(5 + rawpacket[3] + 256 * rawpacket[4]);
        }
        
        // Convert bytes to a Frame structure
        public EZFrame bytes2Frame(byte[] rawframe)
        {
            EZFrame frame = new EZFrame();
            ushort payloadlen = (ushort)(rawframe[4] + 256 * rawframe[5]);

            frame.SenderID = rawframe[1];
            frame.ReceiverID = rawframe[2];
            frame.Ftype = rawframe[3];
            frame.PayloadLength = payloadlen;

            frame.Payload = new byte[payloadlen];
            int i;
            for (i = 0; i < payloadlen; i++)
                frame.Payload[i] = rawframe[6 + i];

            return frame;
        }


        // Convert bytes to a packet structure
        public EZPacket bytes2Packet(byte[] rawpacket)
        {
            EZPacket packet = new EZPacket();
            ushort messagelen = (ushort)(rawpacket[3] + 256 * rawpacket[4]);

            packet.SenderID = rawpacket[0];
            packet.ReceiverID = rawpacket[1];
            packet.SenderNodeType = rawpacket[2];
            packet.MessageLength = messagelen;

            packet.Message = new byte[messagelen];
            int i;
            for (i = 0; i < messagelen; i++)
                packet.Message[i] = rawpacket[5 + i];

            return packet;
        }

                
        // initiate transmission of a packet
        public void sendPacket(EZPacket pack)
        {
            byte receiver_id = pack.ReceiverID;
            CurrentPacket = packet2Bytes(pack);

            // retrieving length of raw packet data
            CurrentPacketLength = getRawPacketLength(CurrentPacket);

            // computing number of frames required
            TotalFrames = CurrentPacketLength / MAX_FRAME_PAYLOAD_LENGTH + 1;

            // zeroing number of frames sent
            FramesSent = 0;
            // clearing timeout flag
            PacketSendTimeout = false;
            // marking the starting time for possible timeouts
            TransmissionStartTime = DateTime.Now;
            // setting Packet Receiver global
            PacketReceiverID = receiver_id;
            // setting state to sending...
            state = stSending;
            // now sending first frame
            sendNextFrame();

        }

        // send the next frame of the current packet
        public void sendNextFrame()
        {
            // creating the first frame
            EZFrame frame = new EZFrame();
            frame.SenderID = NodeID;
            frame.ReceiverID = PacketReceiverID;
            frame.Ftype = t_Data;
            ushort payloadlen, index;
            index = (ushort)(FramesSent * MAX_FRAME_PAYLOAD_LENGTH);
            if (FramesSent == TotalFrames - 1)
                payloadlen = (ushort)(CurrentPacketLength % MAX_FRAME_PAYLOAD_LENGTH);
            else
                payloadlen = MAX_FRAME_PAYLOAD_LENGTH;
            int i;

            frame.PayloadLength = payloadlen;
            frame.Payload = new byte[payloadlen];

            for (i = 0; i < payloadlen; i++)
                frame.Payload[i] = CurrentPacket[i + index];
            // frame created
            byte[] rawframe = frame2Bytes(frame);
            int rawframelen = getRawFrameLength(rawframe);
            // stream the packet through the serial port with a delay per character

            CommPort.Write(rawframe, 0, rawframelen);


            // god forbid, packet was sent. Changing state now
            state = stWaitingAcknowledge;
            // marking frame transmission time for possible acknowledge timeout
            FrameTransmissionStartTime = DateTime.Now;

        }

        

        // create an acknowledgement frame
        public EZFrame makeAcknowledge(byte receiver_id)
        {
            EZFrame frame = new EZFrame();

            frame.SenderID = NodeID;
            frame.ReceiverID = receiver_id;
            frame.Ftype = t_Acknowledge;
            frame.PayloadLength = 2;
            frame.Payload = new byte[frame.PayloadLength];
            frame.Payload[0] = 0;
            frame.Payload[1] = 0;

            return frame;
        }

        // send acknowledgment  
        public void sendAcknowledge(byte receiver_id)
        {
            EZFrame ackframe = makeAcknowledge(receiver_id);
            // converting struct to  bytes
            byte[] rawackframe = frame2Bytes(ackframe);
            // retrieving length
            ushort framelength = getRawFrameLength(rawackframe);

            // sending bytes
            
            CommPort.Write(rawackframe, 0, framelength);

            // done

        }

        
        // handling internal state according to input and triggering appropriate actions
        public void transitionAction(EZFrame frame)
        {
            switch (state)
            {
                case stIdle: // not in process process of receiving or sending
                    if (frame.Ftype == t_Ping)
                    {// received a ping frame. will acknowledge immediately...
                        sendAcknowledge(frame.SenderID);
                        state = stIdle; // state remains idle
                    }
                    else if (frame.Ftype == t_Acknowledge) // ignoring a random acknowledge
                        state = stIdle;
                    else
                    { // any other type of frame
                        // now looking up the incomplete Packets Queue
                        EZStationPacketFSM incompletePack = findIncompletePacket(frame.SenderID);
                        if (incompletePack != null)
                        {
                            incompletePack.transitionAction(frame);
                        }
                        else
                        {// adding a new incomplete pack
                            InsertNewIncompletePacket(frame);
                        }

                        state = stIdle; // still idle
                    }
                    break;
                case stSending: // in the process of sending a packet. still, handling the incoming

                    if (frame.Ftype == t_Ping)
                    {// received a ping frame. will acknowledge immediately...
                        sendAcknowledge(frame.SenderID);
                        state = stSending; // state remains stSending
                    }
                    else if (frame.Ftype == t_Acknowledge) // ignoring a random acknowledge
                        state = stSending; // keep on sending
                    else
                    { // any other type of frame
                        // now looking up the incomplete Packets Queue
                        EZStationPacketFSM incompletePack = findIncompletePacket(frame.SenderID);
                        if (incompletePack != null)
                        {
                            incompletePack.transitionAction(frame);
                        }
                        else
                        {// adding a new incomplete pack
                            InsertNewIncompletePacket(frame);
                        }

                        state = stSending; // still Sending
                    }
                    
                    break;

                case stWaitingAcknowledge: // expecting a frame reception acknowledgement
                        if (frame.Ftype == t_Ping) {// casual ping. immediate reply
                            sendAcknowledge(frame.SenderID);
                            // state remains
                            state = stWaitingAcknowledge;
                        }
                        else if (frame.Ftype == t_Acknowledge)
                        { // received and acknowledge frame
                            if (frame.SenderID == PacketReceiverID)
                            {
                                // increasing number of sent frames
                                FramesSent++;
                                if (FramesSent == TotalFrames)
                                {// all sent

                                    PacketReceiverID = 0;
                                    state = stIdle;
                                }
                                else
                                { // need to send more frames
                                    state = stSending;
                                    sendNextFrame();
                                }
                            }
                            else
                                state = stWaitingAcknowledge;


                        }
                        else
                        { // assigning frame to proper Incomplete Packet recipient
                            // any other type of frame
                            // now looking up the incomplete Packets Queue
                            EZStationPacketFSM incompletePack = findIncompletePacket(frame.SenderID);
                            if (incompletePack != null)
                            {
                                incompletePack.transitionAction(frame);
                            }
                            else
                            {// adding a new incomplete pack
                                InsertNewIncompletePacket(frame);
                            }

                            state = stWaitingAcknowledge; // still Waiting for acknowledge

                        }
                    break;
            } // end big state switch

        }
                    
        // clear the serial buffer
        public void flushSerialBuffer(SerialBuffer serialport)
        {
            serialport.Clear();
            
        }

        // processEvents consults with the state and initiates packet handling 
        // additionally, if a timeout occurs, it re-initiates transmission
        public void processEvents()
        {
            int i, j;

            // 1. Checking Possible Timeouts while sending a packet
            if (state == stWaitingAcknowledge)
            { // waiting for acknowledge 
                // checking time ellapsed
                DateTime currentTime = DateTime.Now;
                TimeSpan transmissionElapsed = currentTime - TransmissionStartTime;
                if (transmissionElapsed.Seconds > PACKET_SENDING_TIMEOUT)
                { // transmission timedout
                    // raising error flag
                    PacketSendTimeout = true;
                    // quiting everything and resetting
                    PacketSenderID = 0;
                    PacketReceiverID = 0;
                    TotalFrames = 0;
                    FramesSent = 0;
                    FramesReceived = 0;
                    state = stIdle;
                }
                else
                {
                    TimeSpan elapsed = currentTime - FrameTransmissionStartTime;
                    if (elapsed.Milliseconds > FRAME_ACKNOWLEDGE_TIMEOUT)
                    { // timeout occured
                        // attempting to resend the frame
                        state = stSending; // sending again
                        sendNextFrame();
                    }
                }
            }
            else // 2. Checking outbound packets queue to initiate next transmission
                if (state == stIdle)
                {
                    if (OutboundPacketsNum > 0)
                    { // outbound queue is not empty. must initiate transmission
                        // removing a packet from the outbound queue  
                        EZPacket outpacket = OutboundPackets[0];

                        OutboundPacketsNum--;

                        // shifting the Outbound queue to the left

                        for (i = 1; i <= OutboundPacketsNum; i++)
                            OutboundPackets[i - 1] = OutboundPackets[i];


                        // outbound queue shifted
                        // Now, sending packet
                        sendPacket(outpacket);
                    }
                }

            // 3. Checking for ready packets in the incomplete Packets Queue
            for (i = 0; i < IncompletePacketsNum; i++) 
                if (IncompletePackets[i].state == stPacketReady) {
                    // the packet is assembled and ready
                // increase number of packets received
                if (ReceivedPacketsNum < MAX_RECEIVED_PACKETS_NUM)
                {
                    ReceivedPacketsNum++;
                    ReceivedPackets[ReceivedPacketsNum - 1] = bytes2Packet(IncompletePackets[i].CurrentPacket);
                }
                else // overflow
                    PacketQueueOverflow = true;

                // decreasing number of incomplete packets
                IncompletePacketsNum--;
                // deleting the Incomplete Packet entry from the Queue
                for (j=i; j<IncompletePacketsNum; j++)
                    IncompletePackets[j] = IncompletePackets[j+1];
                
                // clearing residual pointer
                IncompletePackets[IncompletePacketsNum] = null;
                }

            
           
            // 4. Checking Possible Timeouts while receiving
            for (i=0; i<IncompletePacketsNum; i++) 
            if (IncompletePackets[i].state == stReceiving)
            { // receiving a packet
                DateTime currentTime = DateTime.Now;
                TimeSpan receptionElapsed = currentTime - IncompletePackets[i].ReceptionStartTime;
                if (receptionElapsed.Seconds > PACKET_SENDING_TIMEOUT)
                { // packet's timedout
                    // raising timeout flag flag (must be handled shortly though...)
                    PacketReceiveTimeout = true;
                    PacketReceiveTimeoutSenderID = IncompletePackets[i].PacketSenderID;
                    // deleting the Packet Reception FSM
                    for (j = i; j < IncompletePacketsNum - 1; j++)
                        IncompletePackets[j] = IncompletePackets[j + 1];
                    // decreasing number of incomplete packets
                    IncompletePacketsNum--;
                    // clearing residual pointer
                    IncompletePackets[IncompletePacketsNum] = null;
     
                    
                }
                else
                {
                    // checking time elapsed since last acknowledgement
                    TimeSpan elapsed = currentTime - IncompletePackets[i].FrameReceptionStartTime;
                    if (elapsed.Milliseconds > FRAME_ACKNOWLEDGE_TIMEOUT)
                    { // next frame timedout
                        // sending a new acknowledgement
                        sendAcknowledge(PacketReceiverID);
                        // reseting time
                        IncompletePackets[i].FrameReceptionStartTime = DateTime.Now;
                        IncompletePackets[i].state = stReceiving;
                    }
                }
            } 
            
            
            
        }


                



        // processData. processData assembles the frames and calls to transitionAction
        public void processData(SerialBuffer sBuffer) {
        int i;
    
        int numBytesToRead = sBuffer.bytesAvailable(); // let's see what we 've got...
  
        do {
            if (remainingBytes==0) { // previous frame was fully read
                if (numBytesToRead>=6) { // nead at least 6 bytes to read frame header
                        // get the first 6 bytes (starter(1)+sender-receiverid(2)+frame type(1)+payload length (2))
                        // reading first 6 bytes into FrameHead
                        
                        sBuffer.readBytes(FrameHead, 6);
                        // now cheking for framestarter character mismatch
                        if (FrameHead[0]!=framestarter) flushSerialBuffer(sBuffer);
                            else // reamaining bytes should be the payload length plus the terminator and two CRC bytes 
                              remainingBytes = FrameHead[4] + FrameHead[5] * 256 +3; 
                    }
            }
            else if (numBytesToRead >= remainingBytes)
            { // it's time to get the remaining frame(s)
                int totalBytes = remainingBytes + 6; // calculate total length
                byte[] buffer = new byte[totalBytes];
                byte[] remBuffer = new byte[remainingBytes];
                // now reading remaining bytes as estimated using the frame header
                // going altogether
                sBuffer.readBytes(remBuffer, remainingBytes);

                // tailoring bufhead and rembuffer into buffer

                for (i = 0; i < totalBytes; i++)
                    if (i < 6) buffer[i] = FrameHead[i];
                    else buffer[i] = remBuffer[i - 6];

                // now handling the message...
                // checking terminator and CRC
                ushort CRC = (ushort)(buffer[totalBytes - 2] + 256 * buffer[totalBytes - 1]);

                if ((buffer[totalBytes - 3] == frameterminator) && (checkCRC(buffer, totalBytes - 2, CRC)))
                {
                    EZFrame frame = bytes2Frame(buffer);
                    // done
                    if (frame.ReceiverID == NodeID)  // packet addressed to this node
                        transitionAction(frame); // change internal state and act
                }


                // clearing remaining bytes
                remainingBytes = 0;
            }

            numBytesToRead = sBuffer.bytesAvailable() ;
        
        } while ((numBytesToRead >= remainingBytes)&&(remainingBytes>0));

        }

        public void resetFlags()
        {
            PacketQueueOverflow = false;
            PacketSendTimeout = false;
            PacketReceiveTimeout = false;
        }

        // clear PacketSendTimeout flag
        public void clearPacketSendTimeoutFlag() {
            PacketSendTimeout = false;
        }

        // clear PacketReceiveTimeout Flag
        public void clearPacketReceiveTimeoutFlag() {
  
            PacketReceiveTimeout = false;
        }

        // clear PacketQueueOverflow flag
        public void clearPacketQueueOverflowFlag() {
  
            PacketQueueOverflow = false;
        }
  
        // number of packets available in queue             
        public int packetsAvailable() {

            return ReceivedPacketsNum;
        }


        public Boolean getPacketQueueOverflow() {
 
            return PacketQueueOverflow;
        }

        public Boolean getPacketReceiveTimeout() {
            Boolean f = PacketReceiveTimeout;
            PacketReceiveTimeout = false;
            return f;
        }

        Boolean getPackeSendTimeout() {
            Boolean f = PacketSendTimeout;
            PacketSendTimeout = false;
            return f;
        }

        public void clearFlags()
        {

            PacketQueueOverflow = false;
            PacketReceiveTimeout = false;
            PacketSendTimeout = false;
        }


        // get a packet from the queue
        public EZPacket getNextAvailablePacket()
        {
            EZPacket pack;
            // pack is now pointing to the allocated Packet that the queue points in position 0
            pack = ReceivedPackets[0];
            // reducing number of available packets
            ReceivedPacketsNum--;
            // clearing a possible overflow flag
            clearPacketQueueOverflowFlag();
            // shifting contents of queue to the left
            int i;
            for (i = packetsAvailable() - 1; i > 0; i--)
                ReceivedPackets[i - 1] = ReceivedPackets[i];

            return pack;
        }

        public void resetPacketTransceiver()
        {
            ReceivedPacketsNum = 0;

            clearFlags();
        }

        


        // examines if the outbound packet queue is fulll
        public Boolean OutboundQueueFull()
        {
            return (OutboundPacketsNum >= MAX_OUTBOUND_PACKETS_NUM);
        }
 
        // pushes a new packet into the outbound queue
        public Boolean pushOutboundPacket(EZPacket pack)
        {
            Boolean success = false;
            if (!OutboundQueueFull())
            {
                OutboundPacketsNum++;
                OutboundPackets[OutboundPacketsNum - 1] = pack;
                success = true;
            }
            else
                success = false;
            return success;
        }

    }
}
