using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobosapienRFControl
{
    class EZStationPacketFSM
    {   
        // ************* Packet Related fields ***********************
        // sender ID
        public byte PacketSenderID;
        // packet receive timeout flag
        public Boolean PacketReceiveTimeout;
        // the current packet as raw bytes
        public byte[] CurrentPacket;
        
        // packet Reception Start Time
        public DateTime ReceptionStartTime;
        // a frame reception start time
        public DateTime FrameReceptionStartTime;

        // packet length
        public int CurrentPacketLength;
        // number of frames received so far
        public int FramesReceived;
        // total number of frames required for current packet
        public int TotalFrames;

        // the EZRoboNet device used by this Packet FSM
        public EZRoboNetDevice NetDevice;

        // the FSM's state
        public int state;

        public EZStationPacketFSM(EZRoboNetDevice device, EZFrame frame)
        {
            // registering the Net Device
            NetDevice = device;

            // capturing time of first reception
            ReceptionStartTime = DateTime.Now;
            // acknowledging
            NetDevice.sendAcknowledge(PacketSenderID);
            // retrieving sender id

            PacketSenderID = frame.SenderID;
            // calculating total packet length
            ushort packetlength = (ushort)(frame.Payload[3] + 256 * frame.Payload[4] + 5);
            CurrentPacketLength = packetlength;
            // calculating total number of frames required for full packet transmission
            TotalFrames = packetlength / EZRoboNetDevice.MAX_FRAME_PAYLOAD_LENGTH + 1;
            // allocating memory space for the packet being received
            CurrentPacket = new byte[packetlength];
            // now filling initial space in CurrentPacket
            int i;
            for (i = 0; i < frame.PayloadLength; i++)
                CurrentPacket[i] = frame.Payload[i];
            // done copying

            // setting number of frames received to 1
            FramesReceived = 1;
            // sending an acknowledgement frame
            NetDevice.sendAcknowledge(frame.SenderID);

            if (FramesReceived < TotalFrames)
            { // need to receive more
                state = EZRoboNetDevice.stReceiving;
                // marrking time of acknowledgement
                FrameReceptionStartTime = DateTime.Now;
            }
            else
            {
                state = EZRoboNetDevice.stPacketReady;
                FramesReceived = 0;
                TotalFrames = 0;
            }
        }


        // handle next frame
        public void receivedNewFrame(EZFrame frame)
        {
            // sending acknowledgement
            NetDevice.sendAcknowledge(PacketSenderID);
            // copying payload bytes into the packet
            int i;
            int startindex = EZRoboNetDevice.MAX_FRAME_PAYLOAD_LENGTH * FramesReceived;
            for (i = 0; i < frame.PayloadLength; i++)
                CurrentPacket[startindex + i] = frame.Payload[i];
            // frame payload copied
            // increasing number of received frames
            FramesReceived++;



            // now changing state
            if (FramesReceived == TotalFrames) state = EZRoboNetDevice.stPacketReady;
            else
            {
                state = EZRoboNetDevice.stReceiving;
                // marking time for next frame arrival possible timeout
                FrameReceptionStartTime = DateTime.Now;
            }

        }



        // handling internal state according to input and triggering appropriate actions
        public void transitionAction(EZFrame frame)
        {
            switch (state)
            {
                
                case EZRoboNetDevice.stReceiving: // in the process of receiving frames
                    switch (frame.Ftype)
                    {
                        case EZRoboNetDevice.t_Ping: // just reply and go back to wait for the next frame in the sequence
                            NetDevice.sendAcknowledge(frame.SenderID);

                            state = EZRoboNetDevice.stReceiving;
                            break;
                        case EZRoboNetDevice.t_Data: // Data frame arrived. See if we can fill the packet some more
                            if (PacketSenderID == frame.SenderID) // received a frame from the original sender
                                receivedNewFrame(frame);
                            else
                            { // packet does not originate fron the current sender
                                // disposing inetercepted frame

                                state = EZRoboNetDevice.stReceiving; // still waiting for a new frame
                            }
                            break;
                        case EZRoboNetDevice.t_Acknowledge: // acknowledgment packet arrived
                            // do nothing. dispose the frame

                            break;
                    }
                    break;

                case EZRoboNetDevice.stPacketReady: // a packet is ready to be handled
                    switch (frame.Ftype)
                    {
                        case EZRoboNetDevice.t_Ping: //  ping frame
                            NetDevice.sendAcknowledge(frame.SenderID);

                            state = EZRoboNetDevice.stPacketReady;
                            break;
                        case EZRoboNetDevice.t_Acknowledge: // acknowledge frame
                            // ignore

                            state = EZRoboNetDevice.stPacketReady;
                            break;
                        case EZRoboNetDevice.t_Data: // a data frame
                            // ignore until packeready state has been served

                            state = EZRoboNetDevice.stPacketReady;
                            break;
                    }
                    break;
            } // end big state switch

        }



    }
}
