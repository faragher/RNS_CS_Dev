// MIT License
//
// Copyright (c) 2016-2022 Mark Qvist / unsigned.io
// C# Port (c) 2023 Michael Faragher / betweentheborders.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// This is considered pre-alpha of a minimum viable product. No warranties are provided and compatibility is not assured.

namespace RNS
{
    public class Reticulum
    {
        public const int MTU = 500;
        const int MAX_QUEUED_ANNOUNCES = 16384;
        const int QUEUED_ANNOUNCE_LIFE = 60 * 60 * 24;

        public const int ANNOUNCE_CAP = 2;


        const int MINIMUM_BITRATE = 500;


        public const int DEFAULT_PER_HOP_TIMEOUT = 6;



        public const int TRUNCATED_HASHLENGTH = 128;

        public const int HEADER_MINSIZE = 2 + 1 + (TRUNCATED_HASHLENGTH / 8) * 1;
        public const int HEADER_MAXSIZE = 2 + 1 + (TRUNCATED_HASHLENGTH / 8) * 2;
        public const int IFAC_MIN_SIZE = 1;
        byte[] IFAC_SALT = ParseHexToByteArray("adf54d882c9a9b80771eb4995d702d4a3e733391b2a0f53f416d9f907e55cff8");

        int MDU = MTU - HEADER_MAXSIZE - IFAC_MIN_SIZE;

        int RESOURCE_CACHE = 24 * 60 * 60;
        int JOB_INTERVAL = 5 * 60;
        int CLEAN_INTERVAL = 15 * 60;
        int PERSIST_INTERVAL = 60 * 60 * 12;

        //    router           = None
        //    config = None //ToDo FixMe

        //    # The default configuration path will be expanded to a directory
        //# named ".reticulum" inside the current users home directory
        public string userdir = Environment.SpecialFolder.UserProfile.ToString();
        string configdir = "";
        string configpath = "";
        string storagepath = "";
        string cachepath = "";


        public static byte[] ParseHexToByteArray(string S)
        {
            byte[] buffer = new byte[S.Length / 2];
            int i = 0;
            while (i < buffer.Length)
            {
                buffer[i] = byte.Parse(S.Substring(i * 2, 2), System.Globalization.NumberStyles.HexNumber);
                i++;
            }
            return buffer;
        }

        public class Packet
        {
            byte HeaderA;
            byte Hops;
            byte Context;
            byte[] AddressA = new byte[16];
            byte[] AddressB = new byte[16];
            byte[] DataBuffer = new byte[465];
            byte[] Data;
            byte[] IFAC = new byte[64];
            byte PropagationType;
            byte DestinationType;
            byte PacketType;
            int DataLength;
            bool TypeTwoHeader;
            const byte IFAC_Header =             0b10000000;
            const byte Header_Type_Header =      0b01000000;
            const byte Propagation_Type_Header = 0b00110000;
            const byte Destination_Type_Header = 0b00001100;
            const byte Packet_Type_Header =      0b00000011;
            bool IFAC_Flag;

            int Position;
            public Packet(byte[] Payload, Interface Inter)
            {


                HeaderA = Payload[0];
                Hops = Payload[1];

                Position = 2;

                IFAC_Flag = ((HeaderA & IFAC_Header) == 0b10000000) ? true : false;
                TypeTwoHeader = ((HeaderA & Header_Type_Header) == 0b01000000) ? true : false;
                PropagationType = (byte)((HeaderA & Propagation_Type_Header) >> 4);
                DestinationType = (byte)((HeaderA & Destination_Type_Header) >> 2);
                PacketType = (byte)(HeaderA & Packet_Type_Header);

                if (IFAC_Flag)
                {
                    if (Inter.ifac_size > 0)
                    {
                        for (int i = 0; i < Inter.ifac_size; i++)
                        {
                            IFAC[i] = Payload[Position];
                            Position++;
                        }
                    }
                }
                for (int i = 0; i < 16; i++)
                {
                    AddressA[i] = Payload[Position];
                    Position++;
                }
                if (TypeTwoHeader)
                {
                    for (int i = 0; i < 16; i++)
                    {
                        AddressB[i] = Payload[Position];
                        Position++;
                    }
                }
                Context = Payload[Position];
                Position++;
                DataLength = Payload.Length - Position;
                Data = new byte[DataLength];
                for(int i = 0; i < DataLength; i++)
                {
                    Data[i] = Payload[Position];
                    Position++;
                }

            }
            public void Verbose()
            {
                Console.WriteLine("Packet contents:");
                Console.WriteLine("IFAC: "+IFAC_Flag.ToString());
                if (IFAC_Flag) { 
                    Console.WriteLine("  Length: "+IFAC.Length.ToString());
                    Console.WriteLine("  "+IFAC.ToString());
                }
                string buffer = TypeTwoHeader ? "2" : "1";
                Console.WriteLine("Header Type: "+buffer);
                Console.WriteLine("Propagation Type: "+PropagationType.ToString()+" ("+Transport.PropagationType[PropagationType]+")");
                Console.WriteLine("Destination Type: " + DestinationType.ToString() + " ("+Transport.DestinationType[DestinationType]+")");
                Console.WriteLine("Packet Type: " + PacketType.ToString() + " (" + Transport.PacketType[PacketType] + ")");
                Console.WriteLine("Hops: " + Hops);
                Console.Write("Address: ");
                foreach(byte B in AddressA)
                {
                    Console.Write(B.ToString("X") + " ");
                }
                Console.WriteLine();
                if (TypeTwoHeader)
                {
                    Console.Write("Address 2: ");
                    foreach (byte B in AddressB)
                    {
                        Console.Write(B.ToString("X") + " ");
                    }
                    Console.WriteLine() ;
                }
                Console.WriteLine("Context: " + Context.ToString("X"));
                Console.WriteLine("Data Length: " + DataLength);
                Console.Write("Data Content: ");
                foreach(byte B in Data)
                {
                    Console.Write(B.ToString("X")+" ");
                }
                Console.WriteLine();
                Console.WriteLine("End packet");




            }
        }


    }

    public static class Util
    {
        public static string PrettyHexRep(byte[] data)
        {
            string buffer = "<";
            foreach(byte b in data)
            {
                buffer += b.ToString("X2");
            }
            buffer += ">";
            return buffer;
        }

        public static byte[] UTF8_to_Bytes(string Input)
        {
            return System.Text.Encoding.UTF8.GetBytes(Input);
        }

        public static byte[] TruncateHash(byte[] data, int length, bool fromEnd = true)
        {
            byte[] hash = new byte[length];
            int index = 0;
            if (fromEnd)
            {
                index = data.Length - length;
            }
            Array.Copy(data, index, hash, 0, length);
            return hash;
        }
    }
}