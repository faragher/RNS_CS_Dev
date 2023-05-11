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
    public class Link
    {

        //object CURVE = RNS.Identity.CURVE; //ToDo FixMe
        const int ECPUBSIZE = 32 + 32;
        const int KEYSIZE = 32;

        const int MDU = ((RNS.Reticulum.MTU - RNS.Reticulum.IFAC_MIN_SIZE - RNS.Reticulum.HEADER_MINSIZE - RNS.Identity.FERNET_OVERHEAD) / RNS.Identity.AES128_BLOCKSIZE) * RNS.Identity.AES128_BLOCKSIZE - 1;

        const int ESTABLISHMENT_TIMEOUT_PER_HOP = RNS.Reticulum.DEFAULT_PER_HOP_TIMEOUT;

        const int TRAFFIC_TIMEOUT_FACTOR = 6;
        const int KEEPALIVE_TIMEOUT_FACTOR = 4;

        const int STALE_GRACE = 2;

        const int KEEPALIVE = 360;

        public const int STALE_TIME = 2 * KEEPALIVE;

        const byte PENDING = 0x00;
        const byte HANDSHAKE = 0x01;
        const byte ACTIVE = 0x02;
        const byte STALE = 0x03;
        const byte CLOSED = 0x04;

        const byte TIMEOUT = 0x01;
        const byte INITIATOR_CLOSED = 0x02;
        const byte DESTINATION_CLOSED = 0x03;

        const byte ACCEPT_NONE = 0x00;
        const byte ACCEPT_APP = 0x01;
        const byte ACCEPT_ALL = 0x02;
        byte[] resource_strategies = {ACCEPT_NONE, ACCEPT_APP, ACCEPT_ALL};
    }
}