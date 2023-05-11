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
    public class Interface
    {
        public bool IN = false;
        public bool OUT = false;
        public bool FWD = false;
        public bool RPT = false;
        public string name = "";

        // Interface mode definitions
        const byte MODE_FULL = 0x01;
        const byte MODE_POINT_TO_POINT = 0x02;
        const byte MODE_ACCESS_POINT = 0x03;
        const byte MODE_ROAMING = 0x04;
        const byte MODE_BOUNDARY = 0x05;
        const byte MODE_GATEWAY = 0x06;

        // Which interface modes a Transport Node
        // should actively discover paths for.
        byte[] DISCOVER_PATHS_FOR = {MODE_ACCESS_POINT, MODE_GATEWAY};
        

        int rxb;
        int txb;
        bool online;

        public CallbackClass Callbacks;

        int ANNOUNCE_CAP;
        public int ifac_size;
        //List<>

        public Interface()
        {
            rxb = 0;
            txb = 0;
            online = false;
            if(ANNOUNCE_CAP == 0)
            {
                ANNOUNCE_CAP = RNS.Reticulum.ANNOUNCE_CAP;
            }
            Callbacks = new CallbackClass();
        }

        public class CallbackArgs : EventArgs
        {
            public byte[] Message { get; private set; }
            public RNS.Interface Interface { get; private set; }
            public CallbackArgs(byte[] _message, RNS.Interface _interface)
            {
                Message = _message;
                Interface = _interface;
            }
        }

        public class CallbackClass
        {
            //public delegate void CallbackEventHandler(object sender, CallbackArgs args);
            public event EventHandler<CallbackArgs>? CallbackEventHandler;
            public void Process_Inbound(byte[] _message, RNS.Interface _interface)
            {
                OnCallback(new CallbackArgs(_message, _interface));
            }
            protected virtual void OnCallback(CallbackArgs e)
            {
                EventHandler<CallbackArgs>? handler = CallbackEventHandler;
                if (handler != null)
                {
                    handler(this, e);
                }
            }
        }

    }
}