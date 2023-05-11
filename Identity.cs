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
    public class Identity
    {
        string CURVE = "Curve25519";


        const int KEYSIZE = 256 * 2;


        public const int FERNET_OVERHEAD = RNS.Cryptography.Fernet.FERNET_OVERHEAD;
        public const int AES128_BLOCKSIZE = 16;          // In bytes
        int HASHLENGTH = 256;         // In bits
        int SIGLENGTH = KEYSIZE;     // In bits

        int NAME_HASH_LENGTH = 80;
        int TRUNCATED_HASHLENGTH = RNS.Reticulum.TRUNCATED_HASHLENGTH;

        //known_destinations = {} //toDo FixMe
    }
}