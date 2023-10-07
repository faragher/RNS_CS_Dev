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
        const int HASHLENGTH = 256;         // In bits
        int SIGLENGTH = KEYSIZE;     // In bits

        public const int NAME_HASH_LENGTH = 80;
        public static int TRUNCATED_HASHLENGTH = RNS.Reticulum.TRUNCATED_HASHLENGTH;
        public byte[] Hash = new byte[HASHLENGTH / 8];

        List<Destination> known_destinations = new List<Destination>();

        Identity(bool create_keys = false)
        {
           string prv = "";
            byte[] prv_bytes = null;
            string sig_prv = "";
            byte[] sig_prv_bytes = null;

            string pub = "";
            byte[] pub_bytes = null;
            string sig_pub = "";
            byte[] sig_pub_bytes = null;

            //Hash = null;
            //HexHash = null;

            if (create_keys)
            {
                //Create_Keys();
            }
        }

        public string PrettyName
        {
            get { return RNS.Util.PrettyHexRep(Hash); }
        }

        public static byte[] FullHash(byte[] data)
        {
            // Functional, but placing under RNS.Crypto for generalization
            // return System.Security.Cryptography.SHA256.HashData(data);

            return RNS.Cryptography.SHA256(data);
        }

        public static byte[] Truncated_Hash(byte[] data)
        {
            byte[] hash = new byte[TRUNCATED_HASHLENGTH/8];
            byte[] fullhash = FullHash(data);
            Array.Copy(fullhash, fullhash.Length-(TRUNCATED_HASHLENGTH/8), hash, 0, TRUNCATED_HASHLENGTH/8);
            return hash;

        }

        public static byte[] Get_Random_Hash()
        {

            System.Security.Cryptography.RandomNumberGenerator random = System.Security.Cryptography.RandomNumberGenerator.Create();
            byte[] hashseed = new byte[TRUNCATED_HASHLENGTH/8];
            random.GetBytes(hashseed);
            return Truncated_Hash(hashseed);

        }

        public static void Persist_Data()
        {
            //if (!RNS.Transport.owner.isConnectedToSharedInstance)
            //{
            //    Save_Known_Destinations();
            //}
        }

        public static void Exit_Handler()
        {
            Persist_Data();
        }
    }
}