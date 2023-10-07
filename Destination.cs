namespace RNS
{

    public class Destination
    {
        const byte SINGLE = 0x00;
        const byte GROUP = 0x01;
        const byte PLAIN = 0x02;
        const byte LINK = 0x03;
        static byte[] types = new byte[] { SINGLE, GROUP, PLAIN, LINK };

        const byte PROVE_NONE = 0x21;
        const byte PROVE_APP = 0x22;
        const byte PROVE_ALL = 0x23;
        static byte[] proof_strategies = new byte[] { PROVE_NONE, PROVE_APP, PROVE_ALL };

        const byte ALLOW_NONE = 0x00;
        const byte ALLOW_ALL = 0x01;
        const byte ALLOW_LIST = 0x02;
        static byte[] request_policies = new byte[] { ALLOW_NONE, ALLOW_ALL, ALLOW_LIST };

        const byte IN = 0x11;
        const byte OUT = 0x12;
        static byte[] directions = new byte[] { IN, OUT };

        const byte PR_TAG_WINDOW = 30;

        static string Expand_Name(Identity _ID, string _app_name, List<string> _aspects)
        {
            if (_app_name.Contains("."))
            {
                throw new ArgumentException("Dots cannot be used in app names");
            }

            string name = _app_name;
            foreach(string S in _aspects)
            {
                if (S.Contains(".")) throw new ArgumentException("Dots cannot be used in aspects");
                name+="."+S;
            }

            if (_ID != null)
            {
                name+="."+_ID.PrettyName;
            }

            return name;
        }

        public static byte[] Hash(Identity _ID, string _app_name, List<string> aspects)
        {
            string FullName = Expand_Name(_ID, _app_name, aspects);
            byte[] Hash = Identity.FullHash(Util.UTF8_to_Bytes(FullName));
            byte[] TruncatedHash = Util.TruncateHash(Hash, Identity.NAME_HASH_LENGTH/8);
            byte[] addr_hash_material = TruncatedHash;

            foreach (byte b in Hash)
            {
                Console.Write(b.ToString("X2"));
            }

            if (_ID != null)
            {
                addr_hash_material = new byte[TruncatedHash.Length + _ID.Hash.Length];
                Array.Copy(TruncatedHash,0,addr_hash_material,0,TruncatedHash.Length);
                Array.Copy(_ID.Hash,0,addr_hash_material,TruncatedHash.Length,_ID.Hash.Length);
            }

            Console.WriteLine(" Name Hash Length  = " + (Identity.NAME_HASH_LENGTH / 8));

            return Util.TruncateHash(addr_hash_material, Identity.NAME_HASH_LENGTH/8);
            
        }


    }
}