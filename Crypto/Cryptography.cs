using Org.BouncyCastle.Math.EC.Rfc8032;
using Org.BouncyCastle.Crypto;
using Org.BouncyCastle.Crypto.Signers;
using Org.BouncyCastle.Crypto.Parameters;
using Org.BouncyCastle.Security;
using Org.BouncyCastle.Crypto.Generators;
using Org.BouncyCastle.Crypto.Paddings;
using Org.BouncyCastle.Crypto.Engines;
using System.Text;

namespace RNS
{
    public static class Cryptography
    {
        public class Fernet
        {
            // 
            // !!!WARNING!!!
            // This is a modified implementation of Fernet. Do not attempt
            // to use for standard Fernet implementations or replace with
            // a Fernet library.
            //
            // Typical Fernet has the following structure, base64url encoded:
            //     1 byte version code
            //     8 byte timestamp
            //     16 byte AES-128 IV
            //     16 byte * n data blocks
            //     32 byte HMAC
            //
            // Custom implementation omits version and timestamp, saving 9 bytes.
            //    16 byte AES-128 IV
            //    16 byte * n data blocks
            //    32 byte HMAC
            //
            // This results in an overhead of 48 bytes (IV+HMAC) instead of 
            // 57 bytes (Version+Timestamp+IV+HMAC)
            // 
            public const int FERNET_OVERHEAD = 48; //Bytes
        }

        public static byte[] SHA256(byte[] data)
        {
            return System.Security.Cryptography.SHA256.HashData(data);
        }

        public static byte[] SHA512(byte[] data)
        {
            return System.Security.Cryptography.SHA512.HashData(data);
        }

        //public class HMAC
        //{
            
        //}

        public static byte[] HMAC256(byte[] key, byte[] message)
        {
            Org.BouncyCastle.Crypto.Macs.HMac hmac = new Org.BouncyCastle.Crypto.Macs.HMac(new Org.BouncyCastle.Crypto.Digests.Sha256Digest());
            hmac.Init(new KeyParameter(key));
            byte[] result = new byte[hmac.GetMacSize()];
            hmac.BlockUpdate(message, 0, message.Length);
            hmac.DoFinal(result, 0);
            return result;
        }

        public static byte[] HMAC512(byte[] key, byte[] message)
        {
            Org.BouncyCastle.Crypto.Macs.HMac hmac = new Org.BouncyCastle.Crypto.Macs.HMac(new Org.BouncyCastle.Crypto.Digests.Sha512Digest());
            hmac.Init(new KeyParameter(key));
            byte[] result = new byte[hmac.GetMacSize()];
            hmac.BlockUpdate(message, 0, message.Length);
            hmac.DoFinal(result, 0);
            return result;
        }

        public static class AES_128_CBC
        {
            public static byte[] encrypt(byte[] plaintext, byte[] key, byte[] iv)
            {
                ICipherParameters Param = new KeyParameter(key);
                ParametersWithIV ParamIV = new ParametersWithIV(Param, iv);
                Org.BouncyCastle.Crypto.IBufferedCipher cipher = Org.BouncyCastle.Security.CipherUtilities.GetCipher("AES/CBC/PKCS7Padding");
                cipher.Init(true, ParamIV);
                return cipher.DoFinal(plaintext, 0, plaintext.Length);
            }

            public static byte[] decrypt(byte[] ciphertext, byte[] key, byte[] iv)
            {
                ICipherParameters Param = new KeyParameter(key);
                ParametersWithIV ParamIV = new ParametersWithIV(Param, iv);
                Org.BouncyCastle.Crypto.IBufferedCipher cipher = Org.BouncyCastle.Security.CipherUtilities.GetCipher("AES/CBC/PKCS7Padding");
                cipher.Init(false, ParamIV);
                return cipher.DoFinal(ciphertext, 0, ciphertext.Length);
            }
        }

        public class Ed25519PrivateKey
        {
            byte[] seed;
            Ed25519PrivateKeyParameters sk;
            Ed25519PublicKeyParameters vk;

            public Ed25519PrivateKey(byte[] _seed)
            {
                seed = _seed;
                sk = new Ed25519PrivateKeyParameters(seed,0);
                vk = sk.GeneratePublicKey();
                if (sk == null) { Console.WriteLine("The Signing key is still null"); }


                
            }

            public Ed25519PrivateKey()
            {
                seed = System.Security.Cryptography.RandomNumberGenerator.GetBytes(32);
                from_private_bytes(seed);
            }

            public void BIT()
            {
                Ed25519PrivateKeyParameters Prv = new Ed25519PrivateKeyParameters(seed,0);
                Ed25519PublicKeyParameters Pub = Prv.GeneratePublicKey();
                byte[] message = new byte[] { (byte)'a',(byte)'b',(byte)'c' };
                ISigner sig = SignerUtilities.GetSigner("Ed25519");
                sig.Init(true, Prv);
                sig.BlockUpdate(message, 0, message.Length);
                byte[] signature =  sig.GenerateSignature();
                sig.Init(false, Pub);
                sig.BlockUpdate(message, 0, message.Length);
                Console.WriteLine(sig.VerifySignature(signature));
                signature = sign(message);
                sig.Init(false, Pub);
                sig.BlockUpdate(message, 0, message.Length);
                Console.WriteLine(sig.VerifySignature(signature));

            }

            public void generate()
            {
                seed = System.Security.Cryptography.RandomNumberGenerator.GetBytes(32);
                from_private_bytes(seed);
            }

            public void from_private_bytes(byte[] _seed)
            {
                seed = _seed;
                
                sk = new Ed25519PrivateKeyParameters(seed, 0);
                vk = sk.GeneratePublicKey();
            }

            public Ed25519PublicKeyParameters public_key()
            {
                vk = sk.GeneratePublicKey();

                return vk;
            }

            public byte[] sign(byte[] message)
            {
                //byte[] buffer;
                ISigner sig = SignerUtilities.GetSigner("Ed25519");
                sig.Init(true, sk);
                if (sk == null) { Console.WriteLine("The Signing key is still null and is signing!"); }
                sig.BlockUpdate(message, 0, message.Length);
                return sig.GenerateSignature();
            }

        }

        public class Ed25519PublicKey
        {
            byte[] seed;
            public Ed25519PublicKeyParameters vk;

            public Ed25519PublicKey(byte[] _seed)
            {
                seed= _seed;
                vk = new Ed25519PublicKeyParameters(_seed,0);
            }

            public Ed25519PublicKey(Ed25519PublicKeyParameters _vk)
            {
                vk= _vk;
            }

            public bool verify(byte[] signature, byte[] message)
            {
                if (vk == null) { Console.WriteLine("The Verifying key is still null"); }
                ISigner sig = SignerUtilities.GetSigner("Ed25519");
                sig.Init(false, vk);
                sig.BlockUpdate(message, 0, message.Length);
                return sig.VerifySignature(signature);
                
            }

            public void SET(Ed25519PublicKeyParameters Pub)
            {
                vk= Pub;
            }

            public void BIT()
            {
                Ed25519PrivateKeyParameters Prv = new Ed25519PrivateKeyParameters(System.Security.Cryptography.RandomNumberGenerator.GetBytes(32), 0);
                Ed25519PublicKeyParameters Pub = Prv.GeneratePublicKey();
                vk = Pub;
                byte[] message = new byte[] { (byte)'a', (byte)'b', (byte)'c' };
                ISigner sig = SignerUtilities.GetSigner("Ed25519");
                sig.Init(true, Prv);
                sig.BlockUpdate(message, 0, message.Length);
                byte[] signature = sig.GenerateSignature();
                sig.Init(false, Pub);
                sig.BlockUpdate(message, 0, message.Length);
                Console.WriteLine(sig.VerifySignature(signature));
                Console.WriteLine(verify(signature, message));
                if (verify(signature, message))
                {
                    Console.WriteLine("Ed25519 signature valid");

                }
                else
                {

                    Console.WriteLine("Ed25519 signature invalid");

                }
            }
        }




        // Testing and scratchpad. To be removed.
        public static void TestMe()
        {

            IAsymmetricCipherKeyPairGenerator g = new Ed25519KeyPairGenerator();
            ISigner sig = SignerUtilities.GetSigner("Ed25519");
            KeyGenerationParameters P = new Ed25519KeyGenerationParameters(new SecureRandom());
            g.Init(P);
            //IAsymmetricCipherKeyPairGenerator g = GeneratorUtilities.GetKeyPairGenerator("Ed25519");
            AsymmetricCipherKeyPair kp = g.GenerateKeyPair();
            AsymmetricKeyParameter sKey = kp.Private;
            AsymmetricKeyParameter vKey = kp.Public;

            sig.Init(true, sKey);
            byte[] data = new byte[] { (byte)'a', (byte)'b', (byte)'c', (byte)'d', (byte)'e', (byte)'f', (byte)'g', (byte)'h'};
            sig.BlockUpdate(data, 0, data.Length);
            byte[] sigBytes = sig.GenerateSignature();
            sig.Init(false, vKey);
            sig.BlockUpdate(data, 0, data.Length);
            if (!sig.VerifySignature(sigBytes))
            {
                Console.WriteLine("Verification Failed");
            }
            else
            {
                Console.WriteLine("Verification Passed");
            }
            Org.BouncyCastle.Crypto.IBufferedCipher Cyp = Org.BouncyCastle.Security.CipherUtilities.GetCipher("AES/CBC/PKCS5Padding");
            

            ICipherParameters Param = new KeyParameter(new byte[] {0x00, 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f });
            ParametersWithIV ParamIV = new ParametersWithIV(Param, new byte[] { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
            Cyp.Init(true,ParamIV);
            byte[] vomit = Cyp.DoFinal(data, 0, data.Length);
            foreach (byte b in vomit)
            {
                Console.Write(b.ToString("X"));
            }
            Console.WriteLine();
            Cyp.Init(false, ParamIV);
            vomit = Cyp.DoFinal(vomit, 0, vomit.Length);
            foreach(byte b in vomit)
            {
                Console.Write(b.ToString("X"));
            }
            Console.WriteLine();
            
            



        }




        //public class BCEngine
        //{
        //    private readonly Encoding _encoding;
        //    private readonly IBlockCipher _blockCipher;
        //    private PaddedBufferedBlockCipher _cipher;
        //    private IBlockCipherPadding _padding;

        //    public BCEngine(IBlockCipher blockCipher, Encoding encoding)
        //    {
        //        _blockCipher = blockCipher;
        //        _encoding = encoding;
        //    }

        //    public void SetPadding(IBlockCipherPadding padding)
        //    {
        //        if (padding != null)
        //            _padding = padding;
        //    }

        //    public string Encrypt(string plain, string key)
        //    {
        //        byte[] result = BouncyCastleCrypto(true, _encoding.GetBytes(plain), key);
        //        return Convert.ToBase64String(result);
        //    }

        //    public string Decrypt(string cipher, string key)
        //    {
        //        byte[] result = BouncyCastleCrypto(false, Convert.FromBase64String(cipher), key);
        //        return _encoding.GetString(result);
        //    }




            //private byte[] BouncyCastleCrypto(bool forEncrypt, byte[] input, string key)
            //{
            //    try
            //    {
            //        _cipher = _padding == null ? new PaddedBufferedBlockCipher(_blockCipher) : new PaddedBufferedBlockCipher(_blockCipher, _padding);
            //        byte[] keyByte = _encoding.GetBytes(key);
            //        _cipher.Init(forEncrypt, new KeyParameter(keyByte));
            //        return _cipher.DoFinal(input);
            //    }
            //    catch (Org.BouncyCastle.Crypto.CryptoException ex)
            //    {
            //        throw new CryptoException("Exception", ex);
            //    }
            //}

            //public string AESEncryption(string plain, string key, bool fips)
            //{
            //    BCEngine bcEngine = new BCEngine(new AesEngine(), _encoding);
            //    bcEngine.SetPadding(_padding);
            //    return bcEngine.Encrypt(plain, key);
            //}

            //public string AESDecryption(string cipher, string key, bool fips)
            //{
            //    BCEngine bcEngine = new BCEngine(new AesEngine(), _encoding);
            //    bcEngine.SetPadding(_padding);
            //    return bcEngine.Decrypt(cipher, key);
            //}
        //}

        public enum DigestType
        {
            SHA256,
            SHA512
        }
    }

    public class CryptoEngine
    {
        private readonly IBlockCipher _blockCipher;
        private PaddedBufferedBlockCipher _cipher;
        private IBlockCipherPadding _padding;
    }

}
