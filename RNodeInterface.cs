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

using System;
using System.IO; 
using System.IO.Ports;

namespace RNode
{
	public class RNodeInterface: RNS.Interface
	{
		SerialPort Port;

		int Timeout = 1000;


		//RNode Serial Specs
		int SerialSpeed = 115200;
		int SerialDataBits = 8;
		StopBits SerialStopBits = StopBits.One;
		Parity SerialParity = Parity.None;


		//Stack Specs
		int HW_MTU = 508;

		//Network
		List<byte[]> Packet_Queue = new List<byte[]>();
		string ID_Callsign = "";
		int ID_Interval = 0;
		int Announce_Rate_Target;

		//RNode Fixed Config
		int RSSI_Offset = 157;
		const int FB_Pixel_Width = 64;
		const int FB_Bits_Per_Pixel = 1;
		const int FB_Pixels_Per_Byte = 8 / FB_Bits_Per_Pixel;
		const int FB_Bytes_Per_Line = FB_Pixel_Width / FB_Pixels_Per_Byte;
		const int MAX_CHUNK = 32768;
		const int FREQ_MIN = 137000000;
		const int FREQ_MAX = 1020000000;
		const int CALLSIGN_MAX_LEN = 32;
		const int REQUIRED_FW_VER_MAJ = 1;
		const int REQUIRED_FW_VER_MIN = 52;
		const int RECONNECT_WAIT = 5000;

		//RNode state
		bool isOnline = false;
		bool isDetected = false;
		bool isDetached = false;
		bool isReconnecting = false;
		bool Firmware_OK = false;
		bool Interface_Ready = false;
		bool FlowControl = false;
		DateTime? First_TX;
		Thread ReceiveThread;
		public string Name { get; private set; } = "Undefined Interface";
		RNS.Transport Owner;
		int Last_ID;
		int Reconnect_W;
		bool isValidConfig = false;
		bool should_ID = false;

		//Desired State
		uint Frequency;
		uint Bandwidth;
		byte SF;
		byte CR;
		byte TXPower;
		byte State;

		//Reported State
		int R_Frequency;
		int R_Bandwidth;
		int R_SF;
		byte R_CR;
		byte R_TXPower;
		byte R_State;
		byte R_Lock;
		int R_Stat_RX;
		int R_Stat_TX;
		int R_Stat_RSSI;
		float R_Stat_SNR;
		byte R_Random;
		int Bitrate;
		float Bitrate_kbps;

		int RXB;
		int TXB;

		bool DebugOutput = true;

		//RNode Hard/firmware
		int Maj_Version;
		int Min_Version;
		byte Platform;
		byte MCU;
		bool hasDisplay = false;


		public RNodeInterface(RNS.Transport _owner, string _name, string _port, uint _frequency = 0, uint _bandwidth = 0, byte _txpower = 0, byte _sf = 0, byte _cr = 0, bool _flow_control = false, int _id_interval = 0, string _id_callsign = "")
		{
			if(System.Environment.OSVersion.Platform == PlatformID.Other)
            {
				throw new Exception("Invalid platform. Library currently supports Windows and Linux. If you see this message on either of these platforms, please file a bug report");
            }

			RXB = 0;
			TXB = 0;

			Owner = _owner;
			Name = _name;
			Port = new SerialPort(_port);

			isOnline = false;
			isDetached = false;
			isReconnecting = false;

			Frequency = _frequency;
			Bandwidth = _bandwidth;
			TXPower = _txpower;
			SF = _sf;
			CR = _cr;
			State = KISS.RADIO_STATE_OFF;
			Bitrate = 0;
			Platform = 0x00;
			MCU = 0x00;
			isDetected = false;
			Firmware_OK = false;
			Maj_Version = 0;
			Min_Version = 0;

			Last_ID = 0;
			First_TX = null;
			Reconnect_W = RECONNECT_WAIT;
			
			R_Frequency = 0;
			R_Bandwidth = 0;
			R_TXPower = 0;
			R_SF = 0;
			R_CR = 0;
			R_State = KISS.RADIO_STATE_OFF;
			R_Lock = 0;
			R_Stat_RX = 0;
			R_Stat_TX = 0;
			R_Stat_RSSI	= 0;
			R_Random = 0;

			Packet_Queue.Clear();
			FlowControl = _flow_control;
			Interface_Ready = false;
			Announce_Rate_Target = 0;

			isValidConfig = true;
			if(Frequency < FREQ_MIN || Frequency > FREQ_MAX)
            {
				isValidConfig = false;
				Console.WriteLine("Invalid frequency configured for " + name);
            }
			if (TXPower<0 ||TXPower>17)
			{
				isValidConfig = false;
				Console.WriteLine("Invalid TX power configured for " + name);
			}
			if (Bandwidth < 7800 || Bandwidth > 500000)
			{
				isValidConfig = false;
				Console.WriteLine("Invalid bandwidth configured for " + name);
			}
			if (SF< 7 || SF>12)
			{
				isValidConfig = false;
				Console.WriteLine("Invalid spreading factor configured for " + name);
			}
			if (CR<5 || CR>8)
			{
				isValidConfig = false;
				Console.WriteLine("Invalid coding rate configured for " + name);
			}
			if (_id_interval > 0 && _id_callsign != "")
			{
				if (ID_Callsign.Length <= CALLSIGN_MAX_LEN)
				{
					ID_Callsign = _id_callsign;
					ID_Interval = _id_interval;
					should_ID = true;
				}
				else
				{
					Console.WriteLine("The ID callsign for " + Name + " exceeds the max length of " + CALLSIGN_MAX_LEN.ToString());
					isValidConfig = false;
				}
			}
            else
            {
				ID_Interval = 0;
				ID_Callsign = "";
            }

            if (!isValidConfig)
            {
				throw new ArgumentException("The configuration for " + name + " contains errors; interface offline");
            }

            try
            {
				Port.Open();
                if (Port.IsOpen)
                {
					Configure_Device();
                }
                else
                {
					throw new IOException("Could not open serial port " + Port.PortName);
                }
            }
			catch (Exception e)
            {
				Console.WriteLine("Could not open serial port for interface " + Name);
				Console.WriteLine("Contained exception was: "+e.ToString());
				Console.WriteLine("Will attempt to bring this interface up periodically");
				if(!isDetached && !isReconnecting)
                {
					Thread Reconnect = new Thread(Reconnect_Port);
					Reconnect.Start();
                }
            }



		}

		public bool BIT()
		{
			Console.WriteLine("Beginning Built-In Test");
			Console.WriteLine("Detecting serial ports:");
			DetectRadio();
			return true;
		}

		public bool InitializeSerial(string TargetPort)
		{
			CloseRadio();
			ConfigureSerialPort(ref Port, TargetPort);
			Port.Open();
			return true;
		}

		public void Open_Port()
		{
			Console.WriteLine("Opening serial port " + Port.PortName);
			Port.BaudRate = SerialSpeed;
			Port.DataBits = SerialDataBits;
			Port.Parity = SerialParity;
			Port.StopBits = SerialStopBits;
			Port.RtsEnable = false;
			Port.ReadTimeout = 250;
			Port.WriteTimeout = 250;
		}

		void ConfigureRadio()
		{

		}

		void setFrequency()
		{
			byte c1 = (byte)(Frequency >> 24);
			byte c2 = (byte)((byte)(Frequency >> 16) & 0xFF);
			byte c3 = (byte)((byte)(Frequency >> 8) & 0xFF);
			byte c4 = (byte)((byte)(Frequency) & 0xFF);
			List<byte> data = new List<byte>() { c1, c2, c3, c4 };
			data = KISS.Escape(data);
			byte[] Command = new byte[data.Count + 3];
			for (int i = 2; i < data.Count; i++)
			{
				Command[i] = (byte)data[i - 2];
			}
			Command[0] = KISS.FEND;
			Command[Command.Length - 1] = KISS.FEND;
			Command[1] = KISS.CMD_FREQUENCY;
			try
			{
				Port.Write(Command, 0, Command.Length);
			}
			catch
			{
				throw new IOException("An IO error occurred while configuring frequency for " + Port.PortName);
			}

		}
		void setBandwidth()
		{

			byte c1 = (byte)(Bandwidth >> 24);
			byte c2 = (byte)((byte)(Bandwidth >> 16) & 0xFF);
			byte c3 = (byte)((byte)(Bandwidth >> 8) & 0xFF);
			byte c4 = (byte)((byte)(Bandwidth) & 0xFF);
			List<byte> data = new List<byte>() { c1, c2, c3, c4 };
			data = KISS.Escape(data);
			byte[] Command = new byte[data.Count + 3];
			for (int i = 2; i < data.Count; i++)
			{
				Command[i] = (byte)data[i - 2];
			}
			Command[0] = KISS.FEND;
			Command[Command.Length - 1] = KISS.FEND;
			Command[1] = KISS.CMD_BANDWIDTH;
			try
			{
				Port.Write(Command, 0, Command.Length);
			}
			catch
			{
				throw new IOException("An IO error occurred while configuring bandwidth for " + Port.PortName);
			}
		}
		void setTXPower()
		{
			byte[] Command = new byte[] { KISS.FEND, KISS.CMD_TXPOWER, TXPower, KISS.FEND };
			try
			{
				Port.Write(Command, 0, Command.Length);
			}
			catch
			{
				throw new IOException("An IO error occurred while configuring TX power for " + Port.PortName);
			}
		}
		void setSpreadingFactor()
		{
			byte[] Command = new byte[] { KISS.FEND, KISS.CMD_SF, SF, KISS.FEND };
			try
			{
				Port.Write(Command, 0, Command.Length);
			}
			catch
			{
				throw new IOException("An IO error occurred while configuring spreading factor for " + Port.PortName);
			}
		}
		void setCodingRate()
		{
			byte[] Command = new byte[] { KISS.FEND, KISS.CMD_CR, CR, KISS.FEND };
			try
			{
				Port.Write(Command, 0, Command.Length);
			}
			catch
			{
				throw new IOException("An IO error occurred while configuring coding rate for " + Port.PortName);
			}
		}
		void setRadioState(byte TargetState)
		{
			State = TargetState;
			byte[] Command = new byte[] { KISS.FEND, KISS.CMD_RADIO_STATE, TargetState, KISS.FEND };
			try
			{
				Port.Write(Command, 0, Command.Length);
			}
			catch
			{
				throw new IOException("An IO error occurred while configuring radio state for " + Port.PortName);
			}
		}

		string DetectRadio()
		{
			CloseRadio();
			string[] DetectedPorts = SerialPort.GetPortNames();
			SerialPort TestPort = new SerialPort();
			if (DetectedPorts == null || DetectedPorts.Length == 0) {
				Console.WriteLine("No serial ports detected");
				return null;
			}
			foreach (string DetectedPort in DetectedPorts)
			{
				try
				{
					Console.WriteLine(DetectedPort);
					ConfigureSerialPort(ref TestPort, DetectedPort);
					TestPort.Open();
					Console.WriteLine("Port " + DetectedPort + " opened.");
					Console.WriteLine("Interrogating:");
					SendDetectCommand(TestPort);
					ReceiveTestResponse(TestPort);
					TestPort.Close();
					Console.WriteLine(DetectedPort + " closed.");
				}
				catch
				{
					Console.WriteLine(DetectedPort + " not an RNode or non-functional");
				}

			}
			return null;
		}

		bool SendDetectCommand(SerialPort Ser)
		{
			byte[] Command = { KISS.FEND, KISS.CMD_DETECT, KISS.DETECT_REQ, KISS.FEND, KISS.CMD_FW_VERSION, 0x00, KISS.FEND, KISS.CMD_PLATFORM, 0x00, KISS.FEND, KISS.CMD_MCU, 0x00, KISS.FEND };
			try
			{
				Ser.Write(Command, 0, Command.Length);
				return true;
			}
			catch
			{
				throw new IOException("An IO error occurred while detecting hardware for " + Port.PortName.ToString());
				return false;
			}
		}

		public void Send(byte[] data)
		{
			Process_Outgoing(data);
		}

		void SendFrequencyTest(SerialPort Ser)
		{
			byte[] Command = { KISS.FEND, KISS.CMD_FREQUENCY, 0x36, 0x89, 0xCA, 0xDB, 0xDC, KISS.FEND };
			Console.Write("Frequency: Sending ");
			foreach (byte b in Command)
			{
				Console.Write(b.ToString("X") + " ");
			}
			Console.WriteLine();

		}

		void Leave()
		{
			byte[] Command = new byte[] { KISS.FEND, KISS.CMD_LEAVE, 0xFF, KISS.FEND };
			try
			{
				Port.Write(Command, 0, Command.Length);
			}
			catch
			{
				throw new IOException("An IO error occurred while sending host left command to device");

			}
		}

		void Hard_Reset()
		{
			byte[] Command = new byte[] { KISS.FEND, KISS.CMD_RESET, 0xF8, KISS.FEND };
			try
			{
				Port.Write(Command, 0, Command.Length);
			}
			catch
			{
				throw new IOException("An IO error occurred while restarting device");
			}
			Thread.Sleep(225);
		}

		public void DisableBacklight()
		{
			
			byte[] Command = { KISS.FEND, 0x45, 0x00, KISS.FEND };
			if (!isOnline) return;
			try
			{
				Port.Write(Command, 0, Command.Length);
			}
			catch
			{
				throw new IOException("Cannot disable backlight on " + Port.PortName.ToString());
			}
		}

		public void EnableBacklight()
		{
			
			byte[] Command = { KISS.FEND, 0x45, 0x01, KISS.FEND };
            if (!isOnline)  return;
			try
			{
				Port.Write(Command, 0, Command.Length);
			}
			catch
			{
				throw new IOException("Cannot enable backlight on " + Port.PortName.ToString());
			}
		}

		void ReceiveRawResponse(SerialPort Ser)
		{
			Console.WriteLine("Begin raw receive");
			bool isInFrame = false;
			List<byte> Buffer = new List<byte>();
			DateTime TimeToFail = DateTime.UtcNow;
			TimeToFail = TimeToFail.AddSeconds(2);
			while (Ser.BytesToRead == 0 && TimeToFail > DateTime.UtcNow)
			{
				System.Threading.Thread.Sleep(100);
			}
			while (Ser.BytesToRead > 0)
			{
				byte b = (byte)Ser.ReadByte();
				Console.Write(b.ToString("X") + " ");

			}
			Console.WriteLine();
			Console.WriteLine("End raw receive");
		}

		void ReceiveTestResponse(SerialPort Ser)
		{
			bool isInFrame = false;
			List<byte> Buffer = new List<byte>();
			DateTime TimeToFail = DateTime.UtcNow;
			TimeToFail = TimeToFail.AddSeconds(2);
			while (Ser.BytesToRead == 0 && TimeToFail > DateTime.UtcNow)
			{
				System.Threading.Thread.Sleep(100);
			}
			while (Ser.BytesToRead > 0)
			{
				byte b = (byte)Ser.ReadByte();
				if (!isInFrame && b == 0xc0)
				{
					isInFrame = true;
				}
				else if (isInFrame && b == 0xc0)
				{
					isInFrame = false;
				}
				else if (!isInFrame)
				{
					Console.WriteLine("Error: Byte received out of frame");
				}
				else if (isInFrame)
				{
					if (b == 0x08)
					{
						Console.Write("Detect sent: ");
						if (Ser.ReadByte() == 0x46)
						{
							Console.WriteLine("Proper response received.");
						}
						else
						{
							Console.WriteLine("Inappropriate response received.");
						}

					}

					if (b == 0x50)
					{
						try
						{
							Console.WriteLine("Firmware version: " + Ser.ReadByte().ToString() + "." + Ser.ReadByte().ToString());
						}
						catch
						{
							Console.WriteLine("Error reading firmware version");
						}
					}

					if (b == 0x48)
					{
						Console.Write("Platform: ");
						switch (Ser.ReadByte())
						{
							case 0x80:
								Console.WriteLine("ESP32");
								break;
							case 0x90:
								Console.WriteLine("AVR");
								break;
							default:
								Console.WriteLine("Undefined");
								break;
						}
					}

					if (b == 0x49)
					{
						Console.Write("MCU: ");
						switch (Ser.ReadByte())
						{
							case 0x81:
								Console.WriteLine("ESP32");
								break;
							case 0x91:
								Console.WriteLine("1284P");
								break;
							case 0x92:
								Console.WriteLine("2560");
								break;
							default:
								Console.WriteLine("Undefined");
								break;
						}
					}

					//Console.Write(b);
					//Console.Write(" ");
				}
			}
			Console.WriteLine();
		}

		void ProcessIncoming(List<byte> Payload)
		{
			RXB += Payload.Count;
			if (DebugOutput)
			{
				Console.WriteLine("Data received:");
				foreach (byte b in Payload)
				{
					Console.Write((char)b);
				}
				Console.WriteLine("");
				Console.WriteLine("RSSI: " + R_Stat_RSSI.ToString());
				Console.WriteLine("SNR: " + R_Stat_SNR.ToString());
			}
			//ToDo Add Event
			Callbacks.Process_Inbound(Payload.ToArray(), this);
			R_Stat_RSSI = 0;
			R_Stat_SNR = 0;
		}

		void ConfigureSerialPort(ref SerialPort PortOfInterest, string TargetPortName)
		{
			PortOfInterest = new SerialPort(TargetPortName, SerialSpeed, Parity.None, SerialDataBits, SerialStopBits);
		}


		void UpdateBitrate()
		{
			try
			{
				Bitrate = (int)(R_SF * 1000 * (4.0f / R_CR) / (MathF.Pow(2, R_SF) / (R_Bandwidth / 1000)));
				if (Bitrate <= 0)
				{
					Bitrate = 0;
					return;
				}

				Bitrate_kbps = MathF.Round(Bitrate / 1000.0f, 2);
				Console.WriteLine("On air bitrate is now " + Bitrate_kbps.ToString() + " kbps");
			}
			catch
			{
				Bitrate = 0;
			}
		}

		void Reconnect_Port()
        {
			isReconnecting = true;
			while (!isOnline && !isDetached)
			{
				try
				{
					Thread.Sleep(5000);
					Console.WriteLine("Attempting to reconnect serial port " + Port.PortName + " for " + Name);
					Port.Open();
					if (Port.IsOpen)
					{
						Configure_Device();
					}
				}

				catch (Exception ex)
				{
					Console.WriteLine("Error while reconnecting port. The contained exception was: "+ex.Message);
				}
			}
			isReconnecting = false;
            if (isOnline)
            {
				Console.WriteLine("Reconnected port "+Port.PortName+" for "+Name);
            }
                
            
        }

		public void CloseRadio()
		{
			if (Port == null) { return; }
			if (Port.IsOpen)
			{
				Port.Close();
			}
		}

		public void Queue(byte[] Payload)
		{
			Packet_Queue.Add(Payload);
		}

		public void Force_Queue()
		{
			Process_Queue();
		}

		public void Configure_Device()
        {
			R_Frequency = 0;
			R_Bandwidth = 0;
			R_TXPower = 0;
			R_SF = 0;
			R_CR = 0;
			R_State = 0;
			R_Lock = 0;

			Port.BaudRate = SerialSpeed;
			Port.DataBits = SerialDataBits;
			Port.Parity = SerialParity;
			Port.StopBits = SerialStopBits;
			Port.RtsEnable = false;
			Port.ReadTimeout = 250;
			Port.WriteTimeout = 250;

			Thread.Sleep(2000);

			ReceiveThread = new Thread(ReceiveLoop);
			ReceiveThread.Start();

			Detect();
			Thread.Sleep(200);

            if (!isDetected)
            {
				Console.WriteLine("Could not detect device for " + Port.PortName);
				Port.Close();
				return;
            }
            else
            {
				if(Platform == KISS.PLATFORM_ESP32) { hasDisplay = true; }
            }
			Console.WriteLine("Serial port " + Port.PortName + " is now open");
			Console.WriteLine("Configuring RNode interface...");
			InitRadio();
            if (ValidateRadioState())
            {
				Interface_Ready = true;
				Console.WriteLine(Name+" is configured and powered up");
				Thread.Sleep(300);
				isOnline = true;
            }
            else
            {
				Console.WriteLine("After configuring " + Name + " the reported radio parameters did not match your configuration.");
				Console.WriteLine("Make sure that your hardware actually supports the parameters specified in the configuration");
				Console.WriteLine("Aborting RNode startup");
				Port.Close();
				return;
            }
		}

		public void Send_Plaintext(string Payload)
		{

			Queue(System.Text.Encoding.ASCII.GetBytes(Payload));

		}

		void Process_Queue() //debug
		{
			if (Packet_Queue.Count > 0)
			{
				//Console.WriteLine("I have a packet!");
				byte[] data = Packet_Queue[0];
				Packet_Queue.RemoveAt(0);
				Interface_Ready = true;
				Process_Outgoing(data);
			}
			else if (Packet_Queue.Count == 0)
			{
				Interface_Ready = true;
			}
		}

		void Process_Outgoing(byte[] Payload)
		{
			int DataLen = Payload.Length;
			if (isOnline)
			{
				//Console.WriteLine("Online!");
				if (Interface_Ready)
				{
					if (FlowControl)
					{
						Interface_Ready = false;
					}
					if (Payload.SequenceEqual(UTF8_to_Bytes(ID_Callsign)))
					{
						First_TX = null;
					}
					else
					{
						if (First_TX == null)
						{
							First_TX = DateTime.UtcNow;
						}
					}
					int datalen = Payload.Length;
					Payload = KISS.Escape(Payload.ToList<byte>()).ToArray();
					byte[] Output = new byte[Payload.Length + 3];
					Payload.CopyTo(Output, 2);
					Output[0] = 0xC0;
					Output[Output.Length - 1] = 0xC0;
					try
					{
						Port.Write(Output, 0, Output.Length);
						TXB += datalen;
					}
					catch
					{
						throw new IOException("Process_Outgoing has failed.");
					}

				}
				else
				{
					Queue(Payload);
				}
			}
		}


		public void ReceiveLoop()
		{
			Console.WriteLine("ReceiveLoop running");
			if (Port == null || !Port.IsOpen)
			{
				Console.WriteLine("Port is not properly initialized");
				return;
			}
			bool isInFrame = false;
			bool Escape = false;
			byte Command = KISS.CMD_UNKNOWN;
			byte Buffer;
			List<byte> DataBuffer = new List<byte>();
			List<byte> CommandBuffer = new List<byte>();
			DateTime Last_Read = DateTime.UtcNow;
			while (Port.IsOpen)
			{
				if (Port.BytesToRead > 0)
				{
					Buffer = (byte)Port.ReadByte();
					if (isInFrame && Buffer == KISS.FEND && Command == KISS.CMD_DATA)
					{
						isInFrame = false;
						ProcessIncoming(DataBuffer);
						DataBuffer.Clear();
						CommandBuffer.Clear();
					}
					else if (Buffer == KISS.FEND)
					{
						isInFrame = true;
						Command = KISS.CMD_UNKNOWN;
						DataBuffer.Clear();
						CommandBuffer.Clear();
					}
					else if (isInFrame && DataBuffer.Count < HW_MTU)
					{
						if (DataBuffer.Count == 0 && Command == KISS.CMD_UNKNOWN) { Command = Buffer; }
						else if (Command == KISS.CMD_DATA)
						{
							if (Buffer == KISS.FESC) { Escape = true; }
							else
							{
								if (Escape)
								{
									if (Buffer == KISS.TFEND) { Buffer = KISS.FEND; }
									if (Buffer == KISS.TFESC) { Buffer = KISS.FESC; }
									Escape = false;
								}
								DataBuffer.Add(Buffer);
							}
						}
						else if (Command == KISS.CMD_FREQUENCY)
						{
							if (Buffer == KISS.FESC) { Escape = true; }
							else
							{
								if (Escape)
								{
									if (Buffer == KISS.TFEND) { Buffer = KISS.FEND; }
									if (Buffer == KISS.TFESC) { Buffer = KISS.FESC; }
									Escape = false;
								}
								CommandBuffer.Add(Buffer);
								if (CommandBuffer.Count == 4)
								{
									R_Frequency = CommandBuffer[0] << 24 | CommandBuffer[1] << 16 | CommandBuffer[2] << 8 | CommandBuffer[3];
									Console.WriteLine("Radio reporting frequency is " + (R_Frequency / 1000000.0).ToString() + "MHz");
									UpdateBitrate();
								}
							}
						}
						else if (Command == KISS.CMD_BANDWIDTH)
						{
							if (Buffer == KISS.FESC) { Escape = true; }
							else
							{
								if (Escape)
								{
									if (Buffer == KISS.TFEND) { Buffer = KISS.FEND; }
									if (Buffer == KISS.TFESC) { Buffer = KISS.FESC; }
									Escape = false;
								}
								CommandBuffer.Add(Buffer);
								if (CommandBuffer.Count == 4)
								{
									R_Bandwidth = CommandBuffer[0] << 24 | CommandBuffer[1] << 16 | CommandBuffer[2] << 8 | CommandBuffer[3];
									Console.WriteLine("Radio reporting bandwidth is " + (R_Bandwidth / 1000.0).ToString() + "KHz");
									UpdateBitrate();
								}
							}
						}
						else if (Command == KISS.CMD_TXPOWER)
						{
							R_TXPower = Buffer;
							Console.WriteLine("Radio reporting TX power is " + R_TXPower.ToString() + " dBm");
						}
						else if (Command == KISS.CMD_SF)
						{
							R_SF = Buffer;
							Console.WriteLine("Radio reporting spreading factor is " + R_SF.ToString());
						}
						else if (Command == KISS.CMD_CR)
						{
							R_CR = Buffer;
							Console.WriteLine("Radio reporting coding rate is " + R_CR.ToString());
						}
						else if (Command == KISS.CMD_RADIO_STATE)
						{
							R_State = Buffer;
							if (R_State == 0) {
								Console.WriteLine(Name + ": Radio reporting state is offline.");
							}
						}
						else if (Command == KISS.CMD_RADIO_LOCK)
						{
							R_Lock = Buffer;
						}
						else if (Command == KISS.CMD_FW_VERSION)
						{
							if (Buffer == KISS.FESC) { Escape = true; }
							else
							{
								if (Escape)
								{
									if (Buffer == KISS.TFEND) { Buffer = KISS.FEND; }
									if (Buffer == KISS.TFESC) { Buffer = KISS.FESC; }
									Escape = false;
								}
								CommandBuffer.Add(Buffer);
								if (CommandBuffer.Count == 2)
								{
									Maj_Version = (int)CommandBuffer[0];
									Min_Version = (int)CommandBuffer[1];
									Validate_Firmware();
								}

							}
						}
						else if (Command == KISS.CMD_STAT_RX)
						{
							if (Buffer == KISS.FESC) { Escape = true; }
							else
							{
								if (Escape)
								{
									if (Buffer == KISS.TFEND) { Buffer = KISS.FEND; }
									if (Buffer == KISS.TFESC) { Buffer = KISS.FESC; }
									Escape = false;
								}
								CommandBuffer.Add(Buffer);
								if (CommandBuffer.Count == 4)
								{
									R_Stat_RX = CommandBuffer[0] << 24 | CommandBuffer[1] << 16 | CommandBuffer[2] << 8 | CommandBuffer[3];
								}
							}
						}
						else if (Command == KISS.CMD_STAT_TX)
						{
							if (Buffer == KISS.FESC) { Escape = true; }
							else
							{
								if (Escape)
								{
									if (Buffer == KISS.TFEND) { Buffer = KISS.FEND; }
									if (Buffer == KISS.TFESC) { Buffer = KISS.FESC; }
									Escape = false;
								}
								CommandBuffer.Add(Buffer);
								if (CommandBuffer.Count == 4)
								{
									R_Stat_TX = CommandBuffer[0] << 24 | CommandBuffer[1] << 16 | CommandBuffer[2] << 8 | CommandBuffer[3];
								}
							}
						}
						else if (Command == KISS.CMD_STAT_RSSI)
						{
							R_Stat_RSSI = Buffer - RSSI_Offset;
						}
						else if (Command == KISS.CMD_STAT_SNR)
						{
							R_Stat_SNR = (float)(sbyte)Buffer / 4.0f;
						}
						else if (Command == KISS.CMD_RANDOM)
						{
							R_Random = Buffer;
						}
						else if (Command == KISS.CMD_PLATFORM)
						{
							Platform = Buffer;
						}
						else if (Command == KISS.CMD_MCU)
						{
							MCU = Buffer;
						}
						else if (Command == KISS.CMD_ERROR)
						{
							if (Buffer == KISS.ERROR_INITRADIO)
							{
								throw new IOException("Radio initialization failure");
							}
							else if (Buffer == KISS.ERROR_TXFAILED)
							{
								throw new IOException("Hardware transmit failure");
							}
							else
							{
								throw new IOException("Unknown hardware failure");
							}
						}
						else if (Command == KISS.CMD_RESET)
						{
							if (Buffer == 0xF8 && Platform == KISS.PLATFORM_ESP32 && isOnline)
							{
								throw new IOException("ESP32 reset while device online");
							}
						}
						else if (Command == KISS.CMD_READY)
						{
							Console.WriteLine("Received CMD_READY");
							Process_Queue();
						}
						else if (Command == KISS.CMD_DETECT)
						{
							if (Buffer == KISS.DETECT_RESP)
							{
								isDetected = true;
							}
							else
							{
								isDetected = false;
							}
						}
					}
				}
				else
				{
					if (DataBuffer.Count > 0 && DateTime.UtcNow > Last_Read.AddMilliseconds(Timeout))
					{
						Console.WriteLine("Serial Read Timeout");
						DataBuffer.Clear();
						isInFrame = false;
						Command = KISS.CMD_UNKNOWN;
						Escape = false;
					}

					if (ID_Interval > 0 && ID_Callsign != null)
                    {
						if(First_TX != null)
                        {
							if (DateTime.UtcNow > First_TX.Value.AddSeconds(ID_Interval))
                            {
								Console.WriteLine("Interface " + Name + " is transmitting beacon data: " + ID_Callsign.ToString());
								Process_Outgoing(UTF8_to_Bytes(ID_Callsign));
                            }
                        }
                    }

					System.Threading.Thread.Sleep(80);
				}
			}
			Console.WriteLine("Receive loop terminated.");

		}

		byte[] UTF8_to_Bytes(string Input)
        {
			return System.Text.Encoding.UTF8.GetBytes(Input);
        }

		public void Detect()
		{
			isDetected = SendDetectCommand(Port);
		}

		void Validate_Firmware()
        {
			if(Maj_Version>=REQUIRED_FW_VER_MAJ && Min_Version >= REQUIRED_FW_VER_MIN)
            {
				Firmware_OK = true;
				return;
            }

			Console.WriteLine("The firmware version of the connected RNode is "+Maj_Version.ToString()+ "."+Min_Version.ToString());
			Console.WriteLine("This library requires at least version " + Maj_Version.ToString() + "." + Min_Version.ToString());
			Console.WriteLine("Please update your RNode firmware with rnodeconf from https://github.com/markqvist/rnodeconfigutil/");
			//ToDo RNS.Panic();
        }

		public bool ValidateRadioState()
		{
			isValidConfig = true;
			Console.WriteLine("Waiting for config validation");
			System.Threading.Thread.Sleep(250);
			if (R_Frequency > 0 && Math.Abs(R_Frequency - Frequency) > 500)
			{
				isValidConfig = false;
				Console.WriteLine("Frequency mismatch");
				Console.WriteLine("Expected " + Frequency.ToString() + " - got " + R_Frequency.ToString());
			}
			if (R_Bandwidth != Bandwidth)
			{
				isValidConfig = false;
				Console.WriteLine("Bandwidth mismatch");
				Console.WriteLine("Expected " + Bandwidth.ToString() + " - got " + R_Bandwidth.ToString());
			}
			if (R_TXPower != TXPower)
			{
				isValidConfig = false;
				Console.WriteLine("TX power mismatch"); 
				Console.WriteLine("Expected " + TXPower.ToString() + " - got " + R_TXPower.ToString());
			}
			if (R_SF != SF)
			{
				isValidConfig = false;
				Console.WriteLine("Spreading factor mismatch");
				Console.WriteLine("Expected " + SF.ToString() + " - got " + R_SF.ToString());
			}
			if (R_State != State)
			{
				isValidConfig = false;
				Console.WriteLine("Radio state mismatch");
			}
			return isValidConfig;
		}

		public void InitRadio()
		{
			setFrequency();
			setBandwidth();
			setTXPower();
			setSpreadingFactor();
			setCodingRate();
			setRadioState(KISS.RADIO_STATE_ON);
		}

		public void DefineRatio(uint CenterFrequency, uint BW, byte Code_Rade, byte SpreadFactor, byte TX_Power)
		{
			Frequency = CenterFrequency;
			Bandwidth = BW;
			SF = SpreadFactor;
			CR = Code_Rade;
			TXPower = TX_Power;

		}

		public void Detach()
		{
			isDetached = true;
			Disable_External_Framebuffer();
			setRadioState(KISS.RADIO_STATE_OFF);
			Leave();
		}

		public void Disable_External_Framebuffer()
		{
			if (hasDisplay && isOnline)
			{
				byte[] Command = new byte[] { KISS.FEND, KISS.CMD_FB_EXT, 0x00, KISS.FEND };
				try
				{
					Port.Write(Command, 0, Command.Length);
				}
				catch
				{
					throw new IOException("An IO error occurred while disabling external framebuffer on device");
				}
			}
		}

		public void Enable_External_Framebuffer()
		{
			if (hasDisplay && isOnline)
			{
				byte[] Command = new byte[] { KISS.FEND, KISS.CMD_FB_EXT, 0x01, KISS.FEND };
				try
				{
					Port.Write(Command, 0, Command.Length);
				}
				catch
				{
					throw new IOException("An IO error occurred while enabling external framebuffer on device");
				}
			}
		}

		public void Display_Image(byte[] ImageData)
		{
			if (hasDisplay)
			{
				int lines = ImageData.Length / 8;
				for (int i = 0; i < lines; i++)
				{

					int LineStart = i * FB_Bytes_Per_Line;
					int LineEnd = LineStart + FB_Bytes_Per_Line;
					byte[] LineData = new byte[FB_Bytes_Per_Line];
					Array.Copy(ImageData, LineStart, LineData, 0, FB_Bytes_Per_Line);
					Write_Framebuffer(i, LineData);
				}
			}
		}

		void Write_Framebuffer(int Line, byte[] LineData)
		{
			if (hasDisplay && isOnline)
			{

				LineData = KISS.Escape(LineData.ToList()).ToArray();
				byte[] Command = new byte[LineData.Length + 4];
				Array.Copy(LineData, 0, Command, 3, LineData.Length);
				Command[2] = (byte)Line;
				Command[0] = KISS.FEND;
				Command[1] = KISS.CMD_FB_WRITE;
				Command[Command.Length - 1] = KISS.FEND;
				try
				{
					Port.Write(Command, 0, Command.Length);
				}
				catch
				{
					throw new IOException("An IO error occurred while writing framebuffer data");
				}

			}
		}

		class KISS
		{
			public const byte FEND = 0xC0;
			public const byte FESC = 0xDB;
			public const byte TFEND = 0xDC;
			public const byte TFESC = 0xDD;
			public const byte CMD_UNKNOWN = 0xFE;
			public const byte CMD_DATA = 0x00;
			public const byte CMD_FREQUENCY = 0x01;
			public const byte CMD_BANDWIDTH = 0x02;
			public const byte CMD_TXPOWER = 0x03;
			public const byte CMD_SF = 0x04;
			public const byte CMD_CR = 0x05;
			public const byte CMD_RADIO_STATE = 0x06;
			public const byte CMD_RADIO_LOCK = 0x07;
			public const byte CMD_DETECT = 0x08;
			public const byte CMD_LEAVE = 0x0A;
			public const byte CMD_READY = 0x0F;
			public const byte CMD_STAT_RX = 0x21;
			public const byte CMD_STAT_TX = 0x22;
			public const byte CMD_STAT_RSSI = 0x23;
			public const byte CMD_STAT_SNR = 0x24;
			public const byte CMD_BLINK = 0x30;
			public const byte CMD_RANDOM = 0x40;
			public const byte CMD_FB_EXT = 0x41;
			public const byte CMD_FB_READ = 0x42;
			public const byte CMD_FB_WRITE = 0x43;
			public const byte CMD_PLATFORM = 0x48;
			public const byte CMD_MCU = 0x49;
			public const byte CMD_FW_VERSION = 0x50;
			public const byte CMD_ROM_READ = 0x51;
			public const byte CMD_RESET = 0x55;
			public const byte DETECT_REQ = 0x73;
			public const byte DETECT_RESP = 0x46;
			public const byte RADIO_STATE_OFF = 0x00;
			public const byte RADIO_STATE_ON = 0x01;
			public const byte RADIO_STATE_ASK = 0xFF;
			public const byte CMD_ERROR = 0x90;
			public const byte ERROR_INITRADIO = 0x01;
			public const byte ERROR_TXFAILED = 0x02;
			public const byte ERROR_EEPROM_LOCKED = 0x03;
			public const byte PLATFORM_AVR = 0x90;
			public const byte PLATFORM_ESP32 = 0x80;

			


	

			public static List<byte> Escape(List<byte> Input)
			{
				List<byte> Output = new List<byte>();
				foreach (byte b in Input)
				{
					if (b == 0xDB)
					{
						Output.Add(0xDB);
						Output.Add(0xDD);
					}
					else if (b == 0xC0)
					{
						Output.Add(0xDB);
						Output.Add(0xDC);
					}
					else
					{
						Output.Add(b);
					}
				}
				return Output;
			}
		}
	}


}