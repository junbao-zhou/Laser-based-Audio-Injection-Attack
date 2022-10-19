using System;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Runtime.InteropServices;

namespace C__Serial
{
	static class Program
	{
		public static SerialPort serialPort = null;

		public static SerialPort openSerialPort(string comName, int baud)
		{
			serialPort = new SerialPort();
			serialPort.PortName = comName;
			serialPort.BaudRate = baud;
			serialPort.DataBits = 8;
			serialPort.StopBits = StopBits.One;
			serialPort.Parity = Parity.None;
			serialPort.Open();
			// serialPort.DataReceived += recieveData;
			Console.WriteLine("Open Serial : " + comName);
			return serialPort;
		}
		static void Main(string[] args)
		{
			Console.WriteLine("Avaiable COMs : ");
			foreach (string s in SerialPort.GetPortNames())
			{
				Console.WriteLine(s);
			}
			Console.Write("Please selecte a COM : ");
			string s_in = Console.ReadLine();
			Console.WriteLine("You've selected COM : " + s_in);
			SerialPort serial = openSerialPort(s_in, 115200);

			const int max = 0xFF;
			const int screen = 255;
			new Task(() =>
			{
				while (true)
				{
					byte i = Convert.ToByte(Console.ReadKey().KeyChar);
					serialPort.Write(new byte[] { i }, 0, 1);
				}
			}).Start();
			while (true)
			{
				int b1 = serialPort.ReadByte();
				// int b2 = serialPort.ReadByte();
				// int num = b1 | (b2 << 8);
				int num = b1;
				int count = screen * num / max;
				Console.WriteLine(new String(' ', Math.Min(Math.Max(count, 0), screen)) + "|" + new String(' ', Math.Min(Math.Max(screen - count, 0), screen)) + "|" + num);
				// Console.WriteLine(Convert.ToUInt32(b1));
				// Console.WriteLine("recieve : " + Convert.ToString(b, 16));
			}
		}

	}
}
