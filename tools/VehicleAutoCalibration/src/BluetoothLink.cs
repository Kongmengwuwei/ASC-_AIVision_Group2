using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Diagnostics;
using System.Reflection;
using System.Text;
using System.Threading;

internal sealed class BluetoothLink : IDisposable
{
    private readonly SerialPort port;
    private Stream bluetoothStream;
    private object bluetoothClient;
    private Thread bluetoothReader;
    private readonly string bluetoothAddress;
    private readonly bool externalBluetooth;
    private bool autoStopOnDispose = true;
    private readonly object transportSync = new object();
    private volatile bool disposing;
    private volatile bool readerStop;
    private readonly object sync = new object();
    private readonly StringBuilder received = new StringBuilder();
    private readonly AutoResetEvent receivedEvent = new AutoResetEvent(false);

    public BluetoothLink(string portName)
    {
        if (portName.StartsWith("BTH-INPROC:", StringComparison.OrdinalIgnoreCase))
        {
            bluetoothAddress = portName.Substring(11).Replace(":", string.Empty).Replace("-", string.Empty);
            autoStopOnDispose = false;
            ResumeBluetooth();
            return;
        }
        if (portName.StartsWith("BTH:", StringComparison.OrdinalIgnoreCase))
        {
            bluetoothAddress = portName.Substring(4).Replace(":", string.Empty).Replace("-", string.Empty);
            externalBluetooth = true;
            autoStopOnDispose = false;
            return;
        }

        port = new SerialPort(portName, 115200, Parity.None, 8, StopBits.One);
        port.Handshake = Handshake.None;
        port.DtrEnable = false;
        port.RtsEnable = false;
        port.ReadTimeout = 300;
        port.WriteTimeout = 1000;
        port.Encoding = Encoding.ASCII;
        port.DataReceived += OnDataReceived;
        port.Open();
        Thread.Sleep(300);
        port.DiscardInBuffer();
        port.DiscardOutBuffer();
    }

    public string PortName
    {
        get { return port != null ? port.PortName : "Bluetooth RFCOMM"; }
    }

    public static string[] GetAvailablePorts()
    {
        string[] ports = SerialPort.GetPortNames();
        Array.Sort(ports, StringComparer.OrdinalIgnoreCase);
        return ports;
    }

    public void VerifyFirmware()
    {
        ClearReceived();
        Send("status");
        string response = WaitForText("STATUS", TimeSpan.FromSeconds(3));
        if (response == null)
        {
            throw new InvalidOperationException(
                "No STATUS reply. The selected COM port is not the car Bluetooth link, " +
                "or the calibration firmware has not been flashed.");
        }
    }

    public void VerifyCalibrationInterface()
    {
        ClearReceived();
        Set("start.delay", 0.0);
        string response = WaitForText("OK start.delay", TimeSpan.FromSeconds(2));
        if (response == null)
        {
            throw new InvalidOperationException(
                "The car does not provide delayed-start calibration support. Flash the latest calibration firmware.");
        }
    }

    public void Send(string content)
    {
        if (externalBluetooth)
        {
            RunExternalBluetoothCommand(content);
            return;
        }
        string frame = "[" + content + "]";
        if (port != null)
        {
            port.Write(frame);
        }
        else
        {
            ResumeBluetooth();
            byte[] bytes = Encoding.ASCII.GetBytes(frame);
            lock (transportSync)
            {
                bluetoothStream.Write(bytes, 0, bytes.Length);
                bluetoothStream.Flush();
            }
        }
        Console.WriteLine("TX " + frame);
    }

    public void Set(string name, double value)
    {
        Send(string.Format(
            System.Globalization.CultureInfo.InvariantCulture,
            "slider,{0},{1:F6}", name, value));
        Thread.Sleep(80);
    }

    public void Stop()
    {
        try { Send("stop"); }
        catch { }
    }

    public string SnapshotReceived()
    {
        lock (sync)
        {
            return received.ToString();
        }
    }

    public void Dispose()
    {
        /* Do not reconnect a deliberately suspended RFCOMM link just to send stop. */
        if (autoStopOnDispose && (port != null || bluetoothStream != null))
        {
            try { Stop(); } catch { }
        }
        disposing = true;
        SuspendForPoseMonitoring();
        if (port != null && port.IsOpen)
        {
            port.Close();
        }
        if (port != null) { port.Dispose(); }
        receivedEvent.Dispose();
    }

    public void SuspendForPoseMonitoring()
    {
        if (bluetoothAddress == null || externalBluetooth) { return; }
        readerStop = true;
        Thread reader;
        lock (transportSync)
        {
            reader = bluetoothReader;
            if (bluetoothStream != null)
            {
                try { bluetoothStream.Dispose(); } catch { }
                bluetoothStream = null;
            }
            if (bluetoothClient != null)
            {
                try
                {
                    MethodInfo close = bluetoothClient.GetType().GetMethod("Close", Type.EmptyTypes);
                    if (close != null) { close.Invoke(bluetoothClient, null); }
                }
                catch { }
                bluetoothClient = null;
            }
            bluetoothReader = null;
        }
        if (reader != null && reader.IsAlive && reader != Thread.CurrentThread) { reader.Join(800); }
    }

    private void ResumeBluetooth()
    {
        if (bluetoothAddress == null || externalBluetooth || disposing) { return; }
        lock (transportSync)
        {
            if (bluetoothStream != null) { return; }
            readerStop = false;
            bluetoothClient = OpenBluetoothClient(bluetoothAddress, out bluetoothStream);
            bluetoothReader = new Thread(ReadBluetoothLoop);
            bluetoothReader.IsBackground = true;
            bluetoothReader.Name = "Vehicle calibration Bluetooth reader";
            bluetoothReader.Start();
        }
        Thread.Sleep(250);
    }

    private static object OpenBluetoothClient(string addressText, out Stream networkStream)
    {
        const string hcPcPath = @"D:\Tools\HC-PC(V1.2.5).exe";
        const string resourceName = "BluetoothApp.lib.InTheHand.Net.Personal.dll";
        if (!File.Exists(hcPcPath))
        {
            throw new FileNotFoundException("HC-PC Bluetooth library source was not found.", hcPcPath);
        }

        Assembly appAssembly = Assembly.LoadFrom(hcPcPath);
        Assembly bluetoothAssembly;
        using (Stream resource = appAssembly.GetManifestResourceStream(resourceName))
        {
            if (resource == null) { throw new InvalidOperationException("Embedded Bluetooth library was not found in HC-PC."); }
            byte[] data = new byte[resource.Length];
            int offset = 0;
            while (offset < data.Length)
            {
                int count = resource.Read(data, offset, data.Length - offset);
                if (count <= 0) { throw new EndOfStreamException("Could not read the embedded Bluetooth library."); }
                offset += count;
            }
            bluetoothAssembly = Assembly.Load(data);
        }

        Type clientType = bluetoothAssembly.GetType("InTheHand.Net.Sockets.BluetoothClient", true);
        Type addressType = bluetoothAssembly.GetType("InTheHand.Net.BluetoothAddress", true);
        Type serviceType = bluetoothAssembly.GetType("InTheHand.Net.Bluetooth.BluetoothService", true);
        object address = addressType.GetMethod("Parse", new Type[] { typeof(string) })
                                    .Invoke(null, new object[] { addressText });
        Guid serialPortService = (Guid)serviceType.GetField("SerialPort", BindingFlags.Public | BindingFlags.Static)
                                                   .GetValue(null);

        object client = Activator.CreateInstance(clientType);
        try
        {
            clientType.GetMethod("Connect", new Type[] { addressType, typeof(Guid) })
                      .Invoke(client, new object[] { address, serialPortService });
            networkStream = (Stream)clientType.GetMethod("GetStream", Type.EmptyTypes).Invoke(client, null);
            networkStream.ReadTimeout = 500;
            networkStream.WriteTimeout = 1000;
            return client;
        }
        catch (TargetInvocationException ex)
        {
            Exception cause = ex.InnerException ?? ex;
            throw new InvalidOperationException("Bluetooth RFCOMM connection failed: " + cause.Message, cause);
        }
    }

    private void RunExternalBluetoothCommand(string content)
    {
        Exception last = null;
        for (int attempt = 0; attempt < 3; ++attempt)
        {
            try
            {
                RunExternalBluetoothCommandOnce(content);
                /* Windows RFCOMM rejects rapid reconnects while the previous socket is retiring. */
                Thread.Sleep(500);
                return;
            }
            catch (Exception ex)
            {
                last = ex;
                Thread.Sleep(1000);
            }
        }
        throw new IOException("Bluetooth command failed after three attempts: " + last.Message, last);
    }

    private void RunExternalBluetoothCommandOnce(string content)
    {
        string executable = Assembly.GetExecutingAssembly().Location;
        string encoded = Convert.ToBase64String(Encoding.UTF8.GetBytes(content));
        ProcessStartInfo start = new ProcessStartInfo();
        start.FileName = executable;
        start.Arguments = "--rfcomm-child " + bluetoothAddress + " " + encoded;
        start.UseShellExecute = false;
        start.CreateNoWindow = true;
        start.RedirectStandardOutput = true;
        start.RedirectStandardError = true;

        using (Process process = Process.Start(start))
        {
            string output = process.StandardOutput.ReadToEnd();
            string error = process.StandardError.ReadToEnd();
            if (!process.WaitForExit(20000))
            {
                try { process.Kill(); } catch { }
                throw new TimeoutException("Bluetooth command helper timed out.");
            }
            if (!string.IsNullOrEmpty(output)) { AppendReceived(output); }
            if (process.ExitCode != 0)
            {
                throw new IOException("Bluetooth command helper failed: " + error.Trim());
            }
        }
    }

    private void ReadBluetoothLoop()
    {
        byte[] buffer = new byte[1024];
        while (!disposing && !readerStop)
        {
            try
            {
                int count = bluetoothStream.Read(buffer, 0, buffer.Length);
                if (count <= 0) { break; }
                AppendReceived(Encoding.ASCII.GetString(buffer, 0, count));
            }
            catch (IOException)
            {
                if (!disposing && !readerStop) { Thread.Sleep(20); }
            }
            catch (ObjectDisposedException) { break; }
        }
    }

    private void OnDataReceived(object sender, SerialDataReceivedEventArgs args)
    {
        try
        {
            string text = port.ReadExisting();
            if (text.Length == 0)
            {
                return;
            }
            AppendReceived(text);
        }
        catch { }
    }

    private void AppendReceived(string text)
    {
        lock (sync)
        {
            received.Append(text);
            if (received.Length > 32768)
            {
                received.Remove(0, received.Length - 16384);
            }
        }
        Console.Write(text);
        receivedEvent.Set();
    }

    private void ClearReceived()
    {
        lock (sync)
        {
            received.Length = 0;
        }
    }

    private string WaitForText(string expected, TimeSpan timeout)
    {
        DateTime end = DateTime.UtcNow + timeout;
        while (DateTime.UtcNow < end)
        {
            string text = SnapshotReceived();
            if (text.IndexOf(expected, StringComparison.OrdinalIgnoreCase) >= 0)
            {
                return text;
            }
            receivedEvent.WaitOne(100);
        }
        return null;
    }
}
