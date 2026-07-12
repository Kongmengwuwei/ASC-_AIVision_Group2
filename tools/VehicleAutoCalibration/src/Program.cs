using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading;

internal static class Program
{
    private static CalibrationRunner activeRunner;
    private static BluetoothLink activeLink;

    private static int Main(string[] args)
    {
        Console.OutputEncoding = System.Text.Encoding.UTF8;
        string rfcommChildAddress = GetOption(args, "--rfcomm-child");
        if (!string.IsNullOrWhiteSpace(rfcommChildAddress))
        {
            return RunRfcommChild(rfcommChildAddress, GetOptionValueAfter(args, rfcommChildAddress));
        }
        string monitorPath = GetOption(args, "--monitor");
        string portName = GetOption(args, "--port");
        string bluetoothAddress = GetOption(args, "--bt-address");
        string phase = GetOption(args, "--phase") ?? "kinematics";
        bool dryRun = args.Any(delegate(string arg)
        {
            return string.Equals(arg, "--dry-run", StringComparison.OrdinalIgnoreCase);
        });
        bool linkDiagnostic = args.Any(delegate(string arg)
        {
            return string.Equals(arg, "--link-diagnostic", StringComparison.OrdinalIgnoreCase);
        });
        bool skipBaseline = args.Any(delegate(string arg)
        {
            return string.Equals(arg, "--skip-baseline", StringComparison.OrdinalIgnoreCase);
        });

        Console.CancelKeyPress += delegate(object sender, ConsoleCancelEventArgs eventArgs)
        {
            eventArgs.Cancel = true;
            Console.WriteLine("\nEmergency stop requested...");
            if (activeRunner != null) { activeRunner.Cancel(); }
            else if (activeLink != null) { activeLink.Stop(); }
        };

        try
        {
            PoseMonitorReader poseReader = new PoseMonitorReader(monitorPath);
            Console.WriteLine("Pose source: " + poseReader.LatestPosePath);

            if (dryRun)
            {
                CalibrationRunner dryRunner = new CalibrationRunner(
                    poseReader, null, CreateOutputDirectory());
                dryRunner.VerifyStationaryPose();
                return 0;
            }

            if (!string.IsNullOrWhiteSpace(bluetoothAddress))
            {
                portName = "BTH:" + bluetoothAddress;
            }
            else if (string.IsNullOrWhiteSpace(portName))
            {
                string[] ports = BluetoothLink.GetAvailablePorts();
                if (ports.Length == 0)
                {
                    throw new InvalidOperationException(
                        "No COM ports are available. Disconnect DAP, connect the HC-04 Bluetooth SPP port, then retry.");
                }
                Console.WriteLine("Available ports: " + string.Join(", ", ports));
                Console.Write("Bluetooth COM port: ");
                portName = Console.ReadLine();
            }

            string outputDirectory = CreateOutputDirectory();
            if (!linkDiagnostic)
            {
                CalibrationRunner preflight = new CalibrationRunner(poseReader, null, outputDirectory);
                preflight.VerifyStationaryPose();
                preflight.EnsureSafeStartingArea();
            }

            using (BluetoothLink link = new BluetoothLink(portName))
            {
                activeLink = link;
                link.VerifyFirmware();
                link.VerifyCalibrationInterface();

                if (linkDiagnostic)
                {
                    DiagnosePoseWhileConnected(poseReader, TimeSpan.FromSeconds(15));
                    return 0;
                }

                CalibrationRunner runner = new CalibrationRunner(poseReader, link, outputDirectory);
                activeRunner = runner;
                if (!skipBaseline) { runner.ApplyBaselineParameters(); }

                Console.WriteLine();
                Console.WriteLine("SAFETY CHECK:");
                Console.WriteLine("- DAP cable disconnected");
                Console.WriteLine("- car is on the ground in the central open area");
                Console.WriteLine("- at least 0.8 m clearance in front/back/left/right");
                Console.WriteLine("- upper-computer pose is updating continuously");
                Console.Write("Type RUN to begin motion: ");
                if (!string.Equals(Console.ReadLine(), "RUN", StringComparison.Ordinal))
                {
                    Console.WriteLine("Cancelled before motion.");
                    return 2;
                }

                if (phase == "kinematics" || phase == "all") { runner.RunKinematics(); }
                if (phase == "yaw" || phase == "all") { runner.RunYawTuning(); }
                if (phase == "position" || phase == "all") { runner.RunPositionTuning(); }
                if (phase == "guide" || phase == "all") { runner.RunGuideTuning(); }
                if (phase == "scurve" || phase == "all") { runner.RunScurveTuning(); }
                if (phase == "validation") { runner.RunFinalValidation(); }
                if (phase == "lateral-validation") { runner.RunLateralValidation(); }

                string[] valid = { "kinematics", "yaw", "position", "guide", "scurve", "validation", "lateral-validation", "all" };
                if (!valid.Contains(phase))
                {
                    throw new ArgumentException("Unknown phase: " + phase);
                }

                if (!skipBaseline) { runner.SaveRecommendedParameters(); }
                Console.WriteLine("\nCalibration phase completed. Results: " + outputDirectory);
                activeRunner = null;
                activeLink = null;
            }
            return 0;
        }
        catch (OperationCanceledException ex)
        {
            Console.Error.WriteLine(ex.Message);
            return 130;
        }
        catch (Exception ex)
        {
            try { if (activeLink != null) { activeLink.Stop(); } } catch { }
            Console.Error.WriteLine("ERROR: " + ex.Message);
            Console.Error.WriteLine(ex.StackTrace);
            return 1;
        }
    }

    private static string GetOption(string[] args, string name)
    {
        for (int i = 0; i + 1 < args.Length; ++i)
        {
            if (string.Equals(args[i], name, StringComparison.OrdinalIgnoreCase))
            {
                return args[i + 1];
            }
        }
        return null;
    }

    private static string GetOptionValueAfter(string[] args, string value)
    {
        for (int i = 0; i + 1 < args.Length; ++i)
        {
            if (string.Equals(args[i], value, StringComparison.OrdinalIgnoreCase))
            {
                return args[i + 1];
            }
        }
        return null;
    }

    private static int RunRfcommChild(string address, string encodedCommand)
    {
        if (string.IsNullOrWhiteSpace(encodedCommand))
        {
            Console.Error.WriteLine("Missing encoded RFCOMM command.");
            return 2;
        }
        try
        {
            string command = System.Text.Encoding.UTF8.GetString(Convert.FromBase64String(encodedCommand));
            using (BluetoothLink link = new BluetoothLink("BTH-INPROC:" + address))
            {
                link.Send(command);
                Thread.Sleep(600);
            }
            return 0;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine(ex.Message);
            return 1;
        }
    }

    private static void WaitForPoseRecovery(PoseMonitorReader poseReader, TimeSpan timeout)
    {
        DateTimeOffset end = DateTimeOffset.Now + timeout;
        DateTimeOffset stableStart = DateTimeOffset.MinValue;
        DateTimeOffset lastTimestamp = DateTimeOffset.MinValue;
        Exception last = null;
        while (DateTimeOffset.Now < end)
        {
            try
            {
                PoseSample sample = poseReader.ReadFresh(TimeSpan.FromMilliseconds(700));
                if (sample.Timestamp > lastTimestamp)
                {
                    if (stableStart == DateTimeOffset.MinValue) { stableStart = DateTimeOffset.Now; }
                    lastTimestamp = sample.Timestamp;
                    if ((DateTimeOffset.Now - stableStart).TotalSeconds >= 4.0) { return; }
                }
            }
            catch (Exception ex)
            {
                last = ex;
                stableStart = DateTimeOffset.MinValue;
                lastTimestamp = DateTimeOffset.MinValue;
            }
            Thread.Sleep(80);
        }
        throw new IOException("Pose did not recover after Bluetooth connection: " +
                              (last == null ? "unknown error" : last.Message), last);
    }

    private static void DiagnosePoseWhileConnected(PoseMonitorReader poseReader, TimeSpan duration)
    {
        DateTimeOffset end = DateTimeOffset.Now + duration;
        double maximumAgeMs = 0.0;
        int stale700Count = 0;
        int sampleCount = 0;
        while (DateTimeOffset.Now < end)
        {
            PoseSample sample = poseReader.ReadFresh(TimeSpan.FromSeconds(10));
            double ageMs = (DateTimeOffset.Now - sample.Timestamp).TotalMilliseconds;
            maximumAgeMs = Math.Max(maximumAgeMs, ageMs);
            if (ageMs > 700.0) { ++stale700Count; }
            ++sampleCount;
            Thread.Sleep(100);
        }
        Console.WriteLine("LINK_DIAGNOSTIC samples={0} max_pose_age_ms={1:F0} stale_over_700ms={2}",
                          sampleCount, maximumAgeMs, stale700Count);
    }

    private static string CreateOutputDirectory()
    {
        string root = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "results");
        string path = Path.Combine(root, DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture));
        Directory.CreateDirectory(path);
        return path;
    }
}
