using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading;

internal sealed class PoseSample
{
    public DateTimeOffset Timestamp;
    public double X;
    public double Y;
    public double YawDeg;

    public PoseSample Clone()
    {
        return (PoseSample)MemberwiseClone();
    }
}

internal sealed class PoseMonitorReader
{
    public const double MetersPerMapUnit = 0.20;
    private readonly string latestPosePath;

    public PoseMonitorReader(string configuredPath)
    {
        latestPosePath = ResolveLatestPosePath(configuredPath);
    }

    public string LatestPosePath
    {
        get { return latestPosePath; }
    }

    public PoseSample ReadFresh(TimeSpan maxAge)
    {
        Exception last = null;
        for (int attempt = 0; attempt < 8; ++attempt)
        {
            try
            {
                string text = File.ReadAllText(latestPosePath).Trim();
                string[] fields = text.Split(',');
                if (fields.Length < 4)
                {
                    throw new InvalidDataException("latest_pose.txt has fewer than four fields");
                }

                PoseSample sample = new PoseSample();
                sample.Timestamp = DateTimeOffset.Parse(fields[0], CultureInfo.InvariantCulture);
                sample.X = double.Parse(fields[1], CultureInfo.InvariantCulture);
                sample.Y = double.Parse(fields[2], CultureInfo.InvariantCulture);
                sample.YawDeg = double.Parse(fields[3], CultureInfo.InvariantCulture);

                if (DateTimeOffset.Now - sample.Timestamp > maxAge)
                {
                    throw new InvalidOperationException(
                        "pose is stale by " +
                        (DateTimeOffset.Now - sample.Timestamp).TotalSeconds.ToString("F2", CultureInfo.InvariantCulture) +
                        " seconds");
                }
                return sample;
            }
            catch (Exception ex)
            {
                last = ex;
                Thread.Sleep(20);
            }
        }

        throw new IOException("Cannot read a fresh upper-computer pose: " + last.Message, last);
    }

    public IList<PoseSample> CollectUnique(TimeSpan duration, TimeSpan maxAge)
    {
        List<PoseSample> samples = new List<PoseSample>();
        DateTimeOffset end = DateTimeOffset.Now + duration;
        DateTimeOffset lastTimestamp = DateTimeOffset.MinValue;

        while (DateTimeOffset.Now < end)
        {
            PoseSample sample = ReadFresh(maxAge);
            if (sample.Timestamp != lastTimestamp)
            {
                samples.Add(sample);
                lastTimestamp = sample.Timestamp;
            }
            Thread.Sleep(10);
        }
        return samples;
    }

    private static string ResolveLatestPosePath(string configuredPath)
    {
        if (!string.IsNullOrWhiteSpace(configuredPath))
        {
            string path = configuredPath;
            if (Directory.Exists(path))
            {
                path = Path.Combine(path, "logs", "latest_pose.txt");
            }
            if (File.Exists(path))
            {
                return Path.GetFullPath(path);
            }
            throw new FileNotFoundException("Configured monitor pose file was not found", path);
        }

        IEnumerable<string> candidates = Enumerable.Empty<string>();
        try
        {
            candidates = Directory.EnumerateFiles(
                    @"D:\workwork", "latest_pose.txt", SearchOption.AllDirectories)
                .Where(delegate(string path)
                {
                    return path.IndexOf("SmartCarPoseMonitor", StringComparison.OrdinalIgnoreCase) >= 0;
                });
        }
        catch { }

        string newest = candidates
            .OrderByDescending(delegate(string path) { return File.GetLastWriteTimeUtc(path); })
            .FirstOrDefault();
        if (newest == null)
        {
            throw new FileNotFoundException(
                "Cannot find SmartCarPoseMonitor\\logs\\latest_pose.txt under D:\\workwork");
        }
        return newest;
    }
}
