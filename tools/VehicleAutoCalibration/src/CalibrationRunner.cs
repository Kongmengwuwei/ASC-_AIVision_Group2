using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;

internal struct Vec2
{
    public double X;
    public double Y;

    public Vec2(double x, double y) { X = x; Y = y; }
    public double Length { get { return Math.Sqrt(X * X + Y * Y); } }
    public Vec2 Normalized()
    {
        double length = Length;
        if (length < 1e-9) { throw new InvalidOperationException("zero-length direction"); }
        return new Vec2(X / length, Y / length);
    }
    public Vec2 Perpendicular() { return new Vec2(-Y, X); }
    public static double Dot(Vec2 a, Vec2 b) { return a.X * b.X + a.Y * b.Y; }
    public static Vec2 operator +(Vec2 a, Vec2 b) { return new Vec2(a.X + b.X, a.Y + b.Y); }
    public static Vec2 operator -(Vec2 a, Vec2 b) { return new Vec2(a.X - b.X, a.Y - b.Y); }
    public static Vec2 operator *(Vec2 a, double scale) { return new Vec2(a.X * scale, a.Y * scale); }
}

internal sealed class TrialResult
{
    public string Name;
    public List<PoseSample> Samples = new List<PoseSample>();
    public PoseSample Start;
    public PoseSample End;
    public double DurationSeconds;

    public Vec2 DisplacementMeters
    {
        get
        {
            return new Vec2(
                (End.X - Start.X) * PoseMonitorReader.MetersPerMapUnit,
                (End.Y - Start.Y) * PoseMonitorReader.MetersPerMapUnit);
        }
    }
}

internal sealed class CandidateScore
{
    public string Name;
    public double Value;
    public double Score;
}

internal sealed class CalibrationRunner
{
    private readonly PoseMonitorReader poseReader;
    private readonly BluetoothLink link;
    private readonly string outputDirectory;
    private volatile bool cancelled;
    private Vec2 forwardAxis;
    private Vec2 lateralAxis;
    private bool axesValid;

    /* Encoder/wheel hardware scale. Do not absorb short-path braking error here. */
    public double LinearScale = 1.000000;
    public double AngularScale = 1.0;
    public double LateralScale = 0.901589;
    public double Coupling = 0.000000;
    public double YawKp = 6.0;
    public double YawKd = 10.5;
    public double YawFf = 5.0;
    public double PositionKp = 1.1;
    public double PositionKd = 0.25;
    public double GuideKp = 0.0;
    public double GuideMin = 0.0;
    public double GuideMax = 12.0;
    public double GuideDeadbandCm = 0.25;
    public double ScurveAccel = 1.05;
    public double ScurveJerk = 3.50;

    public CalibrationRunner(PoseMonitorReader poseReader, BluetoothLink link, string outputDirectory)
    {
        this.poseReader = poseReader;
        this.link = link;
        this.outputDirectory = outputDirectory;
        Directory.CreateDirectory(outputDirectory);
    }

    public void Cancel()
    {
        cancelled = true;
        link.Stop();
    }

    public void VerifyStationaryPose()
    {
        IList<PoseSample> samples = poseReader.CollectUnique(
            TimeSpan.FromSeconds(5), TimeSpan.FromSeconds(3));
        if (samples.Count < 20)
        {
            throw new InvalidOperationException("Upper-computer pose rate is too low: " + samples.Count);
        }

        double meanX = samples.Average(delegate(PoseSample p) { return p.X; });
        double meanY = samples.Average(delegate(PoseSample p) { return p.Y; });
        double stdX = StdDev(samples.Select(delegate(PoseSample p) { return p.X; }));
        double stdY = StdDev(samples.Select(delegate(PoseSample p) { return p.Y; }));
        double positionNoiseMm = Math.Sqrt(stdX * stdX + stdY * stdY) *
                                 PoseMonitorReader.MetersPerMapUnit * 1000.0;

        Console.WriteLine(
            "Pose: X={0:F4}, Y={1:F4}, yaw={2:F3} deg, rate={3:F1} Hz, noise={4:F2} mm RMS",
            meanX, meanY, CircularMeanDeg(samples),
            (samples.Count - 1) /
            (samples[samples.Count - 1].Timestamp - samples[0].Timestamp).TotalSeconds,
            positionNoiseMm);

        if (positionNoiseMm > 8.0)
        {
            throw new InvalidOperationException(
                "Upper-computer stationary position noise exceeds 8 mm; fix camera detection first.");
        }
    }

    public void ApplyBaselineParameters()
    {
        link.Set("kin.linear", LinearScale);
        link.Set("kin.angular", AngularScale);
        link.Set("kin.lateral", LateralScale);
        link.Set("kin.coupling", Coupling);
        ApplyYaw(YawKp, YawKd, YawFf);
        link.Set("pos.kp", PositionKp);
        link.Set("pos.ki", 0.0);
        link.Set("pos.kd", PositionKd);
        link.Set("guide.kp", GuideKp);
        link.Set("guide.min", GuideMin);
        link.Set("guide.max", GuideMax);
        link.Set("guide.deadband", GuideDeadbandCm);
        link.Set("scurve.amax", ScurveAccel);
        link.Set("scurve.jmax", ScurveJerk);
    }

    public void EnsureSafeStartingArea()
    {
        PoseSample pose = poseReader.ReadFresh(TimeSpan.FromSeconds(3));
        if (pose.X < 2.5 || pose.X > 11.5 || pose.Y < 2.5 || pose.Y > 7.5)
        {
            throw new InvalidOperationException(string.Format(
                CultureInfo.InvariantCulture,
                "Car is too close to the map edge (X={0:F2}, Y={1:F2}). " +
                "Place it in the central open area before calibration.", pose.X, pose.Y));
        }
    }

    public void RunKinematics()
    {
        const double distanceM = 0.40;
        Console.WriteLine("\n=== Kinematics: longitudinal scale ===");
        link.Set("guide.kp", 0.0);
        link.Set("kin.linear", LinearScale);
        link.Set("kin.lateral", LateralScale);
        link.Set("kin.coupling", Coupling);

        TrialResult forward = RunTranslation("linear_forward", "forward", distanceM, 40.0);
        if (forward.DisplacementMeters.Length < 0.15)
        {
            throw new InvalidOperationException("Forward move was too small; check motor power and Bluetooth mode.");
        }
        forwardAxis = forward.DisplacementMeters.Normalized();
        lateralAxis = forwardAxis.Perpendicular();
        axesValid = true;

        TrialResult backward = RunTranslation("linear_backward", "backward", distanceM, 40.0);
        double actualLinear = 0.5 * (
            Vec2.Dot(forward.DisplacementMeters, forwardAxis) -
            Vec2.Dot(backward.DisplacementMeters, forwardAxis));
        double closedLoopLinearSuggestion =
            Clamp(LinearScale * actualLinear / distanceM, 0.5, 1.5);
        Console.WriteLine(
            "Longitudinal actual={0:F4} m. Hardware kin.linear remains locked at {1:F6}; " +
            "closed-loop-only suggestion {2:F6} is not applied.",
            actualLinear, LinearScale, closedLoopLinearSuggestion);

        TrialResult linearValidationForward = RunTranslation("linear_validate_forward", "forward", distanceM, 40.0);
        TrialResult linearValidationBackward = RunTranslation("linear_validate_backward", "backward", distanceM, 40.0);
        double validatedLinear = 0.5 * (
            Vec2.Dot(linearValidationForward.DisplacementMeters, forwardAxis) -
            Vec2.Dot(linearValidationBackward.DisplacementMeters, forwardAxis));
        Console.WriteLine(
            "Linear validation actual={0:F4} m; hardware kin.linear still {1:F6}.",
            validatedLinear, LinearScale);

        Console.WriteLine("\n=== Kinematics: lateral scale and cross coupling ===");
        TrialResult left = RunTranslation("lateral_left", "left", distanceM, 40.0);
        Vec2 leftOrthogonal = left.DisplacementMeters -
                              forwardAxis * Vec2.Dot(left.DisplacementMeters, forwardAxis);
        if (leftOrthogonal.Length < 0.15)
        {
            throw new InvalidOperationException("Left move was too small to establish the lateral axis.");
        }
        lateralAxis = leftOrthogonal.Normalized();
        TrialResult right = RunTranslation("lateral_right", "right", distanceM, 40.0);

        double leftLateral = Vec2.Dot(left.DisplacementMeters, lateralAxis);
        double rightLateral = Vec2.Dot(right.DisplacementMeters, lateralAxis);
        if (Math.Abs(leftLateral) < 0.05 || Math.Abs(rightLateral) < 0.05)
        {
            throw new InvalidOperationException(
                "Lateral calibration displacement is too small for a stable coupling estimate.");
        }
        double actualLateral = 0.5 * (leftLateral - rightLateral);
        double residualLeft = Vec2.Dot(left.DisplacementMeters, forwardAxis) / leftLateral;
        double residualRight = Vec2.Dot(right.DisplacementMeters, forwardAxis) / rightLateral;
        double residualCoupling = 0.5 * (residualLeft + residualRight);
        double baselineCoupling = Coupling;
        bool couplingCandidateApplied = false;

        LateralScale = Clamp(LateralScale * actualLateral / distanceM, 0.5, 1.5);
        if (Math.Abs(residualCoupling) >= 0.01)
        {
            Coupling = Clamp(baselineCoupling + residualCoupling, -0.20, 0.20);
            couplingCandidateApplied = Math.Abs(Coupling - baselineCoupling) > 1e-9;
        }
        Console.WriteLine(
            "Lateral actual={0:F4} m, residual dx/dy={1:F5} -> kin.lateral={2:F6}, kin.coupling={3:F6}",
            actualLateral, residualCoupling, LateralScale, Coupling);
        link.Set("kin.lateral", LateralScale);
        link.Set("kin.coupling", Coupling);

        TrialResult leftValidation = RunTranslation("lateral_validate_left", "left", distanceM, 40.0);
        TrialResult rightValidation = RunTranslation("lateral_validate_right", "right", distanceM, 40.0);
        PrintTranslationValidation(leftValidation, lateralAxis, distanceM);
        PrintTranslationValidation(rightValidation, lateralAxis * -1.0, distanceM);
        double validatedLeft = Vec2.Dot(leftValidation.DisplacementMeters, lateralAxis);
        double validatedRight = Vec2.Dot(rightValidation.DisplacementMeters, lateralAxis);
        if (Math.Abs(validatedLeft) < 0.05 || Math.Abs(validatedRight) < 0.05)
        {
            throw new InvalidOperationException(
                "Lateral validation displacement is too small for a stable coupling estimate.");
        }
        double validatedLateral = 0.5 * (validatedLeft - validatedRight);
        double validatedResidualCoupling = 0.5 * (
            Vec2.Dot(leftValidation.DisplacementMeters, forwardAxis) / validatedLeft +
            Vec2.Dot(rightValidation.DisplacementMeters, forwardAxis) / validatedRight);
        LateralScale = Clamp(LateralScale * validatedLateral / distanceM, 0.5, 1.5);
        if (couplingCandidateApplied &&
            Math.Abs(validatedResidualCoupling) >= Math.Abs(residualCoupling))
        {
            Coupling = baselineCoupling;
            Console.WriteLine(
                "Coupling candidate did not improve cross-axis drift; rolled back to {0:F6}.",
                Coupling);
        }
        link.Set("kin.lateral", LateralScale);
        link.Set("kin.coupling", Coupling);
        Console.WriteLine(
            "Lateral refinement actual={0:F4} m residual={1:F5} -> lateral={2:F6}, coupling={3:F6}",
            validatedLateral, validatedResidualCoupling, LateralScale, Coupling);
        link.Set("guide.kp", GuideKp);
    }

    public void RunYawTuning()
    {
        Console.WriteLine("\n=== Yaw PID and minimum feedforward ===");
        double[] kpCandidates = { 4.5, 6.0, 7.5 };
        double[] kdCandidates = { 6.5, 8.5, 10.5 };
        double[] ffCandidates = { 5.0, 8.0, 11.0 };

        YawKp = ChooseYawParameter("yaw.kp", kpCandidates, YawKp, YawKd, YawFf);
        YawKd = ChooseYawParameter("yaw.kd", kdCandidates, YawKp, YawKd, YawFf);
        YawFf = ChooseYawParameter("yaw.ff", ffCandidates, YawKp, YawKd, YawFf);
        ApplyYaw(YawKp, YawKd, YawFf);
        SaveRecommendedParameters();
    }

    public void RunPositionTuning()
    {
        EnsureAxes();
        Console.WriteLine("\n=== Near-target position PID ===");
        link.Set("guide.kp", 0.0);

        double[] kpCandidates = { 0.8, 1.1, 1.4 };
        double[] kdCandidates = { 0.25, 0.50, 0.80 };
        PositionKp = ChooseTranslationParameter("pos.kp", kpCandidates, PositionKp, PositionKd);
        PositionKd = ChooseTranslationParameter("pos.kd", kdCandidates, PositionKp, PositionKd);

        link.Set("pos.kp", PositionKp);
        link.Set("pos.kd", PositionKd);
        link.Set("guide.kp", GuideKp);
        SaveRecommendedParameters();
    }

    public void RunGuideTuning()
    {
        EnsureAxes();
        Console.WriteLine("\n=== Straight-segment normal guidance ===");
        link.Set("guide.min", GuideMin);
        link.Set("guide.max", GuideMax);
        link.Set("guide.deadband", GuideDeadbandCm);

        double[] candidates = { 0.0, 2.5, 4.0, 6.0 };
        CandidateScore best = null;
        foreach (double candidate in candidates)
        {
            ThrowIfCancelled();
            link.Set("guide.kp", candidate);
            TrialResult forward = RunTranslation("guide_forward_" + candidate.ToString("F2", CultureInfo.InvariantCulture),
                                                 "forward", 0.50, 35.0);
            TrialResult backward = RunTranslation("guide_backward_" + candidate.ToString("F2", CultureInfo.InvariantCulture),
                                                  "backward", 0.50, 35.0);
            double score = CrossTrackScore(forward, forwardAxis) +
                           CrossTrackScore(backward, forwardAxis * -1.0);
            Console.WriteLine("guide.kp={0:F2}, score={1:F4}", candidate, score);
            if (best == null || score < best.Score)
            {
                best = new CandidateScore { Name = "guide.kp", Value = candidate, Score = score };
            }
        }
        GuideKp = best.Value;
        link.Set("guide.kp", GuideKp);
        SaveRecommendedParameters();
    }

    public void RunScurveTuning()
    {
        EnsureAxes();
        Console.WriteLine("\n=== S-curve acceleration and jerk ===");
        double[] accelCandidates = { 0.65, 0.85, 1.05 };
        double[] jerkCandidates = { 3.5, 4.7, 6.0 };

        ScurveAccel = ChooseScurveParameter("scurve.amax", accelCandidates, ScurveAccel, ScurveJerk);
        ScurveJerk = ChooseScurveParameter("scurve.jmax", jerkCandidates, ScurveAccel, ScurveJerk);
        link.Set("scurve.amax", ScurveAccel);
        link.Set("scurve.jmax", ScurveJerk);
        SaveRecommendedParameters();
    }

    public void RunFinalValidation()
    {
        Console.WriteLine("\n=== Power-cycle source-default validation ===");
        EnsureAxes();

        TrialResult forward = RunTranslation("final_forward", "forward", 0.40, 40.0);
        TrialResult backward = RunTranslation("final_backward", "backward", 0.40, 40.0);
        PrintTranslationValidation(forward, forwardAxis, 0.40);
        PrintTranslationValidation(backward, forwardAxis * -1.0, 0.40);

        TrialResult left = RunTranslation("final_left", "left", 0.40, 35.0);
        Vec2 leftOrthogonal = left.DisplacementMeters -
                              forwardAxis * Vec2.Dot(left.DisplacementMeters, forwardAxis);
        if (leftOrthogonal.Length < 0.15)
        {
            throw new InvalidOperationException("Final left validation displacement was too small.");
        }
        lateralAxis = leftOrthogonal.Normalized();
        TrialResult right = RunTranslation("final_right", "right", 0.40, 35.0);
        PrintTranslationValidation(left, lateralAxis, 0.40);
        PrintTranslationValidation(right, lateralAxis * -1.0, 0.40);

        TrialResult ccw = RunTurn("final_yaw_ccw", "turn90");
        TrialResult cw = RunTurn("final_yaw_cw", "turn90r");
        Console.WriteLine("VALIDATE yaw ccw_error={0:F2}deg cw_error={1:F2}deg " +
                          "ccw_drift={2:F2}cm cw_drift={3:F2}cm",
                          Math.Abs(Math.Abs(WrapDeg(ccw.End.YawDeg - ccw.Start.YawDeg)) - 90.0),
                          Math.Abs(Math.Abs(WrapDeg(cw.End.YawDeg - cw.Start.YawDeg)) - 90.0),
                          ccw.DisplacementMeters.Length * 100.0,
                          cw.DisplacementMeters.Length * 100.0);
    }

    public void RunLateralValidation()
    {
        Console.WriteLine("\n=== Lateral coupling validation ===");
        EnsureAxes();
        TrialResult left = RunTranslation("coupling_left", "left", 0.40, 35.0);
        Vec2 leftOrthogonal = left.DisplacementMeters -
                              forwardAxis * Vec2.Dot(left.DisplacementMeters, forwardAxis);
        if (leftOrthogonal.Length < 0.15)
        {
            throw new InvalidOperationException("Lateral coupling validation displacement was too small.");
        }
        lateralAxis = leftOrthogonal.Normalized();
        TrialResult right = RunTranslation("coupling_right", "right", 0.40, 35.0);
        PrintTranslationValidation(left, lateralAxis, 0.40);
        PrintTranslationValidation(right, lateralAxis * -1.0, 0.40);
    }

    public void SaveRecommendedParameters()
    {
        string path = Path.Combine(outputDirectory, "recommended_parameters.txt");
        StringBuilder text = new StringBuilder();
        text.AppendLine("# Auto-calibration result " + DateTimeOffset.Now.ToString("O", CultureInfo.InvariantCulture));
        text.AppendLine("kin.linear=" + LinearScale.ToString("F6", CultureInfo.InvariantCulture));
        text.AppendLine("kin.angular=" + AngularScale.ToString("F6", CultureInfo.InvariantCulture));
        text.AppendLine("kin.lateral=" + LateralScale.ToString("F6", CultureInfo.InvariantCulture));
        text.AppendLine("kin.coupling=" + Coupling.ToString("F6", CultureInfo.InvariantCulture));
        text.AppendLine("yaw.kp=" + YawKp.ToString("F6", CultureInfo.InvariantCulture));
        text.AppendLine("yaw.ki=0.000000");
        text.AppendLine("yaw.kd=" + YawKd.ToString("F6", CultureInfo.InvariantCulture));
        text.AppendLine("yaw.ff=" + YawFf.ToString("F6", CultureInfo.InvariantCulture));
        text.AppendLine("pos.kp=" + PositionKp.ToString("F6", CultureInfo.InvariantCulture));
        text.AppendLine("pos.ki=0.000000");
        text.AppendLine("pos.kd=" + PositionKd.ToString("F6", CultureInfo.InvariantCulture));
        text.AppendLine("guide.kp=" + GuideKp.ToString("F6", CultureInfo.InvariantCulture));
        text.AppendLine("guide.min_cmps=" + GuideMin.ToString("F6", CultureInfo.InvariantCulture));
        text.AppendLine("guide.max_cmps=" + GuideMax.ToString("F6", CultureInfo.InvariantCulture));
        text.AppendLine("guide.deadband_cm=" + GuideDeadbandCm.ToString("F6", CultureInfo.InvariantCulture));
        text.AppendLine("scurve.amax_mps2=" + ScurveAccel.ToString("F6", CultureInfo.InvariantCulture));
        text.AppendLine("scurve.jmax_mps3=" + ScurveJerk.ToString("F6", CultureInfo.InvariantCulture));
        File.WriteAllText(path, text.ToString(), Encoding.UTF8);
        Console.WriteLine("Saved recommendations: " + path);
    }

    private TrialResult RunTranslation(string name, string direction, double distanceM, double speedCmps)
    {
        link.SuspendForPoseMonitoring();
        WaitForFreshPoseAfterRadioRelease();
        PoseSample start = AveragePose(poseReader.CollectUnique(
            TimeSpan.FromMilliseconds(500), TimeSpan.FromSeconds(3)));
        TrialResult result = new TrialResult { Name = name, Start = start };
        DateTimeOffset wallStart;
        PoseSample motionStart = null;
        PoseSample previous = start;
        DateTimeOffset lastPoseTimestamp = start.Timestamp;
        bool started = false;
        double stableSeconds = 0.0;
        double timeoutSeconds = 8.0 + Math.Max(8.0, distanceM / (speedCmps * 0.01) * 3.5 + 5.0);

        Console.WriteLine("\nMOVE {0}: {1} {2:F0} cm at {3:F0} cm/s", name, direction, distanceM * 100.0, speedCmps);
        link.Set("speed", speedCmps);
        link.Set("position", distanceM * 100.0);
        link.Send(direction);
        Thread.Sleep(100);
        link.Set("start.delay", 8.0);
        link.Send("start");
        wallStart = DateTimeOffset.Now;
        link.SuspendForPoseMonitoring();
        WaitForFreshPoseAfterRadioRelease();

        try
        {
            while ((DateTimeOffset.Now - wallStart).TotalSeconds < timeoutSeconds)
            {
                ThrowIfCancelled();
                PoseSample current = poseReader.ReadFresh(
                    started ? TimeSpan.FromSeconds(3) : TimeSpan.FromSeconds(5));
                if (current.Timestamp == lastPoseTimestamp)
                {
                    Thread.Sleep(10);
                    continue;
                }

                result.Samples.Add(current.Clone());
                Vec2 displacement = PoseDeltaMeters(start, current);
                if (displacement.Length > 0.01 && !started)
                {
                    started = true;
                    motionStart = current.Clone();
                }
                if (displacement.Length > distanceM * 1.8 + 0.20)
                {
                    throw new InvalidOperationException("Translation exceeded the safety displacement envelope.");
                }
                EnsureInsideMap(current);

                double dt = (current.Timestamp - previous.Timestamp).TotalSeconds;
                if (dt > 0.001)
                {
                    double speed = PoseDeltaMeters(previous, current).Length / dt;
                    double yawRate = Math.Abs(WrapDeg(current.YawDeg - previous.YawDeg)) / dt;
                    /* Camera yaw has about 0.2-0.4 deg frame noise; 3 deg/s caused false non-settling. */
                    if (started && displacement.Length >= distanceM * 0.65 &&
                        speed < 0.018 && yawRate < 10.0)
                    {
                        stableSeconds += dt;
                    }
                    else
                    {
                        stableSeconds = 0.0;
                    }
                }

                previous = current;
                lastPoseTimestamp = current.Timestamp;
                if (started && stableSeconds >= 0.8)
                {
                    break;
                }
            }
        }
        finally
        {
            link.Stop();
            link.SuspendForPoseMonitoring();
            WaitForFreshPoseAfterRadioRelease();
        }

        if (!started)
        {
            throw new InvalidOperationException("Vehicle did not move before timeout.");
        }
        if (stableSeconds < 0.8)
        {
            throw new TimeoutException("Vehicle did not settle before the motion timeout.");
        }

        result.End = AveragePose(poseReader.CollectUnique(
            TimeSpan.FromMilliseconds(700), TimeSpan.FromSeconds(3)));
        result.DurationSeconds = (result.End.Timestamp - motionStart.Timestamp).TotalSeconds;
        SaveTrial(result);
        Thread.Sleep(400);
        return result;
    }

    private TrialResult RunTurn(string name, string command)
    {
        link.SuspendForPoseMonitoring();
        WaitForFreshPoseAfterRadioRelease();
        PoseSample start = AveragePose(poseReader.CollectUnique(
            TimeSpan.FromMilliseconds(500), TimeSpan.FromSeconds(3)));
        TrialResult result = new TrialResult { Name = name, Start = start };
        DateTimeOffset wallStart;
        PoseSample motionStart = null;
        PoseSample previous = start;
        DateTimeOffset lastTimestamp = start.Timestamp;
        bool started = false;
        double stableSeconds = 0.0;
        double accumulatedYaw = 0.0;

        Console.WriteLine("\nTURN {0}: {1}", name, command);
        link.Set("start.delay", 8.0);
        link.Send(command);
        wallStart = DateTimeOffset.Now;
        link.SuspendForPoseMonitoring();
        WaitForFreshPoseAfterRadioRelease();
        try
        {
            while ((DateTimeOffset.Now - wallStart).TotalSeconds < 22.0)
            {
                ThrowIfCancelled();
                PoseSample current = poseReader.ReadFresh(
                    started ? TimeSpan.FromSeconds(3) : TimeSpan.FromSeconds(5));
                if (current.Timestamp == lastTimestamp)
                {
                    Thread.Sleep(10);
                    continue;
                }
                result.Samples.Add(current.Clone());
                double dyaw = WrapDeg(current.YawDeg - previous.YawDeg);
                accumulatedYaw += dyaw;
                if (Math.Abs(accumulatedYaw) > 3.0 && !started)
                {
                    started = true;
                    motionStart = current.Clone();
                }
                if (Math.Abs(accumulatedYaw) > 140.0)
                {
                    throw new InvalidOperationException("Yaw exceeded the 140 degree safety envelope for a 90 degree test.");
                }

                double dt = (current.Timestamp - previous.Timestamp).TotalSeconds;
                if (dt > 0.001 && started && Math.Abs(accumulatedYaw) >= 60.0 &&
                    Math.Abs(dyaw / dt) < 10.0)
                {
                    stableSeconds += dt;
                }
                else
                {
                    stableSeconds = 0.0;
                }
                previous = current;
                lastTimestamp = current.Timestamp;
                if (started && stableSeconds >= 0.8) { break; }
            }
        }
        finally
        {
            link.Stop();
            link.SuspendForPoseMonitoring();
            WaitForFreshPoseAfterRadioRelease();
        }

        if (!started || stableSeconds < 0.8)
        {
            throw new TimeoutException("Yaw test failed to start or settle.");
        }
        result.End = AveragePose(poseReader.CollectUnique(
            TimeSpan.FromMilliseconds(600), TimeSpan.FromSeconds(3)));
        result.DurationSeconds = (result.End.Timestamp - motionStart.Timestamp).TotalSeconds;
        SaveTrial(result);
        Thread.Sleep(300);
        return result;
    }

    private double ChooseYawParameter(string parameter, double[] candidates,
                                      double kp, double kd, double ff)
    {
        CandidateScore best = null;
        foreach (double candidate in candidates)
        {
            if (parameter == "yaw.kp") { kp = candidate; }
            if (parameter == "yaw.kd") { kd = candidate; }
            if (parameter == "yaw.ff") { ff = candidate; }
            ApplyYaw(kp, kd, ff);
            TrialResult ccw = RunTurn(parameter.Replace('.', '_') + "_" + candidate.ToString("F2") + "_ccw", "turn90");
            TrialResult cw = RunTurn(parameter.Replace('.', '_') + "_" + candidate.ToString("F2") + "_cw", "turn90r");
            double score = YawScore(ccw) + YawScore(cw);
            Console.WriteLine("{0}={1:F3}, score={2:F4}", parameter, candidate, score);
            if (best == null || score < best.Score)
            {
                best = new CandidateScore { Name = parameter, Value = candidate, Score = score };
            }
        }
        link.Set(parameter, best.Value);
        return best.Value;
    }

    private double ChooseTranslationParameter(string parameter, double[] candidates,
                                              double kp, double kd)
    {
        CandidateScore best = null;
        foreach (double candidate in candidates)
        {
            if (parameter == "pos.kp") { kp = candidate; }
            if (parameter == "pos.kd") { kd = candidate; }
            link.Set("pos.kp", kp);
            link.Set("pos.kd", kd);
            TrialResult forward = RunTranslation(parameter.Replace('.', '_') + "_" + candidate.ToString("F2") + "_forward",
                                                 "forward", 0.35, 35.0);
            TrialResult backward = RunTranslation(parameter.Replace('.', '_') + "_" + candidate.ToString("F2") + "_backward",
                                                  "backward", 0.35, 35.0);
            double score = TranslationScore(forward, forwardAxis, 0.35) +
                           TranslationScore(backward, forwardAxis * -1.0, 0.35);
            Console.WriteLine("{0}={1:F3}, score={2:F4}", parameter, candidate, score);
            if (best == null || score < best.Score)
            {
                best = new CandidateScore { Name = parameter, Value = candidate, Score = score };
            }
        }
        link.Set(parameter, best.Value);
        return best.Value;
    }

    private double ChooseScurveParameter(string parameter, double[] candidates,
                                         double accel, double jerk)
    {
        CandidateScore best = null;
        foreach (double candidate in candidates)
        {
            if (parameter == "scurve.amax") { accel = candidate; }
            if (parameter == "scurve.jmax") { jerk = candidate; }
            link.Set("scurve.amax", accel);
            link.Set("scurve.jmax", jerk);
            TrialResult forward = RunTranslation(parameter.Replace('.', '_') + "_" + candidate.ToString("F2") + "_forward",
                                                 "forward", 0.45, 40.0);
            TrialResult backward = RunTranslation(parameter.Replace('.', '_') + "_" + candidate.ToString("F2") + "_backward",
                                                  "backward", 0.45, 40.0);
            double accuracy = TranslationScore(forward, forwardAxis, 0.45) +
                              TranslationScore(backward, forwardAxis * -1.0, 0.45);
            double score = accuracy + 0.02 * (forward.DurationSeconds + backward.DurationSeconds);
            Console.WriteLine("{0}={1:F3}, score={2:F4}", parameter, candidate, score);
            if (best == null || score < best.Score)
            {
                best = new CandidateScore { Name = parameter, Value = candidate, Score = score };
            }
        }
        link.Set(parameter, best.Value);
        return best.Value;
    }

    private void ApplyYaw(double kp, double kd, double ff)
    {
        link.Set("yaw.kp", kp);
        link.Set("yaw.ki", 0.0);
        link.Set("yaw.kd", kd);
        link.Set("yaw.ff", ff);
    }

    private void EnsureAxes()
    {
        if (axesValid) { return; }
        link.Set("guide.kp", 0.0);
        TrialResult forward = RunTranslation("axis_probe_forward", "forward", 0.30, 25.0);
        forwardAxis = forward.DisplacementMeters.Normalized();
        lateralAxis = forwardAxis.Perpendicular();
        RunTranslation("axis_probe_backward", "backward", 0.30, 25.0);
        axesValid = true;
        link.Set("guide.kp", GuideKp);
    }

    private static double TranslationScore(TrialResult trial, Vec2 axis, double expectedM)
    {
        Vec2 final = trial.DisplacementMeters;
        Vec2 target = axis * expectedM;
        double finalError = (final - target).Length;
        double maxProgress = 0.0;
        double rmsCross = 0.0;
        int count = 0;
        foreach (PoseSample sample in trial.Samples)
        {
            Vec2 delta = PoseDeltaMeters(trial.Start, sample);
            maxProgress = Math.Max(maxProgress, Vec2.Dot(delta, axis));
            double cross = Vec2.Dot(delta, axis.Perpendicular());
            rmsCross += cross * cross;
            ++count;
        }
        rmsCross = count > 0 ? Math.Sqrt(rmsCross / count) : 1.0;
        double overshoot = Math.Max(0.0, maxProgress - expectedM);
        return finalError * 4.0 + overshoot * 3.0 + rmsCross;
    }

    private static double CrossTrackScore(TrialResult trial, Vec2 axis)
    {
        double sumSquares = 0.0;
        double max = 0.0;
        int count = 0;
        foreach (PoseSample sample in trial.Samples)
        {
            double cross = Math.Abs(Vec2.Dot(PoseDeltaMeters(trial.Start, sample), axis.Perpendicular()));
            sumSquares += cross * cross;
            max = Math.Max(max, cross);
            ++count;
        }
        double rms = count > 0 ? Math.Sqrt(sumSquares / count) : 1.0;
        return rms + 0.5 * max;
    }

    private static double YawScore(TrialResult trial)
    {
        double accumulated = 0.0;
        double maxAbs = 0.0;
        PoseSample previous = trial.Start;
        foreach (PoseSample sample in trial.Samples)
        {
            accumulated += WrapDeg(sample.YawDeg - previous.YawDeg);
            maxAbs = Math.Max(maxAbs, Math.Abs(accumulated));
            previous = sample;
        }
        double finalError = Math.Abs(Math.Abs(accumulated) - 90.0);
        double overshoot = Math.Max(0.0, maxAbs - 90.0);
        double translationCm = trial.DisplacementMeters.Length * 100.0;
        return trial.DurationSeconds + 0.12 * overshoot + 0.25 * finalError + 0.08 * translationCm;
    }

    private static void PrintTranslationValidation(TrialResult result, Vec2 axis, double expectedM)
    {
        Vec2 delta = result.DisplacementMeters;
        double along = Vec2.Dot(delta, axis);
        double cross = Vec2.Dot(delta, axis.Perpendicular());
        Console.WriteLine(
            "VALIDATE {0}: along={1:F4}m error={2:F2}cm cross={3:F2}cm yaw_delta={4:F2}deg",
            result.Name, along, (along - expectedM) * 100.0, cross * 100.0,
            WrapDeg(result.End.YawDeg - result.Start.YawDeg));
    }

    private void SaveTrial(TrialResult result)
    {
        string safeName = string.Concat(result.Name.Select(delegate(char ch)
        {
            return char.IsLetterOrDigit(ch) || ch == '-' || ch == '_' ? ch : '_';
        }));
        string path = Path.Combine(outputDirectory,
            DateTime.Now.ToString("yyyyMMdd_HHmmss_fff", CultureInfo.InvariantCulture) + "_" + safeName + ".csv");
        using (StreamWriter writer = new StreamWriter(path, false, Encoding.UTF8))
        {
            writer.WriteLine("timestamp,x_map,y_map,yaw_deg,x_rel_m,y_rel_m");
            foreach (PoseSample sample in result.Samples)
            {
                Vec2 delta = PoseDeltaMeters(result.Start, sample);
                writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "{0},{1:F6},{2:F6},{3:F6},{4:F6},{5:F6}",
                    sample.Timestamp.ToString("O", CultureInfo.InvariantCulture),
                    sample.X, sample.Y, sample.YawDeg, delta.X, delta.Y));
            }
        }
    }

    private static PoseSample AveragePose(IList<PoseSample> samples)
    {
        if (samples == null || samples.Count == 0)
        {
            throw new InvalidOperationException("No pose samples were collected.");
        }
        return new PoseSample
        {
            Timestamp = samples[samples.Count - 1].Timestamp,
            X = samples.Average(delegate(PoseSample p) { return p.X; }),
            Y = samples.Average(delegate(PoseSample p) { return p.Y; }),
            YawDeg = CircularMeanDeg(samples)
        };
    }

    private static double CircularMeanDeg(IList<PoseSample> samples)
    {
        double sin = samples.Average(delegate(PoseSample p) { return Math.Sin(p.YawDeg * Math.PI / 180.0); });
        double cos = samples.Average(delegate(PoseSample p) { return Math.Cos(p.YawDeg * Math.PI / 180.0); });
        double angle = Math.Atan2(sin, cos) * 180.0 / Math.PI;
        return angle < 0.0 ? angle + 360.0 : angle;
    }

    private static Vec2 PoseDeltaMeters(PoseSample from, PoseSample to)
    {
        return new Vec2(
            (to.X - from.X) * PoseMonitorReader.MetersPerMapUnit,
            (to.Y - from.Y) * PoseMonitorReader.MetersPerMapUnit);
    }

    private static void EnsureInsideMap(PoseSample pose)
    {
        if (pose.X < 0.25 || pose.X > 13.75 || pose.Y < 0.25 || pose.Y > 9.75)
        {
            throw new InvalidOperationException("Upper-computer pose entered the map-edge safety margin.");
        }
    }

    private void WaitForFreshPoseAfterRadioRelease()
    {
        DateTimeOffset end = DateTimeOffset.Now + TimeSpan.FromSeconds(12);
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
                    if ((DateTimeOffset.Now - stableStart).TotalSeconds >= 2.0) { return; }
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
        throw new IOException("Pose did not recover after releasing the Bluetooth radio: " +
                              (last == null ? "timestamp did not advance" : last.Message), last);
    }

    private void ThrowIfCancelled()
    {
        if (cancelled) { throw new OperationCanceledException("Calibration cancelled by user."); }
    }

    private static double WrapDeg(double angle)
    {
        while (angle > 180.0) { angle -= 360.0; }
        while (angle < -180.0) { angle += 360.0; }
        return angle;
    }

    private static double Clamp(double value, double min, double max)
    {
        return Math.Max(min, Math.Min(max, value));
    }

    private static double StdDev(IEnumerable<double> values)
    {
        double[] array = values.ToArray();
        if (array.Length == 0) { return 0.0; }
        double mean = array.Average();
        return Math.Sqrt(array.Average(delegate(double value)
        {
            double error = value - mean;
            return error * error;
        }));
    }
}
