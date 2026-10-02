using System.Text.Json;
using DrvDrillLidar.Driver;
using DrvDrillLidar.Diagnostics;
using DrvDrillLidar.Platform;
using DrvDrillLidar;
using static DrvDrillLidar.Diagnostics.DriverDiagnosticsExtensions;
using CommandCompletionValue = DrvDrillLidar.Platform.CommandCompletion;

/// <summary>
/// 距离探测控制器（tunnel-distance-detection）：401/402 开/关编排、
/// task.result 轮次回写与 SSE 重连 results/latest 回填。
/// </summary>
namespace DrvDrillLidar.Detection
{
internal sealed class DistanceDetectionController : LidarTaskController
{
    private readonly IDetectionRegisterSink Sink;
    private DetectionStateType _lifecycleState = DetectionStateType.Idle;

    private static readonly IReadOnlyDictionary<string, StepFailureAction> WaitFailureActions =
        FailCommandOn(
            "prepare.failed",
            "detection.failed",
            "task.start-failed",
            "task.cancel-failed");

    private static readonly IReadOnlyDictionary<string, StepFailureAction> FinishFailureActions =
        FailureMap(
            ("prepare.failed", StepFailureAction.FailCommand),
            ("detection.failed", StepFailureAction.FailCommand),
            ("finalize.failed", StepFailureAction.CancelAndFinalize),
            ("task.start-failed", StepFailureAction.FailCommand),
            ("task.cancel-failed", StepFailureAction.FailCommand));

    public override string ApplicationId => ApplicationIds.Detection;

    /// <summary>
    /// TMBS 生命周期的内部投影，仅供控制器门禁、收尾和测试观察；不绑定 Edge 模型点。
    /// </summary>
    internal DetectionStateType LifecycleState
    {
        get { lock (Sync) return _lifecycleState; }
        private set { lock (Sync) _lifecycleState = value; }
    }

    /// <summary>命令内核使用的探测目标判定；完成依据不依赖公开状态寄存器。</summary>
    internal bool IsGoalAlreadySatisfied(DetectionCommand command)
    {
        lock (Sync)
        {
            var appStatus = TaskStatusMapping.GetApplicationStatus(LatestStatus);
            return command switch
            {
                DetectionCommand.StartDetection => string.Equals(appStatus, "detection.running", StringComparison.Ordinal),
                DetectionCommand.StopDetection => TaskId is null,
                _ => false,
            };
        }
    }

    internal CommandAdmission AdmitCommand(DetectionCommand command) =>
        CommandAdmission.Accepted(
            (int)command,
            IsGoalAlreadySatisfied(command),
            allowInterrupt: command == DetectionCommand.StopDetection && CanInterruptActiveCommand,
            replayOutcome: CommandExecutionOutcome.GoalAlreadySatisfied);

    public DistanceDetectionController(IDetectionRegisterSink sink, IDriverDiagnostics? diagnostics = null, TimeSpan? waitTimeout = null)
        : base(diagnostics, waitTimeout)
    {
        Sink = sink;
    }

    public override bool HandleCommand(int cmdValue)
    {
        if (!Enum.IsDefined(typeof(DetectionCommand), cmdValue))
        {
            Diagnostics.Warning("command.unknown", "收到未知探测命令", Field("app", ApplicationId), Field("command", cmdValue));
            Sink.WriteError(DetectionErrorCode.UnknownCommand, new ErrorContext { Command = cmdValue });
            RejectCommandStart((int)DetectionErrorCode.UnknownCommand);
            return false;
        }

        var cmd = (DetectionCommand)cmdValue;
        TaskStatus? status;
        lock (Sync) { status = LatestStatus; }
        var supersedesActive = CurrentCommandLease?.SupersedesActive == true;

        if (!DetectionStatusMapping.CanExecuteCommand(cmd, status)
            && !(cmd == DetectionCommand.StopDetection && supersedesActive && TaskId is null))
        {
            // 探测边界：不可执行状态一律忽略，不写错误码
            Diagnostics.Information("command.replayed", "探测命令在当前状态下幂等完成", Field("app", ApplicationId), Field("command", (int)cmd), Field("status", TaskStatusMapping.GetApplicationStatus(status)));
            CompleteCommandStart(CommandCompletionValue.Succeeded(CommandExecutionOutcome.GoalAlreadySatisfied));
            return true;
        }

        var appStatus = TaskStatusMapping.GetApplicationStatus(status);
        Sink.WriteFunctionStatus(0);
        switch (cmd)
        {
            case DetectionCommand.StartDetection:
                if (TaskId is null)
                    QueueStartCreate();
                else if (appStatus is "detection.failed" or "prepare.failed")
                    QueueRetryThenStart();
                else
                    QueueStart();
                break;
            case DetectionCommand.StopDetection:
                if (TaskId is null && supersedesActive)
                    ReplaceSteps(CreateApplicationStep((_, _) => Task.FromResult(true)));
                else
                    QueueStopFlow(appStatus);
                break;
        }

        return true;
    }

    public override void ApplyStatus(TaskStatus status)
    {
        string? currentTaskId;
        lock (Sync) { currentTaskId = TaskId; }
        if (string.IsNullOrWhiteSpace(currentTaskId))
            return;
        if (!string.Equals(status.TaskId, currentTaskId, StringComparison.Ordinal))
            return;

        if (!RecordStatus(status))
            return;
        Diagnostics.Information(
            "task.status-applied",
            "已应用探测任务状态",
            Field("app", ApplicationId),
            Field("taskId", status.TaskId),
            Field("status", TaskStatusMapping.GetApplicationStatus(status)),
            Field("stateVersion", status.StateVersion));
        ApplyStatusCore(status);
    }

    public void ApplyCycle(DetectionCycle cycle)
    {
        lock (Sync)
        {
            if (cycle.Sequence <= LatestAppliedCycleSequence)
                return;
            LatestAppliedCycleSequence = cycle.Sequence;
        }

        if (!string.Equals(cycle.Status, "valid", StringComparison.OrdinalIgnoreCase))
        {
            Sink.WriteFunctionStatus(0);
            if (!string.IsNullOrWhiteSpace(cycle.Error?.Code))
            {
                Diagnostics.Warning("detection.cycle.invalid", "探测轮次无效", Field("app", ApplicationId), Field("taskId", TaskId), Field("sequence", cycle.Sequence), Field("stateVersion", cycle.TaskStateVersion), Field("status", cycle.Status), Field("sourceCode", cycle.Error!.Code));
                Sink.WriteError(
                    DetectionErrorMapper.MapCycle(cycle.Error!.Code),
                    new ErrorContext
                    {
                        SourceCode = cycle.Error.Code,
                        Sequence = cycle.Sequence,
                        StateVersion = cycle.TaskStateVersion,
                        Status = cycle.Status,
                    });
            }
            return;
        }

        if (cycle.ResultSchemaVersion != 1)
        {
            RejectCurrentCycle(cycle, "unsupported-result-schema");
            return;
        }

        if (!TryValidateRanges(cycle.Ranges, out var allMeasured, out var hasIndeterminate))
        {
            RejectCurrentCycle(cycle, "invalid-ranges");
            return;
        }

        if (!allMeasured)
        {
            Sink.WriteFunctionStatus(0);
            if (!hasIndeterminate)
                Sink.ClearError(new ErrorContext { Sequence = cycle.Sequence, StateVersion = cycle.TaskStateVersion });
            return;
        }

        if (string.Equals(cycle.GeometryMode, "full", StringComparison.Ordinal))
        {
            if (!TryGetFullProjection(cycle, out var projection))
            {
                RejectCurrentCycle(cycle, "invalid-full-geometry");
                return;
            }

            Sink.WriteFunctionStatus(0);
            Sink.WriteFrontDistance(projection.FrontDistance);
            Sink.WriteLeftDistance(projection.LeftDistance);
            Sink.WriteRightDistance(projection.RightDistance);
            Sink.WriteYaw(projection.YawDeg);
            Sink.WriteFrontNormal(projection.FrontNormal!);
            LogAndClearValidCycle(cycle);
            Sink.WriteFunctionStatus(2);
            return;
        }

        if (cycle.GeometryMode is "range-only" or "distance-only")
        {
            Sink.WriteFunctionStatus(0);
            Sink.WriteFrontDistance(cycle.Ranges!.Front!.DistanceM!.Value);
            Sink.WriteLeftDistance(cycle.Ranges.Left!.DistanceM!.Value);
            Sink.WriteRightDistance(cycle.Ranges.Right!.DistanceM!.Value);
            LogAndClearValidCycle(cycle);
            Sink.WriteFunctionStatus(1);
            return;
        }

        RejectCurrentCycle(cycle, "invalid-geometry-mode");
    }

    private void LogAndClearValidCycle(DetectionCycle cycle)
    {
        Diagnostics.Debug(
            "detection.cycle.valid",
            "收到有效探测轮次",
            () =>
            [
                Field("app", ApplicationId),
                Field("taskId", TaskId),
                Field("sequence", cycle.Sequence),
                Field("stateVersion", cycle.TaskStateVersion),
                Field("pointCount", cycle.PointCount),
                Field("geometryMode", cycle.GeometryMode),
            ]);
        Sink.ClearError(new ErrorContext { Sequence = cycle.Sequence, StateVersion = cycle.TaskStateVersion });
    }

    private void RejectCurrentCycle(DetectionCycle cycle, string reason)
    {
        Sink.WriteFunctionStatus(0);
        Diagnostics.Warning(
            "sse.result-rejected",
            "探测轮次结构不符合 current 契约，已跳过",
            Field("app", ApplicationId),
            Field("taskId", TaskId),
            Field("sequence", cycle.Sequence),
            Field("stateVersion", cycle.TaskStateVersion),
            Field("reason", reason));
    }

    private static bool TryValidateRanges(
        DetectionRanges? ranges,
        out bool allMeasured,
        out bool hasIndeterminate)
    {
        allMeasured = false;
        hasIndeterminate = false;
        if (ranges?.Front is null || ranges.Left is null || ranges.Right is null)
            return false;

        var values = new[] { ranges.Front, ranges.Left, ranges.Right };
        foreach (var value in values)
        {
            if (string.Equals(value.DistanceState, "measured", StringComparison.Ordinal))
            {
                if (value.DistanceM is not { } distance || !double.IsFinite(distance)
                    || distance < 0 || value.SupportCellCount < 20)
                    return false;
            }
            else if (value.DistanceState is "no-return" or "indeterminate")
            {
                if (value.DistanceM is not null || value.SupportCellCount != 0)
                    return false;
            }
            else
            {
                return false;
            }
        }

        allMeasured = values.All(value => value.DistanceState == "measured");
        hasIndeterminate = values.Any(value => value.DistanceState == "indeterminate");
        return true;
    }

    private static bool TryGetFullProjection(DetectionCycle cycle, out CycleProjection projection)
    {
        projection = default;
        var geometry = cycle.Geometry;
        var normal = FormatFrontNormal(geometry?.FrontNormal);
        if (geometry?.YawDeg is not { } yaw || !double.IsFinite(yaw) || normal is null
            || !TryValidateWall(geometry.Walls?.Front, out var front)
            || !TryValidateWall(geometry.Walls?.Left, out var left)
            || !TryValidateWall(geometry.Walls?.Right, out var right))
            return false;

        projection = new CycleProjection(front, left, right, yaw, normal);
        return true;
    }

    private static bool TryValidateWall(GeometryWallMeasurement? wall, out double distance)
    {
        distance = 0;
        if (wall?.FittedDistanceM is not { } fitted || !double.IsFinite(fitted) || fitted < 0
            || wall.RmseM is not { } rmse || !double.IsFinite(rmse) || rmse < 0
            || wall.InlierRatio is not { } ratio || !double.IsFinite(ratio) || ratio < 0 || ratio > 1
            || wall.InlierPointCount is not { } count || count < 0)
            return false;
        distance = fitted;
        return true;
    }

    private readonly record struct CycleProjection(
        double FrontDistance,
        double LeftDistance,
        double RightDistance,
        double YawDeg,
        double[]? FrontNormal);

    internal bool ApplyCycleIfCurrent(
        DetectionCycle cycle,
        CommandLease? expectedLease,
        string expectedTaskId)
    {
        return RunIfCurrentLease(expectedLease, () =>
        {
            if (!string.Equals(TaskId, expectedTaskId, StringComparison.Ordinal))
                return;
            if (!string.Equals(
                    TaskStatusMapping.GetApplicationStatus(LatestStatus),
                    "detection.running",
                    StringComparison.Ordinal))
            {
                Sink.WriteFunctionStatus(0);
                return;
            }
            ApplyCycle(cycle);
        });
    }

    /// <summary>探测应用门面入口：负责 task.result 业务字段校验，再回写轮次结果。</summary>
    public void ApplyResultEvent(JsonElement data)
    {
        string? expectedTaskId;
        lock (Sync) { expectedTaskId = TaskId; }
        var reason = TaskResultValidator.Validate(data, expectedTaskId, out var cycle);
        if (reason is not null)
        {
            Diagnostics.Warning("sse.result-rejected", "探测结果事件校验失败，已跳过", Field("app", ApplicationId), Field("taskId", TaskId), Field("reason", reason));
            if (expectedTaskId is not null && ShouldInvalidateRejectedResult(data, expectedTaskId))
            {
                RunIfCurrentLease(null, () =>
                {
                    if (string.Equals(TaskId, expectedTaskId, StringComparison.Ordinal))
                        Sink.WriteFunctionStatus(0);
                });
            }
            return;
        }

        if (cycle is not null)
        {
            RunIfCurrentLease(null, () =>
            {
                if (!string.Equals(TaskId, expectedTaskId, StringComparison.Ordinal))
                    return;
                if (!string.Equals(
                        TaskStatusMapping.GetApplicationStatus(LatestStatus),
                        "detection.running",
                        StringComparison.Ordinal))
                {
                    Sink.WriteFunctionStatus(0);
                    return;
                }
                ApplyCycle(cycle);
            });
        }
    }

    private bool ShouldInvalidateRejectedResult(JsonElement data, string expectedTaskId)
    {
        if (data.ValueKind != JsonValueKind.Object
            || !data.TryGetProperty("applicationId", out var applicationId)
            || applicationId.ValueKind != JsonValueKind.String
            || !string.Equals(applicationId.GetString(), ApplicationIds.Detection, StringComparison.Ordinal)
            || !data.TryGetProperty("taskId", out var taskId)
            || taskId.ValueKind != JsonValueKind.String
            || !string.Equals(taskId.GetString(), expectedTaskId, StringComparison.Ordinal))
            return false;

        if (!data.TryGetProperty("cycle", out var cycle)
            || cycle.ValueKind != JsonValueKind.Object
            || !cycle.TryGetProperty("sequence", out var sequence)
            || sequence.ValueKind != JsonValueKind.Number
            || !sequence.TryGetInt64(out var value))
            return true;

        lock (Sync) { return value > LatestAppliedCycleSequence; }
    }

    /// <summary>
    /// 校验并返回掌子面法向数组（长度=3、全有限），供 Object 寄存器直接写入；
    /// 缺失或非法（长度≠3、含非有限值）返回 null，调用方应保留寄存器旧值。
    /// </summary>
    public static double[]? FormatFrontNormal(double[]? normal)
    {
        if (normal is not { Length: 3 } values)
            return null;
        foreach (var value in values)
        {
            if (!double.IsFinite(value))
                return null;
        }
        if (values.All(value => value == 0))
            return null;
        return values;
    }

    /// <summary>
    /// SSE 重连对账：用 results/latest 的最新轮次（任意状态）回填数值寄存器、法向与错误码。
    /// </summary>
    public async Task BackfillLatestResultAsync(IDetectionResultApi api, CancellationToken ct = default)
    {
        string? taskId;
        string? appStatus;
        var commandLease = ActiveCommandLease;
        lock (Sync)
        {
            taskId = TaskId;
            appStatus = TaskStatusMapping.GetApplicationStatus(LatestStatus);
        }
        if (string.IsNullOrWhiteSpace(taskId) || TaskStatusMapping.IsTerminalStatus(appStatus))
            return;
        if (!string.Equals(appStatus, "detection.running", StringComparison.Ordinal))
        {
            Sink.WriteFunctionStatus(0);
            return;
        }

        using var linkedLeaseCts = commandLease is null
            ? null
            : CancellationTokenSource.CreateLinkedTokenSource(commandLease.RevocationToken, ct);
        var operationCt = linkedLeaseCts?.Token ?? ct;

        try
        {
            var latest = await api.GetLatestResultAsync(taskId, operationCt);
            if (latest?.LatestCycle is { } cycle)
            {
                RunIfCurrentLease(commandLease, () =>
                {
                    if (!string.Equals(TaskId, taskId, StringComparison.Ordinal))
                        return;
                    if (!string.Equals(
                            TaskStatusMapping.GetApplicationStatus(LatestStatus),
                            "detection.running",
                            StringComparison.Ordinal))
                    {
                        Sink.WriteFunctionStatus(0);
                        return;
                    }
                    ApplyCycle(cycle);
                });
            }
        }
        catch (Exception ex) when (ex is LidarApiException or HttpRequestException or TaskCanceledException)
        {
            if (commandLease is not null && !IsCurrentCommandLease(commandLease))
                return;
            if (ex is LidarApiException { Code: "TASK_NOT_FOUND" })
            {
                HandleTaskNotFound(commandLease, taskId);
                return;
            }
            Diagnostics.Warning("detection.backfill-failed", "探测结果重连回填失败", Field("app", ApplicationId), Field("taskId", taskId), Field("reason", ex.Message));
        }
    }

    protected override void ApplyStatusCore(TaskStatus status)
    {
        var mapped = DetectionStatusMapping.MapStatus(TaskStatusMapping.GetApplicationStatus(status));
        LifecycleState = mapped;
        Sink.WriteTaskId(status.TaskId ?? "");
        if (!string.Equals(TaskStatusMapping.GetApplicationStatus(status), "detection.running", StringComparison.Ordinal))
            Sink.WriteFunctionStatus(0);

        if (status.Error?.Code is { } errorCode)
            WriteBackendError(errorCode);
        else if (mapped != DetectionStateType.Error)
            Sink.ClearError();

        // 无编排动作时任务进入终态 → 直接回到 Idle
        if (TaskStatusMapping.IsTerminalStatus(TaskStatusMapping.GetApplicationStatus(status)))
        {
            bool hasSteps;
            lock (Sync) { hasSteps = Steps.Count > 0; }
            if (!hasSteps)
                FinalizeTask();
        }
    }

    protected override void ClearTaskRegisters()
    {
        Sink.WriteFunctionStatus(0);
        Sink.WriteTaskId("");
    }

    protected override void WriteDriverError(DriverErrorCode code) => Sink.WriteError(code);

    protected override void WritePlatformError(PlatformErrorCode code, ErrorContext? context = null) =>
        Sink.WriteError(code, context);

    private void WriteBackendError(string? code, int? httpStatus = null)
    {
        var context = new ErrorContext { SourceCode = code, HttpStatus = httpStatus };
        if (DetectionErrorMapper.TryMap(code, out var detection))
        {
            Sink.WriteError(detection, context);
            return;
        }

        Sink.WriteError(PlatformErrorMapper.Map(code), context);
    }

    protected override void FailBackendError(LidarApiException ex, CommandLease? lease)
    {
        lock (Sync)
        {
            if (lease is not null && !IsCurrentCommandLeaseLocked(lease))
                return;
            Diagnostics.Error("tmbs.failure", "TMBS 探测请求失败", ex, Field("app", ApplicationId), Field("taskId", TaskId), Field("sourceCode", ex.Code), Field("httpStatus", ex.HttpStatus));
            WriteStateFailed();
            WriteBackendError(ex.Code, ex.HttpStatus);
            Steps.Clear();
        }
        CompleteCommand(lease, CommandCompletionValue.Failed(
            BackendErrorValue(ex.Code),
            CommandFailureKind.Backend));
    }

    protected override void FailObservedStatus(TaskStatus status, CommandLease? lease)
    {
        var code = status.Error?.Code;
        lock (Sync)
        {
            if (lease is not null && !IsCurrentCommandLeaseLocked(lease))
                return;
            Diagnostics.Error("task.status-failed", "TMBS 探测任务进入失败状态", fields: [Field("app", ApplicationId), Field("taskId", TaskId), Field("status", TaskStatusMapping.GetApplicationStatus(status)), Field("stateVersion", status.StateVersion), Field("sourceCode", code)]);
            WriteStateFailed();
            WriteBackendError(code);
            Steps.Clear();
        }
        CompleteCommand(lease, CommandCompletionValue.Failed(
            BackendErrorValue(code),
            CommandFailureKind.ObservedStatus));
    }

    protected override void WriteStateFailed() =>
        LifecycleState = DetectionStateType.Error;

    protected override bool HandleSendActionFailure(Step step, LidarApiException ex)
    {
        if (string.Equals(step.Action, "finish", StringComparison.Ordinal))
        {
            Diagnostics.Warning("task.finish-recovery", "finish 失败，内部转 cancel", Field("app", ApplicationId), Field("taskId", TaskId), Field("sourceCode", ex.Code), Field("httpStatus", ex.HttpStatus));
            ReplaceStepsIfCurrent(step.Lease,
                CreateStep(StepKind.SendAction, "cancel"),
                WaitForStatus("task.cancelled", FinalizeTask, CancelFailureActions));
            return true;
        }

        return false;
    }

    protected override bool HandleStepFailure(Step step, TaskStatus status)
    {
        if (!string.Equals(TaskStatusMapping.GetApplicationStatus(status), "finalize.failed", StringComparison.Ordinal))
            return false;

        Diagnostics.Warning("task.finalize-recovery", "finalize.failed，内部转 cancel", Field("app", ApplicationId), Field("taskId", TaskId), Field("status", TaskStatusMapping.GetApplicationStatus(status)), Field("sourceCode", status.Error?.Code));
        ReplaceStepsIfCurrent(step.Lease,
            CreateStep(StepKind.SendAction, "cancel"),
            WaitForStatus("task.cancelled", FinalizeTask, CancelFailureActions));
        return true;
    }

    private void QueueStartCreate()
    {
        ReplaceSteps(
            CreateStep(StepKind.CreateTask),
            WaitForStatus("detection.ready"),
            CreateStep(StepKind.SendAction, "start-detection"),
            WaitForStatus("detection.running"));
    }

    private void QueueRetryThenStart()
    {
        ReplaceSteps(
            CreateStep(StepKind.SendAction, "retry"),
            WaitForStatus("detection.ready"),
            CreateStep(StepKind.SendAction, "start-detection"),
            WaitForStatus("detection.running"));
    }

    private void QueueStart()
    {
        ReplaceSteps(
            CreateStep(StepKind.SendAction, "start-detection"),
            WaitForStatus("detection.running"));
    }

    private void QueueStopFlow(string? appStatus)
    {
        if (appStatus is "prepare.running" or "task.start-failed" or "task.cancel-failed")
        {
            ReplaceSteps(
                CreateStep(StepKind.SendAction, "cancel"),
                WaitForStatus("task.cancelled", FinalizeTask, CancelFailureActions));
            return;
        }

        if (string.Equals(appStatus, "detection.running", StringComparison.Ordinal))
        {
            ReplaceSteps(
                CreateStep(StepKind.SendAction, "stop-detection"),
                WaitForStatus("detection.stopped"),
                CreateStep(StepKind.SendAction, "finish"),
                WaitForStatus("task.finished", FinalizeTask, FinishFailureActions));
            return;
        }

        // ready / stopped / failed / finalize.failed → 直接 finish；finish 失败内部 cancel
        ReplaceSteps(
            CreateStep(StepKind.SendAction, "finish"),
            WaitForStatus("task.finished", FinalizeTask, FinishFailureActions));
    }

    private static int BackendErrorValue(string? code) =>
        DetectionErrorMapper.TryMap(code, out var detection)
            ? (int)detection
            : (int)PlatformErrorMapper.Map(code);

    private Step WaitForStatus(
        string status,
        Action? onComplete = null,
        IReadOnlyDictionary<string, StepFailureAction>? failureActions = null) =>
        CreateStep(
            StepKind.WaitStatus,
            targetStatuses: [status],
            onComplete: onComplete,
            failureActions: failureActions ?? WaitFailureActions);

    private static IReadOnlyDictionary<string, StepFailureAction> CancelFailureActions =>
        FailCommandOn("task.cancel-failed");

    private void FinalizeTask()
    {
        LifecycleState = DetectionStateType.Idle;
        Sink.WriteFunctionStatus(0);
        Sink.ClearError();
        ClearTask();
    }
}

}
