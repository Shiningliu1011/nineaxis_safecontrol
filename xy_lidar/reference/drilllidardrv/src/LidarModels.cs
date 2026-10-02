using System.Text.Json;
using System.Text.Json.Serialization;

namespace DrvDrillLidar;

// ─────────────────────────── 枚举 ───────────────────────────

/// <summary>
/// 建模命令枚举。仅公开六条阶段目标命令。
/// </summary>
public enum ScanCommand
{
    /// <summary>开始左侧扫描：无任务时创建建模任务，trigger-scan(batch=1)。</summary>
    ScanLeft = 101,

    /// <summary>开始右侧扫描：复用当前任务，trigger-scan(batch=2)。</summary>
    ScanRight = 102,

    /// <summary>开始建模，进入 compute.completed。</summary>
    StartCompute = 201,

    /// <summary>取消任务，进入 task.cancelled。</summary>
    CancelTask = 399,

}

public enum CommandDisposition { None = 0, Accepted = 1, Busy = 2, Replayed = 3, Rejected = 4 }
internal static class CommandContractErrors { public const int Busy = 1304; }
public enum CommandExecutionState { Idle = 0, Executing = 1, Succeeded = 2, Failed = 3 }
/// <summary>
/// 对外公开的命令状态：0=Idle、1=Executing、2=Succeeded、3=Failed。
/// 该状态只服务于上位显示/判断，不参与驱动内部命令门禁或完成事实。
/// </summary>
internal enum CommandStatusState { Idle = 0, Executing = 1, Succeeded = 2, Failed = 3 }
public enum CommandExecutionOutcome { None = 0, GoalReached = 1, GoalAlreadySatisfied = 2, Cancelled = 3, AlreadyFinished = 4, AlreadyAborted = 5 }
/// <summary>
/// 命令回执的纯平铺 DTO。
///
/// 该类型直接对应 Object 寄存器 wire shape；不保留旧的 attempt/execution
/// 嵌套节点或兼容别名。attempt 与 execution 字段仍分别表示同一回执中的
/// 受理结果和执行结果。
/// </summary>
public sealed class CommandReceipt
{
    public long runtimeEpoch { get; set; }
    public long revision { get; set; }
    public int attemptCmd { get; set; }
    public int attemptDisposition { get; set; }
    public int attemptError { get; set; }
    public int executionCmd { get; set; }
    public int executionState { get; set; }
    public int executionOutcome { get; set; }
    public int executionError { get; set; }
}

/// <summary>
/// 探测命令枚举：应用层只暴露开/关，任务生命周期由驱动内部编排。
/// </summary>
public enum DetectionCommand
{
    /// <summary>开（开始探测）：创建/恢复任务并 start-detection。</summary>
    StartDetection = 401,

    /// <summary>关（停止探测）：内部 stop→finish（或 cancel），结束后回到 Idle。</summary>
    StopDetection = 402,
}

/// <summary>
/// 建模状态枚举：旧值 0-8 不变，追加 9=Preparing、10=Aborted、11=Resetting。
/// </summary>
internal enum ScanStateType
{
    Idle = 0,
    Scanning = 1,
    BatchDone = 2,
    Finishing = 3,
    ScanDone = 4,
    ComputeRunning = 5,
    ComputeDone = 6,
    Failed = 7,
    Cancelled = 8,
    Preparing = 9,
    Aborted = 10,
    Resetting = 11,
}

/// <summary>
/// 探测状态枚举（与建模完全分离）。
/// </summary>
internal enum DetectionStateType
{
    Idle = 0,
    Preparing = 1,
    Ready = 2,
    Running = 3,
    Stopping = 4,
    Stopped = 5,
    Error = 6,
    Finishing = 7,
    Cancelling = 8,
    Finished = 9,
    Cancelled = 10,
    Aborted = 11,
}

/// <summary>
/// 驱动工作模式（Normal=连接真实设备；Simulate=本地虚拟流程）。
/// </summary>
public enum DeviceMode
{
    Normal = 0,
    Simulate = 1,
}

// ─────────────────────────── 请求体 ───────────────────────────

/// <summary>创建应用任务请求体。</summary>
public class TaskCreateRequest
{
    [JsonPropertyName("deviceIds")]
    public List<int> DeviceIds { get; set; } = [];
}

/// <summary>
/// trigger-scan 动作体。
/// </summary>
public class TriggerScanInput
{
    [JsonPropertyName("durationSec")]
    public double? DurationSec { get; set; }

    [JsonPropertyName("batch")]
    public int Batch { get; set; }

    [JsonPropertyName("overwrite")]
    public bool? Overwrite { get; set; }
}

/// <summary>
/// start-compute 动作体：由三个建模输入寄存器（法向/导航点/轮廓线）在驱动内组装；
/// 缺失拦截 3101、结构非法拦截 3102。
/// </summary>
public class StartComputeInput
{
    [JsonPropertyName("computeInput")]
    public ProcessingConfig? ComputeInput { get; set; }
}

/// <summary>
/// 算法处理配置；使用扩展字段透传，避免对算法 schema 做强绑定。
/// </summary>
public class ProcessingConfig
{
    [JsonExtensionData]
    public Dictionary<string, JsonElement>? Extensions { get; set; }
}

// ─────────────────────────── 任务状态 ───────────────────────────

/// <summary>任务错误对象（TaskStatus.error）。</summary>
public class TaskError
{
    [JsonPropertyName("code")]
    public string? Code { get; set; }

    [JsonPropertyName("message")]
    public string? Message { get; set; }

    [JsonPropertyName("details")]
    public JsonElement? Details { get; set; }
}

/// <summary>TMBS 公共任务状态（status / SSE task.updated / 创建与动作回执同构）。</summary>
public class TaskStatus
{
    [JsonPropertyName("taskId")]
    public string? TaskId { get; set; }

    [JsonPropertyName("applicationId")]
    public string? ApplicationId { get; set; }

    [JsonPropertyName("lifecycleStatus")]
    public string? LifecycleStatus { get; set; }

    [JsonPropertyName("applicationStatus")]
    public string? ApplicationStatus { get; set; }

    [JsonPropertyName("stateVersion")]
    public long? StateVersion { get; set; }

    [JsonPropertyName("legacyReadOnly")]
    public bool LegacyReadOnly { get; set; }

    [JsonPropertyName("deviceIds")]
    public List<int>? DeviceIds { get; set; }

    [JsonPropertyName("parameterSchemaVersion")]
    public string? ParameterSchemaVersion { get; set; }

    [JsonPropertyName("phaseSchemaVersion")]
    public string? PhaseSchemaVersion { get; set; }

    [JsonPropertyName("phase")]
    public TaskPhase? Phase { get; set; }

    [JsonPropertyName("applicationState")]
    public ApplicationState? ApplicationState { get; set; }

    [JsonPropertyName("progress")]
    public int? Progress { get; set; }

    [JsonPropertyName("stepLabel")]
    public string? StepLabel { get; set; }

    [JsonPropertyName("error")]
    public TaskError? Error { get; set; }

    [JsonPropertyName("createdAt")]
    public string? CreatedAt { get; set; }

    [JsonPropertyName("updatedAt")]
    public string? UpdatedAt { get; set; }

    [JsonPropertyName("terminalAt")]
    public string? TerminalAt { get; set; }

    [JsonExtensionData]
    public Dictionary<string, JsonElement>? Extensions { get; set; }
}

/// <summary>v1 任务 phase 结构化字段（REST status/detail 含 items；SSE 紧凑版无 items）。</summary>
public class TaskPhase
{
    [JsonPropertyName("currentId")]
    public string? CurrentId { get; set; }

    [JsonPropertyName("state")]
    public string? State { get; set; }

    [JsonPropertyName("items")]
    public List<TaskPhaseItem>? Items { get; set; }
}

/// <summary>phase 内单一步骤。</summary>
public class TaskPhaseItem
{
    [JsonPropertyName("id")]
    public string? Id { get; set; }

    [JsonPropertyName("state")]
    public string? State { get; set; }
}

/// <summary>v1 应用状态（applicationState.status 为稳定状态 ID）。</summary>
public class ApplicationState
{
    [JsonPropertyName("status")]
    public string? Status { get; set; }

    [JsonPropertyName("currentStep")]
    public CurrentStep? CurrentStep { get; set; }
}

/// <summary>当前 step（仅建模 compute 阶段等场景上报；探测应用恒为 null）。</summary>
public class CurrentStep
{
    [JsonPropertyName("id")]
    public string? Id { get; set; }

    [JsonPropertyName("progress")]
    public int? Progress { get; set; }
}

/// <summary>任务详情响应（GET /application-tasks/{taskId}）。</summary>
public class TaskDetailResponse
{
    [JsonPropertyName("taskId")]
    public string? TaskId { get; set; }

    [JsonPropertyName("applicationId")]
    public string? ApplicationId { get; set; }

    [JsonPropertyName("applicationStatus")]
    public string? ApplicationStatus { get; set; }

    [JsonPropertyName("applicationDetail")]
    public ApplicationDetail? ApplicationDetail { get; set; }

    [JsonExtensionData]
    public Dictionary<string, JsonElement>? Extensions { get; set; }
}

/// <summary>任务详情中的应用段信息。</summary>
public class ApplicationDetail
{
    [JsonPropertyName("batchCount")]
    public int? BatchCount { get; set; }

    [JsonPropertyName("batches")]
    public List<BatchDetail>? Batches { get; set; }

    [JsonExtensionData]
    public Dictionary<string, JsonElement>? Extensions { get; set; }
}

/// <summary>单批次扫描详情。</summary>
public class BatchDetail
{
    [JsonPropertyName("batch")]
    public int Batch { get; set; }

    [JsonPropertyName("deviceIds")]
    public List<int>? DeviceIds { get; set; }

    [JsonPropertyName("durationSec")]
    public double? DurationSec { get; set; }

    [JsonPropertyName("devicePointCounts")]
    public Dictionary<string, long>? DevicePointCounts { get; set; }

    [JsonPropertyName("status")]
    public string? Status { get; set; }
}

// ─────────────────────────── 任务产物 ───────────────────────────

/// <summary>任务产物元数据。</summary>
public class ArtifactInfo
{
    [JsonPropertyName("artifactId")]
    public string? ArtifactId { get; set; }

    [JsonPropertyName("artifactType")]
    public string? ArtifactType { get; set; }

    [JsonPropertyName("format")]
    public string? Format { get; set; }

    [JsonPropertyName("sizeBytes")]
    public long? SizeBytes { get; set; }

    [JsonPropertyName("mediaType")]
    public string? MediaType { get; set; }

    [JsonPropertyName("checksum")]
    public ArtifactChecksum? Checksum { get; set; }

    [JsonPropertyName("metadata")]
    public ArtifactPointCloudMetadata? Metadata { get; set; }

    [JsonPropertyName("createdAt")]
    public string? CreatedAt { get; set; }

    [JsonPropertyName("contentUrl")]
    public string? ContentUrl { get; set; }
}

public class ArtifactChecksum
{
    [JsonPropertyName("algorithm")]
    public string? Algorithm { get; set; }

    [JsonPropertyName("value")]
    public string? Value { get; set; }
}

public class ArtifactPointCloudMetadata
{
    [JsonPropertyName("fields")]
    public List<string>? Fields { get; set; }

    [JsonPropertyName("hasRgb")]
    public bool HasRgb { get; set; }

    [JsonPropertyName("pointCount")]
    public long? PointCount { get; set; }

    [JsonPropertyName("vertexCount")]
    public long? VertexCount { get; set; }

    [JsonPropertyName("triangleCount")]
    public long? TriangleCount { get; set; }
}

/// <summary>任务产物列表响应（GET /application-tasks/{taskId}/artifacts）。</summary>
public class ArtifactListResponse
{
    [JsonPropertyName("items")]
    public List<ArtifactInfo>? Items { get; set; }

    public List<ArtifactInfo> GetAll() => Items ?? [];
}

// ─────────────────────────── 探测结果 ───────────────────────────

/// <summary>单轮探测结果。</summary>
public class DetectionCycle
{
    [JsonPropertyName("sequence")]
    public long Sequence { get; set; }

    [JsonPropertyName("taskStateVersion")]
    public long? TaskStateVersion { get; set; }

    [JsonPropertyName("segment")]
    public int? Segment { get; set; }

    [JsonPropertyName("status")]
    public string? Status { get; set; }

    [JsonPropertyName("createdAt")]
    public string? CreatedAt { get; set; }

    [JsonPropertyName("pointCount")]
    public int? PointCount { get; set; }

    [JsonPropertyName("resultSchemaVersion")]
    public int? ResultSchemaVersion { get; set; }

    [JsonPropertyName("geometryMode")]
    public string? GeometryMode { get; set; }

    [JsonPropertyName("ranges")]
    public DetectionRanges? Ranges { get; set; }

    [JsonPropertyName("geometry")]
    public DetectionGeometry? Geometry { get; set; }

    [JsonPropertyName("warnings")]
    public List<DetectionWarning>? Warnings { get; set; }

    [JsonPropertyName("captureDurationMs")]
    public long? CaptureDurationMs { get; set; }

    [JsonPropertyName("computeDurationMs")]
    public long? ComputeDurationMs { get; set; }

    [JsonPropertyName("totalDurationMs")]
    public long? TotalDurationMs { get; set; }

    // Legacy result fields are deserialized for historical read compatibility only.
    [JsonPropertyName("yawDeg")]
    public double? YawDeg { get; set; }

    [JsonPropertyName("frontNormal")]
    public double[]? FrontNormal { get; set; }

    [JsonPropertyName("walls")]
    public WallMeasurements? Walls { get; set; }

    [JsonPropertyName("error")]
    public CycleError? Error { get; set; }
}

public class DetectionRanges
{
    [JsonPropertyName("front")]
    public DetectionRange? Front { get; set; }
    [JsonPropertyName("left")]
    public DetectionRange? Left { get; set; }
    [JsonPropertyName("right")]
    public DetectionRange? Right { get; set; }
}

public class DetectionRange
{
    [JsonPropertyName("distanceState")]
    public string? DistanceState { get; set; }
    [JsonPropertyName("distanceM")]
    public double? DistanceM { get; set; }
    [JsonPropertyName("supportCellCount")]
    public int SupportCellCount { get; set; }
}

public class DetectionGeometry
{
    [JsonPropertyName("yawDeg")]
    public double? YawDeg { get; set; }
    [JsonPropertyName("frontNormal")]
    public double[]? FrontNormal { get; set; }
    [JsonPropertyName("walls")]
    public GeometryWallMeasurements? Walls { get; set; }
}

public class GeometryWallMeasurements
{
    [JsonPropertyName("front")]
    public GeometryWallMeasurement? Front { get; set; }
    [JsonPropertyName("left")]
    public GeometryWallMeasurement? Left { get; set; }
    [JsonPropertyName("right")]
    public GeometryWallMeasurement? Right { get; set; }
}

public class GeometryWallMeasurement
{
    [JsonPropertyName("fittedDistanceM")]
    public double? FittedDistanceM { get; set; }
    [JsonPropertyName("rmseM")]
    public double? RmseM { get; set; }
    [JsonPropertyName("inlierRatio")]
    public double? InlierRatio { get; set; }
    [JsonPropertyName("inlierPointCount")]
    public int? InlierPointCount { get; set; }
}

public class DetectionWarning
{
    [JsonPropertyName("code")]
    public string? Code { get; set; }
    [JsonPropertyName("message")]
    public string? Message { get; set; }
}

/// <summary>三面墙测量集合。</summary>
public class WallMeasurements
{
    [JsonPropertyName("front")]
    public WallMeasurement? Front { get; set; }

    [JsonPropertyName("left")]
    public WallMeasurement? Left { get; set; }

    [JsonPropertyName("right")]
    public WallMeasurement? Right { get; set; }
}

/// <summary>单墙拟合测量。</summary>
public class WallMeasurement
{
    [JsonPropertyName("fittedDistanceM")]
    public double? FittedDistanceM { get; set; }

    [JsonPropertyName("nearestDistanceM")]
    public double? NearestDistanceM { get; set; }

    [JsonPropertyName("pointCount")]
    public int? PointCount { get; set; }

    [JsonPropertyName("inlierRatio")]
    public double? InlierRatio { get; set; }

    [JsonPropertyName("rmseM")]
    public double? RmseM { get; set; }
}

/// <summary>单轮错误（invalid/aborted 轮次）。</summary>
public class CycleError
{
    [JsonPropertyName("code")]
    public string? Code { get; set; }

    [JsonPropertyName("message")]
    public string? Message { get; set; }
}

/// <summary>最新结果响应（GET .../results/latest）。</summary>
public class LatestResultResponse
{
    [JsonPropertyName("taskId")]
    public string? TaskId { get; set; }

    [JsonPropertyName("latestCycle")]
    public DetectionCycle? LatestCycle { get; set; }

    [JsonPropertyName("latestValidResult")]
    public DetectionCycle? LatestValidResult { get; set; }
}

/// <summary>结果历史响应（GET .../results）。</summary>
public class ResultHistoryResponse
{
    [JsonPropertyName("items")]
    public List<DetectionCycle>? Items { get; set; }

    [JsonPropertyName("total")]
    public int Total { get; set; }

    [JsonPropertyName("limit")]
    public int Limit { get; set; }

    [JsonPropertyName("offset")]
    public int Offset { get; set; }
}

// ─────────────────────────── 设备 / SSE 快照 ───────────────────────────

/// <summary>
/// 公共设备投影（platform.snapshot / device.updated / 设备只读 REST 接口）。
/// 只包含 deviceId、三态 availability 与 lastSeen 观测字段。
/// </summary>
public class DeviceInfo
{
    [JsonPropertyName("deviceId")]
    public int DeviceId { get; set; }

    /// <summary>三态可用性：online / offline / error。</summary>
    [JsonPropertyName("availability")]
    public string? Availability { get; set; }

    [JsonPropertyName("lastSeen")]
    public double? LastSeen { get; set; }
}

/// <summary>SSE 首帧 platform.snapshot 的 data。</summary>
public class PlatformSnapshot
{
    [JsonPropertyName("devices")]
    public List<DeviceInfo>? Devices { get; set; }

    [JsonPropertyName("globalWorkflow")]
    public GlobalWorkflow? GlobalWorkflow { get; set; }

    [JsonPropertyName("tasks")]
    public List<TaskStatus>? Tasks { get; set; }

    [JsonPropertyName("activeLidarArrayRegistrationRun")]
    public JsonElement? ActiveLidarArrayRegistrationRun { get; set; }
}

/// <summary>全局排他槽投影。</summary>
public class GlobalWorkflow
{
    [JsonPropertyName("owner")]
    public GlobalWorkflowOwner? Owner { get; set; }
}

/// <summary>全局排他槽持有者。</summary>
public class GlobalWorkflowOwner
{
    [JsonPropertyName("ownerType")]
    public string? OwnerType { get; set; }

    [JsonPropertyName("ownerId")]
    public string? OwnerId { get; set; }
}

// ─────────────────────────── 错误 / 异常 ───────────────────────────

/// <summary>TMBS 统一错误结构。</summary>
public class ErrorEnvelope
{
    [JsonPropertyName("code")]
    public string? Code { get; set; }

    [JsonPropertyName("message")]
    public string? Message { get; set; }

    [JsonPropertyName("details")]
    public JsonElement? Details { get; set; }
}

/// <summary>
/// TMBS HTTP 调用失败抛出的异常，携带 HTTP 状态、字符串错误码与 details。
/// </summary>
public class LidarApiException : Exception
{
    public int HttpStatus { get; }

    public string? Code { get; }

    public JsonElement? Details { get; }

    public LidarApiException(
        int httpStatus,
        string message,
        string? code = null,
        JsonElement? details = null,
        Exception? inner = null)
        : base(message, inner)
    {
        HttpStatus = httpStatus;
        Code = code;
        Details = details;
    }
}

// ─────────────────────────── SSE ───────────────────────────

/// <summary>
/// SSE 事件外壳（events.v1）。<c>data</c> 保留为原始 JsonElement，按 category/type 二次反序列化。
/// </summary>
public class SseEnvelope
{
    [JsonPropertyName("eventId")]
    public long? EventId { get; set; }

    [JsonPropertyName("eventVersion")]
    public string? EventVersion { get; set; }

    [JsonPropertyName("category")]
    public string? Category { get; set; }

    [JsonPropertyName("type")]
    public string? Type { get; set; }

    [JsonPropertyName("resourceId")]
    public string? ResourceId { get; set; }

    [JsonPropertyName("emittedAt")]
    public string? EmittedAt { get; set; }

    [JsonPropertyName("data")]
    public JsonElement Data { get; set; }
}
