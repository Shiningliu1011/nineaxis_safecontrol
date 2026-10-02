using System.Text.Json;
using DrvDrillLidar;
using DrvDrillLidar.Platform;

namespace DrvDrillLidar.Detection;

internal static class TaskResultValidator
{
    public static string? Validate(JsonElement data, string? currentTaskId, out DetectionCycle? cycle)
    {
        cycle = null;
        if (data.ValueKind != JsonValueKind.Object)
            return "data 不是对象";

        if (!data.TryGetProperty("applicationId", out var appId)
            || appId.ValueKind != JsonValueKind.String
            || !string.Equals(appId.GetString(), ApplicationIds.Detection, StringComparison.Ordinal))
            return "applicationId 不是 tunnel-distance-detection";

        if (!data.TryGetProperty("taskId", out var taskIdElement)
            || taskIdElement.ValueKind != JsonValueKind.String
            || string.IsNullOrWhiteSpace(taskIdElement.GetString()))
            return "taskId 缺失或非字符串";

        if (currentTaskId is null)
            return null;

        if (!string.Equals(taskIdElement.GetString(), currentTaskId, StringComparison.Ordinal))
            return $"taskId 与当前任务不一致（{taskIdElement.GetString()} vs {currentTaskId}）";

        if (!data.TryGetProperty("stateVersion", out var stateVersionElement)
            || stateVersionElement.ValueKind != JsonValueKind.Number
            || !stateVersionElement.TryGetInt64(out var stateVersion)
            || stateVersion <= 0)
            return "stateVersion 缺失或非法";

        if (!data.TryGetProperty("cycle", out var cycleElement)
            || cycleElement.ValueKind != JsonValueKind.Object)
            return "cycle 缺失或不是对象";

        DetectionCycle? parsed;
        try
        {
            parsed = cycleElement.Deserialize<DetectionCycle>(
                new JsonSerializerOptions { PropertyNameCaseInsensitive = true });
        }
        catch (JsonException ex)
        {
            return $"cycle 反序列化失败: {ex.Message}";
        }

        if (parsed is null)
            return "cycle 反序列化结果为空";
        if (parsed.TaskStateVersion != stateVersion)
            return $"cycle.taskStateVersion（{parsed.TaskStateVersion}）与事件 stateVersion（{stateVersion}）不一致";
        if (!string.Equals(parsed.Status, "valid", StringComparison.OrdinalIgnoreCase)
            && !string.Equals(parsed.Status, "invalid", StringComparison.OrdinalIgnoreCase)
            && !string.Equals(parsed.Status, "aborted", StringComparison.OrdinalIgnoreCase))
            return $"cycle.status 非法: {parsed.Status ?? "(null)"}";

        cycle = parsed;
        return null;
    }
}
