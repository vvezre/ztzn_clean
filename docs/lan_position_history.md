# 局域网历史轨迹接口

请求区域 2：

```http
GET /api/t-railcar/position-history/250006?areaNumber=2
Authorization: Bearer 局域网登录获得的Token
```

小车地址沿用 `http://小车IP:7899`；模拟器使用 `http://电脑IP:7899`，设备编号换成 `999999`。

返回示例：

```json
{
  "success": true,
  "data": {
    "areaNumber": 2,
    "points": [{"x": 100, "y": 200}, {"x": 105, "y": 201}],
    "coordinateReady": true,
    "rtkFixAvailable": true
  }
}
```

- 区域 1、3 等只需将 `areaNumber` 换成对应编号，与清扫先后顺序无关。
- 不传 `areaNumber`：保持原接口，返回全部保留轨迹，`data` 不增加 `areaNumber`。
- 按采集时正在记录的区域归属，不根据坐标猜测区域。连接桥及非记录阶段的轨迹不混入区域查询；仍在全量查询中保留。
- 所有区域使用同一建模原点，x/y 为厘米整数，不在每个区域重新归零。
- 采样间隔至少 1 秒，同一记录区域/桥内移动不足 3 厘米不重复存点；所有区域及连接桥合计最多保留最新 1500 点。切换记录区域不会绕过 1 秒限制。
- 每次 `start_modeling` 都新建模型，区域点、连接点、路径预览及轨迹查询切换到新一轮。无需传 `restart`，旧客户端传 `restart: false` 也会重新开始。已保存路线不删除，新增区域不清空之前区域。
- 对应区域没有轨迹（尚未记录、旧点没有归属或已超出总上限被淘汰）返回 `points: []`。旧版未带归属的轨迹不猜测分区。
- 尚无建模原点时小车返回 `coordinateReady: false`；RTK 丢失时 `rtkFixAvailable: false`，已有轨迹保留，不追加无效点。这两个标志表示当前定位状态，不表示该区域是否已有轨迹。
- `areaNumber` 为 1～999999999 的整数，只传一次。格式错误返回 HTTP 400；未登录或 Token 无效返回 401；设备编号不是当前设备返回 404。

实时位置接口和返回格式不变。

重新开始建图请求：

```http
POST /api/t-railcar/command
Authorization: Bearer 局域网登录获得的Token
Content-Type: application/json
```

```json
{"productId": "250006", "command": "start_modeling", "params": {}}
```

使用返回的 `commandId` 查询 `/api/command-status/{commandId}`，确认 `status: "SUCCEEDED"` 后刷新区域点、连接点、预览和轨迹，并清空前端缓存的上一轮图形。
仅查看当前建模时使用 `get_modeling_state` / 查询点位接口，不要重新发送 `start_modeling`，否则本轮记录也会被重置。
