# 局域网自动清扫位置与历史轨迹

这些是局域网清扫位置、当前轨迹和已归档清扫记录接口，部署新版小车程序后生效。现有建模位置和建模轨迹接口不变。

小车地址：`http://192.168.0.175:7899`，设备编号 `250006`。
模拟器地址：`http://电脑当前IP:7899`，设备编号 `999999`。
两个接口都携带局域网登录获取的 `Authorization: Bearer Token`。

## 实时位置

```http
GET /api/t-railcar/cleaning-realtime-position/250006
Authorization: Bearer Token
```

```json
{
  "success": true,
  "data": {
    "taskName": "9.2演示",
    "runId": "clean_abc123",
    "x": 123,
    "y": 456,
    "coordinateReady": false,
    "rtkFixAvailable": true,
    "controlState": "RUNNING",
    "isCleaning": true
  }
}
```

- x/y 单位为厘米，使用本次执行路线关联模型的原点（东为 x 正方向、北为 y 正方向），与模型路线图一致。小车离开原点后仍正常返回有效 x/y。
- coordinateReady 在此接口中表示小车当前是否位于任务原点允许的 20cm 范围内；它不再决定 x/y 是否可用。未在原点或无法判断时为 false。
- controlState 返回 IDLE、RUNNING、RETURNING、STOPPED、START_FAILED、COMPLETE，分别表示未启动、运行中、独立返航中、已停止、启动失败和正常完成；独立返航正常到达后返回 IDLE。
- isCleaning 仅在自动清扫已经接受启动、正处于 READY 或 RUNNING 阶段时为 true；暂停、停止中、已停止、完成、启动失败、独立返航及手动控制均为 false。
- 原点随本轮任务固定，不读取当前建模会话，也不因切换页面或修改新的建模草稿而变化。
- 后台最多每秒更新约 5 次；前端可约 500ms 查询一次，只更新图上的位置标记，无需刷新整个页面。
- 第一次成功开始清扫之前：taskName/runId 和 x/y 为 null，controlState 为 IDLE；若已选择路线，coordinateReady 仍按小车是否位于该任务原点返回。启动条件拒绝后 controlState 为 START_FAILED。
- 新一轮自动清扫通过启动校验后，会在启动响应返回前同步生成新的 runId 并替换上一轮 points；因此轮询不会短暂读到上一轮轨迹。任务终止后先归档为清扫日志，归档成功后当前 points 立即清空。
- 坐标系原点无法确定时 x/y 为 null。RTK 非固定解、超时或无有效坐标时 x/y 为 null，rtkFixAvailable 表示当前 RTK 固定解可用状态。不要将 null 当成 0 绘图。
- 停止后仍可查询在最近一轮任务坐标系中的实时位置，但不继续追加历史轨迹。

## 本轮清扫历史轨迹（不分区域）

```http
GET /api/t-railcar/cleaning-position-history/250006
Authorization: Bearer Token
```

```json
{
  "success": true,
  "data": {
    "taskName": "9.2演示",
    "runId": "clean_abc123",
    "points": [
      {"x": 0, "y": 0},
      {"x": 5, "y": 1},
      {"x": 11, "y": 2}
    ],
    "coordinateReady": true,
    "rtkFixAvailable": true,
    "simplified": false,
    "pointLimitExceeded": false
  }
}
```

- 返回本轮实际 RTK 行驶轨迹，不是预先规划的路线。不传 areaNumber，包含清扫、换行、跨桥和本轮返回路段。
- 每隔至少 1 秒采样，同位置或移动不足 3cm 不重复插入；开始及停止/暂停/完成时额外保留有效端点。
- 新一轮成功启动后产生新 runId，开始新轨迹；重复启动、启动失败、只选择路线或重新进入页面，不会错误创建清扫日志。
- 暂停不追加，继续同一路线、同一返回模式时沿用 runId；新的完整启动是新一轮。正常完成、明确停车或执行失败后，先把本轮轨迹归档到清扫日志，再清空本接口的 points。
- 两个接口数据的 runId 应一致。前端遇到新的 runId 清空旧图；分别请求时如果跨过了任务切换，应重新查询，不能混用两轮坐标。
- 返回全部保留点，始终按实际采集顺序。超过 1500 点后优先移除直线/近直线内部冗余点，不删除起点、不从头截断，不删分段断点和明显拐弯/折返点。
- 压缩对**已采集厘米整数点**的累计几何偏差控制在 5cm 内（不代表 RTK 实际测量精度，也不能补出采样间隔内漏采的形状）。simplified=true 表示有中间冗余点被压缩。
- 若在这些保护条件下仍超过 1500 个点，保留必要点并返回 pointLimitExceeded=true；1500 是保形优先的软上限，不强删关键形状。

### RTK 中断或暂停后，避免画出虚假的连接直线

恢复后第一个轨迹点可能带 `breakBefore: true`，例如：

```json
{"x": 300, "y": 500, "breakBefore": true}
```

前端从这个点重新起笔，不与上一个点连线；后续点照常连接。它表示中间未记录位置，不能假设小车走了一条直线。没有 breakBefore 的点保持普通 x/y 格式。

RTK 丢失期间，已有 points 保留，返回中的定位标志反映当前状态，不表示历史点无效。

## 已归档清扫记录

列表接口只返回摘要，按开始时间倒序排列：

```http
GET /api/t-railcar/cleaning-logs?productId=250006&page=1&pageSize=20
Authorization: Bearer Token
```

```json
{
  "code": 200,
  "message": "success",
  "data": {
    "list": [{
      "id": "log_20260916_001",
      "runId": "clean_abc123",
      "productId": "250006",
      "serialNumber": "-T01250006",
      "taskName": "9.15演示",
      "startTime": "2026-09-16T08:32:11+08:00",
      "endTime": "2026-09-16T09:14:29+08:00",
      "durationSeconds": 2538,
      "distanceMeters": null,
      "status": "COMPLETED",
      "endReason": "路线正常完成"
    }],
    "total": 1,
    "page": 1,
    "pageSize": 20
  }
}
```

详情接口返回本次实际轨迹和启动时冻结的规划路线快照：

```http
GET /api/t-railcar/cleaning-logs/log_20260916_001
Authorization: Bearer Token
```

`trajectory.points` 每项包含 `x`、`y`、`heading`、`timestamp`；`plannedRoute`
包含当时的 `areaPoints`、`linkPoints`、`pathPoints`。`distanceMeters` 暂时固定
返回 `null`，以后接入可靠里程源时保持字段不变，仅补充数值。

状态只有 `RUNNING`、`COMPLETED`、`STOPPED`、`FAILED`。归档文件写入失败时不会
清空当前轨迹，避免任务数据永久丢失。

## 错误返回

- HTTP 401：缺少或无效的局域网 Token。
- HTTP 404：设备编号不是当前局域网设备。
- HTTP 500：位置数据读取异常。

只查询这些接口不会发送任何运动、建模、任务切换命令。当前轨迹保存在 `modeling_models/cleaning_history/latest.json`，归档日志保存在 `modeling_models/cleaning_logs/`（实际前缀由 MODELING_STORE_DIR 决定），不改已保存路线和建模记录。
