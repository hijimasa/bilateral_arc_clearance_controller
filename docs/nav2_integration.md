# Nav2統合ガイド

[English](en/nav2_integration.md) | 日本語

## 推奨する適用場所

BACの主な適用場所はController Serverである。BACはglobal planを局所的な意図として参照しながら、
障害物点と現在速度から次の速度指令を選び、必要ならpathから局所的に外れて進行する。
これはNav2がcontrollerに割り当てる「局所環境で実行可能なcontrol effortを生成する」責務に一致する。

- [Nav2 Controller Server](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/controller_server/)
- [Writing a Controller Plugin](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html)

```text
Planner → BAC / DWB / MPPI → Velocity Smoother → Collision Monitor → base
```

## 他の拡張点との違い

| 目的 | 適切な場所 | BACとの関係 |
|---|---|---|
| pathを参照しつつ局所回避して`cmd_vel`を生成 | Controller plugin | BACの主用途 |
| DWBの候補生成を保ち左右clearanceを加点 | DWB critic | bilateral項だけを導入したい場合の代案 |
| sensorを統合・保持し障害物表現を作る | local costmap layer | 候補円弧の選択は担当しない |
| 地図上の固定領域で速度を制限 | Speed Filter | live obstacleへの操舵とは別 |
| 最終段で停止・減速 | Collision Monitor | BACと併用する独立防護層 |
| controller選択とrecoveryの編成 | BT / ControllerSelector | 狭路だけBACを選ぶ用途 |

BACはcandidate生成、矩形swept contact、停止可能性、左右中心化、後退escapeを一体で扱うため、現状の
設計をDWB criticだけへ移すと意味が変わる。一方、左右clearance scoreだけを既存DWBへ追加したい場合は、
DWBがcritic pluginとtrajectory generatorを公開しているため、custom criticの方が小さい変更になる。

## Controller plugin設定

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "bac::BacController"
      scan_topic: /scan       # 空ならcostmapのLETHAL cellを使用
      scan_timeout: 0.5
      scan_downsample: 1
      scan_min_points: 10
      footprint.front: 0.5
      footprint.rear: -0.5
      footprint.width: 0.95
      safety_margin.front: 0.2
      safety_margin.rear: 0.2
      safety_margin.side: 0.2
      avoid_margin.side: 0.6
      limits.v_max: 0.4
      limits.v_min: 0.0       # 後方観測を確認後に負値へ変更
      limits.w_max: 1.0
      limits.acc_v: 0.8
      limits.acc_w: 2.5
      control_period: 0.05
      stop_decel: 0.8
      weights.balance: 4.0
      sim_time: 2.5
```

pluginはplanをTFでbase frameへ変換する。planの`frame_id`欠落またはTF失敗はcontroller errorになる。
生スキャンもbase frameへ変換し、未受信、古い、有効測定不足、TF失敗の場合はcostmapのlethal cellへ
fallbackする。costmap入力ではcell中心の量子化を`costmap_margin_compensation`で扱う。

このfallbackは可用性を優先する一方、観測sourceが切り替わることを意味する。「生スキャンに基づく
地図―odom誤差への低感度」を運用上の要件にする場合は、scan sourceの状態を監視し、fallback中を診断へ
出すか、上位supervisor / Collision Monitorでfail-stopを構成する。

## Collision Monitorとの併用

[Collision Monitor](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/)
はcostmapとtrajectory plannerを迂回してsensorを読み、最後段で速度を制限または停止する。操舵して
障害物を回り込むcontrollerではない。Nav2公式の構成どおり、Velocity Smootherを使う場合も
Collision Monitorを`cmd_vel`後処理列の最後に置く。

BACは観測とmodelに基づいて余裕の大きい候補を選び、Collision Monitorはその結果を独立に監視する。
両者を併用しても、実機安全規格への適合が自動的に得られるわけではない。zone、source timeout、停止距離、
速度、sensor coverageを実機で検証する必要がある。

## controllerを切り替える場合

開空間ではRPP/MPPI、狭い開口ではBACのように使い分ける場合、Controller Serverへ複数pluginを登録し、
[ControllerSelector](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/bt_plugins/actions/ControllerSelector/)
またはapplication側から`FollowPath.controller_id`を選ぶ。切替条件はBAC内部へ暗黙に埋め込まず、
task、場所、通路分類、perception confidenceなど、観測可能な条件として記録する方が評価しやすい。

## `bac_filter_node`

`bac_filter_node`は既存controllerの`cmd_vel`を仮想円弧へ変換し、生スキャンを使って整形する。

```bash
ros2 run bilateral_arc_clearance_controller bac_filter_node \
  --ros-args -r scan:=/scan -r odom:=/odom \
             -r cmd_vel_in:=/nav_cmd_vel -r cmd_vel_out:=/cmd_vel
```

障害物が`influence_range`外なら入力を透過し、`AVOIDING`ではcore出力を使い、scanまたはodom途絶時は
停止する。`avoid_status`は`0=CLEAR`、`1=AVOIDING`、`2=STOP`である。

これは上位controllerが選んだtrajectoryを再構成するため、純回頭、後退、上位controllerの加速度modelを
完全には保存しない。Nav2の主要統合にはController pluginを使い、単純な最終停止・減速にはCollision
Monitorを使う。filter nodeは評価、既存systemへの段階導入、非Nav2 command sourceとの統合に限定する。

filter nodeはTFを参照しない。LaserScanがbase frameでなければ`sensor.x/y/yaw`を固定外部パラメータとして
設定する。

## 実機導入チェック

- footprintとsensor extrinsicsを実測する。
- `stop_decel`を実機が全条件で達成できる減速度以下へ設定する。
- `control_period`とController Server frequencyを一致させる。
- 後方coverageがなければ`limits.v_min=0.0`にする。
- scan / odom / TF timeoutとfallback方針を決める。
- Velocity Smootherと下位controllerの加速度制限をBACの仮定と一致させる。
- Collision Monitorを最後段へ置き、BACなしの場合も停止できることを試験する。
- 実機条件でcore test、Nav2 benchmark、遅延・欠落・外れ値試験を再実行する。
