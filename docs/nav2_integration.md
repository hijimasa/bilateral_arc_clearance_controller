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
      motion_model.type: diff_drive
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
地図―odom誤差への低感度」を運用上の要件にする場合は、標準 `diagnostics` topicを監視するか、上位
supervisor / Collision Monitorでfail-stopを構成する。BACは `raw_scan`、設定された `costmap`、または
`costmap_fallback` を報告し、fallback時はWARN levelで理由を含める。周期は
`diagnostics_publish_period` で設定し、0以下で無効になる。

## Ackermann指令contract

`motion_model.type: ackermann`とともに、実測した最小旋回半径`turn_radius_min`を設定する。車両モデルは
この2つで尽きており、粒度はnav2 MPPIの`AckermannConstraints`と同じである。install対象の
[Ackermann設定例](../config/bac_controller_ackermann.yaml)に全体を示す。`turn_radius_min`が正でない
場合はconfigureに失敗する。

Nav2への出力は`TwistStamped`のままで、`linear.x`が車体前進速度、`angular.z`がヨーレートである。BACは
候補を車体曲率でsampleし、常に
`|angular.z| <= min(limits.w_max, |linear.x| / turn_radius_min)`を満たす指令のみを出すため、選ばれた
円弧は必ず幾何的には追従可能である。その場旋回指令は出さず、その場旋回候補は候補格子にも現れない。

下位のAckermann controllerは、このbody twistを自身の操舵interfaceへ変換する必要がある。wheelbase `L`の
自転車モデルなら`delta = atan(L * angular.z / linear.x)`である。BACは操舵jointの実測値を読まず、
road-wheelの操舵速度もモデル化しないため、操舵速度制限・操舵追従誤差・停止中の中立復帰などは下位
controllerの責務である。`turn_radius_min`は幾何的な最小値ではなく、この追従誤差を含めた実際の
旋回円を覆う値を選ぶ。

前進のみの設定（`limits.v_min = 0`）では、車体後方のgoalには到達できない。BACはその場旋回を捏造せず
停止するため、標準behavior treeの`BackUp`などNav2側のrecoveryを設定するか、後方センサcoverageを
確保したうえで`limits.v_min < 0`で後退を許可する。

## 全方向指令contract

`motion_model.type: omni`とともに、横速度の権限`limits.vy_max`を正の値で設定する。0のままだと
configureに失敗する。全体は install対象の[全方向設定例](../config/bac_controller_omni.yaml)に示す。

**このモデルは`cmd_vel`の`linear.y`を出力する。** 下位の車両controllerがこれをhonorしなければ、BACが
避けたつもりの障害物へ直進する。`diff_drive`と`ackermann`では`linear.y`は常に0であり、既存の利用者は
影響を受けない。同様に、現在速度の入力も`linear.y`を読む。odometryのtwistに横速度が埋まっていない
全方向platformでは、加速度窓の起点が常に0になり、横方向の応答が実際より鈍く見積もられる。

回避を担うのは横速度であり、ヨーレートではない。ヨーレートは姿勢規範として決まり、候補生成の前に
確定して全候補で共通である。

**goal姿勢は指定できる。** Nav2は要求されたgoal姿勢をplan末尾のposeに載せており、adapterはそれを
base frameへ変換してcoreへ渡す。goalの手前1.5 mから姿勢規範を経路接線からgoal姿勢へフェードさせ、
0.5 m以内では完全にgoal姿勢が支配する。ヨーが操舵入力ではないため、**位置を横移動で詰めながら要求
された方位を保って到着できる**。実測のヨー誤差は、goal姿勢0.0 / -1.2 / 1.5 / 2.5 / -2.8 / 3.0 radに
対して0.006〜0.060 radであり、Nav2の`SimpleGoalChecker`既定の`yaw_goal_tolerance` 0.25 radの内側に
収まる（測定条件: `heading_gain` 1.5、goalは(4, 2)、開所）。`heading_gain`を下げると誤差は増える。
同条件の差動二輪は0.541〜2.943 radで、要求に関わらず接近方向が残した姿勢で終わる。

goal姿勢が渡されるのは、pruning後の経路にgoal自身が残っている場合に限る（`max_range`で打ち切られた
先の末尾は通過点であり、その向きはgoal姿勢ではない）。`heading_gain: 0.0`とすれば機体方位を固定した
まま並進する（360°センシングを持つplatform向け）。

**姿勢は経路全体にわたってplanに持たせることもできる。** `plan_yaw_mode: "plan"`（既定は`"off"`）と
すると、planの各poseのorientationがbase frameへ変換されて姿勢規範になり、経路接線は使われない。
横に引いた経路を機体の向きを保ったまま進ませる、start姿勢とgoal姿勢の差を走行中に線形に埋める、
といった指定をplanのpose orientationとして書ける。planのorientationが意味を持つことが前提であり、
NavFn/Smacはそこに経路接線を書き込むため、標準plannerのままでは実質`"off"`と変わらない（姿勢を
書き込むwaypoint followerや自作plannerが使い先である）。goal姿勢のフェードは従来どおり働き、planの
末尾poseのorientationとgoal姿勢は同じ値なので二重指定にはならない。`diff_drive`・`ackermann`や
未知の値ではconfigure時にthrowする。狭所では機首バイアス（隙間へ機体を向ける補正）が働かなくなる
ため、姿勢指定つきの通路走行は機体が回れない分の幅の余裕を要求する。詳細は
[parameters.md](parameters.md)の運動モデルの節を参照。

`diff_drive`と`ackermann`はヨーで操舵するため方位を独立に選べず、goal姿勢を渡しても無視して従来どおり
経路接線に従う。これらのモデルで特定のgoal姿勢が必要なら、Nav2側のrecoveryやcontroller切り替えで
対応する。

**`bac_filter_node` は全方向指令を部分的にしか通さない。** 評価用のこのnodeは仮想pathの合成と速度cap
を前進成分だけで行うため、前進成分があるときは横速度が残るが、純横指令は出力0になる。コンテナ内の実
ノードでの実測（`motion_model.type: omni`、`limits.vy_max: 0.3`、720本の全周clear scanを20 Hzで供給）:
`cmd_vel_in(linear.x = 0.30, linear.y = 0.30)` に対する `cmd_vel_out` は、`/odom` が同じ速度を返すとき
`(0.212132, 0.212132)`、`/odom` が零速度のとき `(0.040000, 0.040000)`、`/odom` が無いとき `(0, 0)` である
（出力は現在速度に依存するので、**odomの状態を書かずにこの数値は意味を持たない**。R18 M15: ここに
書かれていた `(0.234, 0.187)` はどのodom条件でも再現しない）。`cmd_vel_in(linear.x = 0, linear.y = 0.30)`
はどのodom条件でも出力0である。全方向モデルの本来の使い先はNav2 controller plugin側（`bac::BacController`）である。

**横移動は車体側方のセンサ被覆を要求する。** これは`limits.v_min < 0`が後方について負うのと同じ注意で
あり、前方のみのLiDARでは斜行先が見えない。`limits.vy_max`はdrivetrainの能力ではなく、実際に観測できて
いる範囲から決める。

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

install済みの例は次のようにも起動できる。

```bash
ros2 launch bilateral_arc_clearance_controller bac_filter.launch.py
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
- Ackermannでは実際の旋回円を実測し、下位のbody twist→操舵変換を検証する。
- 前進のみ設定で後方goalが与えられたときの停止挙動と、Nav2側recoveryの発火を確認する。
- 後方coverageがなければ`limits.v_min=0.0`にする。
- scan / odom / TF timeoutとfallback方針を決める。
- Velocity Smootherと下位controllerの加速度制限をBACの仮定と一致させる。
- Collision Monitorを最後段へ置き、BACなしの場合も停止できることを試験する。
- 実機条件でcore test、Nav2 benchmark、遅延・欠落・外れ値試験を再実行する。
