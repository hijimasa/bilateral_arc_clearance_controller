# パラメータリファレンス

[English](en/parameters.md) | 日本語

`bac_filter_node` と `bac::BacController` は、下表のコアパラメータを共通名で公開する。
nav2 プラグインでは名前空間（例: `FollowPath.`）を先頭に付ける。単位は SI。

## 車体・余裕

| パラメータ | 既定値 | 説明 |
|---|---:|---|
| `footprint.front` | 0.5 | base 原点から車体前端まで [m] |
| `footprint.rear` | -0.5 | base 原点から車体後端まで [m]。負値 |
| `footprint.width` | 0.95 | 車体幅 [m] |
| `safety_margin.front` | 0.2 | 緊急停止領域の前方余裕 [m] |
| `safety_margin.rear` | 0.2 | 緊急停止領域の後方余裕 [m] |
| `safety_margin.side` | 0.2 | 緊急停止領域の側方余裕 [m] |
| `avoid_margin.side` | 0.6 | クリアランス報酬が飽和する車体側面からの距離 [m] |
| `ignore_box.front` | 0.0 | 自己反射除外箱の前方長 [m] |
| `ignore_box.back` | 0.0 | 自己反射除外箱の後方長 [m] |
| `ignore_box.width` | 0.0 | 自己反射除外箱の幅 [m] |

`costmap_margin_compensation` は nav2 アダプタ専用で、コストマップセル中心の量子化誤差を
安全余裕から控除する。生スキャン入力時の既定値は 0、コストマップのみの場合はセル解像度の
半分である。

## 運動モデル

| パラメータ | 既定値 | 説明 |
|---|---:|---|
| `motion_model.type` | `diff_drive` | 運動学policy。`diff_drive`または`ackermann` |

どちらのモデルもNav2標準の車体指令`(linear.x, angular.z)`、すなわち前進速度とヨーレートを出力する。
車両モデルの粒度はnav2 MPPIの`AckermannConstraints`に合わせてあり、追加のパラメータは持たない。
Ackermannを規定するのは最小旋回半径`turn_radius_min`だけであり、wheelbaseや実舵角、操舵速度といった
road-wheel kinematicsは下位の車両controllerの責務とする。

`ackermann`では候補を車体曲率`kappa = angular.z / linear.x`で生成する。sample・refineする範囲は
後退を含む各速度について`|kappa| <= min(1 / turn_radius_min, limits.w_max / |linear.x|)`であり、
`angular.z = linear.x * kappa`へ変換する。すなわち`limits.w_max`は結果のヨーレートを抑えるだけでなく、
`|linear.x| > limits.w_max * turn_radius_min`となる速度域では候補曲率の範囲自体を狭める。実効的な
ヨーレート上限は`min(limits.w_max, |linear.x| / turn_radius_min)`である。停止中に非ゼロのヨーレートは生成せず、
その場旋回候補は候補格子にも現れない。`ackermann`では`turn_radius_min`が正でなければならず、
満たさない場合はcontrollerのconfigure時にthrowする。

下位の車両controllerは、このbody twistを自身の操舵interfaceへ変換する必要がある。wheelbase `L`の
自転車モデルなら`delta = atan(L * angular.z / linear.x)`である。BACは操舵jointの実測値を読まないため、
実機の操舵追従と操舵速度制限の実行は下位系の責務であり、別途検証する。

## 速度・候補生成

| パラメータ | 既定値 | 説明 |
|---|---:|---|
| `limits.v_max` | 0.4 | 最大前進速度 [m/s] |
| `limits.v_min` | -0.1 | 脱出用の最低後退速度 [m/s]。前方センサのみなら 0 を推奨 |
| `limits.w_max` | 1.0 | 最大角速度絶対値 [rad/s] |
| `limits.acc_v` | 0.8 | 並進 dynamic window の加速度 [m/s²] |
| `limits.acc_w` | 2.5 | 出力ヨーレート制限に使う実機の車体角加速度。0で無効 [rad/s²] |
| `control_period` | 0.05 | ヨーレート出力制限が仮定する制御周期 [s] |
| `window_time` | 0.25 | 並進 dynamic window の時間幅 [s] |
| `v_samples` | 5 | 並進速度サンプル数（停止行は別途追加。回頭行は`diff_drive`のみ） |
| `w_samples` | 25 | 差動二輪のヨーレート、またはAckermannの車体曲率の粗サンプル数 |
| `w_refine_steps` | 3 | 粗い最良ヨーレート／曲率候補の片側をこの本数だけ細分再評価。0で無効 |
| `turn_radius_min` | 0.25 | 最小旋回半径 [m]。`diff_drive`では並進候補（前進・後退とも）のclearance評価が退化しないための下限、`ackermann`では候補曲率そのものを縛る運動学制約であり正の値が必須 |
| `velocity_min` | 0.005 | これ未満の出力速度を 0 に丸める [m/s]。Ackermannでは速度なしのヨーレートが実現不能なため`angular.z`も同時に0にする |
| `angvel_min` | 0.01 | 差動二輪のみ。これ未満の出力角速度を0に丸める [rad/s]。Ackermannでは低速でも小さなヨーレートが有意な曲率を表すため適用せず、曲率が0とみなせるときだけ`angular.z`を0にする |

差動二輪のヨーレート候補とAckermannの曲率候補は、狭所で必要な修正円弧を常に残すため、現在値
まわりの加速度窓ではなく全設定範囲を評価する（原DWAからの意図的な逸脱）。出力段ではどちらのモデルも
`limits.acc_w`と`control_period`から1制御周期で到達可能なヨーレートへクランプし、Ackermannでは
その後さらに`turn_radius_min`の曲率上限を再適用する。クランプ後の定曲率円弧で停止可能性を再検証し、
追加減速が必要なら`v`と`w`を同率で下げて曲率を保存する。比例後の`w`が角減速度範囲を外れる場合は
到達可能範囲へ再制限してその円弧を再検証する。`limits.acc_w`は車体ヨー加速度であり、Ackermannの
road-wheel操舵速度を保証するものではない。加速・操舵過渡中の掃引軌道とjerkは未評価であり、各制限の
実行は下位controllerの責務とする。

## 評価と重み

| パラメータ | 既定値 | 説明 |
|---|---:|---|
| `sim_time` | 2.5 | 候補円弧のロールアウト時間 [s] |
| `station_lateral_weight` | 0.3 | 経路横偏差の重み（`weights.path_dist` 比）。障害物で塞がれた経路区間では 0、経路縦断範囲外（クランプ時）は全重みユークリッド距離 |
| `min_eval_distance` | 1.6 | 低速でも確保する最小評価距離 [m] |
| `eval_lateral_max` | 0.5 | 曲線候補の最大横変位 [m] |
| `cap_adapt_rate` | 0.05 | 密度適応クリアランス上限の EMA 更新率。0 で固定 |
| `weights.clearance` | 2.0 | 左右の小さい方のクリアランス報酬 |
| `weights.balance` | 4.0 | 狭所での左右差ペナルティ |
| `weights.path_dist` | 1.0 | 経路追従コスト（残り射影弧長＋横偏差）の重み |
| `weights.heading` | 0.15 | 終端方位誤差 |
| `weights.hysteresis` | 0.6 | 前回の出力指令（到達可能性クランプとdeadband適用後）との差。差動二輪ではヨーレート差 [score per rad/s]、Ackermannでは曲率差 [score per 1/m] |
| `weights.squeeze` | 0.5 | 側方余裕が小さいときの速度ペナルティ |

`weights.hysteresis`の単位はモデルで異なる。差動二輪では[score per rad/s]でヨーレート差に、
Ackermannでは[score per 1/m]で曲率差にかかるため、同じ重み値でも実効的な強さは変わる。Ackermann側の
項の大きさは概ね`2 / turn_radius_min`に比例し、差動二輪側は`2 * w_max`に比例する。同梱の
Ackermann設定例（`turn_radius_min` 1.0、`w_max` 0.8）では両者がほぼ一致するが、`turn_radius_min`を
小さくすると`weights.hysteresis`が`weights.clearance`を圧倒しうるので、モデルを変えたら重みは
評価し直す。

重みを変更するときは `bac_scenario_harness --strict` を必ず再実行する。特に
`weights.balance` と `weights.hysteresis` は狭路中心収束と操舵振動の交換になる。

## 安全・計算量・状態

| パラメータ | 既定値 | 説明 |
|---|---:|---|
| `stop_decel` | 0.8 | 許容性判定に使う制動能力 [m/s²]。**実機の制動限界以下に設定必須** |
| `brake_reaction_time` | 0.1 | 制動距離に含める反応遅れ [s] |
| `margin_scale_floor` | 0.6 | 停止時の安全余裕スケール下限 |
| `margin_scale_speed` | 0.3 | 安全余裕が 100% になる速度 [m/s] |
| `creep_fraction` | 0.3 | 近接ガバナの最低速度割合 |
| `side_envelope_lookahead` | 1.0 | ガバナの前方判定距離 [m]。衝突コース上の点への線形減速と、狭接近すれ違い予定点の速度包絡に共通 |
| `tight_cruise_fraction` | 0.5 | 両側拘束の狭所で許容する巡航速度割合（tightness に線形）。1.0 で無効 |
| `max_range` | 10.0 | 障害物点の最大距離 [m] |
| `max_points` | 1000 | 点数上限。超過時は等間隔間引き。0 以下で無制限 |
| `influence_range` | 1.2 | `CLEAR` とみなす車体からの距離 [m] |
| `avoiding_latch_ticks` | 30 | 障害物が離れた後に `AVOIDING` を保持する tick 数 |

## ROS アダプタ固有

| パラメータ | 対象 | 既定値 | 説明 |
|---|---|---:|---|
| `scan_topic` | nav2 | 空 | 生スキャン。空ならコストマップの lethal セルを使用 |
| `scan_timeout` | 両方 | 0.5 | スキャン鮮度 [s]。nav2 は古いとコストマップへフォールバック |
| `scan_downsample` | nav2 | 1 | LaserScan の角度方向間引き |
| `scan_min_points` | 両方 | 10 (int) | 有効測定(有限ヒット+`+Inf` 無反射)がこれ未満のスキャンをセンサ異常として棄却 |
| `scan_inf_is_valid` | 両方 | true | `+Inf`・range_max 超を「障害物なしの正常測定」として扱う(costmap の `inf_is_valid` 相当) |
| `cmd_timeout` | filter | 0.5 | 上位指令の途絶判定 [s]。途絶時は出力ゼロ |
| `odom_timeout` | filter | 0.5 | 速度フィードバックの途絶判定 [s]。途絶時は停止 |
| `costmap_margin_compensation` | nav2 | 自動 | セル中心量子化の補償 [m] |
| `diagnostics_publish_period` | nav2 | 1.0 | 標準 `diagnostics` message の周期。0以下で無効 [s] |
| `sensor.x/y/yaw` | filter | 0 | LaserScan フレームの固定 2D 外部パラメータ |
| `virtual_path_length` | filter | 3.0 | 入力 `cmd_vel` から作る仮想経路長 [m] |

フィルタノードは TF を参照しないので `sensor.*` を実機に合わせる。nav2 プラグインは TF から
LaserScan を base frame へ変換する。標準 `diagnostics` 出力には `raw_scan`、`costmap`、
`costmap_fallback`、fallback 理由、BAC status、候補数、選択候補の clearance を含む。

## 内部定数（ROS パラメータ非公開）

手法固有の形状定数で、ロボットや環境によらず変更を想定しないもの（`bac_core.hpp` の
コンパイル時既定値）: `eval_angle_max` 1.05 rad（曲線候補の最大評価角）、
`blocked_near`/`blocked_far` 0.4/1.2 m（評価窓外衝突ペナルティのフェード）、
`side_envelope_headroom` 0.1（側方包絡のクッション比）。C++ から `bac::Params` 経由で
上書きは可能。
