# パラメータリファレンス

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

## 速度・候補生成

| パラメータ | 既定値 | 説明 |
|---|---:|---|
| `limits.v_max` | 0.4 | 最大前進速度 [m/s] |
| `limits.v_min` | -0.1 | 脱出用の最低後退速度 [m/s]。前方センサのみなら 0 を推奨 |
| `limits.w_max` | 1.0 | 最大角速度絶対値 [rad/s] |
| `limits.acc_v` | 0.8 | 並進 dynamic window の加速度 [m/s²] |
| `window_time` | 0.25 | 並進 dynamic window の時間幅 [s] |
| `v_samples` | 5 | 並進速度サンプル数（停止・回頭行は別途追加） |
| `w_samples` | 25 | `[-w_max, w_max]` の角速度サンプル数 |
| `turn_radius_min` | 0.25 | 前進候補の最小旋回半径 [m] |
| `velocity_min` | 0.005 | これ未満の出力速度を 0 に丸める [m/s] |
| `angvel_min` | 0.01 | これ未満の出力角速度を 0 に丸める [rad/s] |

角速度候補は、狭所で必要な修正円弧を常に残すため現在角速度まわりの加速度窓ではなく全範囲を
評価する。したがって BAC コア自体は角加速度を制限しない。下位の速度制御器で実機の角加速度・
jerk 制限を必ず適用する。

## 評価と重み

| パラメータ | 既定値 | 説明 |
|---|---:|---|
| `sim_time` | 2.5 | 候補円弧のロールアウト時間 [s] |
| `score_lookahead` | 2.5 | ローカルゴールまでの経路長 [m] |
| `min_eval_distance` | 1.6 | 低速でも確保する最小評価距離 [m] |
| `eval_angle_max` | 1.05 | 曲線候補の最大評価角 [rad] |
| `eval_lateral_max` | 0.3 | 曲線候補の最大横変位 [m] |
| `goal_los_radius` | 0.45 | ローカルゴール視線の障害物半径 [m]。0 で無効 |
| `los_onpath_radius` | 0.5 | 経路上障害物を視線トリムから除く半径 [m] |
| `cap_adapt_rate` | 0.05 | 密度適応クリアランス上限の EMA 更新率。0 で固定 |
| `blocked_near` | 0.4 | 評価窓外衝突の完全ペナルティ距離 [m] |
| `blocked_far` | 1.2 | 評価窓外衝突のペナルティ消失距離 [m] |
| `weights.clearance` | 2.0 | 左右の小さい方のクリアランス報酬 |
| `weights.balance` | 4.0 | 狭所での左右差ペナルティ |
| `weights.goal_dist` | 1.0 | ロールアウト終端からローカルゴールまでの距離 |
| `weights.heading` | 0.15 | 終端方位誤差 |
| `weights.hysteresis` | 0.6 | 前回選択角速度との差 |
| `weights.squeeze` | 0.3 | 側方余裕が小さいときの速度ペナルティ |

重みを変更するときは `bac_scenario_harness --strict` を必ず再実行する。特に
`weights.balance` と `weights.hysteresis` は狭路中心収束と操舵振動の交換になる。

## 安全・計算量・状態

| パラメータ | 既定値 | 説明 |
|---|---:|---|
| `stop_decel` | 1.0 | 許容性判定に使う制動能力 [m/s²] |
| `brake_reaction_time` | 0.1 | 制動距離に含める反応遅れ [s] |
| `margin_scale_floor` | 0.5 | 停止時の安全余裕スケール下限 |
| `margin_scale_speed` | 0.3 | 安全余裕が 100% になる速度 [m/s] |
| `creep_fraction` | 0.3 | 近接ガバナの最低速度割合 |
| `proximity_governor_range` | 2.0 | ガバナ開始距離（正規化値。1 が緊急境界） |
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
| `costmap_margin_compensation` | nav2 | 自動 | セル中心量子化の補償 [m] |
| `sensor.x/y/yaw` | filter | 0 | LaserScan フレームの固定 2D 外部パラメータ |
| `virtual_path_length` | filter | 3.0 | 入力 `cmd_vel` から作る仮想経路長 [m] |

フィルタノードは TF を参照しないので `sensor.*` を実機に合わせる。nav2 プラグインは TF から
LaserScan を base frame へ変換する。
