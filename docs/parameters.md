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

`costmap_margin_compensation` は nav2 アダプタ専用で、costmapセル中心の量子化誤差を
安全余裕から控除する。生スキャン入力時の既定値は 0、costmapのみの場合はセル解像度の
半分である。

## 運動モデル

| パラメータ | 既定値 | 説明 |
|---|---:|---|
| `motion_model.type` | `diff_drive` | 運動学policy。`diff_drive`、`ackermann`、`omni` |

`diff_drive`と`ackermann`はNav2標準の車体指令`(linear.x, angular.z)`、すなわち前進速度とヨーレートを
出力する（`linear.y`は常に0）。`omni`は`linear.y`も出力するため、下位の車両controllerがこれを honor
する必要がある。
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

`omni`では**回避を担うのは横速度であり、ヨーレートではない**。全方向車体は向きを変えずに障害物を
横へ避けられるため、ヨーを探索しても回避には寄与しない。したがって候補格子は
「前進速度 × 横速度」であり、差動二輪の「前進速度 × ヨーレート」と**同じ2次元**である。三次元格子には
ならない。ただし候補数は同じではない。同梱の全方向設定の値（`limits.v_max` 0.4、`limits.vy_max` 0.3、
`limits.w_max` 1.0、`v_samples` 5、`vy_samples` 15、`w_samples` 25）で現在速度 (0.20 m/s, 0.0 rad/s)
から測ると、全方向96件に対し差動二輪130件である（R18 M5・H2）。**この対は`w_refine_steps`を0に
した粗格子だけの数である。** `bac_core.hpp`の既定は`w_refine_steps = 3`で、同梱の全方向yamlは
これを設定しない。同梱設定をそのまま読み込むと細分候補が2 × 3件加わり、**全方向102件に対し
差動二輪136件**になる（実測、開けた場・経路は直進・上記と同一の現在速度）。細分は粗格子の勝者が
停止行でないときだけ走るので、この+6は勝者が動く行である限り成り立つ。掃引は
[algorithm.md](algorithm.md)を参照。

ヨーレートは探索の自由度ではなく姿勢規範として決まる。局所経路接線への比例制御（利得`heading_gain`）
であり、**候補生成の前に確定して全候補で共通の値を持つ**。この順序に意味がある。ヨーを先に決めるため、
採点し接触判定する軌道と実際に走る軌道が一致する。

狭所ではさらに、左右クリアランスの不均衡を姿勢規範へ加える。斜行する矩形は直進する矩形より広い幅を
掃引するため（0.7 × 0.5 mの車体が55°で0.86 m、直進なら0.5 m）、横方向の余裕が最も乏しい場所でこそ
斜行は高くつく。機体を隙間へ向ければ、車体が既に占めている幅のまま同じ場所へ到達できる。この項は
`tightness`で重み付けされ、かつ「両側がcap以内**かつ**正面が塞がっていない」通路条件でのみ働く。
孤立障害物では左右クリアランスが障害物自身の両縁を指すため、均衡させると障害物へ突っ込む。

速度上限は軸ごとではなく**速度ベクトル**に掛かる。軸ごとなら`hypot(limits.v_max, limits.vy_max)`まで
出てしまう。近接速度ガバナのcapは横軸にも掛かる。ただし速度ベクトルのノルム上限は`|limits.v_min|`まで緩む。
後退候補がgovernorで消えないようにするためで、差動二輪と同じ扱いである（既定の`v_min`は-0.1）。前進軸だけに掛けると、障害物の手前で前進は
減速しながら横滑りは`limits.vy_max`のまま続き、掃引幅が最大になる方向だけが減速対象外になる。

`heading_gain`には一律に良い値が無い。goal姿勢の要求が大きいほど、姿勢を合わせる代償として経路が
伸びる。開所でgoal (4, 2)へ向かい、goal姿勢を要求したときの実測は次のとおりである（直線距離4.47 m）。

| `heading_gain` | goal姿勢 -2.8 radでの走行距離 / 最終ヨー誤差 | goal姿勢 +3.0 radでの走行距離 / 最終ヨー誤差 |
|---:|---|---|
| 0.5 | 4.43 m / 0.602 rad | 4.40 m / 0.517 rad |
| 1.0 | 4.47 m / 0.289 rad | 4.44 m / 0.141 rad |
| 1.5 | 8.63 m / 0.006 rad | 4.47 m / 0.060 rad |
| 3.0 | 8.06 m / 0.003 rad | 4.50 m / 0.005 rad |
| 5.0 | 7.72 m / 0.067 rad | 4.53 m / 0.002 rad |

1.0以下ではNav2既定の`yaw_goal_tolerance` 0.25 radを満たさない場合がある。1.5以上では姿勢は合うが、
極端なgoal姿勢では走行距離が倍近くなり、最終位置誤差も0.29 mから0.37〜0.49 mへ広がる。同梱設定の
1.5は、姿勢を満たす最小の値として選んでいる。

`omni`では`limits.vy_max`が正でなければならない。0のままモデルを選ぶと、操舵できない差動二輪へ黙って
degradeするため、configure時にthrowする。**横移動は車体の側方に対するセンサ被覆を要求する。**
これは`limits.v_min`が後退について負うのと同じ注意である。

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
| `limits.vy_max` | 0.0 | `omni`のみ。横速度の権限 [m/s]。正でなければならない。側方のセンサ被覆が前提 |
| `heading_gain` | 1.5 | `omni`のみ。姿勢規範の比例利得 [1/s]。0で機体方位を固定する（360°センシングを持つ車体向け）。上限は検証しない（下記の実測帯を参照） |
| `vy_samples` | 15 | `omni`のみ。前進速度1行あたりの横速度サンプル数。`w_samples`の対応物で、3以上 |
| `velocity_min` | 0.005 | これ未満の出力速度を 0 に丸める [m/s]。Ackermannでは速度なしのヨーレートが実現不能なため`angular.z`も同時に0にする。適用条件とR19 L10の訂正は[deadband（`velocity_min` / `angvel_min`）の適用条件](#deadbandvelocity_min--angvel_minの適用条件)を参照 |
| `angvel_min` | 0.01 | 差動二輪と全方向モデルで適用し、これ未満の出力角速度を0に丸める [rad/s]。適用条件・4経路・実測内訳は[deadband（`velocity_min` / `angvel_min`）の適用条件](#deadbandvelocity_min--angvel_minの適用条件)を参照 |

差動二輪のヨーレート候補とAckermannの曲率候補は、狭所で必要な修正円弧を常に残すため、現在値
まわりの加速度窓ではなく全設定範囲を評価する（原DWAからの意図的な逸脱）。出力段ではどちらのモデルも
`limits.acc_w`と`control_period`から1制御周期で到達可能なヨーレートへクランプし、Ackermannでは
その後さらに`turn_radius_min`の曲率上限を再適用する。クランプ後の定曲率円弧で停止可能性を再検証し、
追加減速が必要なら`v`と`w`を同率で下げて曲率を保存する。比例後の`w`が角減速度範囲を外れる場合は
到達可能範囲へ再制限してその円弧を再検証する。`limits.acc_w`は車体ヨー加速度であり、Ackermannの
road-wheel操舵速度を保証するものではない。加速・操舵過渡中の掃引軌道とjerkは未評価であり、各制限の
実行は下位controllerの責務とする。

### deadband（`velocity_min` / `angvel_min`）の適用条件

`velocity_min`: これ未満の出力速度を 0 に丸める [m/s]。R19 L10: この丸めのスキップは `applyCommandDeadband` 全体に掛かるので、`angvel_min` と同じく、**丸めた結果が接触前に停止できなくなる場合は丸めを行わない**（`angvel_min` の行にしか書いていなかった）

`angvel_min`: 差動二輪と全方向モデルで適用し、これ未満の出力角速度を0に丸める [rad/s]（R18 M7。以前は「差動二輪のみ」と書いていたが偽）。Ackermannでは低速でも小さなヨーレートが有意な曲率を表すため適用せず、曲率が0とみなせるときだけ`angular.z`を0にする。いずれのモデルでも、丸めた結果が接触前に停止できなくなる場合は丸めを行わない。R19 L12: 経路は**4つ**ある——(1) 丸めが指令を変えない → 丸めた値（＝選択指令）をpublish、(2) 丸めが指令を変え、**かつ丸めた指令が停止可能** → **丸めた値をpublish**（丸めが指令を変えたケースの中ではこれが多数派）、(3) 丸めが指令を変え、丸めた指令は停止不能だが選択指令は停止可能 → **選択指令をそのままpublish**、(4) 丸めた指令も選択指令も停止不能 → `(0, 0, 0)` をpublish。1巡目の対応はここを「3つの分岐」と書き、(2) を落としたうえで (2) の条件から「丸めた指令が停止不能」も落としていた。差動二輪の乱択tickでの実測（`angvel_min` 0.01固定、`main` と同一入力）: 通路＋至近正面点で `current.w` を ±[0.085, 0.124] から引く生成器の400000 tickでは、**deadbandの判定に到達したのが216715 tick**で、その内訳が (1) 197156・(2) 19377・**(3) 182**・(4) 0。残る183285 tickは**全件が緊急層の早期return**である（`BacCore::process()` が判定より前に返る箇所は緊急層と `path.empty()` の2つだけで、`path.empty()` は測って0件だった。183285 + 0 + 216715 = 400000）。障害物clusterを任意方位に1つ置き `current.w` を ±1 で一様に引く生成器の40000 tickでは、到達30712 tickの内訳が (1) 30415・(2) 297・(3) 0・(4) 0 である（R19 2巡目はここで (1) を「総tick数 − (2)」として算出しており、到達しなかったtickを (1) に算入していた。R19検証で判明）。同梱回帰 `testDeadbandIsSkippedRatherThanBreakingAdmissibility` では移動21342 tickのうち (3) が255件である。**この試験の測定条件は上の2行とは違う**——全方向モデルの乱択単発tickを30000回（毎回新しい `BacCore`、閉ループ走行ではない）、しかも `angvel_min` は同梱の0.01ではなく **[0.05, 0.35] rad/s から引いている**（分岐到達を増やして件数を帯にするため。試験のdoc commentにその旨がある）。上で宣言した「`angvel_min` 0.01固定」はこの255/21342には掛からない。(4)は多重防御であり、本ラウンドの上記2条件（合計44万tick）でも到達0件、R18 M8は`velocity_min`まで振った36万tickで発火4112件・(4)到達0件を報告している

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
| `weights.hysteresis` | 0.6 | 前回の出力指令（到達可能性クランプ後、deadbandを適用したならその後。deadbandが接触前停止を壊すため適用されなかったtickでは、選択された指令そのもの。R18 L2）との差。差動二輪ではヨーレート差 [score per rad/s]、Ackermannでは曲率差 [score per 1/m]、`omni`では横速度差 [score per m/s] |
| `weights.squeeze` | 0.5 | 側方余裕が小さいときの速度ペナルティ |

`weights.hysteresis`の単位はモデルで異なる。差動二輪では[score per rad/s]でヨーレート差に、
Ackermannでは[score per 1/m]で曲率差にかかるため、同じ重み値でも実効的な強さは変わる。Ackermann側の
項の大きさは概ね`2 / turn_radius_min`に比例し、差動二輪側は`2 * w_max`に比例する。同梱の
Ackermann設定例（`turn_radius_min` 1.0、`w_max` 0.8）では前者2.0に対し後者1.6であり、一致しない。
さらにAckermannの曲率項は、差動二輪のヨーレート項と違って速度とともに縮まない。したがって
**この重み値はモデル間で移らない**。実測では、差動二輪の既定値0.6を同梱Ackermann設定へそのまま
入れると経路追従が圧倒され、車両はgoalへ収束せず周回する（`bac_ackermann_scenarios`が落ちる）。
同梱のAckermann設定例が配布する値は0.3である。`turn_radius_min`を小さくすると
`weights.hysteresis`が`weights.clearance`を圧倒しうる点も同じ理由によるので、モデルを変えたら
重みは必ず評価し直す。

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
| `scan_topic` | nav2 | 空 | 生スキャン。空ならcostmapの lethal セルを使用 |
| `scan_timeout` | 両方 | 0.5 | スキャン鮮度 [s]。nav2 は古いとcostmapへフォールバック |
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
