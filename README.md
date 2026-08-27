# bilateral_arc_clearance_controller

Bilateral Arc Clearance (BAC, 左右分離円弧クリアランス) 方式のローカルプランナです。
Dynamic Window Approach をベースに、走行可能性の評価を左右分離円弧クリアランスに
置き換えたもので、**余裕があれば経路どおりに速く、狭所では開口の中心へ滑らかに進入する**
挙動を実現します。

- `bac::BacCore` — フレームワーク非依存の C++17 コアライブラリ(std のみ)
- `bac_filter_node` — LaserScan + cmd_vel を使う ROS 2 評価ノード(速度指令を仮想経路化)
- `bac::BacController` — nav2 Controller プラグイン(ROS 2 Jazzy で検証済み、`nav2_benchmark/` 参照)
- `test/` — 閉ループシナリオハーネス(単体で PASS/FAIL 判定、CSV トレース+プロット)

## アルゴリズム概要 (v2)

入力: ロボット座標系の障害物点群 + **ロボット座標系のローカル経路** + 現在速度。
20Hz 程度の周期で `process()` を呼びます。

> v1 は上位の速度指令 (v, w) を意図として受けていましたが、nav2 の純追尾のような
> フィードバック型操舵指令を「持続する弧」として外挿すると通路内で実行不能と誤判定され、
> 意図の重みが崩壊する構造的問題があったため、意図表現を経路に改めました
> (単体テストと nav2 が同一の意図表現を共有します)。

1. **緊急停止**: 車体矩形+現在速度の制動距離を、異方性安全マージンの角丸(楕円角)で
   拡張したゾーンに点があれば即停止。マージンは速度でスケール(停止時 50%)し、
   境界上での停止⇔微動のチャタリングを防ぐ
2. **候補生成 (dynamic window)**: 加速度制限内の (v, w) をサンプリング。
   v=0 行は **turn-then-go**(回頭後の向きで直進した場合)として評価し、
   「その場回頭が常に無害に見える」近視眼を排除。旋回半径 `turn_radius_min` 未満の
   前進候補は除外(極小半径弧はクリアランス幾何が縮退するため)
3. **許容性 (DWA admissibility)**: 弧上の最初の車体衝突点までに制動距離+前方マージンで
   停止できない候補を除外
4. **スコアリング**:
   `score = クリアランス×2.0(車体半幅+回避サイドマージンで飽和) − ローカルゴール距離 − 方位誤差×0.15 − ヒステリシス×0.2×|Δw| − 横スクイーズ×0.3×v`
   - クリアランス評価は最低 `min_eval_distance` (1.6 m) 先まで(時間ベースだけだと
     低速時に近視眼化)、ただし曲線候補は弧角 `eval_angle_max` (60°) まで
     (それ以上の外挿は「いずれ反対側の壁に当たる」誤評価を生む)。
     窓外の車体衝突点はユークリッド距離でフェードする毒として反映
     (`blocked_near`/`blocked_far`: 近い=実脅威、遠い=再決定前提で減衰)
   - ローカルゴール = 経路上 `score_lookahead` (2.5 m) 先の点(障害物上にある場合は先へ送る)。
     最近傍経路距離でなくゴール到達度で測ることで、経路上に居座る未知障害物へ
     引き込まれない
5. **近接ガバナ**: 緊急境界近くではサンプリング速度上限を絞る(クリープ床付き)

重要な設計バランス(スイープ済み): clearance 2.0 / goal_dist 1.0 / hysteresis 0.2 —
ヒステリシスを 0.05 まで下げると狭通路内で操舵が振動し、goal_dist をこれ以上上げると
経路上の未知障害物に引き込まれます。

パラメータは全て `bac::Params` で実行時指定。`Result` は出力に加え選択弧のクリアランス・
ゴール距離・許容候補数等の内部量を返し、チューニングを支援します。

## ビルド

```bash
# ROS 2 ワークスペース (ament)
colcon build --packages-select bilateral_arc_clearance_controller
colcon test  --packages-select bilateral_arc_clearance_controller   # シナリオハーネス実行

# ROS なし (素の CMake): コア + ハーネスのみ
cmake -S . -B build && cmake --build build && ./build/bac_scenario_harness
```

## シナリオハーネス (test/)

レイキャスト LiDAR + 加速度制限アクチュエータ + ユニサイクル運動学の閉ループで
7 シナリオを評価します(意図は毎ティック再計算されるロボット→ゴール直線経路):

| シナリオ | 層 | 内容 |
|---|---|---|
| open_passthrough | REGRESSION | 障害物なし → 完全透過 |
| safety_stop | REGRESSION | 安全マージン内に障害物 → 即時停止維持 |
| avoid_single_obstacle | TARGET | 余裕のある単一障害物 → 迂回して通過・復帰 |
| corridor_wide | TARGET | 幅2.5m通路 → 素直に通過 |
| corridor_narrow_aligned | TARGET | 幅1.7m通路(回避マージンが両壁に干渉)に正対進入 |
| corridor_narrow_offset | TARGET | 同通路に横0.4mオフセットから進入 |
| corridor_narrow_walled | TARGET | 通路入口が壁の唯一の開口(迂回不可能) → 中心へ漏斗状進入 |

**v2 で全 7 シナリオが `--strict` 合格**(v1 は狭通路オフセット進入が未達)。

オプション: `--strict` / `--filter 名前` / `--csv-dir DIR` /
重み上書き `--w-clearance X --w-goal-dist X --w-hysteresis X`(スイープ評価用)。
トレースは CSV 出力され、`python3 test/plot_traces.py --dir traces` で PNG 化できます。

## 評価ノード

```bash
ros2 run bilateral_arc_clearance_controller bac_filter_node \
  --ros-args -r scan:=/scan -r odom:=/odom \
             -r cmd_vel_in:=/nav_cmd_vel -r cmd_vel_out:=/cmd_vel
```

- 上位の (v, w) 指令を「指令弧を `virtual_path_length` (3 m) 延長した仮想経路」に変換して
  コアへ渡し、指令速度を上限としてコアが計画
- status(0=CLEAR/1=AVOIDING/2=STOP)を `avoid_status` に publish。CLEAR 時は入力指令を素通し
- スキャン途絶(`scan_timeout`, デフォルト0.5s)で出力ゼロ
- パラメータ: `footprint.*`, `safety_margin.*`, `avoid_margin.*`, `limits.*`, `weights.*`,
  `sensor.{x,y,yaw}` など

## nav2 プラグイン

`bac::BacController`(`bilateral_arc_clearance_controller_plugin.xml`)は
`nav2_core::Controller` 準拠の薄いアダプタです: nav2 のプランをロボット座標系へ変換し、
点群(`scan_topic` 指定時は生スキャン、未指定時はコストマップ LETHAL セル)とともに
コアへ渡すだけで、計画ロジックはすべてコア側にあります。

コストマップ入力ではセル中心の量子化誤差(最大で解像度の半分)を `safety_margin.*` から
自動控除します(`costmap_margin_compensation`、スキャン直結時は既定 0)。

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "bac::BacController"
      scan_topic: /scan        # 推奨: 生スキャン直結
      limits.v_max: 0.4
      sim_time: 2.5
      score_lookahead: 2.5
```

ROS 2 Jazzy の nav2 でベンチマーク済み(`nav2_benchmark/`): 8 シナリオ中、
迂回不可能な幅 1.7 m 通路へのオフセット進入を含む全シナリオで DWB / MPPI / RPP と
同等の成功率(既知の残課題: 正対進入の 1/5 でリプラン直後に入口で停止する
過渡的フレーク)。
