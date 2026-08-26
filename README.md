# bilateral_arc_clearance_controller

Bilateral Arc Clearance (BAC, 左右分離円弧クリアランス) 方式の障害物回避アルゴリズムです。上位の速度指令 (v, w) を LiDAR 点群に対して整形し、**余裕があれば軽く避け、余裕が少ない狭所では開口の中心へ滑らかに進入する**挙動を実現します。

- `bac::BacCore` — フレームワーク非依存の C++17 コアライブラリ(std のみ)
- `bac_filter_node` — LaserScan + cmd_vel を使う ROS 2 評価ノード
- `bac::BacController` — nav2 Controller プラグイン(nav2_core 検出時のみビルド、**未実機検証の雛形**)
- `test/` — 閉ループシナリオハーネス(単体で PASS/FAIL 判定、CSV トレース+プロット)

## アルゴリズム概要

入力: ロボット座標系の障害物点群 + 上位速度指令 (v, w) + 現在速度。20Hz 程度の周期で `process()` を呼びます。

1. **緊急停止**: 車体矩形+現在速度の制動距離を、異方性安全マージンの角丸(楕円角)で拡張したゾーンに点があれば即停止。現在速度基準(停止中はコンパクト)で、指令の安全性は評価側が担う
2. **候補円弧の左右クリアランス評価** (`evaluateArc`): 各点について候補円弧の経路中心線からの符号付き横オフセットと縦距離を閉形式で計算。min(左, 右) がその弧の実効通過半幅
3. **スコアリング**:
   `score = クリアランス(車体半幅+回避サイドマージンで飽和) − 忠実度×実行可能性×|w−指令w| − ヒステリシス×|w−前回選択w|`
   - 開空間では多数候補が飽和し、忠実度が指令直近を選ぶ(=透過/軽回避)
   - 狭所では max-min クリアランスが支配し、開口の中心を通る弧が創発的に選ばれる
   - **忠実度の実行可能性スケール** = 指令弧自身のクリアランス/車体半幅(下限あり)。指令弧が物理的に通るなら満額(狭通路進入を減衰させない)、通らない指令(障害物の先のゴール等)は追従価値を失う
   - 回避中は候補窓を前回選択 w 中心に張る(コミットメント)。指令 w 中心だと上位の逆操舵で回避方向が障害物正面を横切ってフリップし続ける
4. **速度整形は縦方向のみ**(選択した操舵 w は維持=詰まったら空き方向へ回頭して自力復帰):
   車体が当たる最初の点までの距離割合 × 横スクイーズ比 × 近接ガバナ(脱出クリープ床付き)

重要な設計不変条件(`Weights` のコメント参照): ヒステリシス < 忠実度(逆転すると開空間で解放不能)、クリアランス重みは 1.0 のまま(上げると狭通路入口で外へ流れる)。

パラメータ・重みは全て `bac::Params` で実行時指定。`Result` は出力に加え selected_w / speed_fraction / command_clearance 等の内部量を返し、チューニングを支援します。

## ビルド

```bash
# ROS 2 ワークスペース (ament)
colcon build --packages-select bilateral_arc_clearance_controller
colcon test  --packages-select bilateral_arc_clearance_controller   # シナリオハーネス実行

# ROS なし (素の CMake): コア + ハーネスのみ
cmake -S . -B build && cmake --build build && ./build/bac_scenario_harness
```

## シナリオハーネス (test/)

レイキャスト LiDAR + 加速度制限アクチュエータ + ユニサイクル運動学の閉ループで 7 シナリオを評価します:

| シナリオ | 層 | 内容 |
|---|---|---|
| open_passthrough | REGRESSION | 障害物なし → 完全透過 |
| safety_stop | REGRESSION | 安全マージン内に障害物 → 即時停止維持 |
| avoid_single_obstacle | TARGET | 余裕のある単一障害物 → 迂回して通過・復帰 |
| corridor_wide | TARGET | 幅2.5m通路 → 素直に通過 |
| corridor_narrow_aligned | TARGET | 幅1.7m通路(回避マージンが両壁に干渉)に正対進入 |
| corridor_narrow_offset | TARGET | 同通路に横0.4mオフセットから進入 |
| corridor_narrow_walled | TARGET | 通路入口が壁の唯一の開口(迂回不可能) → 中心へ漏斗状進入 |

オプション: `--strict`(TARGET 未達も fail、CI 用)/ `--filter 名前` / `--csv-dir DIR` / 重み上書き `--w-fidelity X --w-hysteresis X --w-clearance X --viability-floor X`(スイープ評価用)。

トレースは CSV 出力され、`python3 test/plot_traces.py --dir traces` で世界+軌跡+時系列を PNG 化できます。

## 評価ノード

```bash
ros2 run bilateral_arc_clearance_controller bac_filter_node \
  --ros-args -r scan:=/scan -r odom:=/odom \
             -r cmd_vel_in:=/nav_cmd_vel -r cmd_vel_out:=/cmd_vel
```

- status(0=CLEAR/1=AVOIDING/2=STOP)を `avoid_status` に publish。CLEAR 時は入力指令を素通し
- スキャン途絶(`scan_timeout`, デフォルト0.5s)で出力ゼロ
- パラメータ: `footprint.*`, `safety_margin.*`, `avoid_margin.*`, `weights.*`, `reaction_time`, `w_range`, `sensor.{x,y,yaw}` など

## nav2 プラグイン

`bac::BacController`(`bilateral_arc_clearance_controller_plugin.xml`)は Humble の `nav2_core::Controller` API 準拠の雛形です。プラン→純追尾で意図指令を生成し、コストマップの LETHAL セルを点群としてコアへ渡します。**nav2 環境での動作確認は未実施**のため、導入時にビルド・検証してください。

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "bac::BacController"
      lookahead: 0.8
      desired_speed: 0.4
```
