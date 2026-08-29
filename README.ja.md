# bilateral_arc_clearance_controller

[English](README.md) | 日本語

Bilateral Arc Clearance（BAC、左右分離円弧クリアランス）は、候補円弧の左右に残る自由幅を直接評価する、
差動二輪向けのNav2ローカルcontrollerです。DWAの速度候補と停止可能性判定を基礎に、狭い開口では
実観測上の左右クリアランスを釣り合わせ、広い場所ではグローバル経路の追従を優先します。

![BACが候補円弧の左右クリアランスを評価する幾何](docs/images/bac_geometry.svg)

提供する構成要素は次の3つです。

- `bac::BacCore`: ROSに依存しないC++17アルゴリズム
- `bac::BacController`: Nav2 `nav2_core::Controller`プラグイン（ROS 2 Jazzyで検証）
- `bac_filter_node`: 既存`cmd_vel`を生スキャンで整形する評価・レガシー統合用ノード

ライセンスはMIT、現在のパッケージバージョンは0.1.0です。

## 目的と主張の範囲

BACは、地図上の経路だけに障害物回避を委ねず、robot frameの局所観測を各制御周期で評価します。
これにより、単一pose estimateに基づくグローバル経路を有限周期で更新する構成と比べて、
**地図―odom間の横ずれと再計画遅延に対する感度を低減する**ことを目的とします。

これは上流から独立した安全保証ではありません。次の前提が必要です。

- 障害物観測の視野・鮮度・外部パラメータが用途を満たす
- planをbase frameへ変換するTFと、現在速度の推定が利用できる
- footprint、制御周期、制動能力が実機を保守的に表す
- 下位速度制御器が速度・加速度制限を実行できる
- 障害物を制御周期内では静的点群として扱える

生スキャンが有効な間、障害物幾何と左右クリアランスは地図―odom誤差に直接依存しません。一方、
経路追従項はTF変換後のplanに依存し、スキャン異常時のcostmap fallbackは再びcostmapとTFに依存します。
したがって本パッケージが主張するのは、上記前提と評価範囲内での**感度低減**であり、任意の上流異常に
対する不変性ではありません。設計上の性質と実測結果の区別は[アルゴリズムと保証範囲](docs/algorithm.md)
に整理しています。

## 評価結果

ROS 2 Jazzyで、同一車体・NavFn・1 Hz再計画・world・10 Hz local costmap入力・controller候補は
前進のみ・共通0.4 m/s上限・共通actuator加速度制限を使い、公平条件評価として18シナリオ × 3 run × 4 controller = 216 episodeを
実行しました。controller固有の軌道生成、horizon、critic、tuningは必然的に異なります。
共通Nav2 recovery treeの`BackUp`は有効なため、system全体を前進のみにはしていません。

| controller | 成功 | 衝突 | 成功時平均 | 中央値 | 最接近の最悪値 |
|---|---:|---:|---:|---:|---:|
| BAC（matched） | 54/54 | 0 | 29.7 s | 28.4 s | 0.078 m |
| DWB | 48/54 | 2 | 24.6 s | 25.2 s | 0.000 m |
| MPPI（matched） | 51/54 | 0 | 27.9 s | 27.3 s | 0.049 m |
| RPP | 48/54 | 0 | 24.2 s | 24.8 s | 0.016 m |

BACはmatched評価の全episodeを完走し、他設定が完走しなかった出現障害物で3/3、1.5 m通路と
0.25 mの合成自己位置横ずれでも3/3成功しました。一方、Z字路では明確に遅くclearanceも小さく、
clutterの1反復では完走まで82.8 s停止しました。別のBACアブレーション216 episodeでは、左右均衡項を
外した極狭路offsetで、平均時間29.2→42.4 s、clearance 0.227→0.174 m、平均横偏差
0.028→0.053 mとなりました。全variantは54/54完走し、基準BACが後退を選ばなかったためescapeの
寄与は未同定です。

これは限定的な決定論的シミュレーション観測で、独立な成功確率、実機安全evidence、一般的優越の証明
ではありません。条件、限界、従来の機能有効system-level評価、既存controllerとの差は
[公平条件比較とアブレーション](docs/ablation_and_matched_evaluation.md)および
[手法比較](docs/method_comparison.md)を参照してください。

### 動画evidence

[![BACとDWBの左右同期replay](docs/media/bac_vs_dwb_matched_appearing_obstacle_thumbnail.jpg)](docs/media/bac_vs_dwb_matched_appearing_obstacle.mp4)

[25.5秒の左右比較MP4を見る](docs/media/bac_vs_dwb_matched_appearing_obstacle.mp4)。matched
`appearing_obstacle/run1`をsimulation時刻で同期し2倍速再生した。BACは完走し、DWBは最終的に中断した。
画面下には3反復集計（BAC 3/3、DWB 0/3）も区別して表示する。これは保存済み2D ray-cast traceの
replayでGazebo映像ではなく、[入出力hash](docs/media/bac_vs_dwb_matched_appearing_obstacle_evidence.json)を
併記している。

[![Gazeboで余裕を持つ回避と狭所通過を行うBAC](docs/media/bac_gazebo_adaptive_clearance_thumbnail.jpg)](docs/media/bac_gazebo_adaptive_clearance.mp4)

[34.9秒のGazebo MP4を見る](docs/media/bac_gazebo_adaptive_clearance.mp4)。左から右への連続takeで、静的
offset障害物から車体距離0.31 mを保ちつつ0.80 m迂回し、8.3秒で中心線から0.30 m以内へ復帰した後、
幅1.0 mのgateを中心偏差0.014 m以内で通過した。車体0.50 mと左右marginを合わせた必要幅は0.74 mで、
最終x=11.91 m、停止・車体接触0だった。局所障害物horizonは明記した2.5 mである。
[同期telemetry](docs/media/bac_gazebo_adaptive_clearance_telemetry.csv)、
[9判定と入力hash](docs/media/bac_gazebo_adaptive_clearance_evidence.json)、
[再現環境](examples/gazebo/README.md)を保存している。

左右replayは選択runと集計の可視化、Gazebo 1系列は定性的な統合evidenceであり、独立反復、実機evidence、
安全検証の代替ではない。

## Nav2での位置づけ

BACは、plan・局所障害物・現在速度から次の`(v,w)`を選び、経路からの局所逸脱も含めて進行を継続するため、
Nav2ではController Serverのプラグインが主な適用場所です。

```text
Planner → BAC / DWB / MPPI → Velocity Smoother → Collision Monitor → base
```

Collision Monitorは最終段で停止・減速する独立安全層であり、BACの代替ではありません。狭路だけBACを
使う場合は、複数controllerを登録し、BTのControllerSelectorなどで切り替えられます。costmap layer、
DWB critic、Collision Monitor、`bac_filter_node`との使い分けは[Nav2統合ガイド](docs/nav2_integration.md)
にまとめています。

## クイックスタート

ROS 2ワークスペースで依存関係を解決してビルドします。

```bash
cd /path/to/colcon_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select bilateral_arc_clearance_controller
colcon test --packages-select bilateral_arc_clearance_controller
colcon test-result --verbose
```

最小構成例です。車体寸法、制動能力、後方視野は実機に合わせて変更してください。
install対象のより完全な例は [`config/bac_controller.yaml`](config/bac_controller.yaml) にあります。

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "bac::BacController"
      scan_topic: /scan
      footprint.front: 0.5
      footprint.rear: -0.5
      footprint.width: 0.95
      limits.v_max: 0.4
      limits.v_min: 0.0  # 十分な後方観測がある場合だけ負値を使う
      stop_decel: 0.8
```

ROSなしでもcoreと閉ループシナリオを検証できます。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

13本の閉ループシナリオはLiDARレイキャスト、加速度制限アクチュエータ、ユニサイクル運動学を含みます。

```bash
./build/bac_scenario_harness --strict --csv-dir traces
python3 test/plot_traces.py --dir traces
```

## 制約

- 2Dの差動二輪と定曲率円弧を前提とし、holonomic / Ackermann motion modelは持ちません。
- 動的障害物の速度・将来位置は推定しません。
- 角速度目標は1制御周期後に到達可能な範囲へ制限しますが、角加速度過渡中の掃引軌道とjerkは未評価です。
- 後退は後方観測範囲が十分な場合だけ有効にしてください。
- 生スキャン異常時はcostmapへfallbackします。完全なfail-stopはCollision Monitor等の独立層で構成してください。
- 0.1.0はシミュレーション中心です。実機の遅延、滑り、外れ値、周期超過は別途評価が必要です。

## ドキュメント

- [ドキュメント索引](docs/README.md)
- [アルゴリズムと保証範囲](docs/algorithm.md)
- [Nav2統合ガイド](docs/nav2_integration.md)
- [パラメータリファレンス](docs/parameters.md)
- [既存手法との比較と評価](docs/method_comparison.md)
- [BACアブレーションと公平条件比較](docs/ablation_and_matched_evaluation.md)
- [再現可能なGazebo evidence](examples/gazebo/README.md)
- [リリースレビュー履歴](docs/release_review_history.md)
- [Public 公開準備チェックリスト](docs/public_release_checklist.md)

## ライセンス

[MIT License](LICENSE)
