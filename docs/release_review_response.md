# リリース前レビューへの対応

対象レビュー: [release_review_findings.md](release_review_findings.md)（2026-08-28、`4effb2b` 時点）  
対応リビジョン: `bf1156c`（コード）+ 本ドキュメント整備コミット

## Critical

| # | 指摘 | 対応 |
|---|---|---|
| 1 | plan と pose の座標フレーム混在 | **修正**。`transformPlan()` を TF 変換ベースへ全面書き換え（frame_id 欠落・TF 失敗は `nav2_core::ControllerTFError`、最近傍 pose からの窓切り出しに変更）。指摘の通り旧実装は locdrift シナリオで注入誤差を打ち消しており、当時の自己位置ズレ結果は根拠として無効だった。修正後に全評価を再生成した（下記）。 |
| 2 | ズレ量の表記とシナリオ設定の不一致 | **修正**。`corridor_locdrift_15x` は 0.25 m（"15" は通路幅 1.5 m）であることを明記し、README・比較資料の数値を訂正。さらにズレ量スイープ（0.10/0.15/0.20/0.25 m、`results_driftsweep/`）で限界値を実測: BAC は全域で時間・クリアランス不変（28.8 s / 0.22–0.23）、DWB は 0.25 で衝突、RPP は 0.20 から失敗、MPPI は 0.25 でもクリアランス 0.08・時間 +60% で完走。**TF 修正後も差別化は成立**する（機構はハーネスの `path_offset_narrow` が ROS なしで裏付ける）。 |
| 3 | 実験世代の混在 | **修正**。`results/`（正準: 18 シナリオ × 4 controller × 3 run = 216 episode、単一リビジョン）、`results_driftsweep/`、`results_gap_sweep/` に分離し、旧混在データは `results_v010_mixed/` へ隔離。`run_all.sh` が `provenance.json`（BAC commit SHA・dirty 数・Nav2 version・world/設定ハッシュ・run 条件）を出力する。集計スクリプトへの期待件数検査は未実装（残項目）。 |

## High

| # | 指摘 | 対応 |
|---|---|---|
| 4 | 曲線上の矩形掃引の過小評価 | **修正**。body-hit 帯を旋回外側にコーナー超過分 `sqrt((R+w/2)²+L²)−(R+w/2)` だけ拡幅（外側境界は厳密、内側境界は側辺 `R−w/2` が最近接なので不変、直進は不変）。レビューの再現ケース（v=0.4, w=1.0, 0.5 s 後の前外コーナー）を unit test 化。挙動シフトは Z 字路の保守化のみ（0.286→0.254 m/s）。 |
| 5 | 角速度到達性・制動モデル | **一部修正・一部設計判断として明記**。`stop_decel` 既定を 1.0→0.8 m/s² に変更しプラント制動と一致させ、「実制動能力以下に設定必須」を文書化。未使用 `limits.acc_w` を設定例から削除。w の全域一様サンプリングは意図的な設計（修正円弧を常に候補に残す。加速度窓化は狭所中心化の劣化と引き換えになることを開発時に確認済み）であり、原 DWA からの逸脱として文書化した。角加速度込みロールアウトは今後の課題。 |
| 6 | ROS アダプタの fail-safe 不足 | **修正**。filter node に command timeout（途絶時は出力ゼロ）・odom timeout（途絶時は停止）・最小有効スキャン点数（未満は観測なし扱いで停止）を追加。nav2 プラグインに `scan_min_points`（有効点不足のスキャンを棄却）、costmap 走査時の mutex 取得、configure 時のパラメータ検証を追加。契約の適用範囲（有効な観測・速度入力と設定済み下位制御を前提とする controller 特性）は README・比較資料の表現を限定済み。アダプタ単体テストは残項目。 |

## Method comparison

- **Collision Monitor**: 「Nav2 Collision Monitor との関係」節を追加（CM は cmd_vel を制限するが操舵しない独立安全層、BAC は操舵まで扱う controller、併用を推奨構成として明記。併用ベースライン評価は残項目）。
- **「原理的に代替できない」の限定**: レビュー推奨の表現（単一 pose 推定 + 有限周期更新の構成に限定、belief-space planning 等の存在を明記、「免疫」→「地図↔オドメトリ間の座標系誤差の影響を受けない（センサ外部パラメータ誤差・遅延は残る）」）へ修正。
- **gap sweep**: MPPI の緩やかな減速を明記し、BAC の特徴を「応答の大きさ + 狭すぎる開口での進入拒否」に修正。0.10 m 統計の適用範囲（18 シナリオ集合）と拒否境界での最接近（約 0.11 m）を明記。

## 可読性・保守性

- `goal_dist` → `path_dist`、`best_goal_dist` → `best_path_cost` へ改名、SVG の `local goal` ラベルを `path projection` へ変更。
- costmap mutex・configure 時検証・`transformPlan` の最近傍 prune は上記の通り修正。
- README の 16/18 シナリオ表記を統一、未使用パラメータを設定例から削除。
- **残項目**: `BacCore::process()`（約 630 行）の責務分割（`PathStationModel` / `ProximityGovernor` / `CandidateSampler` / `SweptFootprintEvaluator` / `CandidateScorer`）。挙動保存リファクタとして次リリースサイクルで実施予定。

## 追加評価条件（残項目）

scan 遅延・欠落・外れ値、odom バイアス・滑り、制御周期 jitter、外部パラメータ誤差、
角加速度 model mismatch、noisy seed 反復と信頼区間、worst-case 実行時間、
Collision Monitor 併用ベースライン、score 項の ablation は未実施であり、
実機評価と合わせて次フェーズの計画とする。

## 再検証結果（対応後）

| 確認項目 | 結果 |
|---|---|
| plain CMake Release build（-Wpedantic） | 警告 0 |
| core unit test（コーナー掃引テスト含む） | 成功 |
| 13 closed-loop scenarios（--strict） | 成功 |
| Jazzy コンテナ colcon build | 成功 |
| nav2 ベンチマーク再生成（216 + 32 + 24 episode、単一リビジョン） | BAC 54/54・衝突 0・0.10 m 未満接近 0 |
