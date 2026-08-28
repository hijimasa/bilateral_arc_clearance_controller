# リリース再々々レビューへの対応

対象レビュー: [release_rererereview_findings.md](release_rererereview_findings.md)（2026-08-28、`1e188e2` 時点、**条件付き Go**）

新規 Critical / High はなし。条件付き Go の条件(Medium)と Low に対応した。

## Medium

| # | 指摘 | 対応 |
|---|---|---|
| 1 | `RESULTS_ROOT` の実行例がホストパス | **修正**。`nav2_benchmark/README.md` の例をコンテナ内パス `/work/nav2_benchmark/...` に変更し、`/work` マウントの説明を追記。 |
| 2 | episode 欠損でも成功終了できる | **修正**。`run_all.sh` に完全性ゲートを追加: controller × scenario × run の期待集合を実行条件から確定し、各 `episode.json` の存在と必須キー(outcome / success / controller / scenario)を検査。欠損・破損が 1 件でもあれば `completeness.json`(expected / observed / missing / corrupt)を出力して**非ゼロ終了**し、集計へ進まない。既存の第 4 世代正準結果に対して検査を実行し、216/216・破損 0 を確認。 |
| 3 | 性能測定の provenance が非一意 | **修正**。クリーンな最終リビジョン(package HEAD、src/include の dirty 0)で再測定し、`perf/provenance.txt` に HEAD SHA・実装 commit・source tree hash・CPU・コンパイラ・ビルドフラグ・測定条件・結果表を記録。`max` は「保存 2000 反復内の観測最大であり WCET 保証ではない」と明記。`bac_perf_benchmark` は `eval_pts` 列を出力し、`max_points=1000` による間引き(2000/4000 点入力が 1000 点より速く見える理由)を明示。再測定値: 1000 点で p50 391 / p95 404 / 観測最大 548 µs(20 Hz 予算比 p95 0.8%)。 |

## Low

| # | 指摘 | 対応 |
|---|---|---|
| 4 | 手法比較表の角加速度説明が古い | **修正**。BAC 制約欄を「角加速度過渡・jerk は未評価」に更新。 |
| 5 | filter node の rate limit に adapter test がない | **残項目として維持**(前回対応表から継続)。次サイクルで command / odometry / status 組合せ固定の ROS adapter unit test を追加する。 |

## 角加速度過渡について(High 相当の確認事項)

レビューの整理どおり「既知の制約として scope された状態」を維持する。以下の表現は使用しない:
指令軌道全体の力学的到達可能性、過渡込みの stop-before-contact 保証、本 controller 単体での
実機独立安全層。実機安全保証を対象にする場合の追加要件(角加速度付き 1 周期積分 rollout、
adapter test、下位制御器込みの遅延・追従誤差評価)は次サイクルの計画に含める。

## 状態

コード本体はレビュー判定どおりリリース可能。条件付き Go の条件(Medium 1〜3)は本対応で解消。
