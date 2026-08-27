# リリース再々レビューへの対応

対象レビュー: [release_rerereview_findings.md](release_rerereview_findings.md)（2026-08-28、`c745293` 時点）

## Critical

| # | 指摘 | 対応 |
|---|---|---|
| 1 | 進行角の周期正規化不具合 | **修正**。`firstContactArcLength()` の進行角を `std::fmod` で `[0, 2π)` に正規化（`π−asin` 系の交点角で σ(ψ0−θ) が +2π を超え、近接触が一周後扱いで棄却されていた）。反例（v=−0.40, w=−0.50, 接触 0.19 m）を固定 unit test 化。property test は乱数 400 反復に加えて**決定論的な象限グリッド**（前後進 × 左右旋回の全組合せ × 車体周囲の点グリッド、計 5000 超ケース、`eval_angle_max` 窓での真値総当たり）を常設——見逃し 0・幻接触 0。修正後に正準 216 episode・drift sweep・gap sweep を第 4 世代として再生成（下表）。 |

## High

| # | 指摘 | 対応 |
|---|---|---|
| 2 | 角加速度過渡が未評価 | **部分修正として保証範囲を限定**（推奨対応 5 を採用）。保証は「出力角速度の目標値が 1 制御周期後に到達可能」まで、過渡中の掃引軌道・jerk は未評価であることを parameters.md・README に明記。併せて指摘 3・4 を修正: クランプ後円弧の停止可能性再検証は `turn_radius_min` 未満でも実施（厳密接触は任意半径で有効）、filter node の `CLEAR` 透過にも同一の角速度レート制限を適用（到達性契約が両出力経路を覆う）。角加速度込み rollout（最初の 1 周期を角加速度付き積分）は残項目。 |

## Medium

| # | 指摘 | 対応 |
|---|---|---|
| 3 | padding 評価のズレ量表記 | **修正**。0.25 m と訂正済み。 |
| 4 | ベンチ環境の古い説明 | **修正**。「本リポジトリ外の評価環境」記述を追跡済みハーネス+provenance の説明へ置換。`nav2_benchmark/README.md` を現行構成（正準 18 シナリオ、drift/gap sweep、`RESULTS_ROOT` 分離、provenance、`summarize.py` 運用）へ全面更新。 |
| 5 | README の角加速度説明 | **修正**。「core が保証するもの（1 周期後に到達可能な目標値）/ まだ保証しないもの（過渡掃引・jerk）/ 下位制御器へ要求するもの」を分けて記載。 |
| 6 | 実行時間の測定根拠 | **修正**。`bac_perf_benchmark`（warm-up 50 + 2000 反復、p50/p95/max、raw CSV 出力）を追加。実測: 1000 点で p50 388 / p95 404 / max 672 µs、4000 点で p50 322 / p95 336 / max 562 µs（AMD Ryzen 9 5950X、g++ 11.4、-O2）——20 Hz 予算比で最悪 1.3%。条件・raw は `nav2_benchmark/perf/`（provenance.txt + perf_raw.csv）に保存。 |

## 修正後の再生成（第 4 世代、全実験群 bac_dirty=0・bench_dirty=0）

| 確認項目 | 結果 |
|---|---|
| 正準 216 episode | BAC 54/54・衝突 0・0.15 m 未満接近 0（最悪 0.154）・平均 27.8 s |
| drift sweep（0.10〜0.25 m） | BAC 全域不変 28.8 s / clr 0.22〜0.23。0.25 m で DWB 1/2 中断・RPP 0/2 中断 |
| gap sweep | 段階減速 0.39→0.24→0.10→進入拒否（最接近 0.11、衝突なし） |
| core unit（反例 test・象限グリッド含む） | 成功 |
| 13 closed-loop scenarios（--strict） | 成功 |
| plain CMake Release build（-Wpedantic） | 警告 0 |
| Jazzy コンテナ colcon build | 成功 |

## 残項目（次サイクル、[前回対応表](release_rereview_response.md)から継続）

角加速度込み候補 rollout、アダプタ単体テスト、`process()` の責務分割、noisy seed 反復と信頼区間、
外乱系評価、Collision Monitor 併用ベースライン。
