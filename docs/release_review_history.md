# リリースレビュー履歴

[English](en/release_review_history.md) | 日本語

## 現在の状態

- 確認日: 2026-09-01
- 評価対象package: `24466ae`。R11の対象だった作業ツリーは `8fd0d7e`（実装）/ `5951e0b`（文書）/
  `24466ae`（レビュー記録）としてコミットされ、R12はその3コミットを対象とする
- benchmark: `026a17a`（正準）/ `4fed3d2`（公平条件比較・ablation）。R11・R12ともbenchmarkを再実行していない
- 判定: コードレビュー **Go**、Public化P0 **完了**。公開操作は所有者判断

全12回のリリースレビューで確認されたCritical / High / Mediumは対応済みである。第10回のLow 3件、
第11回のLow 4件（L1は挙動変更を伴わない文書化で完了）、第12回のLow 3件も対応済みで、release blockerは
ない。第12回は第11回の対応の再検証であり、第11回がM1を完了と判断した廃案設計記述が公開ヘッダに残存
していたこと、Nav2 adapterの差動二輪coverageが追加ではなく置換されていたこと、および同梱のAckermann
設定例が自身の回帰試験に通らなかったこと（第11回L4の文書化が誤った前提に立っていた）を検出した。
2026-08-29にROS adapter testと入力source diagnosticsを追加し、同日に公平条件比較216 episodeとBAC
ablation 216 episodeを追加した。2026-09-01にはAckermann motion modelを追加し、第11回レビューで
`setParams`の例外安全（High 1件）を修正した。次cycleへ残す設計・評価項目は、角加速度過渡を積分した
rollout、実機外乱評価、Collision Monitor併用baseline、`BacCore::process()`の責務分割、
およびfilter node仮想pathの旋回円clamp（R12 L5）である。Ackermannシナリオの拡充（R11 L5）は
2026-09-01に完了し、安全停止（前進のみ／後退退避）、狭路centering、clutter走破を追加した。Ackermannの検証は決定論的な単体検査と閉ループ回帰のみで、
実車evidenceは無い。

個別文書は各時点の判断を保存する監査証跡であり、途中で撤回された結論も削除していない。特に第8回の
artifact mtimeによるdomain分離監査は第9回で撤回され、正準216 episodeを修正後runnerで再生成した。
命名と更新方法は[レビュー記録規則](reviews/README.md)に従う。

## レビューの要約

| ID | 主な論点 | 最終対応 | 詳細 |
|---:|---|---|---|
| R01 | plan / pose frame混在、評価世代混在、矩形swept footprint、adapter fail-safe、比較範囲 | TF変換、結果分離、幾何修正、入力timeoutと文書限定 | [指摘](reviews/r01-2026-08-28-findings.md) / [対応](reviews/r01-2026-08-28-response.md) |
| R02 | 曲線矩形接触の厳密性、`+Inf` scan、座標誤差主張、角速度到達性、provenance | 閉形式接触、scan意味論、限定表現、出力角速度制限、tracked benchmark | [指摘](reviews/r02-2026-08-28-findings.md) / [対応](reviews/r02-2026-08-28-response.md) |
| R03 | 接触角の周期化、角加速度過渡、performance根拠 | 周期bug修正とproperty test、保証範囲限定、raw付きmicrobenchmark | [指摘](reviews/r03-2026-08-28-findings.md) / [対応](reviews/r03-2026-08-28-response.md) |
| R04 | container path、episode欠損、performance revision | 完全性gate、実行例修正、clean revisionで再測定 | [指摘](reviews/r04-2026-08-28-findings.md) / [対応](reviews/r04-2026-08-28-response.md) |
| R05 | gate status、古いepisode再利用、performance source、raw列 | status伝播、preflight、clean provenance、`eval_pts`追加 | [指摘](reviews/r05-2026-08-28-findings.md) / [対応](reviews/r05-2026-08-28-response.md) |
| R06 | preflight前の上書き、overwrite伝播と削除範囲、file-open異常 | write前preflight、限定削除、host env伝播、I/O fail-fast | [指摘](reviews/r06-2026-08-28-findings.md) / [対応](reviews/r06-2026-08-28-response.md) |
| R07 | 期待外artifact、aggregate / provenance失敗、入力検証 | root全体の集合検査、finalization gate、名前・件数・RTF検証 | [指摘](reviews/r07-2026-08-28-findings.md) / [対応](reviews/r07-2026-08-28-response.md) |
| R08 | 使用中ROS domain再利用、先頭0、trace schema、runner test | PID-domain free-list、10進正規化、trace意味論、shell integration test | [指摘](reviews/r08-2026-08-28-findings.md) / [対応](reviews/r08-2026-08-28-response.md) |
| R09 | 旧dataset監査の証明不足、launch status、pool fail-open | 監査撤回、正準再生成、parent manifest、status伝播、fail-closed | [指摘](reviews/r09-2026-08-28-findings.md) / [対応](reviews/r09-2026-08-28-response.md) |
| R10 | tree hash再現性、manifest同一性、reuse用語、raw archive | tracked source hash、集合/schema検査、用語分離、archive tool | [指摘](reviews/r10-2026-08-28-findings.md) / [対応](reviews/r10-2026-08-28-response.md) |
| R11 | Ackermann対応: 却下設定の半適用、文書と実装の乖離、テストの空振り | 検証先行によるsetParams例外安全、廃案設計記述の訂正、ミューテーション9件を殺すテスト追加 | [指摘](reviews/r11-2026-09-01-findings.md) / [対応](reviews/r11-2026-09-01-response.md) |
| R12 | 同梱Ackermann設定が回帰試験に不合格、公開ヘッダの廃案設計記述、差動二輪adapter coverageの置換 | 設定例の重み訂正と同梱設定シナリオ追加、ヘッダ訂正、既定設定試験の復帰とAckermann試験の分離 | [指摘](reviews/r12-2026-09-01-findings.md) / [対応](reviews/r12-2026-09-01-response.md) |

## 現在の検証contract

- plain CMake Release buildとCTest 7件、ROS 2 Jazzy/Nav2環境ではadapter結合試験を加えたCTest 8件。
  adapter結合試験は既定設定（差動二輪）で実行し、Ackermannのパラメータ配線は独立した試験で検査する。
  両者は同一tickの表と裏を検査するため、`motion_model.type`の誤解決は必ずどちらかが検出する。
  Jazzyコンテナではさらに、`ackermann`ラベル試験の存在と通過、インストール済みAckermann設定、
  実ノードが不正な`motion_model.type`と非正の`turn_radius_min`を拒否することを検査する。
- core unit / property testと17 closed-loop scenarios、Ackermann 10 closed-loop scenarios。Ackermann側は
  同梱設定例そのものを走らせる試験を含み、狭路centeringとclutterでは横偏差・曲率符号反転・1周期あたり
  曲率変化・停止tickを閾値検査する。閾値は正常時と破壊時の実測値を分離するように決めている。
- 差動二輪とAckermannのmotion model単体試験。Ackermann側は候補格子、旋回半径拘束、refinement、
  clearance probe、deadband、実行時のmodel切替、却下設定後の可用性を検査する。
- scan投影・plan変換/pruneの単体試験、およびplugin lifecycle、TF error、scan fallback、speed limit、
  diagnosticsの結合試験。
- benchmark完全性checker 31 tests。
- runner orchestration 63 checks。
- controller × scenario × runの期待集合、episode / trace schema、aggregate、provenance昇格のfail-fast。
- PID、domain、期待label、launch / reap区間、終了statusを持つdomain manifest。
- Git追跡sourceから再現可能なprovenance v2と検証script。
- 検査合格後だけraw datasetとSHA256SUMSを作るrelease archive script。

## 正準release evidence

- 18 scenarios × 3 runs × 4 controllers = 216 episodes。
- BAC commit `1f9911e`、benchmark commit `026a17a`、両worktree dirty 0、provenance v2。
- 90個すべてのdomain IDを再利用し、初回以降の再割当126回、保持区間overlap 0。
- BAC 54/54成功、衝突0、最接近0.136 m。DWB 50/54（衝突4）、MPPI 51/54、RPP 47/54。
- 同じrevisionでdrift sweep 32 episodeとgap sweep 24 episodeを再生成し、全272 episodeで欠損・破損0。
- `release_archive_25f12be/`に旧272 episodeと追加432 episode、両benchmark source世代、
  BAC source snapshot、`SHA256SUMS`を保存した。

3 datasetはいずれもprovenance v2で、`bench_tree_sha`、Git tree object、`worlds_sha`の3/3 digestを
`verify_provenance.py`で再現した。container imageは
`sha256:d58fe8c8f5790cd000cf7bdc1b46395ac2567c231cd592ae6d29426ba9eb2737`である。

## 追加P1評価

- BAC `25f12be`、benchmark `4fed3d2`、両worktree dirty 0。
- 公平条件比較: 18 scenarios × 3 runs × 4 controllers = 216 episodes。BAC 54/54、DWB 48/54
  （衝突2）、MPPI 51/54、RPP 48/54。
- BAC ablation: 18 scenarios × 3 runs × 4 variants = 216 episodes。全variant 54/54、衝突0。
- 両datasetとも期待216 / 観測216 / 欠損0 / 破損0 / 期待外0、domain overlap 0、provenance
  digest 3/3再現。
- 左右均衡項を外した狭路で中心化・clearance・到達時間が悪化。raw scanは完走に必須でなく、
  escapeの寄与は基準BACが後退を選ばなかったため未同定。
- 詳細: [BACアブレーションと公平条件比較](ablation_and_matched_evaluation.md)。
- matched `appearing_obstacle/run1`のBAC/DWB左右同期replayを追加し、3反復集計とデータhashを併記した。
- Gazebo Classic 11 / ROS 2 Humbleの回避・復帰・1.0 m gate連続1系列へ更新した。車体最小離隔0.309 m、
  最大横偏差0.799 m、gate内中心偏差0.013 m、body contact 0、最終x 12.03 m。動画・同期telemetry・
  9判定JSONを保存した。

## 公開時チェック

1. 公開後にarchive、`SHA256SUMS`、source tag、container image digestを同じGitHub Releaseへ添付する。
2. 0.1.0がシミュレーション中心であり、実機安全認証を意味しないことをrelease noteへ残す。
3. tag作成後、外部cloneからsourceとarchiveの参照可能性を再確認する。
