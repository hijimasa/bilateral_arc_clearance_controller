# リリースレビュー履歴

## 現在の状態

- 確認日: 2026-08-28
- package: `59a78d3`
- benchmark: `604780e`
- 判定: **Go**

全10回のリリースレビューで確認されたCritical / High / Mediumと、第10回のLow 3件は対応済みである。
現在のrelease blockerはない。次cycleへ残す項目は、ROS adapter test、角加速度過渡を積分したrollout、
実機外乱評価、Collision Monitor併用baseline、`BacCore::process()`の責務分割である。

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

## 現在の検証contract

- plain CMake Release buildとCTest 2件。
- core unit / property testと13 closed-loop scenarios。
- benchmark完全性checker 31 tests。
- runner orchestration 63 checks。
- controller × scenario × runの期待集合、episode / trace schema、aggregate、provenance昇格のfail-fast。
- PID、domain、期待label、launch / reap区間、終了statusを持つdomain manifest。
- Git追跡sourceから再現可能なprovenance v2と検証script。
- 検査合格後だけraw datasetとSHA256SUMSを作るrelease archive script。

## 正準release evidence

- 18 scenarios × 3 runs × 4 controllers = 216 episodes。
- BAC commit `f1e2a90`、benchmark commit `13becc0`、両worktree dirty 0。
- 90個すべてのdomain IDを再利用し、初回以降の再割当126回、保持区間overlap 0。
- BAC 54/54成功、衝突0、最接近0.139 m。

正準datasetはprovenance v1であり、当時の`bench_tree_sha`はignored生成物を含むためcommitから再現できない。
source同一性は`bench_commit=13becc0`、`bench_dirty=0`、一致する`worlds_sha`で追跡する。次回runから
Git追跡fileだけを使うprovenance v2になる。詳細は第10回対応文書を参照する。

## 公開時チェック

1. `nav2_benchmark/scripts/make_release_archive.sh`で正準、drift、gap datasetを検証・梱包する。
2. archive、`SHA256SUMS`、source tag、container image digestを同じreleaseへ添付する。
3. READMEとmethod comparisonの数値をarchive内のsummaryと照合する。
4. 0.1.0がシミュレーション中心であり、実機安全認証を意味しないことをrelease noteへ残す。
