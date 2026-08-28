# 第10回リリースレビューへの対応

対象レビュー: [release_rererererererererereview_findings.md](release_rererererererererereview_findings.md)(2026-08-28、`83c5e7a` / `13becc0` 時点。判定 **Go**、残 Low 3 件)

Low 3 件と、チェックリストの raw アーカイブ項目に対応した。正準データセットの再生成は行っていない
(理由は Low 1 の項)。

## Low 1 — `bench_tree_sha` を Git 追跡ソースから再現可能にする

**修正**。`find ws/src scripts Dockerfile -type f` を `git ls-files ws/src scripts Dockerfile` へ置き換え、
`LC_ALL=C sort` で照合順序も固定した。これで `__pycache__/*.pyc` などの ignored な生成物は入らない。
併せて、git 自身の tree object id を `bench_tree_object`(`git rev-parse HEAD:nav2_benchmark`)として
記録し、フィールド定義の世代を示す `provenance_version`(現行 2)を追加した。

検証手段として `scripts/verify_provenance.py <results_root>` を追加した。作業ツリーを信用せず、
**記録済み commit の blob から** `bench_tree_sha` / `bench_tree_object` / `worlds_sha` を再計算して
照合する(clean checkout があれば誰でも実行できる)。v1 の provenance に対しては
「旧定義は ignored 生成物を含むため再現不可、ソース同一性は `bench_commit` + `bench_dirty` +
`worlds_sha` で追跡できる」と明示して `worlds_sha` のみ照合する。

**正準データセットは再生成しなかった**。レビューの判定どおり本項は今回の結果を無効にせず、
`bench_commit=13becc0` / `bench_dirty=0` / `worlds_sha` 一致でソース同一性は追跡できる
(`verify_provenance.py results` で再確認済み)。数値・動画を差し替える再生成は、証拠力を増やさずに
公開値を動かすだけなので見送り、次回の実行から v2 provenance になる。

## Low 2 — manifest checker に期待ラベル集合とスキーマ検査

**修正**。`check_domain_manifest.py` は行数比較だけでなく、次を順に検査するようにした。

1. **スキーマ**: CSV ヘッダの完全一致、pid は正整数、domain は 10〜99、ラベル非空、
   status は整数、`reap >= start`。違反は **exit 2**(docstring・実装・テストで統一)。
2. **同一性**: `--controllers` / `--scenarios` / `--runs` から期待ラベル集合を生成し、
   欠落・余分・重複を個別に報告して失敗させる。PID 重複も検出する。
   `run_all.sh` は `--expect-episodes` ではなくこの 3 引数を渡すようになった
   (行数だけ合っていて中身が違う manifest は通らない)。
3. **分離**: 同一 domain の保持区間の重なり 0。
4. **健全性**: 非ゼロ status 0 件。

否定テストを常設に追加: 行数が正しい重複ラベル / 期待外ラベル / PID 重複はいずれも exit 1、
範囲外 domain・pid 0・空ラベル・ヘッダ不一致は exit 2。正準 manifest(216 行)は新検査でも
`missing 0 / extra 0 / duplicate 0` で通過する。

## Low 3 — `reused` の用語

**修正**。出力を `reused_domains=90, reuse_assignments=126` に分けた(前者は 2 回以上使われた
domain ID の個数、後者は初回以降の再割当回数)。`method_comparison.md` と第9回対応資料の
「domain 再利用 90 回」を「90 個すべての domain ID を再利用、初回以降の再割当 126 回」へ訂正し、
README にも両者の定義を明記した。mock 120 episode の場合は `reused_domains=30,
reuse_assignments=30` となり、テストで固定した。

## チェックリスト: raw アーカイブの添付

`scripts/make_release_archive.sh [out_dir]` を追加した。`results*/` は gitignore 対象で
リポジトリに含まれないため、公開時はこれで raw 一式(episode / trace / launch log / manifest /
provenance / summary / plots)をまとめる。各データセットについて**完全性検査・domain manifest 検査・
provenance 照合を先に通してからでなければ梱包しない**(検査に落ちたデータは証跡として出さない)。
出力は `<dataset>.tar.gz` と `SHA256SUMS`。実行確認済み(results 18M / driftsweep 1.2M /
gap_sweep 904K、3 データセットとも検査通過)。

## 検証

| 確認項目 | 結果 |
|---|---|
| `test_run_all.sh`(53 → **63 checks**) | すべて成功 |
| 正準 manifest を新 checker で(期待ラベル集合つき) | 216 episodes / reused_domains=90 / reuse_assignments=126 / overlap 0 |
| manifest 否定対照(重複ラベル・期待外ラベル・PID 重複) | それぞれ exit 1 |
| manifest スキーマ違反(domain 範囲外・pid 0・空ラベル・ヘッダ不一致) | それぞれ exit 2 |
| `verify_provenance.py results` | `worlds_sha` 一致、v1 の tree hash は再現不可と明示、exit 0 |
| `make_release_archive.sh` | 3 データセットとも検査通過・梱包・SHA256SUMS 出力 |
| `test_check_completeness.py -W error::ResourceWarning` | 31/31 成功 |

## 状態

第 10 回レビューの Low 3 件と raw アーカイブ項目を解消。残項目は次サイクル分
(ROS adapter test、角加速度過渡 rollout)のみで、release tag はいつでも作成できる状態。
