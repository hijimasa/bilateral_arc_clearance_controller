# 第7回リリースレビューへの対応

対象レビュー: [release_rerererererereview_findings.md](release_rerererererereview_findings.md)(2026-08-28、`25fb091` / `30cd5fa` 時点。本体 Go・成果物 条件付き Go)

指摘 6 件すべてと、類似問題の掃引に対応した。

## Medium

| # | 指摘 | 対応 |
|---|---|---|
| 1 | 期待集合外の古い episode を許容し、aggregate が複数世代を混在させる | **修正**。(1) `check_completeness.py` に `RESULTS_ROOT` 配下全体を走査する `observed_artifacts()` を追加。**verify** は期待 run ディレクトリ外の `episode.json` / `trace.csv` を `unexpected` として `completeness.json` へ記録し、1 件でもあれば非ゼロ終了(→ aggregate にも provenance 昇格にも進まない)。(2) **preflight** も既定で root 配下の**任意の** episode 出力を拒否し、期待集合内(`OVERWRITE=1` で消去可能)と集合外(新規 `RESULTS_ROOT` が必要)をメッセージで区別。(3) `run_all.sh` は `OVERWRITE=1` でも期待ディレクトリ消去**後に必ず preflight を実行**する構成に変更 — 消去後に残り得るのは集合外の古い episode だけであり、それは拒否される。世代混在は preflight と verify の二重で遮断される。(4) unexpected controller / scenario / run ディレクトリ・集合外 stray trace・期待ディレクトリ内 trace 許容の 5 テストを常設 unit test に追加。README にも集計が root 全体を対象とする理由と併せて明記。subset 統合の運用は導入せず、推奨どおり「毎回新規 `RESULTS_ROOT`」を checker が強制する。 |
| 2 | aggregate / provenance 昇格の失敗が終了 status に伝播しない | **修正**。レビュー提案どおり `aggregate.py` と `mv`(昇格)をそれぞれ `if !` で明示的に gate し、失敗時は stderr へ理由を出して exit 1。**類似掃引として後続処理の前提となる操作も gate**: `mkdir -p "$RESULTS_ROOT"`、`OVERWRITE=1` の `rm -rf`、一時 provenance の書き込み(リダイレクト失敗を含む)。`set -euo pipefail` への全面移行は background job pool と `wait -n || true` の挙動固定が必要なため行わず、明示 gate 方式を維持。episode 内の `mkdir` 失敗は launch 失敗 → episode.json 欠損として完全性ゲートが捕捉する(確認済み)。ホスト入口 `bench.sh` は `set -eu` + `exec docker run` でコンテナ status をそのまま返すことを確認。 |

## Low

| # | 指摘 | 対応 |
|---|---|---|
| 3 | 空集合・不正件数 parameter を拒否しない | **修正**。`run_all.sh` は引数 parse 直後に、空 / 空白のみの `--controllers` / `--scenarios`、重複名、整数でない・1 未満の `--runs` / `--jobs`、正の有限数でない `--rtf` をすべて exit 2 で拒否。`check_completeness.py` 側も空集合・重複・`runs < 1` を `ValueError`(exit 2)にし、6 ケースの unit test を常設。 |
| 4 | perf provenance に相互矛盾する 2 組の測定値 | **修正**。ラベルの無い 4 行(別実行の console summary の消し忘れ)を削除し、「saved perf_raw.csv から計算」と明記された結果表のみを残した。raw CSV・記載値・対応資料の引用値は単一系統で一致。 |
| 5 | `world_file` の open / write / close が未検査 | **修正**。`writeTraceCsv()` の world CSV に trace と同じ open 検査を追加し、さらに trace / world の両方に write / close 後の stream 状態検査(失敗は stderr 警告)を追加。常設テストの `open(...).write(...)` 2 箇所を `with open(...)` へ変更し、`python3 -W error::ResourceWarning` でテスト全体が警告ゼロで通ることを確認。 |
| 6 | filter node rate limiter の ROS adapter test | **残項目として維持**(次サイクル)。 |

## 検証

| 確認項目 | 結果 |
|---|---|
| `test_check_completeness.py`(14 → **23** tests、`-W error::ResourceWarning` 付き) | OK |
| 出荷ブロック逐語抽出テスト(22 checks) | すべて成功 |
| うち B1: `OVERWRITE=1` + 集合外 episode → 期待ディレクトリのみ消去後 preflight が exit 1、集合外は不変 | 確認 |
| うち B2: 集合外 episode のみの root → 拒否・全ファイル hash 不変 | 確認 |
| うち C1/C2: aggregate 失敗 / 昇格失敗 → exit 1、provenance 未昇格 | 確認 |
| うち A: `--runs 0` / `--jobs 0` / 空・重複集合 / `--rtf 0` / 非数 → exit 2、正常引数 → exit 0 | 確認 |
| `bash -n`(run_all.sh / bench.sh) | 成功 |
| package rebuild(Release、-Wall -Wextra -Wpedantic)+ CTest | 警告 0、2/2 成功 |

## 状態

第 7 回レビューの必須項目(unexpected artifact の拒否、aggregate / provenance 昇格の fail-fast 化)と Low 3〜5 を解消。残項目は従来どおり次サイクル分(ROS adapter test、角加速度過渡 rollout)のみ。
