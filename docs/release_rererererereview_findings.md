# bilateral_arc_clearance_controller リリース再々々々々レビュー指摘

レビュー日: 2026-08-28  
対象package: `3140b8d` (`Fifth-review response document`)  
対象benchmark: `87f82c8` (`Propagate the completeness gate, refuse stale episodes, re-pin perf`)  
対応資料: [release_rerererereview_response.md](release_rerererereview_response.md)

## 結論

コントローラ本体に新たなCritical / Highの不具合は確認しなかった。現行HEADでplain CMake Release
build、core unit test、13本のclosed-loop scenarioがすべて成功した。性能測定も変更後harnessを含む
clean package revisionへ紐付けられ、raw CSV、tree hash、集計値の整合を確認した。

benchmark完全性ゲートについても、検査失敗statusの伝播、aggregateの停止、episode内容照合、
既存episodeの既定拒否が実装された。一方、preflightより先に既存結果の`provenance.json`を書き換えるため、
再実行を拒否しただけでも元のprovenanceを破壊できる。また、通常入口の`bench.sh`が`OVERWRITE`を
コンテナへ転送しないため、案内される上書き操作をホストから利用できない。

したがって判定は、**コントローラ本体はGo、リリース成果物は条件付きGoを継続**とする。
preflightを全結果書き込みより前へ移動し、`OVERWRITE`の転送と安全性を整えてからrelease tagを作成する
ことを推奨する。

## Critical

なし。

## High

新規指摘なし。

角加速度過渡中のswept trajectoryとjerk、ROS adapter test、実機遅延・追従誤差は従来どおり
保証対象外または次cycleの残項目である。実験的releaseでは明記済みの制約として扱い、実機の独立した
safety layerとしては主張しない。

## Medium

### 1. preflight拒否前に既存`provenance.json`を上書きする

[`run_all.sh`](../../../../nav2_benchmark/scripts/run_all.sh#L48)は、`RESULTS_ROOT`を作成した直後に
`provenance.json`を書き込む。その後、[既存episodeのpreflight](../../../../nav2_benchmark/scripts/run_all.sh#L91)を
実行する。

この順序では、既存結果へ誤って再実行した場合に次の状態になる。

1. 既存datasetの`provenance.json`が今回の日時、commit、実行条件で上書きされる。
2. preflightが既存`episode.json`または`trace.csv`を検出する。
3. benchmarkは非ゼロ終了するが、元datasetのprovenanceは復元されない。

したがって「既存出力があれば既定で拒否して保護する」という目的を完全には満たしていない。
preflightは`mkdir -p`以外の結果書き込みより前に実行し、拒否時には既存datasetを一切変更しないようにする
必要がある。より安全なのは、preflight通過後にtemporary provenanceを書き、全episodeと完全性検査が
成功した時点で正式な`provenance.json`へrenameする構成である。

### 2. `OVERWRITE=1`が通常のホスト入口からコンテナへ渡らない

preflightのerror messageは`OVERWRITE=1`を案内し、[`run_all.sh`](../../../../nav2_benchmark/scripts/run_all.sh#L98)も
環境変数を参照する。一方、ホスト入口の[`bench.sh`](../../../../nav2_benchmark/scripts/bench.sh#L11)は
`RESULTS_ROOT`と`IMAGE_DIGEST`だけを`docker run -e`で転送し、`OVERWRITE`を転送しない。

そのため通常の実行方法で次を指定しても、コンテナ内では`OVERWRITE`が未設定となり、preflightは
既存出力を拒否する。

```bash
OVERWRITE=1 RESULTS_ROOT=/work/nav2_benchmark/results bash scripts/bench.sh ...
```

少なくとも`bench.sh`へ`-e OVERWRITE`を追加する必要がある。READMEには、上書き対象が期待される
run directoryであること、既存rawが削除されること、新規`RESULTS_ROOT`の利用が推奨であることも
記載する。

## Low

### 3. `OVERWRITE=1`の削除pathを構成する識別子が未検証

[`run_all.sh`](../../../../nav2_benchmark/scripts/run_all.sh#L93)はcommand line由来のcontroller名と
scenario名から削除pathを作り、`OVERWRITE=1`時に[`rm -rf "$out"`](../../../../nav2_benchmark/scripts/run_all.sh#L99)を
実行する。通常のscenario名では期待run directoryだけが対象になるが、`..`や`/`を含む入力を拒否しない。

controller / scenarioを`^[A-Za-z0-9_-]+$`などへ限定し、正規化した`out`が正規化済み
`RESULTS_ROOT`配下であることを削除前に確認する必要がある。可能なら、既存directory全体の削除ではなく、
既知の生成fileだけを対象にする。

### 4. 完全性ゲートの異常系testが常設されていない

対応資料では出荷コード断片を抽出したtestにより、欠損、controller不一致、preflight拒否、上書きを
検証している。ただし、このtestはrepositoryへ保存されておらず、CIやCTestから再実行できない。

完全性判定を独立したPython scriptへ分離し、temporary directoryを使うunit testとして次を常設すると、
shell制御変更時のregressionを検出できる。

- 完全なepisode集合
- 欠損file
- 不正JSON
- 必須key欠落
- controller / scenario不一致
- success型不一致
- 未知outcome
- 既存出力の既定拒否

### 5. performance CSVのopen失敗を検出しない

[`perf_benchmark.cpp`](../test/perf_benchmark.cpp#L28)はCSV pathが指定されても、`std::fopen()`が失敗すると
`fcsv == nullptr`のまま測定を続け、終了status 0を返す。存在しないdirectoryを指定する独立確認でも、
CSVを生成せず成功終了した。

引数が指定されている場合はopen失敗をstderrへ出力して非ゼロ終了させ、`std::fclose()`の結果も必要に
応じて検査することを推奨する。現在保存されているraw CSVは正常に生成されているため、既存性能値への
影響はない。

### 6. filter nodeのrate limiterにROS adapter testがない

`bac_filter_node`の`CLEAR`透過経路へ追加された角速度rate limiterを直接固定するadapter testは
引き続き存在しない。対応資料でも次cycleの残項目として明示されているため、今回のrelease blockerには
しない。実機安全保証を対象にする前には追加が必要である。

## 解消を確認した項目

- 完全性検査の非ゼロ終了が`run_all.sh`全体へ伝播する。
- 完全性検査失敗時はaggregateへ進まない。
- `episode.json`のcontroller / scenario値を期待値と照合する。
- `success`のboolean型と`outcome`の定義値を検査する。
- 既存episode / traceを既定で拒否するpreflightを追加した。
- `OVERWRITE=1`時は期待run directoryだけを対象にする。
- raw性能CSVを`points,eval_pts,iter,us`の4列にした。
- 性能測定をclean package HEAD `ef54f50`へ紐付けた。
- provenanceの対象tree hashは現行対象fileからの再計算値と一致した。
- raw性能CSVは各point count 2,000 sampleで、記載されたp50 / p95 / 観測最大値と一致した。

## 今回の確認結果

| 確認項目 | 結果 |
|---|---|
| package HEAD | `3140b8d` |
| performance測定package HEAD | `ef54f50` |
| benchmark HEAD | `87f82c8` |
| 関連作業ツリー | clean |
| plain CMake Release build (`-Wall -Wextra -Wpedantic`) | 成功、警告なし |
| CTest | 2/2成功 |
| `BacCoreUnit` | 成功 |
| `BacScenarioHarness --strict` | 成功（13 closed-loop scenarios） |
| shell構文検査 (`bash -n`) | 成功 |
| provenance tree hash | 再計算値と一致 |
| 保存raw CSV | 4列、各point count 2,000 sample |
| 保存性能、480点 | p50 194.11 / p95 197.75 / 観測最大235.30 µs |
| 保存性能、1000点 | p50 387.31 / p95 395.46 / 観測最大468.31 µs |
| 保存性能、2000点 | p50 319.77 / p95 325.94 / 観測最大369.71 µs |
| 保存性能、4000点 | p50 321.78 / p95 328.79 / 観測最大406.81 µs |
| 現行HEADでの性能再実行、1000点 | p50 379.9 / p95 385.6 / 観測最大422.0 µs |
| 不正CSV出力先でのperformance benchmark | CSV未生成だが終了status 0（Low 5） |

今回、Dockerで全216 episodeは再実行していない。完全性ゲートはsource review、shell構文検査、
終了status制御の確認を行った。既存datasetのprovenanceを書き換えるため、実データを使ったpreflight拒否の
再実行は行っていない。

## リリース判定チェックリスト

- [x] 完全性検査の失敗を`run_all.sh`全体へ伝播する
- [x] 完全性検査失敗時にaggregateへ進まない
- [x] episodeのcontroller / scenario / success / outcomeを検証する
- [x] 既存episodeの混入をpreflightで検出する
- [x] raw CSVへ`eval_pts`を保存する
- [x] cleanな測定harness revisionへ性能結果を紐付ける
- [x] 性能rawとprovenance集計値を一致させる
- [ ] preflightを`provenance.json`生成より前へ移動する
- [ ] preflight拒否時に既存結果を一切変更しない
- [ ] `bench.sh`から`OVERWRITE`をコンテナへ転送する
- [ ] `OVERWRITE`の削除pathを検証する
- [ ] 完全性ゲートの異常系testをrepositoryへ常設する
- [ ] CSV出力要求時の`fopen()`失敗を非ゼロ終了にする
- [ ] 実機安全保証前に角加速度過渡rolloutとROS adapter testを追加する

release tag前の必須対応は、preflightの順序修正と`OVERWRITE`転送である。削除path検証、完全性testの常設、
performance CSV error handlingはrelease品質を高めるLow項目として、可能であれば同じ修正cycleで対応する。
