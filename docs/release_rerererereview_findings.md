# bilateral_arc_clearance_controller リリース再々々々レビュー指摘

レビュー日: 2026-08-28  
対象package: `34e995b` (`Close the conditional-Go items from the fourth review`)  
対象benchmark: `3db964b` (`Benchmark completeness gate, container-path examples, perf provenance re-pinned`)  
対応資料: [release_rererereview_response.md](release_rererereview_response.md)

## 結論

コントローラ本体に新たなCritical / Highの不具合は確認しなかった。現行HEADでplain CMake Release
build、core unit test、13本のclosed-loop scenarioがすべて成功しており、前回確認した接触判定修正にも
変更はない。`RESULTS_ROOT`の実行例と手法比較表の角加速度説明も適切に修正された。

一方、benchmark完全性ゲートは検査用Pythonの非ゼロ終了をshellが伝播しないため、欠損・破損時にも
集計へ進み、最終的に成功終了できる。また、同じ結果directoryを再利用すると、失敗したepisodeの
代わりに前回の`episode.json`を受理できる。性能測定もcore sourceは固定されているが、変更後の測定
harnessを含むcleanな最終package revisionには紐付いていない。

したがって判定は、**コントローラ本体はGo、リリース成果物は条件付きGoを継続**とする。完全性ゲートの
終了伝播と既存結果の混入防止を修正し、現行clean HEADで性能測定を再取得してからrelease tagを作成する
ことを推奨する。

## Critical

なし。

## High

新規指摘なし。

角加速度過渡中のswept trajectoryとjerkは、前回レビューどおり既知の保証対象外である。
この制約はREADME・parameter reference・手法比較表に明記されている。実機安全保証を対象としない
実験的releaseではblockerとしない。

## Medium

### 1. 完全性ゲートの失敗statusがshellへ伝播しない

[`run_all.sh`](../../../../nav2_benchmark/scripts/run_all.sh#L109)の検査用Pythonは、欠損または破損を
検出すると`sys.exit(1)`する。しかしscript先頭に`set -e`はなく、Python終了statusを検査する
`if`もない。そのためPythonが失敗しても、直後の[aggregate処理](../../../../nav2_benchmark/scripts/run_all.sh#L145)へ
続行する。aggregateと最後の`echo`が成功すれば、`run_all.sh`全体も終了status 0になり得る。

対応資料にある「非ゼロ終了し、集計へ進まない」は現状では成立しない。例えば次のように検査を
明示的にgateする必要がある。

```bash
if ! python3 - "$RESULTS_ROOT" <<PYEOF
# completeness check
PYEOF
then
  exit 1
fi
```

script全体への`set -euo pipefail`追加はbackground jobや`wait`の制御にも影響するため、少なくとも
完全性検査には明示的な終了判定を置く方が安全である。

### 2. 再実行時に古いepisodeを今回の結果として受理できる

[`run_episode()`](../../../../nav2_benchmark/scripts/run_all.sh#L67)は出力directoryを作成するだけで、
既存の`episode.json`や`trace.csv`を拒否・退避・初期化しない。既存結果と同じ`RESULTS_ROOT`へ再実行し、
今回のlaunchが新しい`episode.json`を生成する前に失敗した場合、次の両方が古いファイルを正常結果として
受理する。

- `run_episode()`の`[[ -f "$out/episode.json" ]]`判定
- 実行後の完全性ゲート

その結果、全期待fileが存在していても、複数世代が混ざったsummaryを生成できる。次のいずれかが必要である。

1. 実行前preflightで期待出力先が空であることを確認し、既存時は非ゼロ終了する。
2. timestampまたはrun UUID付きの新規`RESULTS_ROOT`を毎回使用する。
3. 明示的な`--overwrite`指定時だけ、今回対象のfileを限定して初期化する。

既存結果を暗黙に削除するとraw dataを失うため、既定動作は「既存出力があれば拒否」が望ましい。

完全性検査ではさらに、必須keyの存在だけでなく次も確認することを推奨する。

- `episode["controller"] == expected_controller`
- `episode["scenario"] == expected_scenario`
- `success`がbooleanである
- `outcome`が定義済みの値である

### 3. 性能測定が変更後harnessを含むclean revisionへ紐付いていない

[`perf/provenance.txt`](../../../../nav2_benchmark/perf/provenance.txt#L2)は
`package_head: 1e188e2`を記録する一方、現行package HEADは`34e995b`である。測定時点では
`test/perf_benchmark.cpp`が`1e188e2`から変更されていたが、`source_dirty`は`src/include`だけを
検査しており、測定harnessの未コミット変更を記録していない。

`src/include`のtree hashはprovenance記載値と一致し、コア実装は`6b7dba8`から変更されていない。
したがって保存性能値そのものを否定する問題ではないが、「クリーンな最終リビジョンで再測定」という
対応資料の説明と、artifact単体からの再現性は満たしていない。

現行package HEAD `34e995b`の全作業ツリーがcleanであることを確認して再測定し、次を保存すれば解消する。

- 完全なpackage HEAD SHA
- `git status --porcelain`が0件であること
- `src/`、`include/`、`test/perf_benchmark.cpp`、`CMakeLists.txt`を含むtree hash
- compiler、build type、実際のcompile flags
- raw CSVと集計値

## Low

### 4. raw性能CSVに`eval_pts`が保存されない

[`perf_benchmark.cpp`](../test/perf_benchmark.cpp#L81)はconsole summaryへ`eval_pts`を表示するが、
[raw CSV出力](../test/perf_benchmark.cpp#L72)は引き続き`points,iter,us`の3列である。
provenanceには入力点数ごとの評価点数が記録されているため数値解釈は可能だが、raw CSV単体では
`max_points=1000`による間引きを判別できない。

`eval_pts`はpoint countごとに一定なので必須ではないが、対応資料の「eval_pts列を出力」をrawまで
含む説明として読む場合は不整合になる。CSVを`points,eval_pts,iter,us`にすると明確である。

### 5. filter nodeのrate limiterにROS adapter testがない

`bac_filter_node`の`CLEAR`透過経路へ追加された角速度rate limiterを直接固定するadapter testは
引き続き存在しない。対応資料でも次cycleの残項目として明示されているため、今回のrelease blockerには
しない。実機安全保証を対象にする前には、command・odometry・status・scan freshnessの組合せを固定した
testが必要である。

## 確認できた改善

- benchmark READMEの`RESULTS_ROOT`をコンテナ内`/work/...`パスへ修正した。
- 実行条件からcontroller × scenario × runの期待集合を生成する完全性検査を追加した。
- `episode.json`の存在、JSON parse、必須keyを検査するようにした。
- `completeness.json`へexpected / observed / missing / corruptを出力するようにした。
- 性能測定の`max`をWCETではなく2000反復内の観測最大値としてscopeした。
- performance summaryへ`eval_pts`を追加し、`max_points=1000`による間引きを説明した。
- 手法比較表を「角加速度過渡・jerkは未評価」へ更新した。
- 旧文書の性能値をclean core sourceでの再測定値へ更新した。

## 今回の確認結果

| 確認項目 | 結果 |
|---|---|
| package HEAD | `34e995b` |
| benchmark HEAD | `3db964b` |
| 関連作業ツリー | clean |
| plain CMake Release build (`-Wall -Wextra -Wpedantic`) | 成功、警告なし |
| CTest | 2/2成功 |
| `BacCoreUnit` | 成功 |
| `BacScenarioHarness --strict` | 成功（13 closed-loop scenarios） |
| `run_all.sh` / `bench.sh`構文検査 (`bash -n`) | 成功 |
| performance benchmark再実行、480点 | p50 190.2 / p95 226.2 / 観測最大359.3 µs |
| performance benchmark再実行、1000点 | p50 379.1 / p95 387.1 / 観測最大531.2 µs |
| performance benchmark再実行、2000点 | p50 313.5 / p95 317.6 / 観測最大539.5 µs |
| performance benchmark再実行、4000点 | p50 315.7 / p95 319.3 / 観測最大445.9 µs |
| 保存raw CSV | 8000 samples + header、3列 (`points,iter,us`) |
| 保存済み正準結果 | 216 episode、既存集計との一致を前回レビューで確認済み |

今回、Dockerで全216 episodeは再実行していない。完全性ゲートの異常系はsource reviewとshellの
終了status伝播の独立確認によって検証した。

## リリース判定チェックリスト

- [x] `RESULTS_ROOT`のREADME例をコンテナ内pathへ修正する
- [x] 期待episode集合を生成する完全性検査を追加する
- [x] 欠損・破損の詳細を`completeness.json`へ保存する
- [x] 性能の`max`を観測最大値としてscopeする
- [x] performance summaryに評価点数を表示する
- [x] 手法比較表の角加速度制約を現行実装へ同期する
- [ ] 完全性検査の非ゼロ終了を`run_all.sh`全体へ伝播する
- [ ] 完全性検査に失敗した場合はaggregateへ進まない
- [ ] 既存episodeの混入をpreflightまたは新規結果directoryで防止する
- [ ] episode内のcontroller / scenario値を期待値と照合する
- [ ] 現行clean package HEADで性能測定を再取得する
- [ ] performance provenanceへ測定harnessを含むrevision/hashを記録する
- [ ] 必要に応じてraw CSVへ`eval_pts`を保存する
- [ ] 実機安全保証前に角加速度過渡rolloutとROS adapter testを追加する

完全性ゲートの終了伝播と既存結果混入防止はbenchmark結果の世代一貫性に直接関係するため、release tag
作成前の修正を推奨する。性能provenanceはコアsource自体を固定できているものの、対応内容を
「cleanな最終revision」とするには現行HEADでの再取得が必要である。
