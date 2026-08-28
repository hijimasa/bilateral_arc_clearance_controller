# bilateral_arc_clearance_controller 第10回リリースレビュー指摘

レビュー日: 2026-08-28  
対象package: `83c5e7a` (`Regenerated canonical benchmark: numbers from the isolation-proved run`)  
対象benchmark: `13becc0` (`Propagate episode status, fail closed on reap anomalies, record a manifest`)  
対応資料: [release_rerererererererereview_response.md](release_rerererererererereview_response.md)

## 結論

第9回レビューの3件は解消を確認した。

- 正準216 episodeは修正後runnerで再生成され、親processが記録したmanifestは216行、期待label 216件、
  非ゼロstatus 0、同一domainのoverlap 0である。
- episode起動commandの非ゼロstatusはrunner全体へ伝播し、aggregateとprovenance昇格を止める。
- `wait` / PID bookkeeping異常時は既知childを停止・回収してfail-closedになり、domainを再公開しない。
- 保存rawからの再集計値はREADME、method comparison、対応資料の更新値と一致する。

コントローラ本体、benchmark runner、再生成した正準release evidenceに新たなCritical / High / Mediumの
問題は確認しなかった。現時点の判定は**Go**である。

残件はprovenanceと検査contractのLow 3件、および従来から次cycle扱いとしているROS adapter test・
角加速度過渡rolloutである。Low 3件は今回の正準結果を無効にしないが、release後も再現可能な証跡として
扱いやすくするため修正を推奨する。

## Critical

なし。

## High

なし。

## Medium

なし。

## Low

### 1. `bench_tree_sha`がignored生成物を含み、clean commitから再現できない

[`run_all.sh`](../../../../nav2_benchmark/scripts/run_all.sh#L159)は次の対象を`find -type f`で列挙して
`bench_tree_sha`を作る。

```bash
find ws/src scripts Dockerfile -type f | sort | xargs sha256sum | sha256sum
```

この列挙はGit追跡fileだけに限定されず、build / 実行中にsource treeへ作られるignoredな
`__pycache__/*.pyc`も含む。一方、`bench_dirty`は`git status --porcelain`なのでignored fileを数えない。
したがって`bench_dirty: 0`でもtree hashには一時生成物が混ざり、同じcommitから再計算できない。

今回の正準provenanceに記録された値は次のとおりである。

```text
recorded bench_tree_sha: 0fd746690dbe04294a9b9de7d456a0c499001e8fa78d38597a46b420cb3cd35a
clean commit 13becc0:    f888b79e34398117c8dd1fbe1b8d1d08ee5c0cac93583e8a8f3a1a0961142ece
```

`bench_commit=13becc0`、`bench_dirty=0`、`worlds_sha`一致は独立に確認できるため、今回結果のsource同一性は
commitで追跡できる。しかし`bench_tree_sha`自体は再現可能なfingerprintになっていない。

推奨対応:

- Git追跡fileだけをhashするか、clean時は`git rev-parse HEAD:nav2_benchmark`のtree object IDを記録する。
- worktree差分も内容hashへ含める必要がある場合は、追跡file集合とuntracked file集合を明示的に分け、
  `__pycache__`、`.pyc`、build生成物を除外する。
- clean checkoutで再計算した値がprovenanceと一致するtestを追加する。

### 2. manifest checkerが期待episode集合ではなく行数だけを検査する

[`check_domain_manifest.py`](../../../../nav2_benchmark/scripts/check_domain_manifest.py#L44)の
`--expect-episodes`は行数だけを比較する。labelの空文字・重複・期待集合外、PIDやdomainの妥当範囲は
検査しない。例えば異なるdomainを持つ2行へ同じlabelを書いたmanifestは、`--expect-episodes 2`で
終了status 0になることを確認した。

現在の正準manifestについては独立に集合差分を確認し、次のとおり正常だった。

```text
rows 216, unique labels 216, missing 0, extra 0, duplicate labels 0
unique PIDs 216, domain range 10..99, non-zero status 0
```

したがって今回結果に実害はない。ただしchecker単体が「manifestと実行条件の一対一対応」を保証せず、
完全性checkerが検査するartifact集合とも直接結合されていない。

推奨対応:

1. `--controllers`、`--scenarios`、`--runs`から期待label集合を生成し、missing / unexpected / duplicateを
   検査する。あるいはrunnerが期待label fileを生成してcheckerへ渡す。
2. label非空、domain 10〜99、PID正整数、status範囲、CSV headerの完全一致をschemaとして固定する。
3. duplicate label、missing + duplicateで行数だけ一致するmanifest、範囲外domainを否定testへ追加する。
4. docstringの「malformed rowはexit 1」と、実装・既存testのexit 2もどちらかへ統一する。

### 3. `reused`の数値を「再利用回数」と説明している

[`check_domain_manifest.py`](../../../../nav2_benchmark/scripts/check_domain_manifest.py#L61)の`reused`は、
episode総数から初回割当を引いた**再割当回数**ではなく、「2回以上使われたdomain IDの個数」である。
正準runは216 episode / 90 domainなので、出力`90 reused`の意味は次のとおりになる。

- 再利用されたdomain ID: 90個
- 初回以降の再割当: 216 - 90 = 126回

一方、[method comparison](method_comparison.md)と対応資料は「domain再利用90回」と説明しており、
再割当回数として読むと不正確である。overlap 0という本質的な結論には影響しない。

推奨対応:

- checker出力を`reused_domains=90, reuse_assignments=126`のように分ける。
- 文書を「90個すべてのdomain IDを再利用、再割当126回」に訂正する。
- mock 120 episodeの場合も「reused domain 30個、再割当30回」と用語を揃える。

## 解消を確認した項目

- artifact mtimeによる旧datasetの非重複証明を撤回した。
- 正準216 episodeを修正後runnerで再生成した。
- parent launch〜reap区間を記録する`domain_manifest.csv`を追加した。
- domain manifest overlap / 非ゼロstatusをrelease gateにした。
- launch command statusを`run_episode()`、`reap_one()`、run全体へ伝播した。
- Bash 5.1以上の明示検査を追加した。
- 空PID・未知PID・wait異常時にfail-closedでchildを停止・回収する。
- status伝播、manifest否定対照、pool異常の常設testを追加した。
- READMEとmethod comparisonを新しい正準結果へ更新した。
- drift / gap sweepが旧runner由来であることと、90 episode未満でdomain再利用がないことを明記した。

## 今回の確認結果

| 確認項目 | 結果 |
|---|---|
| package HEAD | `83c5e7a` |
| benchmark HEAD | `13becc0` |
| package worktree（本レビュー文書作成前） | clean |
| benchmark関連worktree | clean |
| plain CMake Release build | 成功、警告なし |
| CTest | 2/2成功 |
| `test_check_completeness.py -W error::ResourceWarning` | 31/31成功、警告なし |
| `test_run_all.sh` | 53/53成功 |
| 正準manifest checker | 216 episodes、90 domains、overlap 0、非ゼロstatus 0 |
| 正準manifestとprovenanceの期待label集合 | missing 0、extra 0、duplicate 0 |
| 正準dataset完全性 | expected / observed 216、missing 0、corrupt 0、unexpected 0 |
| raw再集計 | BAC 54/54、DWB 49/54、MPPI 51/54、RPP 48/54。文書と一致 |
| `worlds_sha`再計算 | provenanceと一致 |
| `bench_tree_sha`再計算 | clean commitと不一致（Low 1） |

今回、新たにDocker / ROSで216 episodeを再実行してはいない。対応時に再生成された保存raw、manifest、
provenanceを独立に検査・再集計した。

## リリース判定チェックリスト

- [x] 正準216 episodeを分離保証付きrunnerで再生成する
- [x] manifestで同一domainのoverlap 0を確認する
- [x] manifest label集合が正準実行条件と一致することを今回のrawで確認する
- [x] launch異常とpool bookkeeping異常をfail-closedにする
- [x] README / method comparisonの正準数値をrawと一致させる
- [ ] `bench_tree_sha`をGit追跡sourceから再現可能にする
- [ ] manifest checker自身に期待label集合とschemaの検査を追加する
- [ ] `reused domain数`と`再割当回数`の表現を分ける
- [ ] release公開時にgitignoredな`results*/`のraw archiveとchecksumを添付する
- [ ] 実機安全保証前に角加速度過渡rolloutとROS adapter testを追加する

今回のreleaseを止める指摘はない。Low 1〜3は、次にbenchmarkを回す前に直すと新しいprovenanceと
manifestへそのまま反映できる。公開作業では、repositoryに含まれないraw dataset、manifest、provenance、
summary、trace、launch logを同一archiveへまとめ、checksumをrelease noteへ記録することを推奨する。
