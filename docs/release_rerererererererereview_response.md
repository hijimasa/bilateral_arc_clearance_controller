# 第9回リリースレビューへの対応

対象レビュー: [release_rerererererererereview_findings.md](release_rerererererererereview_findings.md)(2026-08-28、`79e7a34` / `1090ee2` 時点。本体 Go・修正後 runner の通常経路 Go・旧 runner 由来の release evidence 条件付き Go)

指摘 3 件すべてに対応した。**Medium 1 の指摘は妥当**であり、事後監査の主張を撤回して正準 216 episode を
再生成した。

## Medium 1 — 監査の前提が不成立、正準データを再生成

### 指摘の受け入れ

`audit_legacy_domains.py` は `episode.json` の mtime を「background `run_episode` プロセスの終了時刻」と
みなしていたが、実際にはその後に executor shutdown → evaluator 終了 → launch 全体の shutdown
(simulator / Nav2 停止)が続く。したがって mtime は**回収時刻の下限**にすぎず、順位付けの前提が
成立しない。保存データでも `launch.log` の mtime が `episode.json` より全件で遅い(中央値 2.43 秒)
というレビューの独立確認どおりであり、`0 could have overlapped` は「重複がなかった」ことを意味しない。

### 対応

1. **正準 216 episode を修正後 runner で再生成**した(結果は下表)。新データは実行中に
   `domain_manifest.csv` を残し、**親プロセスが記録した起動時刻と実際の回収時刻**で分離を証明する:
   `216 episodes, 90 domains, reused_domains=90, reuse_assignments=126, 0 overlapping`
   (90 個すべての domain ID が再利用され、初回以降の再割当は 126 回。検査が空振りでないことも
   同時に示している。第10回レビュー Low 3 に従い用語を「再利用された domain 数」と
   「再割当回数」に分けた)。
2. 監査スクリプトは削除せず、**指標(indicator)であって証明ではない**ことを docstring と出力
   メッセージに明記し、時刻の下限として `episode.json` と `launch.log` の遅い方を使うよう変更した。
   README と `method_comparison.md` の「同時に生存し得なかったことが確定」「cross-talk の余地はない」
   「再生成不要」という表現は削除・訂正した。
3. 推奨 3(将来 run 用の manifest)を実装: runner が pid・domain・ラベル・起動時刻・**回収時刻**・
   終了 status を親側で追記し、`scripts/check_domain_manifest.py` が
   「同一 domain の保持区間の重なり 0」「非ゼロ status 0 件」を検査、run_all がこれをゲートする。
   episode.json の mtime とは独立した、run ごとの分離証明になる。

## Medium 2 — episode 起動コマンドの status 伝播

推奨 1〜3 をそのまま実装した。

- `run_episode()` は成果物の有無にかかわらず launch status を `return` する(成果物診断は別出力)。
- `reap_one()` は `wait -n -p` の status を PID・domain とともに manifest へ記録し、非ゼロを計数する。
  全 episode 回収後に 1 件でも非ゼロなら、**aggregate と provenance 昇格を行わず非ゼロ終了**する。
- 常設テストを追加: 「有効な `episode.json` / `trace.csv` を書いた**後に** exit 7 する mock」で、
  完全性ゲートは通るが runner 全体は exit 1・provenance 未昇格・`not aggregating` を確認。

## Low 3 — domain pool の fail-closed 化

推奨をすべて実装した。

- 起動時に **bash 5.1 以上を明示検査**(未満は exit 2)。
- `wait` 失敗・空 PID・未知 PID は**すべて fatal**。domain を free-list へ戻さず、
  `kill_pool` で既知の子プロセスを TERM → KILL → `wait` で回収してから exit 1。
  「全 domain を再公開する」fail-open 分岐は削除した。
- 未知 PID で `PID_DOMAIN` が減らずループが停止し得る問題も、この fatal 化で解消。
- 異常系テストを常設: 出荷スクリプトから `kill_pool` / `reap_one` を**逐語抽出**し、
  map に無い子プロセスを回収させると exit 1・FATAL 出力・**free-list への domain 追加なし**・
  以降のコード未到達を固定。

## 検証

| 確認項目 | 結果 |
|---|---|
| `test_run_all.sh`(36 → **53 checks**) | すべて成功 |
| domain 排他(子プロセス側の自己申告) | 120 episode / 90 domain / 30 再利用、overlaps=0 |
| domain 排他(**runner の manifest**: 親の起動〜回収区間) | 120 episode、overlapping 0 |
| manifest 区間が子の自己申告区間を包含すること | 確認(親側が広い) |
| manifest チェッカの否定対照(合成重なり / 非ゼロ status / 不正行) | それぞれ exit 1 / 1 / 2 |
| 否定対照(旧 modulo 方式の runner を `RUN_ALL_SCRIPT` で指定) | 子側・manifest 側の**両検出器が重複を検出**し、runner 自体も fail-closed で失敗 |
| pool 異常時の fail-closed(逐語抽出テスト) | exit 1、domain 再公開なし |
| launch status 伝播(成果物は正常・mock が exit 7) | run 全体 exit 1、provenance 未昇格 |
| `test_check_completeness.py -W error::ResourceWarning` | 31/31 成功 |
| plain CMake Release build + CTest | 警告 0、2/2 成功 |

## 再生成結果(正準 18 シナリオ × 3 run × 4 controller = 216 episode)

生成条件: package `f1e2a90` / bench `13becc0`、**両 worktree dirty 0**、image digest 記録済み、
jobs 4 / rtf 2.0。制御コード(`src` / `include`)は旧データ生成時点(`6b7dba8`)から**差分なし**で、
今回の再生成は runner の分離保証を得るためのものである。

| controller | 成功 | 衝突 | 平均(成功時) | 中央値 | 最接近 最悪値 |
|---|---:|---:|---:|---:|---:|
| BAC | 54/54 | 0 | 27.6 s | 28.4 s | **0.139 m** |
| DWB | 49/54 | 1 | 27.9 s | 25.2 s | 0.000 m |
| MPPI | 51/54 | 0 | 29.0 s | 27.2 s | 0.091 m |
| RPP | 48/54 | 0 | 24.2 s | 24.8 s | 0.017 m |

主張の骨格は再現した(BAC のみ全完走・衝突ゼロ・0.10 m 未満非侵入、`appearing_obstacle` は
BAC 3/3 に対し他 3 手法 0/3、`corridor_locdrift_15x` は BAC 3/3・DWB 衝突 1・RPP 0/3)。
世代差として、BAC の最悪接近が 0.154 → 0.139 m、DWB の衝突が 2 → 1 件、DWB 平均が
`corridor_extreme_offset` の 168 秒復帰 1 件に引かれて 24.6 → 27.9 秒(中央値 25.2 秒)へ動いた。
README・`method_comparison.md`・動画ギャラリーの数値はすべて新データセットへ更新済み。
ズレ量スイープと隙間幅スイープは 90 episode 未満で domain 再利用が起きないため再生成していない
(旧データを継続使用していることを本文に明記)。

## 状態

第 9 回レビューの必須項目(正準データの再生成、launch status 伝播、pool の fail-closed 化)を解消。
残項目は従来どおり次サイクル分(ROS adapter test、角加速度過渡 rollout)のみ。
