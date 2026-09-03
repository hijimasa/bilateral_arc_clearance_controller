# リリースレビュー履歴

[English](en/release_review_history.md) | 日本語

## 現在の状態

- 確認日: 2026-09-02（R19まで）
- 評価対象package: R14までは `13b3d16`、R15〜R18は `feature/omni-motion-model` の各時点（順に
  `616746b` / `a9c6c31` / `dea7662` / `ba544e2`）。この節の以下の記述はR14時点の `13b3d16` に対するもので、
  `main` は `2488248` である（R18 M14）。R11の対象だった作業ツリーは `8fd0d7e`（実装）/ `5951e0b`（文書）/
  `24466ae`（レビュー記録）としてコミットされた。R12はその3コミットを対象とし、その対応は `3521226`
  に入っている。R13は `3bd88cb`（シナリオ拡充、R11 L5の対応）を対象とし、その対応は `112eb35` に
  入っている。R14は `112eb35` までのbranch全体を対象とし、その記録は `0586253`、対応は `13b3d16`
  である
- benchmark: `026a17a`（正準）/ `4fed3d2`（公平条件比較・ablation）。R11・R12ともbenchmarkを再実行していない
- 判定: コードレビュー **Go**、Public化P0 **完了**。公開操作は所有者判断
- R14（`feature/ackermann-motion-model`、`112eb35` を対象）はHigh 1件・Medium 10件に対応済み。判定は
  **Conditional Go**で、main統合そのものは可と結論した。Low 9件は次cycleへ送る（L5・L7・L9は部分的に閉じた）
- R15（`feature/omni-motion-model`、`616746b` を対象）はHigh 4件と統合前条件のMedium、および修正を
  守るためのMediumに対応済み。判定は対応前が **Hold**。残るMedium 6件とLow全件は次cycleへ送る。
  この節が記載する `評価対象package` は `main`（`2488248`）のものであり、全方向モデルはまだ入っていない。
  R15の記録commitは `a1d25f3`、対応commitは `433068a` である
- R16（`a9c6c31` を対象、最重点は未レビューの `433068a`）はHigh 5件と統合前条件のMediumに対応済み。
  判定は対応前が **Hold**。H1の接触は厚みゼロの壁という試験worldの縮退に依存しており、通路を厚みの
  ある箱に改めたうえで、薄壁へ平行に進入する場合の制約を手法の限界として明記した。R16の記録commitは
  `3708fd4`、対応commitは `c74fe7d` / `68a8ed9` / `4739c6f` / `fa15112` / `dea7662` である
- R17（`dea7662` を対象、最重点は未レビューのR16対応commit群）は**対応前の判定が Hold** だった
  （R19 M10の再訂正: ここだけ「判定 **Hold**、対応未了」という旧表現が残っており、同じ段落が
  「R17対応でHigh 5件と統合前条件のMediumを閉じた」と述べているのと自己矛盾していた。英語版と
  日本語版のR15・R16・R18行はいずれも「対応前の判定」形に直っている）。製品コードの
  安全性は収束しており約570万tickで接触0だが、R16対応commit自身が2件のコード回帰と3件の再現しない
  主張を持ち込んだ。High 5件は統合を阻む。R17対応でHigh 5件と統合前条件のMediumを閉じた
  （記録commit `a89f0c4`、対応commit `364fe1a`。R16の対応commitは `c74fe7d` / `68a8ed9` / `4739c6f` /
  `fa15112` / `dea7662` である）
- R18（`ba544e2` を対象、最重点は未レビューの `364fe1a` / `ba544e2`）は**対応前の判定が Hold**
  だった（R19 M10: ここには「Hold、対応未了」と書いてあったが、同じ段落の末尾で対応完了と
  対応commitを述べており自己矛盾していた。R18 H6がR17行の同じ表現に対して行ったのと同じ形に直した）。
  製品コードの側に統合を阻む欠陥は無く、R17 H1の非対称スラブは掃引箱の横区間として幾何的に厳密に
  正しいこと、`vy == 0` の還元がbit完全一致であること、既存2モデルのfixture出力が `main` をビルド
  して一致することが独立に確認された。Holdの理由は、掃引の安全不変条件が接触距離を被検コード自身
  から得ている構造的欠陥1件（H1）と、R17対応資料の主張の誤り5件（H2〜H6）である。うちH2は
  install対象2ファイルを「直した」と報告して実際には触っていないもの、M1は正しかった数値を誤りに
  書き換えたものである。R18対応でHigh 6件と統合前条件のMedium、および安価なLowを閉じた。製品コードの
  挙動は変えていない（`src/` の非コメント差分は0行）。R18の記録commitは `5238150`、対応commitは `8eefe99` である
- R19（`78f6852` を対象、範囲は未レビューの `8eefe99` / `78f6852` に限定）は判定 **Hold**、対応中。
  「新規Highが0件かつ再現しない主張が0件なら収束と見なして統合する」という打ち切り条件を先に決めて
  実施したが、**条件は満たされなかった**（新規High 4件）。製品コードは `ba544e2` から挙動が変わって
  おらず、Highは4件ともR18対応が入れた検証装置と文書にある。H1はR18 H1の自己参照が半分しか
  解消していないこと（接触の「大きさ」の相殺は消えたが「有無」の相殺が残る）、H2は鏡映検査の
  `heading_gain` を上げたことで変異killを2件失ったこと（R14 M5の再発）である。R19対応でHigh 4件と
  統合前条件のMedium、および安価なLowを閉じた。**対応は修正担当と検証担当を分けて5巡行った。**
  独立検証が各巡で見つけたHighは 5 → 3 → 1 → 1 → 0 で、5巡目にして新規の偽が0件になった。
  3巡目だけは担当を分けずに編集し、そこで単一の報告を測らずに書き込んで真の記述を偽で置き換えた
  （検証2名が独立に検出）。製品コード（`src/`）は5巡を通じて非コメント差分0行のままである。
  R19の記録commitは `310f26b`、対応commitは `187c053` である

全19回のリリースレビューのうち、R14までのCritical / High / Mediumは対応済みである。R15〜R19は
`feature/omni-motion-model` に対するもので、`main`（`2488248`）には含まれない。第10回のLow 3件、
第11回のLow 5件（L1は挙動変更を伴わない文書化で完了、L5は2026-09-01のシナリオ拡充で完了）、
第12回のLow 3件、第13回のLow 6件のうち5件も対応済みで、`main`（`2488248`）にrelease blockerはない。
R15〜R18はHighと統合前条件のMediumを閉じ、R19は対応中である（この限定はR18 H6による。
英語版が限定なしに「全レビュー回のCritical / High / Mediumは対応済み」と書いていた）。第13回L7
（`speed_limit_` のデータ競合）は本branch以前からの既存項目として次cycleへ残す。第12回は第11回の対応の再検証であり、第11回がM1を完了と判断した廃案設計記述が公開ヘッダに残存
していたこと、Nav2 adapterの差動二輪coverageが追加ではなく置換されていたこと、および同梱のAckermann
設定例が自身の回帰試験に通らなかったこと（第11回L4の文書化が誤った前提に立っていた）を検出した。
2026-08-29にROS adapter testと入力source diagnosticsを追加し、同日に公平条件比較216 episodeとBAC
ablation 216 episodeを追加した。2026-09-01にはAckermann motion modelを追加し、第11回レビューで
`setParams`の例外安全（High 1件）を修正した。次cycleへ残す設計・評価項目は、角加速度過渡を積分した
rollout、実機外乱評価、Collision Monitor併用baseline、`BacCore::process()`の責務分割、
filter node仮想pathの旋回円clamp（R12 L5、R13がAckermannでは良性と確認したため優先度低）、
および`speed_limit_`のデータ競合（R13 L7、本branch以前からの既存項目）である。Ackermannシナリオの拡充（R11 L5）は
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
| R13 | 同梱設定ガードがyamlに未接続、単一軌道に合わせた閾値、速度governorの閉ループ未被覆、シナリオ数の誤り | シナリオがyamlを直接読む、閾値を摂動帯から再導出（分離不能な1件は削除）、クリアランス検査追加、11本へ訂正 | [指摘](reviews/r13-2026-09-01-findings.md) / [対応](reviews/r13-2026-09-01-response.md) |
| R14 | main統合可否の判断。差動二輪出力段の未固定な挙動変化、同梱設定ガードのキー単位結合、新規2閾値の帯、`limits.w_max` 項の未検証、文書の虚偽記述 | 出力段の意味論を単体回帰で固定、ガードをファイルへ結合、分離しない閾値2件を削除し被覆を単体へ移設、`w_max` 拘束fixture追加、文書訂正 | [指摘](reviews/r14-2026-09-01-findings.md) / [対応](reviews/r14-2026-09-01-response.md) |
| R15 | 全方向モデルのmain統合可否。`vy`が「進行方向」を判断する4か所へ行き渡っておらず、純横移動指令で接触判定・DWA制動判定・emergency層の不変条件が不成立。廊下進入で接触1件。実測値6件が再現せず | 進行方向の判断4か所をベクトルへ一般化、fallbackの合成指令も停止判定へ、property testで不変条件を検査、実測値と本数の訂正 | [指摘](reviews/r15-2026-09-02-findings.md) / [対応](reviews/r15-2026-09-02-response.md) |
| R16 | R15対応の再レビュー。R15 H4が格子点上でのみ解消（0.002 m刻みで接触再現、侵入0.23 m）、deadbandが判定後に走りpublishされるtwistで不変条件が破れる、速度governorが前進軸のまま、R15対応が新設したコードが試験から未実行 | deadbandが出す指令を再判定、governorを進行方向へ一般化、fallbackへ到達する掃引を追加、方向復元を検査。H1は薄壁のrig縮退と判明し厚い壁へ移行のうえ制約を明記 | [指摘](reviews/r16-2026-09-02-findings.md) / [対応](reviews/r16-2026-09-02-response.md) |
| R17 | R16対応の再レビュー。governorの横方向スラブが対称化されて鏡映不変性が壊れた、deadband再判定が恒久停止を作る、第1掃引のSTOP検査が空虚、廊下境界とfilter nodeの記述が実測と食い違う | governorの横方向区間を非対称へ、deadbandは許容性を壊すなら適用しない、第1掃引のSTOP検査を実際に評価、境界と数値を全件再測定 | [指摘](reviews/r17-2026-09-02-findings.md) / [対応](reviews/r17-2026-09-02-response.md) |
| R18 | R17対応の再レビュー（最終）。掃引の安全不変条件が接触距離を被検コード自身から取る自己参照で、車体横幅を半分と見なす変異が「違反0」と印字して通過する。R17対応資料がinstall対象2ファイルの修正を実施せずに報告、deadband分岐のコメントの実測値（14件中14件）が再現しない（7/14）、「判別する回帰は用意できない」が偽、薄壁記述と英語版履歴が偽 | 掃引の接触距離を試験側の独立実装（回転掃引の厳密解）から取り直し、直進指令専用の第3掃引を追加、deadband不適用を数える回帰を追加、左右の支持幅の向きを固定、鏡映検査にヨーを投入。install対象の「same size」・deadbandの実測値・薄壁記述・英語版履歴・filter node数値をすべて実測へ訂正 | [指摘](reviews/r18-2026-09-02-findings.md) / [対応](reviews/r18-2026-09-02-response.md) |
| R19 | R18対応の再レビュー。範囲を `8eefe99` に限定し、打ち切り条件を先に決めて実施。掃引の独立接触検査が評価器のゲートの下流にあり「接触を見落とす向き」の欠陥を検出できない（移動tickの98.6%が未到達、取りこぼし系6変異が生存）、鏡映検査の`heading_gain`引き上げで変異killを2件喪失（R17 H1の部分的な巻き戻しを含む）、候補格子の掃引内訳と`avoid_margin.side`の遷移記述が同梱条件で再現しない | 独立接触検査を評価器のゲートの上へ出し評価器と同じ窓で打ち切る、鏡映検査の`heading_gain`を0へ戻し`cur_w`格子は残す、直進分岐の閾値を評価器に揃える、候補格子・`avoid_margin.side`・接触セル数・被覆閾値・pinのコメントをすべて実測へ | [指摘](reviews/r19-2026-09-02-findings.md) / [対応](reviews/r19-2026-09-02-response.md) |

## 現在の検証contract

- plain CMake Release buildとCTest 10件、ROS 2 Jazzy/Nav2環境ではadapter結合試験を加えたCTest 11件。
  adapter結合試験は既定設定（差動二輪）で実行し、Ackermannのパラメータ配線は独立した試験で検査する。
  両者は同一tickの表と裏を検査するため、`motion_model.type`の誤解決は必ずどちらかが検出する。
  Jazzyコンテナではさらに、`ackermann`ラベル試験の存在と通過、インストール済みAckermann設定、
  実ノードが不正な`motion_model.type`と非正の`turn_radius_min`を拒否することを検査する。
- 全方向モデルの単体試験10件と閉ループ17件。閉ループのうち4件は同一世界を差動二輪参照設定でも走らせて
  差分を検査し、1件はパラメータも無作為化した6000試行のproperty testで、出力した指令が自身の進行方向に
  沿って接触前に停止できることを毎tick検査する（R15 H1・H2・H3はこの形でしか見つからなかった）。同梱の
  全方向設定例そのものを走らせる試験を含む。
- core unit / property testと17 closed-loop scenarios、Ackermann 13 closed-loop scenarios。Ackermann側は
  同梱設定例そのものを走らせる試験を含み、狭路centeringでは横偏差・曲率符号反転・停止tickを、
  clutterではクリアランスと停止tickを閾値検査する。1周期あたりの曲率変化はoffset通路と同梱設定走行で
  検査する。閾値は正常時と破壊時の帯を**同一の摂動格子で両側とも掃引して**決めており、分離する値が
  存在しないと実測された検査は閾値を置かず、対象の被覆は閾値の要らない単体試験へ移している（R14 M3・M4）。
- 差動二輪の出力reachability段の意味論を固定する単体回帰（`test/output_stage_unit.cpp`）。曲率保存減速、
  複数回の反復、旋回不可時に厳密な`(0,0)`と`STOP`を出すことを検査する（R14 M1）。
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
