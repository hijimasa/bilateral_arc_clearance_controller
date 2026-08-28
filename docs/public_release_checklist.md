# Public 公開準備チェックリスト

[English](en/public_release_checklist.md) | 日本語

- 最終更新: 2026-08-29
- 対象: `bilateral_arc_clearance_controller` 0.1.0
- 方針: Public 化、Git tag、GitHub Release、rosdistro 申請は、下記の Private 準備が完了してから行う

この文書は、技術的な公開価値と「ROS ユーザが実際に導入・検証できる状態」を分けて管理する。
チェック済みは実装済みを意味するが、CI のように GitHub 上での初回成功確認が必要な項目は明記する。

## P0: Public 化前の必須項目

| 状態 | 項目 | 完了条件 |
|---|---|---|
| 完了 | 主張範囲の限定 | README と理論文書が、simulation 結果・仮定・非保証を区別する |
| 完了 | ライセンスと著者情報 | MIT LICENSE、source header、`package.xml` の個人名が一致する |
| 完了 | 最小導入資料 | Nav2 設定例、filter 設定例、launch 例が install 対象になる |
| 完了 | framework-free 回帰試験 | core、scenario、scan/plan adapter の CTest が通る |
| 完了 | GitHub Actions | GCC、Clang、ASan/UBSan、ROS 2 Jazzy/Nav2 job の初回greenを確認（2026-08-29） |
| 完了 | Nav2 adapter 結合試験 | lifecycle、TF 失敗、scan stale/invalid、costmap fallback、speed limit を自動試験する |
| 完了 | 公開物の機密・権利監査 | history を含む tracked files に秘密情報、社内 path、第三者データ、再配布不能物がない |
| 完了 | benchmark 公開 archive | `release_archive_1f9911e/`にsource、container digest、272 episodeのraw/summary、SHA256を固定 |
| 完了 | release candidate dry run | clean clone 相当のJazzy/Nav2環境で build/test/install/launch file load が成功する |

P0 がすべて完了するまで repository visibility は変更しない。実機試験は 0.1.0 の公開そのものを
禁止する条件とはしないが、実施していない場合は「simulation-first preview」と release notes に明記する。
2026-08-29時点でP0はすべて完了し、技術的にはPublic化可能である。Hosted CIはpush後の全job greenを
確認した。benchmark archiveはBAC `1f9911e`、benchmark `026a17a`、ROS 2 Jazzy / Nav2 1.3.12、container
image digestを記録し、正準216、drift 32、gap 24 episodeを欠損・破損0で収録する。公開操作、tag、
GitHub Release作成は引き続き所有者が明示的に実施する。
履歴監査ではsecret、秘密鍵、個人環境の絶対path、大容量binaryを検出しなかった。Git commit metadataには
`hijimasa@gmail.com` が残るため、これを公開identityに含めない場合だけPublic化前に履歴方針を再検討する。

## P1: インパクトと信頼性のため強く推奨

| 状態 | 項目 | 目的 |
|---|---|---|
| 完了 | ROS 境界の整理 | scan 投影と plan 変換/pruneを共通化し、ROS lifecycle とfallbackも結合試験する |
| 完了 | 入力源 diagnostics | raw scan 使用、costmap fallback、その理由、candidate/admissible 数を標準診断 topic へ出す |
| 未完了 | RViz debug 表示 | 選択円弧、preview goal、停止・回避状態を任意 topic で可視化する |
| 未完了 | 公平性を揃えた比較 | scan/costmap 更新周期、reverse 可否、速度・加速度条件を揃えた比較を追加する |
| 未完了 | BAC ablation | `weights.balance=0`、escape 無効、raw scan/costmap の差を分解する |
| 未完了 | 現行 baseline | 現行RPP、MPPI、DWB、およびCollision Monitor併用条件を比較範囲に含める |
| 未完了 | 実機 evidence | localization offset、出現障害物、通過不能幅の stop/escape を動画と log で示す |
| 未完了 | 性能予算の CI 化 | core microbenchmark の regression 閾値を、runner ノイズを考慮して設定する |

比較結果には「何を同一にし、何が controller 固有で異なるか」を表で残す。3 回の deterministic run は
独立な統計標本として扱わず、再現性確認と位置付ける。

## P2: 継続的なプロジェクト運営

- `QUALITY_DECLARATION.md` を追加し、実態に合う REP-2004 quality level と未達要件を記す。
- versioning、deprecated parameter、サポート ROS distribution の方針を定める。
- issue template、release notes template、保守応答方針を整備する。
- core の coverage と static analysis を計測し、数値を badge や主張に使う前に CI artifact 化する。
- `BacCore::process()` の責務分割と角加速度過渡を含む rollout を次期設計課題として扱う。

## Public 化後にのみ行う項目

1. repository visibility を Public に変更する。
2. protected branch と required CI checks を有効化する。
3. `v0.1.0` signed/annotated tag と GitHub Release を作成し、検証済み archive を添付する。
4. ROS Index/rosdistro の release 手続きを行う。
5. Nav2 known plugins list への掲載 PR を、公開 URL・使用例・動画を添えて提出する。
6. 公開 URL、issue tracker、release artifact のリンク切れを外部 clone から再確認する。

## Public 化の判定

- **公開可能**: P0 がすべて完了し、未実施の P1 が README と release notes に明記されている。
- **公開延期**: CI、adapter fail-safe、benchmark provenance、権利/機密監査のいずれかが確認できない。
- **インパクトを期待できる公開**: 公開可能条件に加え、公平性を揃えた比較、ablation、最低 1 系列の実機動画がある。

公開の価値は十分にあるが、最初の公開時に最大の説得力を得るには、単なる source 公開ではなく
「再現できる比較」「入力状態を観測できる plugin」「限定条件を明記した実機 evidence」を同時に出す。
