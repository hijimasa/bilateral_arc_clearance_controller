# ドキュメント索引

[English](en/README.md) | 日本語

利用者向けの現行文書と、リリース前検証の履歴を分けて整理する。

## 利用者向け

- [アルゴリズムと保証範囲](algorithm.md): 入出力、候補評価、設計上の性質、前提と非保証事項
- [Nav2統合ガイド](nav2_integration.md): Controller、Collision Monitor、costmap等との役割分担と設定
- [パラメータリファレンス](parameters.md): core・Nav2 plugin・filter nodeの全設定
- [既存手法との比較と評価](method_comparison.md): DWB、MPPI、RPP、VFH/NDとの比較とraw由来の結果
- [BACアブレーションと公平条件比較](ablation_and_matched_evaluation.md): 共通条件、432 episodeの結果、
  因果解釈の限界、Gazebo動画と同期telemetry
- [Gazebo再現環境](../examples/gazebo/README.md): Docker、world、URDF、収録・自動判定script

## 開発・リリース向け

- [Public 公開準備チェックリスト](public_release_checklist.md): Private 期間に完了する項目、公開判定、公開後の作業
- [リリースレビュー履歴](release_review_history.md): 全10回の主要指摘、対応、現在の残項目
- [レビュー記録規則とアーカイブ](reviews/README.md): 命名、必須メタデータ、更新規則
- `reviews/rNN-YYYY-MM-DD-{findings,response}.md`: 各時点の根拠と再現手順を残す監査証跡

個別レビュー文書には、その後撤回・訂正された途中時点の判断も含まれる。現在の判断にはREADME、
本索引から参照する現行文書、およびリリースレビュー履歴の最新状態を使う。

## 図

- [BAC geometry](images/bac_geometry.svg): 候補円弧、path projection、左右クリアランス
- [公平条件・出現障害物](images/matched_appearing_obstacle.png): BAC/DWB/MPPI/RPPの軌跡overlay
- [極狭路offset ablation](images/ablation_extreme_offset.png): BAC 4 variantの軌跡overlay
- [BAC対DWB左右比較GIF](media/bac_vs_dwb_matched_appearing_obstacle_preview.gif)
  ([高画質MP4](media/bac_vs_dwb_matched_appearing_obstacle.mp4)): matched benchmark run 1の同期replay
- [Gazebo adaptive-clearance GIF](media/bac_gazebo_adaptive_clearance_preview.gif)
  ([高画質MP4](media/bac_gazebo_adaptive_clearance.mp4)): 回避・復帰・1.0 mゲート通過の連続1系列
