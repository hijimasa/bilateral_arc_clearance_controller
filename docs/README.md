# ドキュメント索引

[English](en/README.md) | 日本語

利用者向けの現行文書と、リリース前検証の履歴を分けて整理する。

## 利用者向け

- [アルゴリズムと保証範囲](algorithm.md): 入出力、候補評価、設計上の性質、前提と非保証事項
- [Nav2統合ガイド](nav2_integration.md): Controller、Collision Monitor、costmap等との役割分担と設定
- [パラメータリファレンス](parameters.md): core・Nav2 plugin・filter nodeの全設定
- [既存手法との比較と評価](method_comparison.md): DWB、MPPI、RPP、VFH/NDとの比較とraw由来の結果

## 開発・リリース向け

- [リリースレビュー履歴](release_review_history.md): 全10回の主要指摘、対応、現在の残項目
- [レビュー記録規則とアーカイブ](reviews/README.md): 命名、必須メタデータ、更新規則
- `reviews/rNN-YYYY-MM-DD-{findings,response}.md`: 各時点の根拠と再現手順を残す監査証跡

個別レビュー文書には、その後撤回・訂正された途中時点の判断も含まれる。現在の判断にはREADME、
本索引から参照する現行文書、およびリリースレビュー履歴の最新状態を使う。

## 図

- [BAC geometry](images/bac_geometry.svg): 候補円弧、path projection、左右クリアランス
