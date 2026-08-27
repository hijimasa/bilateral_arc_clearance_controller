# 既存手法との比較と BAC の位置づけ

調査・ベンチマーク確認日: 2026-08-27

## 比較範囲

BAC は、矩形の差動二輪ロボットがローカル経路を追従しつつ、2D LiDAR またはコストマップ点群に
反応する問題を対象とする。ここでは、同じ問題に使われる速度空間サンプリング、軌道最適化、
厳密経路追従、および狭所向け反応型ナビゲーションと比較する。

以下は設計上の比較であり、各原論文の異なるデータセットから性能値を横並びにしたものではない。

## 手法系譜

### Dynamic Window Approach (DWA) / DWB

Fox, Burgard, Thrun の DWA は、ロボットの運動学・動力学から到達可能速度を絞り、速度空間で
並進・角速度を探索する反応型衝突回避である。BAC はこの候補生成と「衝突前に停止可能」という
admissibility を継承する。Nav2 の DWB は DWA を軌道生成器と critic のプラグイン構成へ拡張し、
障害物、経路距離、ゴール距離、振動などの critic の和で候補を選ぶ。

BAC の差は、障害物コストを単一の最小距離や costmap 値だけで評価せず、候補円弧の左右それぞれの
自由幅を保持する点にある。狭所では `min(left, right)` と左右差が幾何学的な中心方向を作り、
広所では飽和して経路項へ主導権を戻す。一方、DWB の critic 構成ほど拡張可能ではなく、全方向の
移動を扱う標準の holonomic trajectory generator も持たない。

### Model Predictive Path Integral (MPPI)

Nav2 MPPI は、時系列の制御ノイズから多数の軌道をロールアウトし、motion model と複数 critic で
制御列を更新するモデル予測制御器である。差動二輪、全方向、Ackermann のモデルを選べ、長い制御列と
豊富な critic により滑らかな回避を表現できる。

BAC は各 tick で定曲率円弧を選ぶため、計算とチューニング対象が小さく、左右クリアランスの意味を
ログから追いやすい。その代わり、S 字の制御列、動的障害物の将来予測、非差動二輪モデルは表現しない。

### Regulated Pure Pursuit (RPP)

RPP は pure pursuit に曲率・障害物近接・衝突までの時間による速度調整を加えた厳密経路追従器である。
経路が正しく空いている場面では高速で簡潔だが、基本目的は経路そのものを追うことであり、経路上に
未知障害物が残ったときの横方向の局所迂回を主目的にはしていない。

BAC はローカルゴールへの距離を保ちながら、左右クリアランスが必要なら経路から外れる。したがって
再プラン待ちを減らせる一方、開空間でも RPP より候補評価コストが高く、追従時間も長くなりやすい。

### VFH / Nearness Diagram (ND)

VFH は局所占有情報を極座標ヒストグラムに圧縮し、目標方向と障害物の谷から操舵方向を選ぶ高速な
反応型手法である。ND は高密度・複雑環境を状況分類して処理し、High Safety Narrow Region などの
狭所を明示的に扱う。

これらと BAC は、障害物の左右構造や開口を直接使う点が近い。BAC は極座標方向ではなく実行可能な
定曲率円弧上で車体幅・制動距離を評価し、nav2 のローカル経路を連続的な意図として残す点が異なる。
ND のような離散的状況分類や VFH のヒストグラム記憶は持たない。

## 要約

| 手法 | 探索単位 | 障害物表現 | 狭所の中心化 | 主な強み | 主な制約 |
|---|---|---|---|---|---|
| DWB | 到達可能な局所軌道 | costmap + critic | critic 設計に依存 | Nav2 標準、拡張可能、holonomic 対応 | critic 間の重み調整が大きい |
| MPPI | 時系列制御列 | costmap + critic | 軌道コストに依存 | 多様な運動モデル、滑らかな多段軌道 | 計算量と設定項目が多い |
| RPP | 経路上の追跡曲率 | 衝突時間・近接速度制御 | 経路に依存 | 高速で厳密な経路追従 | 局所的に経路を外れる回避が主目的ではない |
| VFH / ND | 操舵方向・状況別戦略 | 極座標ヒストグラム・近接図 | 開口／狭所を明示 | 反応性、複雑・狭所への設計知見 | 経路・車体動力学との統合は方式ごとに必要 |
| BAC | 定曲率円弧 `(v,w)` | 左右円弧クリアランス | 左右差を直接最小化 | 狭路の幾何中心、説明可能な小規模コア | 差動二輪・静的点群前提、角加速度は下位制御器任せ |

BAC は DWA/DWB の一般的な置き換えというより、狭い開口、経路の横ずれ、経路上の未反映障害物を
重視した特化型 controller である。既存研究に対する性能優位や学術的新規性を、この実装だけから
主張するものではない。

## 同一条件 nav2 ベンチマーク

ワークスペースの `nav2_benchmark` で、ROS 2 Jazzy、同一の矩形車体、速度・加速度上限、NavFn、
1 Hz 再計画、2D LiDAR シミュレータを使い controller のみを交換した。2026-08-28 に、
plan の TF 変換・掃引接触の厳密化（リリースレビュー対応）後の単一リビジョンで全 controller を
同一条件・同一 run 数（18 シナリオ × 各 3 run × 4 controller = 216 episode）で再生成した。
生成条件は `results/provenance.json`（BAC commit SHA、Nav2 version、world/設定ハッシュ）に記録される。

| controller | 成功 episode | 衝突 | 成功時平均到達時間 | 観察された傾向 |
|---|---:|---:|---:|---|
| BAC | 54/54 (100%) | 0 | 27.8 s | 全条件で完走。狭所・劣化条件でも減速幅が小さい |
| DWB | 50/54 (93%) | 2 | 24.6 s | 通常条件は高速。出現障害物・0.25 m ズレで衝突または擦過 |
| MPPI | 51/54 (94%) | 0 | 29.2 s | 通常条件と動的横断に強い。出現障害物で失敗 |
| RPP | 48/54 (89%) | 0 | 24.2 s | 最速。出現障害物と 0.25 m 自己位置ズレで中断 |

平均到達時間は成功 episode のみの算術平均で、失敗を罰する総合スコアではない。controller ごとに
成功したシナリオ集合も異なるため、到達時間の横比較も参考値である。シミュレータは決定論的で
run 間の独立性も限定的なため、上の比率を一般的な成功確率とは解釈しない。

差が最も明確だった条件は次の通り。

- `appearing_obstacle`: BAC 3/3、DWB/MPPI/RPP は各 0/3。経路上へ後から現れた障害物を BAC が
  局所的に迂回した。
- `corridor_locdrift_15x`（1.5 m 通路 + **0.25 m** 自己位置ズレ）: BAC 3/3、DWB 2/3（衝突 1）、
  MPPI 3/3（ただしクリアランス 0.06〜0.08 まで低下）、RPP 0/3（中断）。ズレ量スイープは次節。
- 通常の開空間・広路・狭路では全 controller が成功し、BAC は概ね 1.1〜1.3 倍遅かった。
  速度規制を衝突コース限定にした改修以降、狭所でも巡航速度を大きく落とさない。
- 逆向き 2 連コーナーの Z 字路（1.7 m 幅）は全 controller が成功し、追従系が速い
  （RPP ~21 s、DWB ~21 s、BAC ~38 s）。旋回時の矩形コーナー掃引を正確化した結果、BAC の
  二重旋回はさらに保守的になった。自己位置ズレ変種（`corridor_zigzag_locdrift`）でも
  クリアランスを保って完走する。

### 自己位置ズレ量スイープ（1.5 m 通路、`results_driftsweep/`）

地図↔オドメトリ間の横ズレを 0.10〜0.25 m で掃引した（各 2 episode）。

| ズレ量 [m] | BAC | DWB | MPPI | RPP |
|---:|---|---|---|---|
| 0.10 | 28.8 s / clr 0.23 | 25.2 s / 0.14 | 27.2 s / 0.17 | 24.8 s / 0.17 |
| 0.15 | 28.8 s / **0.23** | 25.2 s / 0.09 | 27.6 s / 0.11 | 25.2 s / 0.11 |
| 0.20 | 28.8 s / **0.23** | 25.2 s / 0.07 | 29.5 s / 0.06 | 27.8 s / 0.06 |
| 0.25 | 28.8 s / **0.22** | 1/2（中断） | 51.8 s / 0.08 | 0/2（中断） |

BAC の到達時間とクリアランスは**ズレ量に対して不変**である（全域で 28.8 s / 0.22〜0.23）。
追従系はズレにほぼ 1:1 でクリアランスを失い、0.25 m では run により中断・衝突・数 cm の擦過の
いずれかに至る（世代間で failure mode が揺れること自体が margin-less 動作の証跡である）。
MPPI は完走する場合もクリアランス 0.08 m・時間 +80% と大きく劣化する。

評価環境(`nav2_benchmark/`: シミュレータ・評価器・launch・world・設定・スクリプト・コンテナ定義)は
親リポジトリで追跡されており、各結果セットの `provenance.json` が BAC commit・ベンチ tree hash・
コンテナ image digest・設定ハッシュを記録する。リリース時は raw episode のアーカイブを併せて
公開する。実機ではセンサ遅延、滑り、点群外れ値、動的物体、制御周期超過を追加評価する。

## 実行層の頑健性契約 — プランナ側対策では代替できないこと

「この頑健性は本来グローバルプランナが保証すべきではないか」という問いに対する検証。
主張の範囲を正確にすると: **単一ポーズ推定に基づくグローバル経路を有限周期で更新する構成**
(標準的な nav2 構成)では、グローバルプランナ単独で以下を代替できない。BAC はロボット座標系の
観測を制御周期で使い、この構成上の弱点を局所的に補う。belief-space / uncertainty-aware planning や
独立安全層(後述の Collision Monitor)といった別のアプローチが存在することは否定しない。

1. **情報の非対称性**: プランナは地図とポーズ推定の上で計画するため、ポーズ推定自体が誤って
   いるとき真の相対幾何を知る手段がない。実行層のうちロボット座標系の障害物幾何・緊急停止判定・
   左右クリアランスは地図↔オドメトリ誤差に直接依存しない。経路追従項(横偏差・終端距離・接線
   方位)は TF 変換後の plan を介して誤差の影響を受けるが、弱い横偏差重みと左右クリアランスの
   作用により、本評価範囲の通路では挙動劣化が観測されなかった(センサ外部パラメータ誤差や
   遅延は別途残る)。
2. **再計画の窓**: 再計画は有限レートであり、計画更新までの間の障害は定義上ローカル層の責務。
   実験では追従系は 1 Hz 再計画・5 Hz コストマップ更新の下でも出現障害物に失敗した。
3. **速度応答の語彙**: 「近接に応じた減速・進入拒否」は制御周期の瞬時幾何の関数であり、
   プランナの表現対象にない(nav2 の speed filter は地図ベースの静的ゾーン)。

### 上流対策の実測: 自己位置ズレをプランナ側マージンで吸収できるか

素直な上流対策として、グローバルコストマップの `footprint_padding` にドリフト相当の余裕を
積んで `corridor_locdrift_15x`(1.5 m 通路 + **0.25 m** 自己位置ズレ)を再走した(各 2 episode)。

| 上流対策 | DWB | MPPI | RPP | BAC(対策なし) |
|---|---|---|---|---|
| なし(基準) | 衝突 | 成功(clr 0.06) | 中断 | **成功**(28.8 s, clr 0.23) |
| padding 0.15 m | **依然衝突** | 中断 | 中断 | – |
| padding 0.30 m | 計画不能 | 計画不能 | 計画不能 | – |

機構は単純で、経路は元々通路中央にあり(誤っているのは座標系)、余裕を積んでも経路は動かない。
余裕がドリフト未満なら追従誤差はそのまま残って衝突し、車体+余裕が通路幅を超えれば
(0.95 + 2×0.30 = 1.55 > 1.5)計画自体が成立せず任務を放棄する。**上流マージンは「安全に
通れない」ことしか保証できない**。狭所通過と誤差耐性の両立は実行層でのみ達成される。

### 実行層としての安全統計

全 18 シナリオ・各 3 run（controller あたり 54 episode、単一リビジョン）の障害物への最接近距離:

| controller | 衝突 | 接近 <0.05 m | 接近 <0.10 m | 最悪値 [m] | 中央値 [m] |
|---|---:|---:|---:|---:|---:|
| BAC | 0 | 0 | 0 | 0.154 | 0.303 |
| DWB | 2 | 8 | 13 | 0.000 | 0.205 |
| MPPI | 0 | 1 | 4 | 0.033 | 0.277 |
| RPP | 0 | 0 | 4 | 0.093 | 0.300 |

幅を掃引した狭窄部(車体 0.95 m)では、DWB/RPP の速度は狭窄幅にほぼ応答せず(片側余裕
5 cm の 1.15 m 隙間でも 0.40 m/s)、MPPI はわずかに減速する(0.39 → 0.38)。BAC は
0.39 → 0.28(幅 1.35)→ 0.10(1.25)→ 進入拒否(1.15、衝突なしで停止)と、応答の大きさと
「狭すぎる開口には入らない」拒否挙動の両方で他と異なる。なお進入拒否時の最接近は約 0.11 m
であり、上表の「0.10 m 非侵入」は 18 シナリオ集合に対する統計で、拒否境界での停止動作までは
一般化しない。

以上から、本パッケージの位置づけは「最速の controller」ではなく、**上流(自己位置・地図・
プランナ・再計画)の状態に依らず成立する、実行層の測定可能な頑健性契約**である:
(a) 全 54 episode(単一リビジョン・全 18 シナリオ)で障害物 0.15 m 以内に非侵入・衝突ゼロ、(b) 速度は障害物への近接に単調応答、
(c) 安全マージン床を下回る通路には進入しない、(d) 行き詰まりでは凍結せず後退して待つ。

## Nav2 Collision Monitor との関係

実行層の安全性を主要な位置づけとする場合、controller だけでなく
[Nav2 Collision Monitor](https://docs.nav2.org/configuration/packages/configuring-collision-monitor.html)
との比較・併用が前提になる。Collision Monitor は生センサをコストマップ・プランナから独立に読み、
stop / slowdown / limit / 速度依存 approach と source timeout を提供する独立安全層である。

役割の違いは次の通り。Collision Monitor は cmd_vel を**制限**する(減速・停止)が、**操舵しない**。
BAC は左右クリアランスに基づく経路からの局所逸脱・狭所中心化・後退エスケープまでを一つの
controller で扱い、危険を「止まって待つ」だけでなく「安全な側へ動いて解消する」。両者は競合せず、
BAC + Collision Monitor の併用は多層安全としてむしろ推奨構成である(Collision Monitor が
最終防壁、BAC がその手前で発動頻度を下げる層)。併用時のベースライン評価は今後の課題とする。

## 参考文献・一次資料

1. D. Fox, W. Burgard, S. Thrun, “The Dynamic Window Approach to Collision Avoidance,” *IEEE Robotics & Automation Magazine*, 4(1), 1997. [CMU publication page](https://publications.ri.cmu.edu/the-dynamic-window-approach-to-collision-avoidance)
2. Navigation2, “DWB Controller.” [official documentation](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html)
3. Navigation2, “Model Predictive Path Integral Controller.” [official source documentation](https://github.com/ros-navigation/navigation2/blob/main/nav2_mppi_controller/README.md)
4. S. Macenski et al., “Regulated Pure Pursuit for Robot Path Tracking,” *Autonomous Robots*, 2023. [arXiv](https://arxiv.org/abs/2305.20026)
5. J. Borenstein, Y. Koren, “The Vector Field Histogram—Fast Obstacle Avoidance for Mobile Robots,” *IEEE Transactions on Robotics and Automation*, 7(3), 1991. [author-hosted PDF](https://public.websites.umich.edu/~ykoren/uploads/The_Vector_Field_HistogramuFast_Obstacle_Avoidance.pdf)
6. J. Minguez, L. Montano, “Nearness Diagram (ND) Navigation: Collision Avoidance in Troublesome Scenarios,” *IEEE Transactions on Robotics and Automation*, 20(1), 2004. [author-hosted PDF](https://webdiis.unizar.es/~jminguez/TRAND.pdf)
