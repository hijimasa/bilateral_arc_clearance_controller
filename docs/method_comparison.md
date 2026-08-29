# 既存手法との比較と BAC の位置づけ

[English](en/method_comparison.md) | 日本語

調査日: 2026-08-27 / リリース候補ベンチマーク確認日: 2026-08-29

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
| BAC | 定曲率円弧 `(v,w)` | 左右円弧クリアランス | 左右差を直接最小化 | 狭路の幾何中心、説明可能な小規模コア | 差動二輪・静的点群前提、角加速度過渡・jerk は未評価 |

BAC は DWA/DWB の一般的な置き換えというより、狭い開口、経路の横ずれ、経路上の未反映障害物を
重視した特化型 controller である。既存研究に対する性能優位や学術的新規性を、この実装だけから
主張するものではない。

## Nav2 system benchmark

2026-08-29に、障害物入力を10 Hz local costmap、controllerの後退候補を無効、並進速度上限と
simulator actuator加速度制限を共通化した
**公平条件比較216 episode**と、左右均衡項・後退候補・入力源を分けた**BACアブレーション216
episode**を追加した。条件、全体表、弱点、因果解釈の範囲は
[BACアブレーションと公平条件比較](ablation_and_matched_evaluation.md)に分離している。

公平条件ではBAC 54/54、DWB 48/54（衝突2）、MPPI 51/54、RPP 48/54だった。BACは出現障害物と
0.25 m横ずれで安定して完走した一方、Z字路では遅くclearanceも小さく、clutterでは1反復で長時間
停止した。アブレーションでは左右均衡項の狭路中心化への寄与が観測されたが、escapeの寄与は
基準BACが後退を選ばなかったため未同定である。

以下は、raw scanと後退を含む通常BACの機能有効状態を比較する従来のsystem-level datasetである。
新しい公平条件比較で置き換えるものではなく、実用構成全体の統合evidenceとして併記する。

ワークスペースの `nav2_benchmark` で、ROS 2 Jazzy、共通の矩形車体、NavFn、1 Hz 再計画、world、
2D LiDARシミュレータを使った。2026-08-29に、BAC `1f9911e`、benchmark `026a17a` のclean
worktreeから、同じ18シナリオ × 各3 run × 4 controller = 216 episodeを再生成した。

ただし、これはcontroller algorithmだけを分離した実験ではない。controller固有の統合経路には次の差がある。

| 要因 | BAC | 比較controller | 解釈上の影響 |
|---|---|---|---|
| 障害物入力 | 20 Hz raw scanを直接使用、costmap fallback | 主に10 Hz local costmap | 観測遅延と前処理が異なる |
| 後退 | `limits.v_min=-0.1` | DWB/RPPは無効、MPPIは-0.15まで許可 | escapeの行動集合が一致しない |
| tuning | BAC固有のlimitとweight | DWB/MPPI/RPP固有設定 | 単一scoreではなく設定済みsystemの比較になる |

以下の結果は統合evidenceと仮説形成には使えるが、差をbilateral clearanceへ因果帰属するものではない。
入力条件を揃えた比較とBAC ablationは上記の別データセットで実施済みである。
このデータセットは、エピソード間の `ROS_DOMAIN_ID` 分離を保証する runner で生成されており、
`results_release_1f9911e/domain_manifest.csv` により「同一 domain の保持区間の重なり 0」(216 episode、
90 個すべての domain ID を再利用、初回以降の再割当 126 回)が検証済みである。生成条件は
`results_release_1f9911e/provenance.json`（BAC commit SHA、
worktree dirty 数 0、Nav2 version、image digest、world/設定ハッシュ、並列数）に記録される。

| controller | 成功 episode | 衝突 | 成功時平均到達時間 | 中央値 | 観察された傾向 |
|---|---:|---:|---:|---:|---|
| BAC | 54/54 (100%) | 0 | 27.7 s | 28.6 s | 全条件で完走。試験集合では唯一、失敗と衝突がともに0 |
| DWB | 50/54 (93%) | 4 | 25.2 s | 25.4 s | 通常条件は高速。出現障害物3件と0.25 mズレ1件で衝突 |
| MPPI | 51/54 (94%) | 0 | 28.6 s | 27.5 s | 通常条件と横断障害物では完走。出現障害物3件でtimeout |
| RPP | 47/54 (87%) | 0 | 24.3 s | 25.0 s | 成功時は最速。出現障害物、0.25 mズレ、極端offsetで中断 |

平均到達時間は成功 episode のみの算術平均で、失敗を罰する総合スコアではない。controller ごとに
成功したシナリオ集合も異なるため、到達時間の横比較も参考値である。成功時の最大到達時間は
BAC 32.6 s、DWB 33.7 s、MPPI 45.1 s、RPP 29.3 sだった。シミュレータは決定論的でrun間の
独立性も限定的なため、上の比率を一般的な成功確率とは解釈しない。

結果差が最も大きく観測された条件は次の通り。

- `appearing_obstacle`: BAC 3/3。DWBは3件とも衝突、MPPIは3件ともtimeout、RPPは3件とも中断した。
  経路上へ後から現れた障害物をBACが局所的に迂回したが、入力更新率などの差も含むsystem-level結果である。
- `corridor_locdrift_15x`（1.5 m 通路 + **0.25 m** 自己位置ズレ）: BAC 3/3、DWB 2/3（衝突 1）、
  MPPI 3/3（ただしクリアランス 0.06〜0.08 まで低下）、RPP 0/3（中断）。ズレ量スイープは次節。
- `corridor_extreme_offset`: BAC/DWB/MPPIは3/3、RPPは2/3（1件中断）。
- 通常の開空間・広路・狭路では全 controller が成功し、BAC は概ね 1.05〜1.1 倍遅かった
  (開空間 16.1 s / RPP 14.9 s、広路 26.1 / 25.0、狭路オフセット 31.3 / 28.6)。
  速度規制を衝突コース限定にした改修以降、狭所でも巡航速度を大きく落とさない。
- 逆向き 2 連コーナーの Z 字路（1.7 m 幅）は全 controller が成功し、追従系が速い
  （RPP 21.1 s、DWB 21.3 s、MPPI 27.4 s、BAC 29.0 s）。旋回時の矩形コーナー掃引を正確化した結果、
  BAC の二重旋回は保守的である。自己位置ズレ変種（`corridor_zigzag_locdrift`）でも
  クリアランスを保って完走する。BAC の全54 episode中の最接近0.136 mはその自己位置ズレ変種で出ている。

### 自己位置ズレ量スイープ（1.5 m通路、`results_driftsweep_release_1f9911e/`）

地図↔オドメトリ間の横ズレを 0.10〜0.25 m で掃引した（各 2 episode）。

| ズレ量 [m] | BAC | DWB | MPPI | RPP |
|---:|---|---|---|---|
| 0.10 | 28.8 s / clr 0.230 | 25.2 s / 0.139 | 27.2 s / 0.171 | 24.8 s / 0.168 |
| 0.15 | 28.8 s / **0.227** | 25.2 s / 0.091 | 27.6 s / 0.116 | 25.2 s / 0.114 |
| 0.20 | 28.8 s / **0.225** | 37.0 s / 0.002 | 28.6 s / 0.063 | 35.7 s / 0.054 |
| 0.25 | 28.8 s / **0.225** | 1/2（1衝突） | 39.4 s / 0.085 | 0/2（2中断） |

このsweepでは、BACは8/8を完走し、平均到達時間28.8 s、クリアランス0.225〜0.230 mだった。
比較対象ではズレの増加に伴うクリアランス低下が観測され、0.25 mではDWBが1/2（1衝突）、
RPPが0/2となった。MPPIは2/2完走したが、最悪クリアランス0.085 m、平均到達時間39.4 sだった。この結果は試した設定と
決定論的simulatorに限られ、ズレ量に対する一般的な不変性やcontroller間の確率的な優劣を示さない。

評価環境(`nav2_benchmark/`: シミュレータ・評価器・launch・world・設定・スクリプト・コンテナ定義)は
親リポジトリで追跡されており、各結果セットの `provenance.json` が BAC commit・ベンチ tree hash・
コンテナ image digest・設定ハッシュを記録する。リリース時は raw episode のアーカイブを併せて
公開する。実機ではセンサ遅延、滑り、点群外れ値、動的物体、制御周期超過を追加評価する。

エピソード間の分離(並列実行時に `ROS_DOMAIN_ID` を共有しないこと)は、現在の runner では
domain free-list とプロセス回収の連動により構造的に保証され、各実行が残す `domain_manifest.csv`
(親プロセスが記録した起動時刻と**実際の回収時刻**)で run ごとに検証される。本ページの正準 18
シナリオの数値は、この runner で再生成したデータセットに基づく。ズレ量スイープ(32 episode)と
ギャップスイープ(24 episode)も同じBAC / benchmark revisionと現行runnerで再生成した。両sweepは
domainプール90未満で、domain IDの再利用は発生していない。

## 実行層で低減する感度 — グローバルプランナ単独との違い

「この性質はグローバルプランナ側だけで実現できるか」という問いに対する、構成上の整理と限定的な
実測である。**単一pose estimateに基づくグローバル経路を有限周期で更新する構成**では、
グローバルプランナ単独は次の制御周期内の処理を担当しない。BACはrobot frameの観測を各制御周期で
評価し、この構成における地図―odom間の横ずれと再計画遅延への感度を局所的に低減する。
belief-space / uncertainty-aware planning、他のlocal controller、独立安全層といった代替・補完手段の
存在を否定する主張ではない。

1. **情報の非対称性**: プランナは地図とポーズ推定の上で計画するため、ポーズ推定自体が誤って
   いるとき真の相対幾何を知る手段がない。実行層のうちロボット座標系の障害物幾何・緊急停止判定・
   左右クリアランスは地図↔オドメトリ誤差に直接依存しない。経路追従項(横偏差・終端距離・接線
   方位)は TF 変換後の plan を介して誤差の影響を受けるが、弱い横偏差重みと左右クリアランスの
   作用により、本評価範囲の通路では挙動劣化が観測されなかった(センサ外部パラメータ誤差や
   遅延は別途残る)。
2. **再計画の窓**: 再計画は有限レートであり、計画更新までの障害への速度応答はローカル層の責務。
   実験では比較controllerは1 Hz再計画と10 Hz local costmap更新の下で出現障害物に失敗した。
3. **速度応答の場所**: 「衝突コース上の点に応じた減速・進入拒否」は制御周期の局所幾何の関数で、
   global plannerのpath出力だけには含まれない。Nav2全体にはDWB / MPPI / RPP、Collision Monitor、
   Speed Filterなど別の局所速度応答手段がある。

### 過去の補助診断: 自己位置ズレをプランナ側マージンで吸収できるか

この診断は今回のrelease candidate datasetより前に実施したもので、同一revisionでは再生成しておらず、
P0 release archiveにも含めていない。今後のmatched-condition評価の動機としてのみ残す。素直な上流対策として、
グローバルコストマップの `footprint_padding` にドリフト相当の余裕を
積んで `corridor_locdrift_15x`(1.5 m 通路 + **0.25 m** 自己位置ズレ)を再走した(各 2 episode)。

| 上流対策 | DWB | MPPI | RPP | BAC(対策なし) |
|---|---|---|---|---|
| なし(基準) | 衝突 | 成功(clr 0.06) | 中断 | **成功**(28.8 s, clr 0.23) |
| padding 0.15 m | **依然衝突** | 中断 | 中断 | – |
| padding 0.30 m | 計画不能 | 計画不能 | 計画不能 | – |

機構は単純で、経路は元々通路中央にあり(誤っているのは座標系)、余裕を積んでも経路は動かない。
この評価では、余裕がdrift未満なら追従誤差が残り、車体+余裕が通路幅を超えると
(0.95 + 2×0.30 = 1.55 > 1.5)計画が成立しなかった。これは**試したpadding設定では**狭所通過と
誤差吸収を両立できなかったことを示すが、任意のplanner、local controller、uncertainty-aware手法が
代替不能であることの証明ではなく、この表は今回の同一revision比較evidenceには含めない。

### 実行層で観測した近接・衝突統計

全 18 シナリオ・各 3 run（controller あたり 54 episode、単一リビジョン）の障害物への最接近距離:

| controller | 衝突 | 接近 <0.05 m | 接近 <0.10 m | 最悪値 [m] | 中央値 [m] |
|---|---:|---:|---:|---:|---:|
| BAC | 0 | 0 | 0 | 0.136 | 0.303 |
| DWB | 4 | 7 | 15 | 0.000 | 0.203 |
| MPPI | 0 | 1 | 4 | 0.027 | 0.287 |
| RPP | 0 | 4 | 7 | 0.010 | 0.281 |

1 runずつの開口幅sweep（車体0.95 m）では、全controllerが1.25 mまで完走した。1.15 mでは
DWB/MPPI/RPPが完走した一方、BACはpath長9.39 mまで進行後にtimeoutし、最接近は0.050 mだった。
BACは1.25 mで38.5 sまで遅くなり、狭さに応じた保守化は観測されたが、「狭すぎる開口を安全に
拒否する」優位性はこの再実行からは主張しない。1 runの探索的境界評価であり、成功率にも一般化しない。

以上から、本パッケージの位置づけは「最速のcontroller」ではなく、robot frameの局所観測を用いて
**特定の座標横ずれと再計画遅延への感度低減を狙う特化型controller**である。本評価範囲では、
(a) 正準54 episodeで衝突0・最接近0.136 m、(b) 0.10〜0.25 m横ずれsweepで8/8成功、
(c) 観測範囲内での後退escapeを確認した。一方、1.15 m開口sweepはtimeoutであり限界も明確になった。
これらはセンサ、TF、odom、
footprint、制動model、下位速度追従を前提としたシミュレーション観測であり、上流異常一般からの独立や
実機安全保証を意味しない。

## Nav2 Collision Monitor との関係

実行層の安全性を主要な位置づけとする場合、controllerだけでなく
[Nav2 Collision Monitor](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/)
との比較・併用が前提になる。Collision Monitor は生センサをコストマップ・プランナから独立に読み、
stop / slowdown / limit / 速度依存 approach と source timeout を提供する独立安全層である。

役割の違いは次の通り。Collision Monitor は cmd_vel を**制限**する(減速・停止)が、**操舵しない**。
BACは左右クリアランスに基づくpathからの局所逸脱・狭所中心化・後退escapeまでを一つの
controllerで扱い、観測とmodelに基づいて余裕の大きい候補を選ぶ。両者は競合せず、
BAC + Collision Monitor の併用は多層安全としてむしろ推奨構成である(Collision Monitor が
最終防壁、BAC がその手前で発動頻度を下げる層)。併用時のベースライン評価は今後の課題とする。

## 参考文献・一次資料

1. D. Fox, W. Burgard, S. Thrun, “The Dynamic Window Approach to Collision Avoidance,” *IEEE Robotics & Automation Magazine*, 4(1), 1997. [CMU publication page](https://publications.ri.cmu.edu/the-dynamic-window-approach-to-collision-avoidance)
2. Navigation2, “DWB Controller.” [official documentation](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html)
3. Navigation2, “Model Predictive Path Integral Controller.” [official source documentation](https://github.com/ros-navigation/navigation2/blob/main/nav2_mppi_controller/README.md)
4. S. Macenski et al., “Regulated Pure Pursuit for Robot Path Tracking,” *Autonomous Robots*, 2023. [arXiv](https://arxiv.org/abs/2305.20026)
5. J. Borenstein, Y. Koren, “The Vector Field Histogram—Fast Obstacle Avoidance for Mobile Robots,” *IEEE Transactions on Robotics and Automation*, 7(3), 1991. [author-hosted PDF](https://public.websites.umich.edu/~ykoren/uploads/The_Vector_Field_HistogramuFast_Obstacle_Avoidance.pdf)
6. J. Minguez, L. Montano, “Nearness Diagram (ND) Navigation: Collision Avoidance in Troublesome Scenarios,” *IEEE Transactions on Robotics and Automation*, 20(1), 2004. [author-hosted PDF](https://webdiis.unizar.es/~jminguez/TRAND.pdf)
