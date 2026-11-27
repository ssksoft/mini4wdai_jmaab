# ヘッドレス版 マニュアル（詳細版）

UE を使わずに、Python と物理エンジン **MuJoCo** だけでミニ四駆を走らせる「ヘッドレス版」の説明です。短い手順は、リポジトリ直下の [README.md](../../README.md)（クイックスタートの手順 1〜3）にあります。

- **対象：** 市販ミニ四駆（マッドレイザー／MAシャーシ）＋ XIAO MG24 Sense ＋ KEIMYO-001 モータードライバ
- **同じもの：** コース、車、ドライバー・電池・モーター・IMU のモデル（`sil_core.py`）、Python の制御（`controllers/*.py`）。これらは UE 版（[manual.md](manual.md)）と**まったく同じもの**を使います。
- **違うもの：** 物理エンジン（UE の Chaos → MuJoCo）と、走っている最中は画面がないこと（走ったあとに 3D で再生はできます。4.5）。

---

## 0. UE 版との使い分け

| | ヘッドレス版 | UE 版 |
|---|---|---|
| 必要なもの | Python 3.11〜3.13 と pip のパッケージ | UE 5.8 と UE プロジェクト |
| 見た目 | 走ったあとに 3D で再生できる（`--view` / `--video`。当たり判定の形をそのまま表示する簡素な見た目）、グラフとログ | 3D で走りを見られる（きれいな見た目） |
| 速さ | 5周を 1〜2 秒で計算 | 実時間の 1/4（既定のスロー）で走る |
| 向いていること | 制御の当たりを付ける、パラメーターの総当たり、どこでコースアウトしたかの確認、自動テスト | 走りを目で確かめる、最終確認、人に見せる |
| 結果の一致 | ラップは UE と数％の差。ぎりぎりの走行は結果が変わることがある（9章） | 基準 |

**おすすめの流れ：**
1. ヘッドレスで制御を書いて、`headless sweep` で良いパラメーターを探します。
2. 良さそうな設定を UE の `start_py` で走らせて確認します。
3. 実機に移します。

---

## 1. 仕組み

```
 MuJoCo（1/960 秒ごと）
   コース = コース部品 FBX の当たり判定（凸形状 538 個）を、UE のレベルと同じ並びで配置
   車     = 車体の箱 ＋ タイヤ 4 個 ＋ ローラー 6 個（いずれも摩擦なし。壁がローラーで車を導く）
      │ 車の姿勢・速度                                ▲ 前後方向の力（駆動・ブレーキ）、タイヤの横グリップ
      ▼                                               │
 sil_core.PyCar：仮想 IMU（416 Hz） → Controller.step(io)（CONTROL_HZ） → KEIMYO-001 ＋ 電池 ＋ モーター
```

- **コース：** `mini4wd_course/fbx/` のコース部品（`SM_Straight` など5種類）の当たり判定（`UCX_` で始まる凸形状）を読み込みます。並びは UE のレベル `Mini4WD_CourseOut` と同じ（`LAYOUT`）で、時計回りです。
- **力のかけ方：** UE のランタイムと同じ式を使います。
  - タイヤが接地しているときだけ、制御の出力（`io.duty`）から計算した駆動力をかける
  - 車軸ごとのタイヤの横グリップ
  - 回転の減衰
- **制御：** UE 版と同じ `sil_core.PyCar` が、IMU の値を作って `Controller.step(io)` を呼び、デューティから駆動力を計算します。制御ファイルは UE 版と共通で、どちらでもそのまま動きます。
- **周回の判定：**
  - 前向きの速度が 0.3 m/s を超えたときにラップの計測を始めます（スタート前のブレーキ待ちは含みません）。
  - ゴールラインを前向きに通過するたびに、1周と数えます。

---

## 2. ファイル構成

```
3dsim/
├─ docs/headless_manual.md        … このファイル
├─ mini4wd_course/
│   ├─ mjsim/m4wd_mj.py           … MuJoCo のコースと車のモデル（LAYOUT, DEFAULT_DESIGN など）
│   ├─ generator/layout_preview.py, fbxread.py … FBX の読み込み・部品の配置（UE と同じ計算）
│   ├─ ue/m4wd_ctrl.py            … IMU の計算など（UE と共通）
│   └─ fbx/                       … コース部品の FBX（Git LFS。約 0.7 MB）
└─ MiniAI_PySim/
    ├─ setup_headless.bat         … セットアップ（.venv の作成、パッケージ導入、環境チェック）
    ├─ headless.bat               … 実行の入口（headless check / <制御> / sweep / view / video）
    ├─ requirements-headless.txt  … 必要なパッケージ（MuJoCo は 4.0.1 に固定）
    ├─ sil/run_headless.py        … ヘッドレス実行の本体
    ├─ sil/replay_headless.py     … 3D 再生（ビューア・動画）
    ├─ sil/sil_core.py            … ドライバー・電池・モーター・IMU のモデル（UE と共通）
    ├─ controllers/               … 制御（UE と共通。local/ は個人用で Git の対象外）
    ├─ tools/headless.py          … headless.bat から呼ばれる入口（Python で直接呼んでもよい）
    ├─ tools/check_headless.py    … 環境チェック
    ├─ tools/sweep_headless.py    … パラメーター探索
    ├─ tools/setup_headless.py    … セットアップの本体
    ├─ tests/test_headless.py     … 自動テスト（MuJoCo がなければスキップ）
    ├─ .venv/                     … セットアップで作られる専用環境（Git の対象外）
    └─ sim_out/                   … 実行結果（Git の対象外）
```

---

## 3. セットアップ

### 3.1 用意するもの

| もの | 内容 |
|---|---|
| Python **3.11〜3.13** | https://www.python.org/ から Windows 版（3.12 推奨）。インストーラーの最初の画面で「Add python.exe to PATH」にチェック。MuJoCo 4.0.1 は 3.10 以前と 3.14 以降の Windows 版がありません |
| リポジトリ | `git clone` → `git lfs pull` 済みのもの（[README](../../README.md) の手順 1）。**ZIP ダウンロードでは動きません**（コース部品の FBX が LFS の引換券のままになるため） |
| ネットワーク | 初回に pip でパッケージ（MuJoCo、numpy、matplotlib、動画用の imageio-ffmpeg など数十 MB）をダウンロードします |

UE は不要です。ヘッドレス版だけを使う場合、LFS の大きなデータ（UE のアセットやスキャンデータ、約 117 MB）はダウンロードしなくても動きます。必要なのは `3dsim/mini4wd_course/fbx/`（約 0.7 MB）だけです。README の手順 1 は、この形で clone します。

```
set GIT_LFS_SKIP_SMUDGE=1
git clone https://github.com/ssksoft/mini4wdai_jmaab.git
cd mini4wdai_jmaab
set GIT_LFS_SKIP_SMUDGE=
git lfs pull --include "3dsim/mini4wd_course/fbx/*"
```

あとで UE 版も使うときは、リポジトリで `git lfs pull` を実行すれば残りが入ります。以前の手順で `git config lfs.fetchinclude …` を設定した場合は、先に `git config --unset lfs.fetchinclude` で外してください（設定が残っていると FBX しか取れません）。

- `GIT_LFS_SKIP_SMUDGE=1` は、clone のときに LFS の本体をダウンロードしないための設定です。
- `lfs.fetchinclude` は、以後の `git lfs pull` / `git pull` で、指定したフォルダだけを取得する設定です。
- UE も使うことになったら、`git config --unset lfs.fetchinclude` のあと、もう一度 `git lfs pull` を実行してください。

### 3.2 自動セットアップ（Windows）

`3dsim\MiniAI_PySim\setup_headless.bat` をダブルクリックします。次の順に処理します。

1. Python を探します（`py -3`、なければ `python`）。3.11 未満なら止まります。
2. `3dsim\MiniAI_PySim\.venv` に専用の仮想環境を作ります。PC 全体の Python には何も入れません。
3. `requirements-headless.txt` のパッケージ（mujoco 4.0.1、numpy、matplotlib、imageio・imageio-ffmpeg、pytest）を入れます。
4. 環境チェック（`headless check`）を実行します。最後に「結果: すべて OK（NG 0 件）」と出れば完了です。

作り直したいとき（Python を入れ替えたときなど）は、コマンドプロンプトで次を実行します。

```
cd /d <リポジトリ>\3dsim\MiniAI_PySim
setup_headless.bat --recreate
```

### 3.3 手動セットアップ（macOS / Linux、または自分で管理したい場合）

```
cd <リポジトリ>/3dsim/MiniAI_PySim
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-headless.txt     # Windows: .venv\Scripts\python
.venv/bin/python tools/headless.py check
```

以降、`headless ...` の代わりに `.venv/bin/python tools/headless.py ...` で実行します。

### 3.4 環境チェック（`headless check`）

| 確認すること | NG のとき |
|---|---|
| Python 3.11 以上 | Python 3.12 を入れ、`setup_headless.bat --recreate` |
| パッケージ mujoco / numpy / matplotlib | `setup_headless.bat` を実行。MuJoCo が 4.0.1 以外なら「注意」が出る（9.2） |
| パッケージ imageio / imageio-ffmpeg | 3D 動画（`--video`）用。v1.1.0 より前にセットアップした場合は `setup_headless.bat` をもう一度実行 |
| `mjsim/m4wd_mj.py` などのファイル | リポジトリが最新か確認（`git pull`） |
| コース部品の FBX が取得済み | `git lfs pull`（ZIP で入手した場合は clone し直す） |
| 制御ファイルを読み込める | 表示されたエラーの行を直す |
| コースのモデルを作れる | 表示されたエラーを確認 |
| 試走（lc_brake、標準モーター、1周） | 1周が 2.05〜2.40 秒（基準 2.219 秒）に入るか。外れたらモデルや MuJoCo の版が変わっている |

結果は `sim_out\check_headless_report.txt` にも保存されます。問い合わせのときは、このファイルを添えてください。

---

## 4. 実行する

### 4.1 基本

```
cd /d <リポジトリ>\3dsim\MiniAI_PySim
headless lc_brake
```

- 制御の名前（`controllers\<名前>.py`、なければ `controllers\local\<名前>.py`）か、`.py` のパスを渡します。
- 周回ごとに `lap 1: 2.219 s` のように表示され、最後に結果のまとめ（JSON）と「結果: 完走 合計 5.803 s -> 出力先」が出ます。

### 4.2 オプション

| オプション | 既定値 | 内容 |
|---|---|---|
| `--motor` | `standard` | `standard`（キット付属）／`machdash_pro`（`sil_core.py` の `MOTORS` に足したものも使える） |
| `--battery` | `nimh2` | `nimh2`（ニッケル水素2本）／`nimh2_low`（使い込み）／`liion1`（リチウムイオン1セル） |
| `--param KEY=VALUE` | — | `Controller(**params)` の値を上書きする（何個でも）。値は JSON として読む（`0.15` `true` `"abc"`） |
| `--laps` | `3` | 完走と判定する周回数 |
| `--mass` | `0.135` | 車重 [kg]（キット 125.9 g ＋ XIAO・基板・配線 約 9 g） |
| `--imu-axes` | `x,y,z` | IMU の取り付け向き（UE 版マニュアル 5.2 と同じ書き方） |
| `--seed` | `0` | IMU のノイズと残留オフセットの乱数の種 |
| `--no-noise` | — | ノイズなしの理想的な IMU にする |
| `--boot` | `0` | 車を離す前に静止させておく時間 [s] |
| `--timeout` | `20` | シミュレーション時間 [s]。超えたら打ち切る |
| `--plot` | — | `plot.png` も作る |
| `--view` | — | 走ったあとに 3D ビューアの窓で再生する（4.5） |
| `--video [follow\|top]` | — | 走ったあとに `run.mp4` を作る。`follow` = 車の後ろから追う（既定）、`top` = コース全体（4.5） |
| `--out` | `sim_out\<制御名>` | 出力先のフォルダ。**同じ制御名で実行すると上書き**されるので、残したいときは変える |
| `--quiet` | — | 何も表示しない（スクリプトから呼ぶとき） |
| `--course-dir` | `3dsim\mini4wd_course` | コースのデータの場所 |

### 4.3 結果の判定（`outcome`）

| 値 | 表示 | 条件 |
|---|---|---|
| `ok` | 完走 | `--laps` の周回数を走りきった |
| `courseout` | コースアウト | 車の高さが床より下になった、0.5 秒以上ひっくり返っていた、コースの範囲から 8 cm 以上出た |
| `stuck` | 停止 | 走り出して 1 秒以上たってから、速度 0.05 m/s 未満が 1.5 秒続いた |
| `timeout` | 時間切れ | `--timeout` を超えた（走ってはいた） |
| `never_started` | 一度も走り出さなかった | `--timeout` まで、前向きの速度が 0.3 m/s を超えなかった |
| `controller_error` | 制御のエラー | `setup()` / `step()` で例外が起きた（ファイル名と行番号を表示） |

### 4.4 Python から呼ぶ

```python
import sys
sys.path.insert(0, r"<リポジトリ>/3dsim/MiniAI_PySim/sil")
import run_headless as RH
r = RH.main(["lc_brake", "--motor", "machdash_pro", "--laps", "5", "--quiet"])
print(r["outcome"], r["laps"], r["total"])
```

戻り値は `summary.json` と同じ内容の辞書です。

### 4.5 3D で見る（ビューア・動画）

毎回の走行で、車の位置と向きを 1/60 秒ごとに `replay.npz` に記録しています。これを同じ MuJoCo のモデルで再生するので、**シミュレーションはやり直さず、結果も変わりません**。計算の速さ（5周 1〜2 秒）もそのままです。

```
headless lc_brake --view            走らせて、そのままビューアで再生
headless lc_brake --video           走らせて、sim_out\lc_brake\run.mp4 を作る
headless view lc_brake              前回の走行をビューアで再生（走らせ直さない）
headless video lc_brake --cam top   前回の走行をコース全体の視点で動画にする
```

**ビューア（`headless view`）**

| 操作 | 内容 |
|---|---|
| スペース | 一時停止・再開 |
| `R` | 最初から |
| 左ドラッグ ／ 右ドラッグ ／ ホイール | 視点を回す ／ 動かす ／ 拡大・縮小 |
| 窓を閉じる | 終了 |

- オプション：`--speed 0.25`（1/4 倍速。既定 1 = 実時間）、`--loop`（繰り返し再生）
- 視点は車を追いかけます。左上に、時刻・速度・デューティ・周回が出ます。最後のコマには結果（`FINISHED` `COURSE OUT` など）が出ます。

**動画（`headless video`）**

| オプション | 既定値 | 内容 |
|---|---|---|
| `--cam` | `follow` | `follow` = 車の後ろから追う ／ `top` = コース全体を斜め上から |
| `--speed` | `1` | 再生の速さ（`0.5` = 1/2 倍速のスロー） |
| `--fps` | `30` | 1 秒あたりのコマ数 |
| `--size` | `1280x720` | 幅x高さ |
| `-o` | `<フォルダ>\run.mp4` | 出力ファイル |

- 画面の上に、時刻・速度・デューティ・周回が入ります。終わりに 1 秒、最後のコマが止まって入ります。
- 書き出しにかかる時間は PC の描画性能しだいです（GPU のある PC なら数秒。描画をソフトウェアで行う環境では 3周で 30 秒ほど）。
- 走らせる名前の代わりに、`--out` で指定したフォルダや、`headless sweep` の `run_###` フォルダも指定できます。

**見た目について**：コースと車は、当たり判定に使っている形（コース部品の凸形状、車の箱と球）をそのまま表示します。色はコース部品ごとの色分けで、実物の色ではありません。走りのようす（どこで浮いたか、どこで壁に当たってコースアウトしたか）を確かめる用途です。見た目の確認は UE 版で行ってください。

---

## 5. 出力ファイル

出力先（既定 `sim_out\<制御名>\`）に次のファイルができます。

| ファイル | 内容 |
|---|---|
| `summary.json` | 結果のまとめ（下の表） |
| `log.csv` | 1/960 秒ごとの記録（下の表）。Excel でも開ける |
| `serial.txt` | `io.print()` の出力（先頭は時刻） |
| `plot.png` | `--plot` のとき。上から順に、速度、デューティ、モーター電流と電池電圧、ジャイロ z / y |
| `replay.npz` | 3D 再生用の記録（1/60 秒ごとの位置・向き・速度・デューティ・周回）。毎回作られる（数十 KB） |
| `run.mp4` | `--video` または `headless video` のとき。3D 再生の動画 |

**summary.json の項目**

| 項目 | 内容 |
|---|---|
| `outcome` / `laps` / `total` | 結果（4.3）、各周のタイム [s]、合計 [s]（完走のときだけ） |
| `sim_s` / `wall_s` | シミュレーション時間 / 計算にかかった実時間 [s] |
| `vpeak_mps` / `zmax_m` | 最高速 [m/s] / 車の最高点の高さ [m]（レーンチェンジでの飛び具合の目安） |
| `controller` / `params` | 使った制御ファイルと上書きしたパラメーター |
| `motor` / `battery` / `mass` / `imu_axes` | 条件 |
| `duty_set` | 制御が一度でも `io.duty` を設定したか |
| `last_print` | `io.print()` の最後の数行 |

**log.csv の列**

| 列 | 内容 |
|---|---|
| `t_s` | 時刻 [s] |
| `duty` | そのステップの平均デューティ（0〜1） |
| `v_mps` / `a_mps2` / `force_N` | 前向きの速度 / 加速度 / 駆動力（マイナスはブレーキ） |
| `i_motor_A` / `v_bat_V` | モーター電流 / 電池の端子電圧 |
| `pc0_V` / `pc1_V` / `pc3_V` / `temp_C` | XIAO のピン電圧（電池電圧 ÷3.2、NTC、M−電圧 ÷3.2）と基板温度 |
| `ax_g` `ay_g` `az_g` / `gx_dps` `gy_dps` `gz_dps` | IMU の値（センサーの軸） |
| `grounded` | タイヤが接地しているか（1 / 0） |

---

## 6. 制御を書く

制御の書き方は UE 版と同じです。詳しくは [manual.md](manual.md) の 5章を見てください。ここでは要点だけまとめます。

```python
class Controller:
    CONTROL_HZ = 500                          # step() を呼ぶ周期 [Hz]
    def __init__(self, DUTY=0.6):             # 調整値は引数にしておく（--param で上書きできる）
        self.DUTY = DUTY
    def setup(self, io):                      # 電源投入時に1回
        io.duty = 0.0                         # 0 = ショートブレーキ
    def step(self, io):                       # 1/CONTROL_HZ 秒ごと
        io.duty = self.DUTY if io.t > 1.0 else 0.0
        io.print("%.2f" % io.gyro[2])         # serial.txt に出る
```

- 入力は、`io.t`（時刻）、`io.dt`、`io.acc` / `io.gyro`（IMU、センサーの軸）、`io.vbat()` / `io.v_mid()` / `io.temp_c()` です。出力は `io.duty` です。
- 時間には `io.t` を使います。`time.time()` や `time.sleep()` は使わないでください。
- **コースの向き：** ヘッドレス版のコースは**時計回り**（`Mini4WD_CourseOut` と同じ）です。UE の `Mini4WD_Oval` は反時計回りなので、向きを決め打ちしない書き方にしておくと両方で動きます。
- 試し書きは `controllers\local\` に置くと Git に入りません。

---

## 7. パラメーター探索（`headless sweep`）

```
headless sweep lc_brake --param BRAKE_S=0.08,0.12,0.16 --param LEAD_DEG=20,30,40 --seeds 0,1,2 --motor machdash_pro --laps 5
```

- `--param 名前=値1,値2,...`：試す値。複数書くと、すべての組み合わせを試します。
- `--seeds 0,1,2`：IMU ノイズの乱数を変えて、同じ組み合わせを何回か走らせます。ノイズで結果が変わらないかを確かめるためです。
- そのほかのオプション（`--motor` `--battery` `--laps` `--mass` など）は、すべての試行に同じものが使われます。
- 結果は `sim_out\sweep_<制御名>_<日時>\` に出ます。
  - `results.csv`：1試行1行。組み合わせ、seed、結果、合計、各周、最高速、出力フォルダ名
  - `run_001\` …：各試行の `summary.json` と `log.csv`

**実行例**（上のコマンドの結果。27回、約24秒）

| BRAKE_S \ LEAD_DEG | 20 | 30 | 40 |
|---|---|---|---|
| 0.08 | 0/3 | 0/3 | 0/3 |
| 0.12 | 0/3 | **3/3（6.427 s）** | 3/3（6.459 s） |
| 0.16 | 0/3 | 3/3（6.427 s） | 3/3（6.459 s） |

完走数 / seed 数と、完走したときの平均合計タイム（5周）です。最後に「全 seed で完走した中で最速」の組み合わせが表示されます。

- 探索で見つかった値は、ヘッドレスの物理でぎりぎり成立している可能性があります。**UE で確かめてから採用してください**（9章）。
- 組み合わせが多いと時間がかかります。1回あたり 1〜1.5 秒が目安です。

---

## 8. カスタマイズ時の編集ポイント

行番号は版によってずれるので、**ファイル名と変数・関数の名前**で探してください。

| 変えたいこと | ファイル | 場所 |
|---|---|---|
| 制御の考え方・調整値 | `MiniAI_PySim/controllers/<名前>.py` | `step()`、`__init__` の引数（`--param` でも上書きできる） |
| モーター・電池の追加、ドライバーの式、IMU のノイズ | `MiniAI_PySim/sil/sil_core.py` | `MOTORS` `BATTERIES` `Keimyo001` `VirtualLSM6DS3`（UE 版と共通。UE 版マニュアル 7.3） |
| コースの並び | `mini4wd_course/mjsim/m4wd_mj.py` | `LAYOUT`（部品の記号は `S` `R` `L` `LC_UP` `LC_DOWN`）。スタートの直線は `START_PIECE`、レーンの位置は `LANE_OFFSET` |
| 車体（重心、ローラーの位置と大きさ、壁との摩擦、接触の硬さ） | `mini4wd_course/mjsim/m4wd_mj.py` | `DEFAULT_DESIGN`（`com_dz` `roller_*` `mu_wall` `solref_t` など） |
| タイヤの摩擦の上限 | `mini4wd_course/mjsim/m4wd_mj.py` | `DEFAULT_DESIGN` の `mu_tire`（横グリップ）。駆動の上限は `sil_core.PyCar` の `mu` |
| 物理の時間刻み | `mini4wd_course/mjsim/m4wd_mj.py` | `DT`（既定 1/960 s）。横グリップの式は UE の刻み `DT_UE` を使っている |
| タイヤの横グリップの式 | `MiniAI_PySim/sil/run_headless.py` | `main()` の `for ax in (0.04, -0.04):`。**UE（`ue/m4wd_drive.py` の `_step`）にも同じ式がある**ので、片方だけ変えないこと |
| 周回・コースアウト・停止の判定 | `MiniAI_PySim/sil/run_headless.py` | `main()` のループの終わり（`offtrack` `outside` `slow_t` `outcome = ...`）。条件は 4.3 |
| グラフの内容 | `MiniAI_PySim/sil/run_headless.py` | `plot()` |
| 3D 再生の視点・表示の文字 | `MiniAI_PySim/sil/replay_headless.py` | `set_follow()`（距離 `distance`・見下ろす角度 `elevation`）、`set_top()`、`caption()` |
| 3D 再生の記録の細かさ | `MiniAI_PySim/sil/replay_headless.py` | `REC_HZ`（既定 60） |
| コースや車の色・光 | `mini4wd_course/mjsim/m4wd_mj.py` | `build_model()` の `rgba`、`_COLORS`、`<light>`・`<headlight>` |
| 環境チェックの試走の基準 | `MiniAI_PySim/tools/check_headless.py` | `REF_LAP1`（モデルを変えたら更新する） |
| 自動テストの基準 | `MiniAI_PySim/tests/test_headless.py` | 各テストの範囲（モデルを変えたら更新する） |
| MuJoCo の版 | `MiniAI_PySim/requirements-headless.txt` | `mujoco==4.0.1`（変えたら 9.3 の結果を確かめ直す） |

**コースを変えるとき**
UE と結果を比べたいなら、UE のレベル（`ue/build_course.py` の `LAYOUT` → `setup_in_ue.py`）も同じ並びにしてください。ヘッドレス版の `LAYOUT` は `Mini4WD_CourseOut` のレベルに合わせてあります。

**新しい部品を作るとき**
`mini4wd_course/generator/build_parts.py` で FBX を作り、`fbx/parts_manifest.json` に登録します。ヘッドレス版は、FBX の中の当たり判定（`UCX_` で始まる形状）だけを使います。

---

## 9. 精度と限界

### 9.1 UE との比較（lc_brake、ニッケル水素、車重 135 g、3周）

| 条件 | ヘッドレス | UE 5.8.4（実測） |
|---|---|---|
| 標準モーター | 完走 5.80 s（2.22 / 1.64 / 1.95） | 完走 6.08 s（2.34 / 1.68 / 2.06） |
| マッハダッシュPRO、`RUN_DUTY` 0.7 | 完走 5.00 s（1.94 / 1.39 / 1.67） | 完走 5.09 s（2.03 / 1.38 / 1.68） |
| マッハダッシュPRO、既定値 | 完走（5周とも） | 3周目でコースアウト |
| マッハダッシュPRO、ブレーキなし | 1周目でコースアウト | 1周目でコースアウト（実機の動画と同じ） |

- ラップは、ヘッドレスのほうが 2〜5％ほど速く出ます。
- UE のほうが、レーンチェンジの坂で車が浮きやすい傾向があります。そのため、「ヘッドレスでぎりぎり完走」は、UE ではコースアウトすることがあります。
- 傾向（どの設定が速いか、ブレーキがないと飛ぶか）は一致します。探索の結果は、最後に UE で確かめてください。

### 9.2 MuJoCo の版による違い

同じ条件でも、MuJoCo の版が違うと、ぎりぎりの走行の結果が変わることがあります。

- 標準モーター：MuJoCo 3.17.0 と 4.0.1 で、ラップの差は 0.01 秒以下でした。
- マッハダッシュPRO（既定値、5周）：4.0.1 では完走しましたが、3.17.0 では5周目にコースアウトしました。

再現性のため、`requirements-headless.txt` で MuJoCo を **4.0.1 に固定**しています。チームで結果を比べるときは、全員が同じ版を使ってください。`headless check` は、4.0.1 以外のときに「注意」を出します。

### 9.3 基準の結果（MuJoCo 4.0.1、lc_brake 既定値、ニッケル水素、5周）

| 条件 | 結果 | ラップ [s] | 最高速 |
|---|---|---|---|
| 標準 | 完走 9.38 s | 2.22 / 1.64 / 1.95 / 1.63 / 1.95 | 3.3 m/s |
| 標準、リチウムイオン | 完走 6.99 s | 1.74 / 1.19 / 1.44 / 1.19 / 1.43 | 4.6 m/s |
| マッハダッシュPRO | 完走 6.43 s | 1.63 / 1.04 / 1.32 / 1.03 / 1.41 | 5.2 m/s |
| マッハダッシュPRO、`RUN_DUTY` 0.7 | 完走 8.03 s | 1.94 / 1.39 / 1.67 / 1.38 / 1.64 | 3.9 m/s |
| マッハダッシュPRO、リチウムイオン | 1周目でコースアウト | — | 4.9 m/s |
| マッハダッシュPRO、ブレーキなし | 1周目でコースアウト | — | 4.0 m/s |

1周目は、2秒のブレーキ待ちのあとの、静止からの加速を含みます。

### 9.4 割り切っている点

- モデルの前提（デューティの平均化、ADC は平均電圧、電池の消耗なし、推定値のモーター抵抗など）は UE 版と同じです（UE 版マニュアル 9章）。
- 車の形は、箱と球の組み合わせで近似しています。ボディの形や空気抵抗は扱いません。
- ローラーとタイヤは摩擦なしの球で、壁との抵抗は `mu_wall` でまとめて表しています。
- 駆動系の回転（空中で車輪が空転する効果）は、既定では扱いません（`DEFAULT_DESIGN` の `air_react`）。

---

## 10. トラブルシューティング

| 症状 | 対処 |
|---|---|
| `setup_headless.bat` で「Python 3.11 以上が必要」 | Python 3.12 を入れる。複数入っている場合は `py -3.12 tools\setup_headless.py --recreate` |
| `pip install` で失敗する | ネットワーク・プロキシを確認する。会社のプロキシ環境では `set HTTPS_PROXY=http://<プロキシ>:<ポート>` のあとに実行する |
| MuJoCo が入らない（`No matching distribution`） | Python の版が 3.11〜3.13 か確認し、Python 3.12 で `--recreate` |
| `headless` と打つと「認識されていません」 | `3dsim\MiniAI_PySim` に移動してから実行する（`headless.bat` はそのフォルダにある） |
| 「.venv not found. Run setup_headless.bat first.」 | `setup_headless.bat` を実行する |
| 「コース部品の FBX が Git LFS の引換券のまま」 | リポジトリで `git lfs pull`。ZIP で入手した場合は clone し直す |
| 「制御ファイルが見つかりません」 | 名前のつづりを確認する。`controllers\` と `controllers\local\` を探している。パスで渡してもよい |
| 結果が「制御のエラー」 | 表示されたファイル名と行番号を直す |
| 結果が「一度も走り出さなかった」 | `io.duty` が 0 のまま（＝ブレーキ）。`serial.txt` や `log.csv` の `duty` 列を見る |
| 同じ条件なのに、人によって結果が違う | `headless check` の「注意」で MuJoCo の版を確認する（9.2）。`--seed` も揃える |
| 文字化けする | `headless.bat` 経由で実行する（`PYTHONUTF8=1` を設定している） |
| UE とタイムが違う | 物理エンジンの違いによるもの（9.1）。最終確認は UE で |
| 「走行の記録（replay.npz）が見つかりません」 | 先にその制御を走らせる。v1.1.0 より前の結果には記録がないので、走らせ直す |
| 「動画の書き出しには imageio と imageio-ffmpeg が必要です」 | `setup_headless.bat` をもう一度実行する |
| 「3D の描画を始められませんでした」 | グラフィックドライバーを更新する。リモートデスクトップ接続中は、直接ログインして実行するか、動画（`--video`）を使う。Linux で画面がない場合は `MUJOCO_GL=egl` を設定する |
| ビューアの窓が出ない・すぐ閉じる | 同上。`headless video` で動画にすれば、窓なしで確認できる |

---

## 11. 自動テストと CI

- **手元で実行する：** `.venv` の Python で実行します。MuJoCo が入っていない環境では、ヘッドレスのテストは自動でスキップされます。
  ```
  cd /d <リポジトリ>\3dsim
  MiniAI_PySim\.venv\Scripts\python -m pytest MiniAI_PySim\tests
  ```
- **テストの中身（`tests/test_headless.py`）：**
  - 標準モーターで完走し、ラップが基準の範囲に入る
  - マッハダッシュでブレーキなしだと1周目にコースアウトする
  - `--param` が制御に届く
  - 制御のエラーが報告される
  - 探索ツールが動く
  - 3D 再生用の記録（`replay.npz`）が最後まで 60 コマ/秒で残る
  - 動画を書き出せる（描画できない環境ではスキップ）
- **GitHub Actions（`.github/workflows/pysim.yml`）：** Pull Request のたびに、`requirements-headless.txt` を入れてテストを実行します。LFS は、コース部品の FBX（約 0.7 MB）だけを取得します。

---

## 12. 検証の記録

2026年11月27日に、Linux 上で次を確認しました。

- **環境：** Python 3.13、mujoco 4.0.1、numpy 2.5（あわせて mujoco 3.17.0 でも確認）
- **セットアップ：** `setup_headless.py` で `.venv` を作り、パッケージを入れ、環境チェックですべて OK になること
- **結果：** 9.3 の基準の結果
- **探索：** 7章の実行例（27回）
- **テスト：** 21件すべて合格
- **3D 再生：** 動画（`follow` / `top`、完走とコースアウト）を書き出して中身を確認。ビューアは仮想ディスプレイ上で窓が開き、再生と表示の文字を確認

Windows の MuJoCo 4.0.1 は、Python 3.11 / 3.12 / 3.13 用の配布物があることを確認しています。ただし、`setup_headless.bat` と `headless.bat`、ビューアの窓を Windows で実際に動かしての確認は、まだしていません。最初に使う人は、チェックシートと同じ要領で結果を記録してください。
