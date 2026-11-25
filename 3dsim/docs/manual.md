# ミニ四駆 Python制御シミュレーター 使い方マニュアル
（市販ミニ四駆 ＋ XIAO MG24 Sense ＋ KEIMYO-001 モータードライバ）

このシミュレーターでは、**Python のクラスで書いた制御**で、ミニ四駆を走らせます。対象は、市販ミニ四駆（マッドレイザー／MAシャーシ）に XIAO MG24 Sense と KEIMYO-001 モータードライバを載せた車です。

- 制御が受け取るのは、マイコンが実機で受け取るのと同じ信号です。
  - IMU（LSM6DS3）の加速度・角速度
  - 電池電圧・M− 電圧・基板温度
- 制御が決めるのは、モーターの PWM デューティ（PC2）です。
- KEIMYO-001・電池・モーターのモデルがデューティを駆動力に変え、車がコースを走ります。
- 制御ファイルを保存して `R` キーを押すだけで、新しい制御で走り直せます。
- 実行する場所は **UE 5.8** の UE プロジェクト `MiniYonkuSim`（リポジトリ `mini4wdai_jmaab` に含まれる）です。3D のコース上で走りを見られます。

---

## 0. 用意するもの

| もの | 内容 |
|---|---|
| Windows 10 / 11 の PC | UE 5.8 が動くもの（独立 GPU を推奨） |
| Unreal Engine 5.8 | Epic Games Launcher からインストールする |
| Git ／ Git LFS | https://git-scm.com/ から Git for Windows をインストールする（Git LFS も同梱） |
| リポジトリ `ssksoft/mini4wdai_jmaab` へのアクセス権 | 非公開リポジトリの場合は、管理者に招待してもらう |

- このマニュアルでは、リポジトリを置いたフォルダを `<リポジトリ>` と書きます（例：`D:\work\mini4wdai_jmaab`）。
- **パスに日本語や空白を含まないフォルダ**に置くことを推奨します。
- 1ページ版の手順は、リポジトリ直下の `README.md`（クイックスタート）にあります。再現できるかを確かめる場合は `docs/verification_checklist.md` に沿って進めてください。

---

## 1. 仕組み

```
 UE（Chaos 物理）       ── 車の姿勢・速度 ──▶ 仮想 LSM6DS3（416 Hz）
        ▲                                               │ io.acc [g], io.gyro [dps]
        │ 前後方向の力                                   ▼
 KEIMYO-001 ＋ 電池 ＋ モーターのモデル ◀── io.duty ── あなたの Controller.step(io)
        │                                               ▲
        └── 電池電圧・M−電圧・基板温度 ── io.vbat() など ──┘
```

- UE のランタイムは、1フレームごとに次の処理を行います。
  1. IMU の値を更新する
  2. その時間の中で `CONTROL_HZ` の周期が来た回数だけ `step(io)` を呼ぶ
  3. `io.duty` の **時間平均** から駆動力を計算する
- `io.t` は**シミュレーションの時刻**です。UE をスロー再生にしても、PC が重くても、制御から見た時間は正しく進みます。

---

## 2. 入手とフォルダ構成

コマンドプロンプト（または Git Bash）で、パスに日本語や空白を含まないフォルダに clone します。

```bat
cd /d D:\work
git lfs install
git clone https://github.com/ssksoft/mini4wdai_jmaab.git
cd mini4wdai_jmaab
git lfs pull
```

- `git lfs install` は PC ごとに最初の1回だけ必要です。
- **ZIP ダウンロード（Code → Download ZIP）は使わないでください。** LFS のファイル（UE のアセットなど）が中身のない数百バイトのファイルになり、UE で「マップのロードに失敗しました … はアセットファイルのようです」と出て開けません。
- UE のアセット（`.uasset` / `.umap`）やコース部品の FBX は **Git LFS** で管理しています。`git lfs pull` が終わっていないと、UE でレベルが開けません。
- 最新版にするときは、`git pull` を実行します（UE を閉じてから実行してください）。

```
mini4wdai_jmaab/
├─ README.md                     … クイックスタート（1ページ版の手順。PDF は GitHub の Releases に添付）
└─ 3dsim/
    ├─ README.md                     … 3dsim の概要・フォルダ構成・Git の使い方
    ├─ CHANGELOG.md                  … 版ごとの変更点
    ├─ docs/
    │   ├─ manual.md                 … このファイル
    │   ├─ verification_checklist.md … 再現できるかを確かめるときのシート
    │   ├─ keyboard_drive_guide.md   … キーボード走行・コース作成の操作ガイド
    │   └─ reports/                  … ラップタイム最適化のレポート
    ├─ MiniYonkuSim/                 … UE プロジェクト（MiniYonkuSim.uproject）
    ├─ mini4wd_course/
    │   ├─ ue/m4wd_drive.py          … UE ランタイム（start_py / stop_py。キーボード走行などもここ）
    │   └─ fbx/ car/ generator/ …    … コース部品・車体データとその生成スクリプト
    └─ MiniAI_PySim/
        ├─ controllers/              … Python の制御
        │   ├─ lc_brake.py           …   サンプル：レーンチェンジ手前のブレーキ制御
        │   ├─ template.py           …   新しく書くときのひな形
        │   └─ local/                …   自分の試し書き用（Git には入らない。5.4）
        ├─ sil/sil_core.py           … ドライバー・電池・モーター・IMU のモデル、PyCar クラス
        ├─ tools/check_env.py        … 環境チェック（4.1）
        ├─ tools/build_docs.py       … リポジトリ直下の README.md から PDF を作る
        └─ tests/test_sil_core.py    … モデルの自動テスト（GitHub Actions で実行）
```

---

## 3. 必要な環境

| ツール | 用途 |
|---|---|
| Unreal Engine 5.8 ＋ UE プロジェクト `MiniYonkuSim`（リポジトリに含まれる） | 走らせる環境 |

**追加でインストールするものはありません。** 制御は UE に内蔵された Python で動きます。

### 3.1 Unreal Engine 側の確認

- `<リポジトリ>\3dsim\MiniYonkuSim\MiniYonkuSim.uproject` を**エクスプローラーでダブルクリック**して開きます。
  - **Epic Games Launcher の「マイプロジェクト」からは開かないでください。** 一覧のサムネイルが、別の場所にある同名の `MiniYonkuSim` を指していることがあります（実際に検証したPCでもそうでした）。どちらを開いたかは、4.1 の環境チェックで確認できます。
  - 初回はシェーダーのコンパイルに時間がかかります（数分〜数十分）。
  - 「別のバージョンのエンジンで作成されています」と出た場合は、5.8 を選んでください。
- **Python Editor Script Plugin** が有効になっていることを確認してください（「編集 → プラグイン」で「Python」を検索）。無効なら有効にして、エディタを再起動します。
- プロジェクトを開くと、`MiniYonkuSim/Content/Python/init_unreal.py` が `mini4wd_course/ue/m4wd_drive.py` を自動で登録します。そのため、`MiniYonkuSim` と `mini4wd_course` は**同じ `3dsim` フォルダに並んでいる必要があります**。
- 制御ファイルの中で、UE 内蔵 Python に入っていないライブラリ（numpy など）を使う場合は、UE の Python にインストールしてください。

  ```bat
  "C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\ThirdParty\Python3\Win64\python.exe" -m pip install numpy
  ```

  **サンプルは標準ライブラリだけで書いてあります。**

---

## 4. クイックスタート

### 4.1 環境チェック（最初に1回）

レベルを開いた状態（**Play していない状態**）で、画面下の入力欄に次を入力して Enter を押します。入力欄の左のドロップダウンは **「Cmd」** のままにします（「Python」を選んだ場合は、先頭の `py ` を外して入力します）。

```
py import m4wd_drive as D; D.check_env()
```

- **結果は画面には出ません。** 入力欄の左にある **「出力ログ」** ボタンを押してログを開くと、`[M4WD check]` で始まる行が並んでいます。Enter を押しても何も起きないように見えるのは、このためです。
- `D.check_env()` は、**今開いているプロジェクトと同じリポジトリ**の `check_env.py` を実行します。パスを打つ必要はありません。
- 起動時のエラーなどで `m4wd_drive` が読み込まれていない場合は、`import m4wd_drive` でエラーになります。そのときは、パスを指定して直接実行します（`<リポジトリ>` は、開いているプロジェクトがあるリポジトリ）。

  ```
  py "<リポジトリ>/3dsim/MiniAI_PySim/tools/check_env.py"
  ```

- 次の項目を確認し、`[OK]` / `[NG]` で表示します。
  - ファイルがそろっているか
  - モデルが正しく計算できるか（自己テスト）
  - ランタイムが登録されているか
  - レベルに車・コース・スタート位置があるか
- 最後に **「結果: すべて OK（NG 0 件）」** と出れば準備完了です。
- 同じ内容が `MiniYonkuSim/Saved/PySim/check_env_report.txt` に保存されます。問い合わせのときは、このファイルを添えてください。
- NG があった場合は 4.2 を見てください。

### 4.2 NG が出たとき

| NG の項目 | 直し方 |
|---|---|
| UE を開いたときに「マップのロードに失敗しました … はアセットファイルのようです」／LFS のファイルが中身のない状態 | LFS の本体がありません。ZIP で入手した場合は `git clone` し直す。clone した場合は `git lfs pull` を実行する |
| ファイル … が見つからない | `git clone` と `git lfs pull` が最後まで終わったか確認する。`git status` で、ファイルが消えていないかを見る |
| 読み込まれた m4wd_drive.py がこのパッケージのものでない／MiniYonkuSim と MiniAI_PySim が同じ 3dsim フォルダにない | 開いているプロジェクトと、実行した `check_env.py` が別のコピーにある。**clone が複数あるとき**に起きる。`py import m4wd_drive as D; D.check_env()` で実行し直す。それでも NG なら、開くプロジェクトを間違えている（Epic ランチャーの一覧から開いた場合など）ので、使いたいリポジトリの `3dsim\MiniYonkuSim\MiniYonkuSim.uproject` をエクスプローラーで開き直す |
| m4wd_drive が読み込まれていない／Content/Python/init_unreal.py がない | `git checkout -- 3dsim/MiniYonkuSim/Content/Python/init_unreal.py` で元に戻し、エディタを再起動する |
| m4wd_drive に start_py がない | 古い `m4wd_drive.py` が読み込まれている。UE を閉じて `git pull` で最新にし、エディタを再起動する |
| ランタイムが登録されていない | 出力ログで `py import m4wd_drive; m4wd_drive.register()` を実行する |
| レベルにコース部品がない | 出力ログで `py "<リポジトリ>/3dsim/mini4wd_course/ue/setup_in_ue.py"` を実行する（コースのレベル `Mini4WD_Oval` を作ります。数分かかります） |
| レベルに車／カメラ／スタート位置（外側）／ゴールラインがない | 出力ログで `py "<リポジトリ>/3dsim/mini4wd_course/ue/car_setup.py"` を実行する |
| モデル自己テストが NG | `MiniAI_PySim/sil/sil_core.py` が書き換わっている。`git diff` で変更を確かめ、不要なら `git checkout -- 3dsim/MiniAI_PySim/sil/sil_core.py` で元に戻す |

直したら、もう一度 4.1 を実行して、すべて OK になることを確かめてください。

> `Mini4WD_CourseOut` レベルでは「参考: 内側スタート位置（M4WD_START_1）はこのレベルにありません」と表示されます。NG ではありません（`lane=1` が使えないだけです）。

### 4.3 UE で走らせる

1. `MiniYonkuSim.uproject` を開きます。コースのレベルは `Mini4WD_Oval` を使います。`Mini4WD_CourseOut` があれば、それでも構いません。開いていない場合は、コンテンツブラウザの `Mini4WD/Maps/Mini4WD_Oval` をダブルクリックします。
2. 画面下の **出力ログ** で、入力欄の左のドロップダウンを **「Python」** にして、次を入力します。

   ```python
   import m4wd_drive as D; D.start_py("lc_brake", motor="machdash_pro")
   ```

   「Cmd」のままの場合は、先頭に `py ` を付けて入力します。

   ```
   py import m4wd_drive as D; D.start_py("lc_brake", motor="machdash_pro")
   ```

   出力ログに `[M4WD] Python control armed: ... -> press Play` と出れば準備完了です。
3. **Play（▶）** を押し、**ゲーム画面を1回クリック**します（キー操作をゲームに渡すため）。
   - 車がスタート位置に戻り、`setup()` が呼ばれます。
   - サンプルは、2秒間ブレーキをかけたままジャイロのオフセットを測ってから走り出します。
4. 画面左上の HUD に、次の情報が表示されます。
   - 「Python制御」、PC2 のデューティ、電流、電池電圧
   - `io.state` の中身（例：`mode=RUN heading=-512`）
   - IMU の値
   - `print>` 行（`io.print` の出力）

| キー | 動作 |
|---|---|
| `R` | **制御ファイルを読み直して**再スタートする（電源の入れ直しに相当） |
| `1` / `2` | 外側／内側レーンから再スタートする |
| `C` / `T` | カメラ切替／スロー再生と等速の切り替え |

- **制御を直す流れ：** エディタで `controllers/lc_brake.py` を保存し、ゲーム画面をクリックしてから `R` を押します。Play を止める必要はありません。
- パラメーターを変えて試すときは、Play を止めてから `start_py` を引数付きで実行し直します。

  ```python
  D.start_py("lc_brake", params={"BRAKE_S": 0.16, "LEAD_DEG": 40}, motor="machdash_pro", laps=5)
  ```

- 結果は `MiniYonkuSim/Saved/PySim/<名前>_<日時>_<回数>/` に保存されます（`log.csv` `serial.txt` `summary.json`）。
- Python 制御をやめるには `D.stop_py()` を実行します。そのあとの Play は元の動作に戻ります（`Mini4WD_Oval` はキーボード走行、`Mini4WD_CourseOut` は実機動画の再現シナリオ）。

---

## 5. コントローラーの書き方

### 5.1 最小の例（`controllers/template.py` と同じ形）

```python
class Controller:
    CONTROL_HZ = 500                     # step() を呼ぶ周期 [Hz]（シミュレーション時間）

    def __init__(self, DUTY=0.6, START_DELAY=1.0):   # 調整したい値は引数にしておく
        self.DUTY = DUTY
        self.START_DELAY = START_DELAY

    def setup(self, io):                 # 電源投入時（t = 0）に1回
        io.duty = 0.0                    # ショートブレーキ = 車を止めておく

    def step(self, io):                  # 1/CONTROL_HZ 秒ごと
        io.duty = 0.0 if io.t < self.START_DELAY else self.DUTY
        io.state["yaw_rate"] = "%.0f" % io.gyro[2]   # HUD に表示される

    def close(self):                     # 終了時（省略可）。自分のファイルに書き出すときなどに使う
        pass
```

### 5.2 `io` でできること

| 名前 | 向き | 内容 |
|---|---|---|
| `io.t` | 入力 | シミュレーション時刻 [s]。電源投入が 0 |
| `io.dt` | 入力 | 前回の `step()` からの経過時間 [s]（= 1/CONTROL_HZ） |
| `io.acc` | 入力 | 加速度 (x, y, z) [g]。**センサーの軸**で表した値。静止して上を向いた軸に +1 g が出る |
| `io.gyro` | 入力 | 角速度 (x, y, z) [dps]。センサーの軸、右手系 |
| `io.vbat()` | 入力 | 電池電圧 [V]。PC0 の値 × 3.2 |
| `io.v_mid()` | 入力 | M− 端子の電圧 [V]。PC3 の値 × 3.2（平均 ＝ (1−デューティ) × 電池電圧） |
| `io.temp_c()` | 入力 | 基板温度 [℃]。PC1 の NTC 電圧から計算した値 |
| `io.adc_volts("PC0")` | 入力 | ピンの生の電圧 [V]（`"PC0"` `"PC1"` `"PC3"`） |
| `io.duty` | **出力** | 0〜1。1 で全力駆動、0 でショートブレーキ。中間はオフの間がブレーキになる。**惰性走行・後退はない**。設定するまではブレーキとして扱う |
| `io.print(...)` | 出力 | Serial の代わり。HUD の `print>` 行と `serial.txt` に出る |
| `io.state[...]` | 出力 | 辞書。中身が HUD に `key=value` の形で出る |

- **IMU の向き：** 既定の取り付けでは、x = 前、y = 左、z = 上 です。このとき、機首上げはジャイロ y がマイナス、左に曲がるとジャイロ z がプラスになります。向きは `imu_axes` で変えられます。書き方は「センサーの x, y, z が、車体のどの軸を向いているか」です。

  | 取り付け例 | imu_axes |
  |---|---|
  | 基板の X が前、部品面が上（既定） | `"x,y,z"` |
  | 基板の X が左、Y が後ろ | `"y,-x,z"` |
  | 基板の X が後ろ、Y が右 | `"-x,-y,z"` |
  | 基板を裏返し（部品面が下）、X が前 | `"x,-y,-z"` |
- **コースの回る向き：** UE の `Mini4WD_Oval` は反時計回り（L カーブ）、`Mini4WD_CourseOut` は時計回り（R カーブ）です。方向を決め打ちしない書き方にしておくと、どちらでも動きます。サンプルは、最初に見つけた坂の方位の符号から向きを判断しています。

### 5.3 書くときの決まり

- 時間は必ず `io.t` / `io.dt` を使ってください。`time.time()` や `time.sleep()` は**使わない**でください（シミュレーションの時間とずれます）。
- `step()` の中で長く待たないでください。待つ処理は「状態」と「時刻」で書きます（サンプルの `mode` / `t_mode` を参照）。
- 制御の中で例外が起きると、その回の走行は止まります。HUD とログに「コントローラーエラー: step() で例外: ZeroDivisionError … (my_ctrl.py 行 42)」のように表示されます。直して `R` を押してください。
- 結果を自分のファイルに書き出したいときは、`close()` で書くのが確実です。`io.print` は `serial.txt` にも残ります。

---

### 5.4 自分の制御の置き場所と Git

- **試し書き**は `MiniAI_PySim/controllers/local/` に置いてください。このフォルダの中身は Git に入りません（`.gitignore` で除外）。`start_py("my_ctrl")` のように名前だけで呼べます。
- **みんなで使う制御**にしたいときは、`controllers/` に置いてコミットします。ファイルの先頭に、何をする制御かをコメントで書いてください。
- 走行結果（`MiniYonkuSim/Saved/`）も Git には入りません。残したい結果は、`docs/reports/` にまとめてからコミットします。

---

## 6. パラメーター一覧

### `D.start_py(controller, ...)`（Play の前に実行）

| 引数 | 既定値 | 内容 |
|---|---|---|
| `controller` | — | `.py` のパス、または `controllers` / `controllers/local` 内の名前（`"lc_brake"`） |
| `params` | `None` | `Controller(**params)` に渡す辞書。例：`{"BRAKE_S": 0.15}` |
| `battery` | `"nimh2"` | `nimh2`（ニッケル水素2本）／`nimh2_low`（使い込み）／`liion1`（リチウムイオン1セル） |
| `motor` | `"standard"` | `standard`（キット付属）／`machdash_pro` |
| `mass` | `0.135` | 車重 [kg] |
| `slow` | `0.25` | 走行中のタイムダイレーション（1/4 速）。等速は `1.0`。等速だと IMU の更新が粗くなる |
| `imu_axes` | `"x,y,z"` | IMU の取り付け向き |
| `noise` / `seed` | `True` / `0` | IMU のノイズ・残留オフセットと、その乱数の種 |
| `laps` | `0` | この周回数で止めて結果を表示する（0 = 止めない） |
| `boot` | `0.0` | 車を離す前に静止させておく時間 [s] |
| `lane` | `0` | スタートレーン（0 = 外側、1 = 内側） |

- 結果（`summary.json`）の `outcome` は次のどれかです。
  - `ok`：完走（`laps` で指定した周回数）
  - `courseout`：コースアウト
  - `error`：制御のエラー
  - `stopped`：Play を止めた
  - `restart`：`R` で再スタートした

---

## 7. カスタマイズ時の編集ポイント

「どこを変えたいか」から、編集するファイルと場所を引けるようにまとめました。
行番号は版によってずれるので、**ファイル名と変数・関数の名前**で探してください。

### 7.1 早見表

| 変えたいこと | ファイル | 場所 |
|---|---|---|
| 制御のしきい値・時間 | `controllers/<名前>.py` | `__init__` の引数（または `start_py` の `params=`） |
| 制御の考え方そのもの | `controllers/<名前>.py` | `step()` |
| 制御の周期 | `controllers/<名前>.py` | `CONTROL_HZ` |
| HUD に出す値 | `controllers/<名前>.py` | `io.state[...]` / `io.print(...)` |
| モーターを追加・調整 | `sil/sil_core.py` | `MOTORS` |
| 電池を追加・調整 | `sil/sil_core.py` | `BATTERIES` |
| 配線・MOSFET の抵抗、基板の熱 | `sil/sil_core.py` | `Keimyo001.__init__`（`r_drv`, `rth`, `tau`）, `Keimyo001.step` |
| 分圧比・NTC の定数 | `sil/sil_core.py` | `VBAT_DIV` `MID_DIV` `ADC_REF` `NTC_R0` `NTC_B` `NTC_PULLUP` |
| IMU のノイズ・オフセット・レンジ・ODR | `sil/sil_core.py` | `VirtualLSM6DS3`（`gbias` `abias` `gauss(0, 0.12)` `±16` `±2000` `odr`） |
| タイヤの摩擦・転がり抵抗 | `sil/sil_core.py` | `PyCar.__init__` の `mu` `roll_decel`（`CarModel.step` で使用） |
| 新しい入力を制御に渡す | `sil/sil_core.py` | `ControllerIO` ＋ `PyCar._advance`（7.4） |
| UE のタイヤ横グリップ・車軸 | `mini4wd_course/ue/m4wd_drive.py` | `MU_TIRE` `AXLE_HALF` `AXLE_H`、`_step` 内の `* 0.8`（横グリップのかかり具合） |
| UE でのコースアウト・完走の判定 | `mini4wd_course/ue/m4wd_drive.py` | `_pyc_update`（`OFFTRACK_Z`、コース範囲 ＋8 cm） |
| UE の HUD の表示 | `mini4wd_course/ue/m4wd_drive.py` | `_pyc_hud` |
| `start_py` の既定値 | `mini4wd_course/ue/m4wd_drive.py` | `def start_py(...)` の引数 |
| MiniAI_PySim の場所 | `mini4wd_course/ue/m4wd_drive.py` | `PY_ROOT`（または環境変数 `M4WD_PYSIM_ROOT`） |
| UE のコース形状 | `mini4wd_course/ue/build_course.py` | `LAYOUT`（変えたら `setup_in_ue.py` でレベルを作り直す） |
| 車体の寸法・ローラー位置（UE） | `mini4wd_course/car/car_spec.json` | 変えたら `ue/car_setup.py` を実行する |

### 7.2 制御ファイル（`controllers/*.py`）

`lc_brake.py` の構成と、変える場所です。

```python
class Controller:
    CONTROL_HZ = 1000                       # ← (1) 制御周期
    def __init__(self, START_WAIT=2.0, RUN_DUTY=1.0, BRAKE_DUTY=0.0, BRAKE_S=0.12, CLIMB_DPS=120.0,
                 REARM_S=0.7, LEAD_DEG=30.0, EARLY_S=0.12, WINDOW_DEG=120.0, PRINT_S=0.1):   # ← (2) 調整値
    def setup(self, io): ...                # ← (3) 初期状態（電源投入時のデューティ、変数の初期化）
    def step(self, io):
        ...  if self.mode == "WAIT": ...    # ← (4) 状態ごとの処理（状態遷移）
        ...  self.psi += ...                # ← (5) 推定（方位の積分など）
        ...  io.state.update(...)           # ← (6) HUD 表示・ログ
```

| 調整値 | 意味 | 効き方の目安 |
|---|---|---|
| `START_WAIT` | 走り出す前に止まっている時間。この間にジャイロのオフセットを測る | 短すぎるとオフセットの推定が粗くなる |
| `RUN_DUTY` | 普段のデューティ | 下げると最高速が下がる（中間のデューティでもオフの間はブレーキになる） |
| `BRAKE_DUTY` | ブレーキ中のデューティ | 0 が最も強い。0.3 などにすると「弱いブレーキ」になる |
| `BRAKE_S` | 坂の入口を検出してからブレーキをかける時間 | 長いほど坂の頂上で遅くなる（安全だがタイムが落ちる） |
| `CLIMB_DPS` | 坂の入口とみなす機首上げの角速度 | 低すぎるとほかの段差で誤検出、高すぎると見逃す |
| `REARM_S` | ブレーキのあと、同じ坂で再検出しないようにする時間 | |
| `LEAD_DEG` | 2回目以降、坂の何度手前（方位）でブレーキを始めるか | 速いモーター・電池ほど大きくする |
| `EARLY_S` | 手前でかけるブレーキの長さ | |
| `WINDOW_DEG` | 坂の位置を覚えたあと、坂から ±何度の範囲の検出だけを有効にするか | 誤検出を防ぐ。狭すぎると本物の坂を見逃す |

サンプルの既定値は、**標準モーター向け**です。UE で実際に走らせた結果は次のとおりです（Mini4WD_CourseOut、ニッケル水素、3周、`slow=0.25`）。

| モーター | params | 結果 |
|---|---|---|
| 標準 | なし（既定値） | 完走 6.08 s（2.34 / 1.68 / 2.06） |
| マッハダッシュPRO | なし（既定値） | 3周目でコースアウト |
| マッハダッシュPRO | `{"BRAKE_S": 0.16, "LEAD_DEG": 40}` | 1周目でコースアウト |
| マッハダッシュPRO | `{"RUN_DUTY": 0.7}` | **完走 5.09 s（2.03 / 1.38 / 1.68）** |
| マッハダッシュPRO | `{"BRAKE_S": 0, "EARLY_S": 0}`（ブレーキなし） | 1周目のレーンチェンジ後にコースアウト（実機の動画と同じ） |

- マッハダッシュでは、1周目に坂のふもとへ約 3.8 m/s で着き、ブレーキ中でも約 2.9 m/s で車が浮いていました。坂のふもとで検出してからブレーキをかけるのでは間に合わないので、ブレーキの時間を延ばしても効きません。
- 速いモーターでは、`RUN_DUTY` で最高速を抑えるか、坂の手前で減速するしくみ（1周目から使える位置の推定など）を足してください。
- モーター・電池・車重を変えたら、`start_py(..., params={...})` で値を変えながら試してください。

### 7.3 車両・電気のモデル（`sil/sil_core.py`）

**モーターの追加** — `MOTORS` に1行足します。

```python
MOTORS = {
    #  キー            表示名                      VMAX[m/s]  F0[N]（ニッケル水素・デューティ100%）  R_m[Ω]
    "standard":     ("標準モーター（キット付属）", 4.0, 8.0 * 0.1259, 1.30),
    "machdash_pro": ("マッハダッシュモーターPRO", 7.0, 13.0 * 0.1259, 0.50),
    "torque_tune2": ("トルクチューン2", 5.2, 10.0 * 0.1259, 0.9),   # ← 例（数値は仮）
}
```

- `VMAX`：ニッケル水素2本・デューティ100%で、平らな直線を走ったときの最高速です。
- `F0`：停止状態から出せる力（タイヤの位置での値）です。実測の初期加速度 [m/s²] × 校正時の車重 0.1259 kg で求めます。
- `R_m`：モーター巻線の抵抗です。テスターで測るか、「電池電圧 ÷ ロック時の電流」で求めます。
- 追加したモーターは、`start_py(..., motor="torque_tune2")` で選べます。

**電池の追加** — `BATTERIES` に `(表示名, 開放電圧[V], 内部抵抗[Ω])` を足します。内部抵抗には、電池ホルダーと接点の抵抗も含めます。

**ドライバーの式を変える** — `Keimyo001.step` で次のように計算しています。

```
I = (d·Voc − e) / (R_m + d²·R_int)
F = kF·I
Vbat = Voc − R_int·d·I
```

- たとえば「PWM 周波数が低いときの電流の途切れ」や「電流制限」を入れたい場合は、ここで `i` を加工します。
- 基板温度の上がり方は `p = i * i * 0.006`（損失）、`rth`、`tau` で決まります。

**IMU** — `VirtualLSM6DS3` の中で次の値を変えられます。

| 項目 | 場所 | 既定値 |
|---|---|---|
| ジャイロの残留オフセット | `gbias` | σ 0.4 dps（電源投入ごとに乱数で決まる一定値） |
| 加速度の残留オフセット | `abias` | σ 0.01 g |
| ノイズ | `gauss(0, 0.0015)`（加速度）、`gauss(0, 0.12)`（ジャイロ） | |
| レンジ | `±16`（g）、`±2000`（dps） | |
| 出力の更新周期 | `odr`（`PyCar` の `imu_odr` 引数） | 416 Hz |

**タイヤの限界** — `PyCar.__init__` の `mu`（摩擦係数 1.0）と `roll_decel`（惰行時の減速 0.25 /s）です。

### 7.4 新しいセンサーを制御に渡す（例：車速）

実機に車輪の回転センサーを付ける想定で、正しい車速を制御に渡す例です。

1. `sil/sil_core.py` の `ControllerIO.__init__` に、項目を足します。

   ```python
   self.wheel_mps = 0.0          # 追加：車輪の周速 [m/s]
   ```

2. `CarModel.step` の最初で車速を覚えます。

   ```python
   def step(self, dt, R, v_world, v_fwd, grounded):
       self.v_fwd = v_fwd            # 追加
   ```

3. `PyCar._advance` の最初で `io` に入れます。

   ```python
   io.wheel_mps = getattr(self, "v_fwd", 0.0)   # 必要ならノイズや量子化を足す
   ```

4. 制御の中では `io.wheel_mps` として使えます。

同じやり方で、モーター電流（`self.drv.i`）や、外部に付けた距離センサーなども渡せます。

### 7.5 物理・コースを変える

- **タイヤの横グリップ：** `m4wd_drive.py` の `_step` の中にあります（`MU_TIRE`、車軸 `AXLE_HALF` ±4 cm、かかり具合 `* 0.8`）。
- **コース：** `build_course.py` の `LAYOUT` を変えて `setup_in_ue.py` を実行します。部品の記号（`S` `R` `L` `LC_UP` `LC_DOWN`）は、`docs/keyboard_drive_guide.md` と同じです。
- **車体：** 重さは `mass` 引数だけで変えられます（プレイ中に上書きします）。重心やローラーの位置は `car_spec.json` を変えて、`car_setup.py` を実行します。

---

## 8. 実機のマイコンに移すとき

実機の XIAO MG24 Sense は Arduino IDE で書きます。Python の `io` と、実機での書き方の対応は次のとおりです。

| Python（io） | 実機（XIAO MG24 Sense ＋ KEIMYO-001） |
|---|---|
| `io.duty = d` | `analogWrite(PC2, (int)(d * 255))` |
| `io.gyro[0..2]` / `io.acc[0..2]` | `imu.readFloatGyroX/Y/Z()` / `imu.readFloatAccelX/Y/Z()`（`PD5` を HIGH にしてから `imu.begin()`） |
| `io.vbat()` | `analogRead(PC0) * 3.3 / 4095 * 3.2`（`analogReadResolution(12)` の場合） |
| `io.v_mid()` | `analogRead(PC3) * 3.3 / 4095 * 3.2` |
| `io.temp_c()` | `analogRead(PC1)` の電圧 → 10k プルアップ・NTC（10k, B=4100）の式で温度に変換 |
| `io.t`, `io.dt` | `micros()` の差 × 1e-6 |
| `CONTROL_HZ` の周期で `step()` | `loop()` の中で `micros()` を見て、一定の周期で処理する |
| `io.print(...)` | `Serial.print(...)` |

- Python は倍精度（double）、マイコンは単精度（float）で計算します。積分などで差が出ることがあります。
- このシミュレーターでは、制御の処理時間を 0 として扱います。実機で重い計算をすると、制御の周期が遅れることがあります。
- 最初は、タイヤを浮かせた状態でデューティとブレーキの動きを確かめてください。

---

## 9. 精度と限界

**KEIMYO-001・電池・モーターのモデル**

- 平均化した DC モーターの式で計算します（電流は連続と仮定）。
  - `I = (d·Voc − e) / (R_m + d²·R_int)`
  - `e = ke·v`（逆起電力）
  - 駆動力 `F = kF·I`
- `kF` と `ke` は、ニッケル水素2本・デューティ100%のときに、UE で校正済みのモーター特性 `F = F0·(1 − v/VMAX)` と一致するように決めています。どちらのモーターも、実機の動画と合わせた値です。
  - 標準モーター：VMAX 4.0 m/s、A0 8 m/s²
  - マッハダッシュPRO：VMAX 7.0 m/s、A0 13 m/s²
- 次の動きは、回路の式から自然に出てきます。
  - デューティを下げたときの速度
  - ショートブレーキの強さ（速度に比例して弱まる）
  - 電池の電圧降下
  - リチウムイオンにしたときの加速・最高速の上がり方
- 加速度は、タイヤの摩擦の上限（μ = 1.0 × 重力）で頭打ちになります。タイヤが浮いているあいだは、駆動力をかけません。

**推定値（実測で置き換えると精度が上がります。7.3 を参照）**

- モーター巻線の抵抗：標準 1.3 Ω、マッハダッシュPRO 0.5 Ω
- 電池：ニッケル水素2本 2.75 V・内部抵抗 0.06 Ω、リチウムイオン 4.0 V・0.04 Ω
- 車重：135 g

**割り切っている点**

- デューティは、ステップの中での時間平均として扱います（PWM の周波数・リップルは扱いません）。電池電圧などのアナログ値も平均値です。
- UE の車の動きは1フレームごとにしか更新されないため、IMU もフレームより細かい変化を表せません。
  - 1/4 速（60 fps）なら、約 4 ms ごとに新しい値が入ります。
  - 等速だと約 17 ms ごとになり、レーンチェンジの段差のような速い変化を取りこぼします。
  - **制御の評価には `slow=0.25` 以下を使ってください。**
- 電池の消耗、5V 昇圧回路（TPS61023）の低電圧リセット、モーターの温度上昇は扱いません。
- 制御の処理時間は 0 として扱います。

---

## 10. 検証の記録

このマニュアルの手順で、実際に次の環境で確認しました（2026年11月21日）。

- 環境：Windows 11、Unreal Engine 5.8.4、UE 内蔵 Python
- 確認した手順：
  - 環境チェック（モデル自己テストを含め、すべて OK）
  - サンプルの走行（7.2 の表）
  - `R` での制御ファイルの読み直し
  - 制御のエラー表示（`ZeroDivisionError` と行番号が出る）
  - `stop_py` で元の動作に戻ること

---

## 11. トラブルシューティング

| 症状 | 対処 |
|---|---|
| `... に class Controller がありません` | クラス名を `Controller` にする |
| `start_py` でいきなりエラー | ファイルの文法エラーなど。`start_py` は先に一度読み込んで確認しているので、出力ログのメッセージを見て直す |
| HUD に「コントローラーエラー」 | 表示された行を直して `R` を押す |
| `R` を押しても制御が変わらない | ファイルを保存したか確認する。ゲーム画面をクリックしてから `R` を押す |
| HUD に「io.duty未設定（ブレーキ扱い）」 | `io.duty` に値を入れていない |
| 車が動かない | `io.duty` が 0（＝ブレーキ）のまま。`io.print` で状態を出して確かめる |
| UE で `start_py` が見つからない | `m4wd_drive.py` が古い。`py import importlib, m4wd_drive; importlib.reload(m4wd_drive); m4wd_drive.register()` のあとに実行する |
| UE で `ModuleNotFoundError: numpy` | UE の Python に入れる（3.1）か、標準ライブラリだけで書く |
| コマンドを入力して Enter を押しても何も起きない | 結果は出力ログに出る。入力欄の左の「出力ログ」ボタンでログを開く。ドロップダウンが「Python」になっている場合は、先頭の `py ` を外す |
| Play しても車が動かない・HUD が出ない | 4.1 の環境チェックを実行し、NG を 4.2 で直す |
| キーを押しても反応しない | ゲーム画面を1回クリックする。マウスを解放するのは `Shift + F1` |
| `start_py` で `FileNotFoundError` | パスを確認する。名前だけ渡す場合は `MiniAI_PySim/controllers/<名前>.py`、なければ `controllers/local/<名前>.py` を探す |
| 同じ設定でも走りが毎回少し違う | IMU ノイズの乱数（`R` のたびに `seed` が1ずつ変わる）と、フレーム時間のばらつきによるもの。再現したいときは `noise=False` にし、`slow` を下げる |
