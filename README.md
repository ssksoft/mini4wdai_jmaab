# mini4wdai_jmaab — クイックスタート（ミニ四駆 Python制御シミュレーター）

市販ミニ四駆 ＋ XIAO MG24 Sense ＋ KEIMYO-001 を、Python で書いた制御で UE のコース上で走らせます。詳しくは [3dsim/docs/manual.md](3dsim/docs/manual.md) を見てください。
**用意するもの**：Windows PC ／ Unreal Engine 5.8 ／ Git for Windows（Git LFS 同梱）／ リポジトリ `ssksoft/mini4wdai_jmaab` へのアクセス権

## 1. 入手する・開く
```
git lfs install
git clone https://github.com/ssksoft/mini4wdai_jmaab.git
cd mini4wdai_jmaab && git lfs pull
```
- **ZIP ダウンロード（Code → Download ZIP）は使わないでください。** LFS のファイル（UE のアセットなど）が中身のない数百バイトのファイルになり、UE で「マップのロードに失敗しました … はアセットファイルのようです」と出て開けません。
- `3dsim\MiniYonkuSim\MiniYonkuSim.uproject` を**エクスプローラーでダブルクリック**します。Epic ランチャーの一覧から開くと、別の場所のコピーが開くことがあります。

## 2. 環境チェック（初回だけ）
画面下の**出力ログ**（Cmd 欄）に入力して Enter を押します。最後に「**結果: すべて OK**」と出れば準備完了です。NG が出たら `3dsim/docs/manual.md` 4.2 の表で直します。
```
py "<リポジトリ>/3dsim/MiniAI_PySim/tools/check_env.py"
```

## 3. 走らせる
出力ログに入力し、**Play（▶）を押してから、ゲーム画面を1回クリック**します。2 秒止まってから走り出し、「**3周 完走**」と出ます（作成者の環境では 6.08 s）。
```
py import m4wd_drive as D; D.start_py("lc_brake", laps=3)
```
- **キー**：`R` 制御ファイルを読み直して再スタート ／ `C` カメラ切替 ／ `T` スロー・等速の切り替え ／ `Esc` 終了
- **条件を変える**：Play を止めてから、引数を付けて `start_py` を実行し直します。
  - `motor="standard"`（既定）／`"machdash_pro"`
  - `battery="nimh2"`（既定）／`"liion1"`
  - `params={"BRAKE_S": 0.15}`（制御の調整値の上書き）
  - `laps=3`（0 = 止めない）

  例：マッハダッシュは既定値のままだとコースアウトしますが、次の設定なら完走します（5.09 s）。
```
py import m4wd_drive as D; D.start_py("lc_brake", motor="machdash_pro", params={"RUN_DUTY": 0.7}, laps=3)
```

## 4. 自分の制御を書く
1. `3dsim\MiniAI_PySim\controllers\template.py` をコピーして、同じフォルダの `local\my_ctrl.py` を作ります（local は Git に入りません）。
2. `step(io)` の中で `io.duty` を決めます。
   - `io.duty`：1 = 全力駆動、0 = ショートブレーキ
   - 使える入力：`io.gyro` ／ `io.acc`（IMU）、`io.vbat()`（電池電圧）、`io.t`（時刻）
   - `io.print()` で HUD に表示します。
3. `D.start_py("my_ctrl")` を実行してから Play します。直したら保存して `R` を押します。

## 結果と困ったとき
- **結果**：`3dsim\MiniYonkuSim\Saved\PySim\<名前>_<日時>\`（`log.csv` ・ `serial.txt` ・ `summary.json`）
- **「マップのロードに失敗しました」**：LFS の本体がありません。ZIP で入手した場合は clone し直し、clone した場合は `git lfs pull` を実行します。
- **車が動かない・HUD が出ない**：手順 2 の環境チェックを実行します。
- **キーが効かない**：ゲーム画面をクリックします（マウスを解放するのは `Shift+F1`）。
- **赤字の「コントローラーエラー」**：表示された行を直して `R` を押します。
- **問い合わせのとき**：`3dsim\MiniYonkuSim\Saved\PySim\check_env_report.txt` を添えてください。

<!-- pdf-end: ここから下は PDF（Releases 添付の1ページ版）に入りません -->

## このリポジトリのフォルダ
| フォルダ | 内容 |
|---|---|
| [3dsim/](3dsim/) | UE 5.8 のシミュレーション一式（詳細は [3dsim/README.md](3dsim/README.md)、変更履歴は [3dsim/CHANGELOG.md](3dsim/CHANGELOG.md)） |
| `control_motor/` `get_imu/` `get_imu_with_run/` `led/` | XIAO MG24 用の Arduino スケッチ |
| `03_models/` `04_data/` `resources/` `xiao_codegen_sample_ert_rtw/`、`Mini4WDAI.prj` | MATLAB / Simulink プロジェクト関連（モデル・データ・コード生成） |
