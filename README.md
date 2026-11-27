# mini4wdai_jmaab — クイックスタート（ミニ四駆 Python制御シミュレーター）

市販ミニ四駆 ＋ XIAO MG24 Sense ＋ KEIMYO-001 を、Python で書いた制御でコース上で走らせます。走らせ方は2つあり、**まずヘッドレス版で動作を確かめてください。**
- **ヘッドレス版（おすすめ）**：Python と物理エンジン MuJoCo だけで計算します。UE のインストールもライセンスの確認も不要で、5周でも1〜2秒で結果が出ます。結果はグラフ・ログと、簡素な 3D 再生で見ます。
- **UE 版**：Unreal Engine 5.8 の画面で、走る様子を見ながら確かめます。最後の確認や、見せるときに使います。

同じ制御ファイルがどちらでも動きます。詳しくは [ヘッドレス版マニュアル](3dsim/docs/headless_manual.md) ／ [UE 版マニュアル](3dsim/docs/manual.md) を見てください。

## 1. 入手する
**用意するもの**：Windows PC ／ Git for Windows（Git LFS 同梱）／ **Python 3.11〜3.13**（python.org、「Add python.exe to PATH」にチェック）

コマンドプロンプトで実行します。ヘッドレス版に要るコース部品のデータだけを取ります。
```
git lfs install
set GIT_LFS_SKIP_SMUDGE=1
git clone https://github.com/ssksoft/mini4wdai_jmaab.git
cd mini4wdai_jmaab
set GIT_LFS_SKIP_SMUDGE=
git lfs pull --include "3dsim/mini4wd_course/fbx/*"
```
- **ZIP ダウンロード（Code → Download ZIP）は使わないでください。** LFS のファイルが中身のない状態になり、どちらの版も動きません。
- UE 版も使うときは、あとでリポジトリのフォルダで `git lfs pull` を実行して、残り（約 120 MB）を取ります。

## 2. ヘッドレス版で走らせる
1. **セットアップ（初回だけ）**：`3dsim\MiniAI_PySim\setup_headless.bat` をダブルクリックします。専用の環境（`.venv`）を作り、MuJoCo などを入れ、環境チェックと試走までします。最後に「**結果: すべて OK**」と出れば準備完了です。
2. **走らせる**：コマンドプロンプトで実行します。
```
cd /d <リポジトリ>\3dsim\MiniAI_PySim
headless lc_brake --plot
```
最後に「**結果: 完走 合計 5.803 s**」と出ます（標準モーター、3周）。結果は `sim_out\lc_brake\` にできます（`plot.png` 速度・デューティ・電流などのグラフ ／ `log.csv` ／ `summary.json` ／ `serial.txt`）。
- **条件を変える**：`--motor standard`（既定）／`machdash_pro` 　`--battery nimh2`（既定）／`liion1` 　`--param 名前=値`（制御の調整値の上書き）　`--laps 5`（周回数、既定 3）。例：`headless lc_brake --motor machdash_pro --param RUN_DUTY=0.7`
- **3D で見る**：`headless lc_brake --view` で、走ったあとに 3D ビューアの窓で再生します。`--video` なら、実機の動画と同じ画角で `run.mp4` に書き出します。前回の走行は `headless view lc_brake` で見直せます。
- **総当たり**：`headless sweep lc_brake --param BRAKE_S=0.08,0.12,0.16 --param LEAD_DEG=20,30,40 --seeds 0,1,2` で全組み合わせを走らせ、完走数・平均タイムの表と最速の組み合わせを出します。

## 3. 自分の制御を書く
1. `3dsim\MiniAI_PySim\controllers\template.py` をコピーして、同じフォルダの `local\my_ctrl.py` を作ります（local は Git に入りません）。
2. `step(io)` の中で `io.duty`（1 = 全力駆動、0 = ショートブレーキ）を決めます。使える入力は `io.gyro` ／ `io.acc`（IMU）、`io.vbat()`（電池電圧）、`io.t`（時刻）。`io.print()` で表示します。
3. まずヘッドレス版で `headless my_ctrl --plot`。良さそうなら UE 版で `D.start_py("my_ctrl")`（手順 4）で確かめます。

## ヘッドレス版で困ったとき
- **環境の確認**：`headless check`（レポートは `sim_out\check_headless_report.txt`）。**「.venv not found」**：先に `setup_headless.bat` を実行します。**「FBX が Git LFS の引換券のまま」**：手順 1 の `git lfs pull --include …` を実行します。
- **MuJoCo が入らない**：Python の版（3.11〜3.13）を確認し、違えば Python 3.12 を入れて `setup_headless.bat --recreate`。
- **UE と結果が違う**：物理エンジンの違いで、ラップは数％ずれ、ぎりぎりの走行は結果が変わることがあります。

<div style="page-break-before:always"></div>

## 4. UE 版で走らせる
**追加で用意するもの**：Unreal Engine 5.8 ／ LFS の残りのデータ（リポジトリで `git lfs pull`）

1. **開く**：`3dsim\MiniYonkuSim\MiniYonkuSim.uproject` を**エクスプローラーでダブルクリック**します。Epic ランチャーの一覧から開くと、別の場所のコピーが開くことがあります。
2. **環境チェック（初回だけ）**：画面下の入力欄（**Cmd**）に入力して Enter。結果は**「出力ログ」ボタンで開くログ**に出ます。最後が「**結果: すべて OK**」なら準備完了、NG は UE 版マニュアル 4.2 で直します。
```
py import m4wd_drive as D; D.check_env()
```
3. **走らせる**：同じ入力欄に入力し、**Play（▶）を押してから、ゲーム画面を1回クリック**します。2 秒止まってから走り出し、「**3周 完走**」と出ます（作成者の環境では 6.08 s）。
```
py import m4wd_drive as D; D.start_py("lc_brake", laps=3)
```
- **キー**：`R` 制御ファイルを読み直して再スタート ／ `C` カメラ切替 ／ `T` スロー・等速の切り替え ／ `Esc` 終了
- **条件を変える**：Play を止めてから、引数を付けて `start_py` を実行し直します。`motor="standard"`（既定）／`"machdash_pro"`、`battery="nimh2"`（既定）／`"liion1"`、`params={"BRAKE_S": 0.15}`、`laps=3`（0 = 止めない）。例：マッハダッシュは既定値のままだとコースアウトしますが、次の設定なら完走します（5.09 s）。
```
py import m4wd_drive as D; D.start_py("lc_brake", motor="machdash_pro", params={"RUN_DUTY": 0.7}, laps=3)
```

## UE 版の結果と困ったとき
- **結果**：`3dsim\MiniYonkuSim\Saved\PySim\<名前>_<日時>\`（`log.csv` ・ `serial.txt` ・ `summary.json`）
- **「マップのロードに失敗しました」**：LFS の本体がありません。リポジトリで `git lfs pull` を実行します（ZIP で入手した場合は clone し直し）。
- **車が動かない・HUD が出ない**：手順 4-2 の環境チェックを実行します。
- **キーが効かない**：ゲーム画面をクリックします（マウスを解放するのは `Shift+F1`）。
- **赤字の「コントローラーエラー」**：表示された行を直して `R` を押します。
- **問い合わせのとき**：`3dsim\MiniYonkuSim\Saved\PySim\check_env_report.txt` を添えてください。

<!-- pdf-end: ここから下は PDF（Releases 添付の1ページ版）に入りません -->

## このリポジトリのフォルダ
| フォルダ | 内容 |
|---|---|
| [3dsim/](3dsim/) | シミュレーション一式（ヘッドレス版・UE 版。マニュアルは [ヘッドレス版 詳細](3dsim/docs/headless_manual.md) ／ [UE 版 詳細](3dsim/docs/manual.md)）（詳細は [3dsim/README.md](3dsim/README.md)、変更履歴は [3dsim/CHANGELOG.md](3dsim/CHANGELOG.md)） |
| `control_motor/` `get_imu/` `get_imu_with_run/` `led/` | XIAO MG24 用の Arduino スケッチ |
| `03_models/` `04_data/` `resources/` `xiao_codegen_sample_ert_rtw/`、`Mini4WDAI.prj` | MATLAB / Simulink プロジェクト関連（モデル・データ・コード生成） |
