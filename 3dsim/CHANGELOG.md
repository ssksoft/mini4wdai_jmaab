# 変更履歴

書き方：新しい版を上に追加します。版番号は `vメジャー.マイナー.パッチ` です。

- 制御の書き方（`io` など）が変わるとき：メジャーを上げる
- 機能を足したとき：マイナーを上げる
- 不具合を直したとき：パッチを上げる

## v1.1.0（2026-11-27）

### 追加：ヘッドレス版（UE なし、Python ＋ MuJoCo）
- UE を使わずに、同じコース・車・制御を MuJoCo で走らせる「ヘッドレス版」を戻しました（v47 まであったものを、今の構成に合わせて組み込み直し）。
  - `mini4wd_course/mjsim/m4wd_mj.py`（コースと車のモデル）、`MiniAI_PySim/sil/run_headless.py`（実行の本体）
  - `MiniAI_PySim/setup_headless.bat`：専用環境（`.venv`）の作成、パッケージの導入、環境チェックまで一度に行います。
  - `MiniAI_PySim/headless.bat`：`headless check`（環境チェック）、`headless <制御>`（実行）、`headless sweep`（パラメーター探索）
- 3D 再生：走ったあとに MuJoCo のビューアで再生（`--view` / `headless view`）、または MP4 動画に書き出し（`--video` / `headless video`。視点は、実機の試走動画・UE の再現シナリオと同じ画角の `real`（既定。レーンチェンジが手前、スタート地点が奥）、車を追う `follow`、真上からの `top`）。走行の記録 `replay.npz` を毎回残し、再生ではシミュレーションをやり直しません（`sil/replay_headless.py`）。動画用に `imageio`・`imageio-ffmpeg` を追加。
  - `requirements-headless.txt`：MuJoCo は 4.0.1 に固定（版によって、ぎりぎりの走行の結果が変わるため）。Python は 3.11〜3.13。
- クイックスタート（リポジトリ直下の `README.md`）をヘッドレス版と統合し、**まずヘッドレス版で動作を確かめる**順に組み替えました（UE のインストールが不要で軽く、ライセンスの確認も要らないため）。PDF は2ページ（1ページ目：ヘッドレス版、2ページ目：UE 版）。
- clone の手順を、ヘッドレス版に要るコース部品の FBX だけを LFS から取る形にしました（UE のデータは使うときに `git lfs pull`）。
- マニュアル詳細版：`docs/headless_manual.md`。
- 自動テスト `tests/test_headless.py` を追加しました（MuJoCo がない環境ではスキップ）。GitHub Actions でもヘッドレス版のテストを実行します（LFS はコース部品の FBX だけを取得）。

### 基準の結果（MuJoCo 4.0.1、lc_brake、ニッケル水素、3周）
- 標準モーター：5.80 s（UE 5.8.4 では 6.08 s）
- マッハダッシュPRO（`RUN_DUTY` 0.7）：5.00 s（UE では 5.09 s）

## v1.0.1（2026-11-25）

- リポジトリを `ssksoft/mini4wdai_jmaab` に移しました（履歴は引き継いでいません。元は `ssksoft/mini4wdai`）。ドキュメントの clone 先とフォルダ名を直しました。
- 「ZIP ダウンロードでは LFS の本体が入らない」注意書きを、README・マニュアル・チェックシートに足しました。
- 環境チェック（`check_env.py`）で、LFS の本体がない（引換券のままの）アセットを見つけるようにしました。
- `docs/keyboard_drive_guide.md` の作成者 PC 固有のパスを `<リポジトリ>` に直しました。
- 環境チェックを `py import m4wd_drive as D; D.check_env()` で実行できるようにしました。開いているプロジェクトのチェックを、パスを打たずに実行できます（clone が複数あるときに、別のコピーの check_env.py を実行して NG になるのを防ぐため）。
- 「環境チェックの結果は出力ログに表示される」ことを、README・マニュアル・チェックシートに書きました。

## v1.0.0（2026-11-25）

GitHub での管理を始めた最初の版です。それまでは zip（v45〜v52）で配っていました。

### 構成
- ドキュメントを `3dsim/docs/` にまとめ、ファイル名を英語にしました。
  - `MiniAI_PySim/使い方マニュアル.md` → `docs/manual.md`
  - `MiniAI_PySim/クイックスタート.md` → リポジトリ直下の `README.md`（クイックスタートを兼ねる）
  - `MiniAI_PySim/検証チェックシート.md` → `docs/verification_checklist.md`
  - `3dsim_操作ガイド.md`（リポジトリ直下）→ `docs/keyboard_drive_guide.md`
  - ラップタイム最適化レポート2本 → `docs/reports/`
- `3dsim/README.md` を日本語の入口に書き直しました。
- 個人用の制御の置き場所として `MiniAI_PySim/controllers/local/` を作りました（Git の対象外）。`start_py("名前")` で、ここにあるファイルも見つかります。
- 次のものを削除しました。
  - `MiniAI_PySim/project_template/`：`init_unreal.py` はリポジトリに入っているため、予備は不要です。
  - `クイックスタート.pdf`：README.md から生成します（`tools/build_docs.py`、Releases に添付）。
  - 検証で作ったテスト用の `controllers/my_ctrl.py`

### 追加
- `MiniAI_PySim/tests/test_sil_core.py`：モデルの自動テスト。
- `MiniAI_PySim/tools/build_docs.py`：クイックスタートの PDF を生成します。
- `.github/workflows/pysim.yml`：push・Pull Request のたびにテストし、タグを push したら Release を作ります。

### これまでの主な内容（zip の時代）
- Python 制御：`start_py` / `stop_py`。制御ファイルは `R` で読み直せます。エラーは行番号付きで表示します。
- KEIMYO-001・電池・モーター・仮想 IMU のモデル（`sil_core.py`）。
- 環境チェック `check_env.py`（ファイル、モデル自己テスト、ランタイム、レベルを確認）。
- 実機の UE 5.8.4 で検証しました（2026-11-21）。
  - 標準モーター：3周完走 6.08 s
  - マッハダッシュPRO（`RUN_DUTY` 0.7）：3周完走 5.09 s
