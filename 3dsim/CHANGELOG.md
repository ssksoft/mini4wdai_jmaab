# 変更履歴

書き方：新しい版を上に追加します。版番号は `vメジャー.マイナー.パッチ` です。

- 制御の書き方（`io` など）が変わるとき：メジャーを上げる
- 機能を足したとき：マイナーを上げる
- 不具合を直したとき：パッチを上げる

## v1.0.1（2026-11-25）

- リポジトリを `ssksoft/mini4wdai_jmaab` に移しました（履歴は引き継いでいません。元は `ssksoft/mini4wdai`）。ドキュメントの clone 先とフォルダ名を直しました。
- 「ZIP ダウンロードでは LFS の本体が入らない」注意書きを、README・マニュアル・チェックシートに足しました。
- 環境チェック（`check_env.py`）で、LFS の本体がない（引換券のままの）アセットを見つけるようにしました。
- `docs/keyboard_drive_guide.md` の作成者 PC 固有のパスを `<リポジトリ>` に直しました。

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
