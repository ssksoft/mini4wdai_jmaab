# 3dsim — ミニ四駆 3D シミュレーション（Unreal Engine 5.8）

タミヤのオーバルホームサーキット（立体レーンチェンジ）とマッドレイザー（MAシャーシ）を、Unreal Engine 上で再現したシミュレーション環境です。

- **Python 制御**：XIAO MG24 Sense ＋ KEIMYO-001 モータードライバを載せた車を、Python で書いた制御で走らせます。
- **キーボード走行**：手で走らせる従来のモードです。
- **再現シナリオ**：実機の動画を再現するモードです。

## はじめ方

手順は、リポジトリ直下の [README.md](../README.md)（クイックスタート）にまとめています。clone から、環境チェック、サンプルの走行、自分の制御の書き方まで載っています。

| ドキュメント | 内容 |
|---|---|
| [../README.md](../README.md) | クイックスタート（リポジトリ直下の README。印刷用 PDF は [Releases](https://github.com/ssksoft/mini4wdai_jmaab/releases) に添付） |
| [docs/manual.md](docs/manual.md) | 使い方マニュアル（制御の書き方、パラメーター、カスタマイズの編集ポイント、精度と限界） |
| [docs/verification_checklist.md](docs/verification_checklist.md) | 別の人が再現できるかを確かめるチェックシート |
| [docs/keyboard_drive_guide.md](docs/keyboard_drive_guide.md) | キーボード走行・コース作成の操作ガイド |
| [docs/reports/](docs/reports/) | ラップタイム最適化のレポート |
| [CHANGELOG.md](CHANGELOG.md) | 版ごとの変更点 |

## フォルダ構成

```
3dsim/
├─ MiniYonkuSim/        UE プロジェクト（Content/Mini4WD に部品・車・レベル、Content/Python/init_unreal.py）
├─ mini4wd_course/      コース・車体の生成スクリプトと元データ
│   ├─ ue/              UE エディタ用スクリプト（m4wd_drive.py = 走行ランタイム、setup_in_ue.py、car_setup.py …）
│   ├─ generator/       寸法からコース部品・車体の FBX を作る Python
│   ├─ fbx/ car/        生成したコース部品・車体の FBX
│   ├─ scan_fbx/        3D スキャンから作った見た目用のデータ
│   └─ opt/             ラップタイム最適化の試行設定（ue/run_opt.py が読む）
├─ MiniAI_PySim/        Python 制御
│   ├─ controllers/     制御（lc_brake.py = サンプル、template.py = ひな形、local/ = 個人用・Git 対象外）
│   ├─ sil/             ドライバー・電池・モーター・IMU のモデル（sil_core.py）
│   ├─ tools/           check_env.py（環境チェック）、build_docs.py（PDF 生成）
│   └─ tests/           モデルの自動テスト（GitHub Actions で実行）
└─ docs/                ドキュメント
```

## Git の使い方

- UE のアセット（`.uasset` / `.umap`）、FBX、画像は **Git LFS** で管理しています（`3dsim/.gitattributes`）。最初に PC ごとに1回、`git lfs install` を実行してください。
- GitHub の **Download ZIP では LFS の本体が入りません**（数百バイトの「引換券」だけになり、UE で開けません）。必ず `git clone` → `git lfs pull` で入手してください。
- UE が作るフォルダ（`Binaries` `Intermediate` `Saved` `DerivedDataCache`）と、個人用の制御（`MiniAI_PySim/controllers/local/`）は Git に入りません（`3dsim/.gitignore`）。
- **コミットする前に**：UE で保存（Ctrl+Shift+S）し、UE を閉じてからにします。
- **バイナリのアセット**はマージできません。複数人で同じアセットを触るときは、`git lfs lock <ファイル>` でロックしてください。
- **変更の入れ方**：作業用のブランチで作業し、Pull Request で `main` に入れます。Pull Request のたびに、GitHub Actions がモデルのテストを実行します。
- **版**：`v1.0.0` のようなタグを付けて push すると、GitHub Actions が Release を作り、クイックスタートの PDF を添付します。
- **モデルのテスト（手元で実行する場合）**：
  ```
  pip install pytest
  pytest 3dsim/MiniAI_PySim/tests
  ```

## 注意

- UE で走らせる確認は、自動テストではできません。UE 側を変えたときは、`docs/verification_checklist.md` の C〜E を手で確認してください。
- LFS のデータは clone のたびに転送されます（100 MB 前後）。たくさんの人に配る場合は、GitHub の LFS の容量・転送量の上限を確認してください。
