# BVHモーション修正ツール

BVHモーションの飛び・ねじれを検出し、補修候補と比較画面を作るWindows向けアプリです。日本語GUIと人工サンプルを同梱しています。Windows版はPythonの追加インストール不要です。

## ダウンロードと使い方

[Windows版](https://github.com/sakanawalk/bvh-motion-repair/releases/latest) · [BOOTH](https://sakanawalk.booth.pm/items/9037981)

1. ZIPをすべて展開し、`モーション修正ツール.exe`を起動します。
2. 「BVHを選ぶ」または「お試しデータを使う」で入力を選びます。
3. 保存先と入力形式を確認して「修正する」を押します。
4. 「修正前後を比較する」で結果を確認します。

元ファイルは上書きせず、実行ごとに別の結果フォルダーを作成します。修正候補は`repaired.bvh`です。詳しい操作は[使い方](guide.html)を参照してください。

## 対応環境と制約

- Windows 64bit
- 約120fps・cm単位・Y-up・[対応プロファイル](docs/PROFILE.md)の人型骨格を持つBVH
- FBX、VMD、30/60fps、任意の骨格には非対応

自動修正は補修候補の作成です。振付の意図やモデル適用後の見た目を確認してから採用してください。
Windows 11の開発環境で、起動・サンプル処理・入力検証・比較画面の再生操作を確認しています。別PCでの動作は未検証です。

## 利用条件

本体には[sakanawalk Software License 1.0](LICENSE)が適用されます。

- 個人・法人とも無料で利用できます。
- 本体・改変版の無断再配布や販売は禁止です。
- 作成した動画・モーションの公開、配布、商用利用は可能です。入力素材の利用条件は別途適用されます。

第三者ライブラリにはそれぞれのライセンスが適用されます。[Windows版のライセンス情報](docs/DESKTOP_LICENSE_AUDIT.md)と同梱の原文を参照してください。過去のMIT版に基づく許諾は遡って取り消しません。確認範囲は[権利・来歴の記録](RIGHTS_REVIEW.txt)に記載しています。
BOOTHの「コーヒー代」は任意の200円です。無料版とファイル・機能・利用条件は同一です。

## ソースから起動

PythonとTkを利用できる環境で実行します。

```console
python -m pip install .
python launch_desktop.py
python -m unittest discover -s tests -v
```

処理仕様は[アルゴリズム](docs/ALGORITHM.md)を参照してください。不具合は[Issues](https://github.com/sakanawalk/bvh-motion-repair/issues)へ報告できます。添付するログや画像の個人情報・素材の利用条件に注意してください。

## 更新履歴

### 0.2.0 r6

- 指数表記のフレーム時間に対応
- 不正なフレーム時間・余分なデータの入力検証を改善
- 比較画面の末尾から再生すると先頭へ戻るよう修正
