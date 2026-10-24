# 対応プロファイルと設定

プロファイル名: `humanoid-xarm-yup-cm-120fps`

Hips と Head が必要です。左右それぞれ以下の直接の親子関係が必要です。

```text
LeftArm -> LeftForeArm -> LeftHand
RightArm -> RightForeArm -> RightHand
LeftUpLeg -> LeftLeg -> LeftFoot -> LeftToeBase
RightUpLeg -> RightLeg -> RightFoot -> RightToeBase
```

その他の中間ボーンは存在できます。骨名の重複・子ボーンの位置チャンネルには対応しません。
腕の子のオフセットはローカルX軸に沿う必要があります（軸外成分はX成分の0.1%以内）。
脚のスケール検査は大腿長10–100cmです。この検査だけで単位や人体の正しさを保証するものではありません。

## 設定

未指定項目には同梱プロファイルの値を使います。未知の設定キーは拒否します。

| キー | 意味 / 初期値 |
|---|---|
| profile | 上記のプロファイル名 |
| pelvis_snap_frames | 検出済み骨盤スナップのうち補修を選択する遷移番号 / 空 |
| glitch_frames | 1フレームの全身異常を前後で補間するフレーム番号 / 空 |
| manual_bridges | `["Left" または "Right", 最初の異常フレーム, 最初の正常フレーム]` の配列 / 空 |
| arm_chain_bridges | 同じ書式で上腕・前腕・手をワールド回転で補間 / 空 |
| pole_sigma_samples | 膝方向のフィルタ幅（標準偏差 サンプル単位） / 6.0 |
| knee_straighten_degrees | 接地IKによる追加の膝伸展上限 / 8.0 |

フレーム番号は0始まりです。補間区間 `[lo, hi)` は `lo-1` と `hi` の境界姿勢でつなぎます。
`pelvis_snap_frames` のfは遷移 `f -> f+1` を表し 修正範囲は `[f-19, f+1)` です。
骨盤スナップは検出結果との一致も検査します。診断候補をそのまま全部指定せず 原本との比較で選別してください。

固定閾値の調整や異なる骨格への対応には現状コード変更と再検証が必要です。
少なくとも 回転ステップ・自然な速い動きの保存・停止区間の誤検出・接地・境界の連続性を確認してください。
