# qr_detector_pkg

ROS 2 Humble用のQRコード検出パッケージです。`sensor_msgs/msg/CompressedImage` を購読し、QRコード文字列を `std_msgs/msg/String` で配信しつつ、枠と文字を描画した `sensor_msgs/msg/CompressedImage` を配信します。

## 依存パッケージのインストール

以下のコマンド例で必要な依存関係をインストールできます。

```bash
sudo apt update
sudo apt install python3-opencv python3-pyzbar python3-numpy
```

Pythonパッケージとして `opencv-python` や `pyzbar` のホイールを使用したい場合は以下のようにpipでインストールしてください。

```bash
python3 -m pip install --user opencv-python pyzbar numpy
```

## ビルド方法

ワークスペースルートで、必ず次のスクリプトでビルドしてください。

```bash
bash ~/inSectWRS2025/script/10_ws_build.sh
```

## ノードの起動

ビルド後、以下のコマンドでノードを起動できます。

```bash
ros2 launch qr_detector_pkg qr_detector.launch.py
```

検出したQRコードの文字列は次のトピックで確認できます。

```bash
ros2 topic echo /qr/text
```

## トピック概要

| 種別 | トピック名 | 型 | 説明 |
| ---- | ----------- | --- | ---- |
| 入力 | `/image_in/compressed` | `sensor_msgs/msg/CompressedImage` | 入力圧縮画像 |
| 出力 | `/qr/text` | `std_msgs/msg/String` | 認識文字列（複数は改行区切り） |
| 出力 | `/image_out/compressed` | `sensor_msgs/msg/CompressedImage` | 枠と文字を描画した圧縮画像 |

## パラメータ

`config/params.yaml` で各種パラメータを調整できます。

| パラメータ名 | 型 | 既定値 | 説明 |
| ------------ | -- | ------ | ---- |
| `input_image_topic` | string | `/image_in/compressed` | 入力画像トピック |
| `output_image_topic` | string | `/image_out/compressed` | 出力画像トピック |
| `text_topic` | string | `/qr/text` | 認識文字列トピック |
| `encode_format` | string | `"jpeg"` | 出力画像圧縮フォーマット |
| `jpeg_quality` | int | `90` | JPEGエンコード品質 |
| `rotate_attempts` | list[int] | `[0, 90, 180, 270]` | 画像を回転して検出を試みる角度 |
| `use_pyzbar_fallback` | bool | `true` | OpenCV検出失敗時にpyzbarで再検出するか |
| `draw_box` | bool | `true` | QRコード枠線を描画するか |
| `draw_text` | bool | `true` | QRコード文字列を画像上に描画するか |
| `max_fps` | int | `30` | 処理する最大フレームレート |
| `skip_rate` | int | `1` | フレーム間引き率（`n`なら`n`フレームに1回処理） |
| `qos_reliability` | string | `"best_effort"` | QoSの信頼性設定（`best_effort`/`reliable`など） |

## ライセンス

Apache License 2.0
