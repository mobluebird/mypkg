# 天文時間計算・配信 ROS 2 パッケージ
[![test](https://github.com/mobluebird/mypkg/actions/workflows/test.yml/badge.svg)](https://github.com/mobluebird/mypkg/actions/workflows/test.yml)
![License](https://img.shields.io/badge/License-BSD--3--Clause-green.svg)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-E95420?style=flat&logo=ubuntu&logoColor=white)
![Python 3.10](https://img.shields.io/badge/Python-3.10-F9DC3E?style=flat&logo=python&logoColor=blue)
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-3399FF?style=flat&logo=ros)

本パッケージは、現在の **UTC（協定世界時）** を取得し、天文時間である **JD（ユリウス日）・GMST（恒星時）・LST（地方恒星時・東京）** を計算し、それぞれ独立したトピックとして配信します。  

## ノードの説明
### talker
- 役割: UTC を取得し、JD・GMST・LST に変換して各トピックにパブリッシュします
- パブリッシュするトピック: `utc_time`、`julian_day`、`gmst`、`lst`
- 更新間隔: 1 秒
### listener
- 役割: 各トピックを受信して画面に表示します（動作確認用）
- サブスクライブするトピック: `utc_time`、`julian_day`、`gmst`、`lst`
- 表示内容: UTC、JD、GMST、LST（Tokyo）

## トピックの説明
| トピック名       | メッセージ型      | 内容                         |
|------------------|-------------------|------------------------------|
| `utc_time`      | `std_msgs/String` | 現在の協定世界時（UTC）を ISO 8601 形式で文字列配信   |
| `julian_day`    | `std_msgs/String` | UTC 時刻を基に計算したユリウス日（JD）を文字列で配信  |
| `gmst`          | `std_msgs/String` | グリニッジ恒星時（GMST）を時間形式（hh:mm:ss）で配信  |
| `lst`           | `std_msgs/String` | 東京の地方恒星時（LST）を時間形式（hh:mm:ss）で配信   |

## 実行方法
- `talker` を起動します
```
$ ros2 run mypkg talker
```
- 別端末で各トピックを確認します
```
$ ros2 topic echo /utc_time
data: '2025-12-24T07:01:36.242501+00:00'
---
```
```
$ ros2 topic echo /julian_day
data: '2461033.79293'
---
```
```
$ ros2 topic echo /gmst
data: '06:12:15.47'
---
```
```
$ ros2 topic echo /lst
data: '15:31:01.51'
---
```

## 動作確認用
`listener` を用いると、すべての天文時間情報をまとめてサブスクライブして表示できます。
```
$ ros2 launch mypkg talk_listen.launch.py
[INFO] [launch]: All log files can be found below /home/moshi/.ros/log/2025-12-24-16-02-37-800976-pR-310738
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [talker-1]: process started with pid [310739]
[INFO] [listener-2]: process started with pid [310741]
[listener-2] [INFO] [1766559759.148814459] [listener]:
[listener-2] UTC : 2025-12-24T07:02:38.242373+00:00
[listener-2] JD  : 2461033.79350
[listener-2] GMST: 06:12:15.59
[listener-2] LST (Tokyo): 15:31:01.60
```

## Python モジュールの説明
### time_utils.py
- 役割: UTC → JD → GMST → LST への計算関数を提供します
- 利用: `talker` で呼び出されます

## テスト環境
- Ubuntu 22.04 LTS
- Python 3.10
- ROS 2 Humble Hawksbill

## 謝辞
- Web記事・資料
  - [タイムゾーンとは？](https://it-infomation.com/time-zone/)
  - [Julian Day Numbers](https://quasar.as.utexas.edu/BillInfo/JulianDatesG.html)
  - [Computing siderial time](https://www.nies.ch/doc/astro/sternzeit.en.php)
  - [GMST・恒星時の考え方（計算式の理論参考）](https://zh.wikipedia.org/wiki/恆星時)
  - [恒星時 (Sidereal Time)](https://eco.mtk.nao.ac.jp/koyomi/wiki/B9B1C0B1BBFE.html)
- 利用・参照したコード
  - このパッケージのコードの一部は、下記のスライド（CC-BY-SA 4.0 by Ryuichi Ueda）のものを、本人の許可を得て自身の著作としたものです。
    - [ryuichiueda/slides_marp/robosys2025](https://github.com/ryuichiueda/slides_marp/tree/master/robosys2025)
  - 計算ロジックは、下記のコードを参考にしました。
    - [astropy.time — 天文時間操作ライブラリ](https://docs.astropy.org/en/latest/time/) © 2011–2025 The Astropy Developers, BSD-3-Clause license

## ライセンス
このソフトウェアパッケージは、3条項BSDライセンスの下、再頒布および使用が許可されます。

## 著作権
© 2025 Motona Shigehisa
