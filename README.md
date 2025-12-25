# UTC 変換 ROS 2 パッケージ
[![test](https://github.com/mobluebird/mypkg/actions/workflows/test.yml/badge.svg)](https://github.com/mobluebird/mypkg/actions/workflows/test.yml)
![License](https://img.shields.io/badge/License-BSD--3--Clause-green.svg)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)
![Python](https://img.shields.io/badge/Python-3.10-yellow.svg)
![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)

本パッケージは、UTC（協定世界時）から **JD（ユリウス日）・GMST（恒星時）・LST（地方恒星時）** を計算し、それぞれ独立した ROS 2 トピックで配信するものです。  
4つのトピックに分けることで、必要な情報だけを個別に受信可能です。

## ノードの説明
### talker
- 役割: UTC を取得し、JD・GMST・LST に変換して各トピックにパブリッシュする
- ノード名: `talker`
- パブリッシュするトピック: `/utc_time`, `/julian_day`, `/gmst`, `/lst`
- 更新間隔: 1 秒
### listener
- 役割: 各トピックを受信して画面に表示する
- ノード名: `listener`
- サブスクライブするトピック: `/utc_time`, `/julian_day`, `/gmst`, `/lst`
- 表示内容: UTC、JD、GMST、LST（Tokyo）

## Python モジュールの説明
### time_utils.py
- 役割: UTC → JD → GMST → LST への計算関数を提供
- 利用: `talker` で呼び出される

## トピックの説明
| トピック名       | メッセージ型      | 内容                         |
|------------------|-------------------|------------------------------|
| `/utc_time`      | `std_msgs/String` | UTC 時刻文字列               |
| `/julian_day`    | `std_msgs/String` | ユリウス日（JD）             |
| `/gmst`          | `std_msgs/String` | 恒星時（GMST）             　|
| `/lst`           | `std_msgs/String` | 東京の地方恒星時（LST）      |

## 実行方法
### UTC, JD, GMST, LST をまとめて表示する場合
- 以下のコマンドで実行可能です。
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
### 個別に表示する場合
- talkerを起動する。
```
$ ros2 run mypkg talker
```
- 別端末で各トピックを確認する。
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

## テスト環境
- Ubuntu 22.04 LTS
- Python 3.10
- ROS 2 Humble

## 謝辞
- Web記事・資料
  - [タイムゾーンとは？](https://it-infomation.com/time-zone/)
  - [Julian Day Numbers](https://quasar.as.utexas.edu/BillInfo/JulianDatesG.html)
  - [Computing siderial time](https://www.nies.ch/doc/astro/sternzeit.en.php)
  - [GMST・恒星時の考え方（計算式の理論参考）](https://zh.wikipedia.org/wiki/恆星時)
  - [恒星時 (Sidereal Time)](https://eco.mtk.nao.ac.jp/koyomi/wiki/B9B1C0B1BBFE.html)
- 利用・参照したコード
  - このパッケージのコードの一部は，下記のスライド（CC-BY-SA 4.0 by Ryuichi Ueda）のものを，本人の許可を得て自身の著作としたものです。
    - [ryuichiueda/slides_marp/robosys2025](https://github.com/ryuichiueda/slides_marp/tree/master/robosys2025)
  - 計算ロジックは、下記のコードを参考にしました。
    - [astropy.time — 天文時間操作ライブラリ](https://docs.astropy.org/en/latest/time/) © 2011–2025 The Astropy Developers, BSD-3-Clause license

## ライセンス
このソフトウェアパッケージは，3条項BSDライセンスの下，再頒布および使用が許可されます。

## 著作権
© 2025 Motona Shigehisa
