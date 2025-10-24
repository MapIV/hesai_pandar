# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


## v2.3.0

### Changed
- ros1ブランチにマージ


## v2.2.1

### Fixed
- センサとの接続が切れて再接続されても、pandar_monitorだけ再接続されない問題を対策


## v2.2.0

### Fixed
- XT32/XT32Mを長時間使用すると定期的に誤った高温値が入力されてエラーが出る問題を対策（MID-5039）<br>
  移動平均処理のため、起動直後１０秒程度は低値を表示します


## v1.3 (v2.1.3)

### Added
- changelogを作成


## v1.2 (v2.1.2)

### Fixed
- signed intの温度データをunsignedで受け取っている箇所を修正
- 温度センサのラベル名が間違っていたため修正
- 一部の環境でlibfmt関連のビルドエラーが発生する問題を対策


## v1.1 (v2.1.1)

### Fixed
- 照射部分を隠すとドライバーが死ぬことがある不具合を修正


## v1.0 (v2.1.0)

初回リリース