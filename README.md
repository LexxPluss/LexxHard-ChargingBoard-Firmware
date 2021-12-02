# LexxPluss Charging Board Software

## Install arduino-cli

```bash
$ brew install arduino-cli
$ arduino-cli config init
```

## Add core

```bash
$ vi ~/Library/Arduino15/arduino-cli.yaml
```

additional_urlsに追加。

```yaml
board_manager:
  additional_urls: [https://mcudude.github.io/MightyCore/package_MCUdude_MightyCore_index.json]
```

```bash
$ arduino-cli core update-index
$ arduino-cli core install MightyCore:avr
```

## Build

```bash
$ git clone https://github.com/LexxPluss/chargingboard.git
$ cd chargingboard
$ arduino-cli compile --fqbn MightyCore:avr:1284:pinout=standard,variant=modelP,BOD=2v7,LTO=Os,clock=16MHz_external
```

## Program

### Program bootloader

下記接続でArduino UNOとCharging Boardを接続して書き込む。

| Name  | UNO pin | ATmega1284p pin | J4 pin |
| ----  | ----    | ----            | ----   |
| RESET | 10      | 9               | 5      |
| MOSI  | 11      | 6               | 4      |
| MISO  | 12      | 7               | 1      |
| SCK   | 13      | 8               | 3      |

```bash
$ arduino-cli burn-bootloader -p /dev/cu.usbxxxx -P arduinoasisp --fqbn MightyCore:avr:1284:pinout=standard,variant=modelP,BOD=2v7,LTO=Os,clock=16MHz_external
```

### Program sketch

Bootloaderが書き込まれていれば下記でSketchを書き込むことができる。

```bash
$ arduino-cli upload -p /dev/cu.usbserial-xxxxxx --fqbn MightyCore:avr:1284:pinout=standard,variant=modelP,BOD=2v7,LTO=Os,clock=16MHz_external
```
