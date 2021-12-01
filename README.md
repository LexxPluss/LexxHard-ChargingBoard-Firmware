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
$ git clone https://github.com/LexxPluss/mbed-os-powerboard.git
$ git clone https://github.com/LexxPluss/chargingboard.git
$ cd chargingboard
$ ln -s ../mbed-os-powerboard/serial_message.hpp .
$ arduino-cli compile --fqbn MightyCore:avr:1284:pinout=standard,variant=modelP,BOD=2v7,LTO=Os,clock=16MHz_external
```

## Program

```bash
$ arduino-cli upload -p /dev/cu.usbserial-xxxxxx --fqbn MightyCore:avr:1284:pinout=standard,variant=modelP,BOD=2v7,LTO=Os,clock=16MHz_external
```
