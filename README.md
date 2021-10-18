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
$ arduino-cli compile --fqbn MightyCore:avr:1284:pinout=standard,variant=modelP,BOD=2v7,LTO=Os,clock=16MHz_external
```
