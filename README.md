## 概要

f446のHALライブラリを使ったプロジェクトのテンプレート。

## 導入方法

### linux (bash)

`~/.bashrc`に以下を追記しておく。
```bash
function f446_hal {
    if [ $# -eq 0 ]; then
        read -p "project name: " project_name
    else 
        project_name=$1
    fi
    git clone git@github.com:KeioRoboticsAssociation/f446_hal_microros.git $project_name
    cd $project_name
    bash ./.install.sh $project_name
}
```

### windows (powershell)

以下の内容をバッチファイルに`f446_hal.bat`として保存し、パスを通す
```bat
@echo off
setlocal enabledelayedexpansion

rem Check if the project name is provided as an argument
if "%~1"=="" (
    set /p project_name="Project name: "
) else (
    set project_name=%~1
)

rem Clone the Git repository
git clone git@github.com:KeioRoboticsAssociation/f446_hal_microros.git %project_name%
if errorlevel 1 (
    echo Failed to clone repository
    exit /b 1
)

rem Change directory to the project directory
cd %project_name%

rem Run the install script
if exist .install.bat (
    call .install.bat %project_name%
) else (
    echo .install.bat not found.
    exit /b 1
)

endlocal
```

## 使い方

- 次のようにしてプロジェクトを作成する。
```bash
f446_hal [プロジェクト名]
```

- 中身の.iocをCubeMXで開いてCode Generateを行う。

- STM32 VSCode Extensionでプロジェクトを開く。

(以上の手順をCLI上で一発で行う方法を知っている人がいたら教えてください)

## Libディレクトリ

- `Air`: GPIO駆動のエアソレノイド制御など空圧まわりのドライバ類。
- `Button`: ユーザーボタンの入力を受け取るクラス。
- `Control`: エンコーダ・モータ用のPI/PID制御器や単位変換ヘルパー。
- `DCMotor`: PWMと方向ピンでDCモータを駆動するドライバ。
- `Encoder`: TIMエンコーダモードを使った位置・速度計測クラス。
- `Perf`: DWTで処理時間/周期を測り計算負荷を確認するクラス。
- `RogiLinkFlex`: rogilinkFlex-halリポジトリと同じ。
- `Servo`: PWMサーボの角度指定や減速比設定ができるユーティリティ。

## その他

- みんながよく使うライブラリはこのテンプレートリポジトリの`Lib`ディレクトリに勝手に追加してもらって構いません。(衝突しないように)
