## 概要

f446のHALライブラリを使ったプロジェクトのテンプレート。

## 導入方法
`~/.bashrc`に以下を追記しておく。
```bash
function f446_hal {
    if [ $# -eq 0 ]; then
        read -p "project name: " project_name
    else 
        project_name=$1
    fi
    git clone git@github.com:KeioRoboticsAssociation/f446_mainboard_template_hal.git $project_name
    cd $project_name
    bash ./.install.sh $project_name
}
```

## 使い方

- 次のようにしてプロジェクトを作成する。
```bash
f446_hal [プロジェクト名]
```

- 中身の.iocをCubeMXで開いてCode Generateを行う。

- STM32 VSCode Extensionでプロジェクトを開く。

(以上の手順をCLI上で一発で行う方法を知っている人がいたら教えてください)

## その他

- みんながよく使うライブラリはこのテンプレートリポジトリの`Lib`ディレクトリに勝手に追加してもらって構いません。(衝突しないように)