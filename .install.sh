if [ $# -ne 1 ]; then
    echo "Usage: ./.install.sh [project_name]"
    exit 1
fi

cd $(dirname $0)

# プロジェクトの名前を変更
project_name=$1
mv ./f446_template_hal.ioc ./${project_name}.ioc
sed 's/f446_template_hal/'${project_name}'/g' ./${project_name}.ioc > ./${project_name}_tmp.ioc
mv ./${project_name}_tmp.ioc ./${project_name}.ioc
rm -rf ./${project_name}_tmp.ioc

# .install.shを削除
rm -rf ./.install.sh

# README.mdの中身を空にする
echo "" > ./README.md

# .gitを削除
rm -rf ./.git