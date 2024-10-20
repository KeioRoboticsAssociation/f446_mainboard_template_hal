if [ $# -ne 1 ]; then
    echo "Usage: ./.install.sh [project_name]"
    exit 1
fi

cd $(dirname $0)

# プロジェクト名を取得
project_name=$1

# f446_template_hal.iocの中身のf446_template_halをプロジェクト名に置き換える
mv ./f446_template_hal.ioc ./${project_name}.ioc
sed 's/f446_template_hal/'${project_name}'/g' ./${project_name}.ioc > ./${project_name}_tmp.ioc
mv ./${project_name}_tmp.ioc ./${project_name}.ioc
rm -rf ./${project_name}_tmp.ioc

# CMakeLists.txtの中身のf446_template_halをプロジェクト名に置き換える
sed 's/f446_template_hal/'${project_name}'/g' ./CMakeLists.txt > ./CMakeLists_tmp.txt
mv ./CMakeLists_tmp.txt ./CMakeLists.txt
rm -rf ./CMakeLists_tmp.txt

# .install.shを削除
rm -rf ./.install.sh
rm -rf ./.install.bat

# README.mdの中身を空にする
echo "" > ./README.md

# .gitを削除
rm -rf ./.git
