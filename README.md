# Tohoku
kibo-RPCのチームTohokuのrepositryです

## Prepare
* ubuntu 16.04
は導入済みとする
ここに書いてるのはすべてこの環境上の話である。windowsはわかりません

### app build
アプリをビルドするためにはここを見る

```sh
sudo apt-get -y install openjdk-8-jdk adb gradle vim 
```
でインストール。その後
Android Studioを入れ,26系を入れる

### testing build
ビルドしたアプリを動かしてみる
以下を`.bashrc`等に書いて`source .bashrc`をする
```sh
export SOURCE_PATH=$HOME/freeflyer
export ANDROID_PATH="${SOURCE_PATH}_android"
export BUILD_PATH=$HOME/freeflyer_build/native
export INSTALL_PATH=$HOME/freeflyer_install/native
export EMULATOR=$HOME/Android/Sdk/emulator/emulator
export AVD="AstrobeeAndroidSim"
export TOHOKU=$HOME/Workspaces/tohoku
export TOHOKU_APK_PASS=$HOME/Workspaces/tohoku
export ROS_IP=$(getent hosts llp | awk '{ print $1 }')
export ROS_MASTER_URI=http://${ROS_IP}:11311
```
Workspacesと書いているところにこのリポジトリをクローンしてきたのでこのようになってるが、特に指定等はない。好きに変えてください。

#### first step

動かすためのシュミレータを導入する
```sh
sudo apt-get install build-essential git

# download(clone)
git clone https://github.com/nasa/astrobee.git $SOURCE_PATH
git clone https://github.com/nasa/astrobee_android.git $ANDROID_PATH

# version select
pushd $SOURCE_PATH
git checkout 70e3df03ff3f880d302812111d0107f3c14dccc0
popd
pushd $ANDROID_PATH
git checkout 5b07e4d626781a6f7e0a9cdf4397375cbe509803
popd

# package install
pushd $SOURCE_PATH
cd scripts/setup
./add_ros_repository.sh
sudo apt-get update
cd debians
./build_install_debians.sh
cd ../
./install_desktop_16_04_packages.sh
sudo rosdep init
rosdep update
popd

# build
pushd $SOURCE_PATH
./scripts/configure.sh -l -F -D
popd
pushd $BUILD_PATH
make -j2
popd
```

以下で実行可能となる
```sh
pushd $BUILD_PATH
source devel/setup.bash
popd
roslaunch astrobee sim.launch dds:=false robot:=sim_pub rviz:=true
```

### second step
これはガイドの32-33ページ目を見たほうがいい
https://jaxa.krpc.jp/download/files/Kibo-RPC_PGManual.pdf

ここでは34ページの `Setting up the Android network and starting the Android Emulator` から説明する

これはandroidスマホのエミュレーターが立つためのコマンドです。
```
cd $ANDROID_PATH/scripts
sudo -E ./launch_emulator.sh -n
```
4.6.7のinstalling apksとは地続きで、このエミュレータを起動したまま行いましょう

これでAPKのインストールをします
```
cd $ANDROID_PATH/core_apks/guest_science_manager
adb install -g -r activity/build/outputs/apk/activity-debug.apk
cd $TOHOKU_APK_PASS
adb install -g -r app/build/outputs/apk/app-debug.apk
```
```
pwd # check eq echo $HOME
wget https://jaxa.krpc.jp/download/files/Kibo-RPC_SimExtMod.zip
unzip Kibo-RPC_SimExtMod.zip
cd sim_extension
chmod +x setup.sh
./setup.sh
```

TODO:: shell script type run