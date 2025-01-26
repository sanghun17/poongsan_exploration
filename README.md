## 설치 및 빌드 가이드

## Dependencies

### Protobuf 설치 확인

`protoc --version` 명령어를 실행했을 때, 출력값이 `2.6.1`이어야 합니다.  
만약 `2.6.1`이 아닌 다른 버전이 출력된다면, 아래 과정을 따라 Protobuf 2.6.1 버전을 설치하세요.

```bash
cd ~
wget https://github.com/protocolbuffers/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gz
tar -xvzf protobuf-2.6.1.tar.gz
cd protobuf-2.6.1
sudo apt update
sudo apt install -y build-essential autoconf automake libtool curl make g++ unzip
./configure
make -j$(nproc)
sudo make install
sudo ldconfig
export PATH=/usr/local/bin:$PATH # 소스에서 설치한 2.6.1 버전 우선화
```

## 워크스페이스 설정 및 빌드

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/sanghun17/poongsan_exploration
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```
