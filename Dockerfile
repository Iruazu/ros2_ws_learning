# ベースイメージとして、公式のROS 2 Humbleデスクトップイメージを使用
FROM osrf/ros:humble-desktop

# 必要なパッケージをインストール
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-bringup

# rootユーザーのホームディレクトリを作業ディレクトリに設定
WORKDIR /root

# コンテナ起動時に常に実行されるコマンド
CMD ["/bin/bash"]