# A. インストール手順
~~~
cd ~/catkin_ws/src/
git clone https://github.com/YOODS/rqt_param_manager
pip install pyyaml
~~~

# B. エンドユーザによる使い方

## B-1. 起動
クライアントとしてRoVIを使用する例：  
※RoVI commit 45c7c96により、現時点では使えなくなっているので注意。45c7c96の一つ前の10fb908に戻せば使える。  
~~~
roscd rovi
git pull origin ramiel
# RoVIの起動 (vgaをsxgaへ読み替え可能。以下同様。)
roslaunch rovi ycam3vga.launch
# 本ツールの起動
roslaunch rovi ycam3vga_livecamera_pm.launch
roslaunch rovi ycam3vga_pshift_pm.launch
~~~

## B-2. 動作、確認
rvizのライブ画面や rosservice call /rovi/pshift_genpc での点群生成で、rqt_param_managerが効いているのが確認できる。


# C. 本ツールのクライアントの作成手順
RoVI以外のパッケージ（仮に xxx とする）をクライアントにする場合の手順は以下。  
(前提として、ROS Parameter Namespace は xxx とする。)  
(例として、パラメータグループA, Bの2つとする。)  

1. ~/catkin_ws/src/xxx/pmjson/ にPM設定ファイル (A_pm.json, B_pm.json) を用意する。(RoVIのをコピーして編集すればよい。)

　dumpYamlの値とparamsの中身を適切に書き換える。

　　YOODS社内規約として、  
　　dumpYamlの値は yaml/*_ld.yaml の * 部分を書き換えればよい。  
　　（yaml/A_ld.yaml, yaml/B_ld.yamlへ。）  

　　paramsの中身はパラメータグループとその構成Parameter pathで決まる。  
　　（paramNameの値の前に、内部的には /xxx/ が付くことに注意。）

2. 上記dumpYamlの値ファイル (~/catkin_ws/src/xxx/yaml/{A,B}_ld.yaml) を用意する。(RoVIのをコピーして編集すればよい。)

　中身も適切に。

　　新規だと関係ないが、RoVIのように既存の yaml/ycam3vga.yaml があるなら、その中から  
　　~/catkin_ws/src/xxx/yaml/{A,B}_ld.yaml  
　　とかぶるパラメータを削除しておくことも忘れずに。  
　　（削除しておかないと次の手順↓で上書きloadされてしまうので。）

3. クライアントのlaunch（やそこからのsh）の中で、  
~~~
ROS_NAMESPACE=/xxx rosparam load yaml/A_ld.yaml
ROS_NAMESPACE=/xxx rosparam load yaml/B_ld.yaml
ROS_NAMESPACE=/xxx rosparam load yaml/CLIENT_BASIC.yaml
~~~
　相当のことをする。

4. パラメータ編集ツールのlaunchファイル (~/catkin_ws/src/xxx/launch/{A,B}_pm.launch) を用意する。(RoVIのをコピーして編集すればよい。)

　例えば  
　https://github.com/YOODS/rovi/blob/ramiel/launch/ycam3vga_livecamera_pm.launch  
　をコピーして、  
~~~
<launch>
  <env name="PMCLIENT_PKG_PATH" value="$(find rovi)" />
  <node ns="rovi" name="ycam3vga_livecamera_pm" pkg="rqt_param_manager" type="rqt_param_manager" args="conffile:=$(find rovi)/pmjson/ycam3vga_livecamera_pm.json" />
</launch>
~~~
　の「rovi」3箇所を「xxx」に書き換え、  
　&lt;node&gt;のnameの値をA_pmに書き換え、  
　conffileの値のycam3vga_livecamera_pm.jsonをA_pm.jsonに書き換え。

5. 後は以下で実行できる。
- クライアントのlaunch実行
- パラメータ編集ツールAのlaunch実行
- パラメータ編集ツールBのlaunch実行

