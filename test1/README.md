# rqt_param_manager テスト１

1. 起動
roviをramielブランチに切り替え、このディレクトリで
~~~
roslaunch test1.launch
~~~
rqt_param_managerは以下のように起動
~~~
ROS_NAMESPACE=test rqt_param_manager conffile:=pm.json
~~~
