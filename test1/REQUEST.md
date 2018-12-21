# rqt_param_manager 改変

1. 起動
~~~
[ROS_NAMESPACE=test] rqt_param_manager [conf:=<config file>] [dump:=<yaml file>]
~~~
- config fileは項目など設定ファイル。*省略時はカレントの[D|d]efault.pmconfを使う*
- yaml fileは「保存」にて書き込まれるyamlファイル。*保存先に存在しないパラメータは追加しないこと*

2. Config file
- テキスト形式のファイルにて、１行が表示項目の１行に対応する。
- 項目分類は以下の３種類
  - パラメータ(Number,String,Array??)
  - サービス(std_srvs/Triggerのみ)
  - コメント
- 各書式について

3. UI

||1列目|2列目|3列目|
|:----|:----|:----|:----|
|1|コメント|
|2|名称|数値||
|3|名称|文字||
|4|名称|ファイル名|[選ぶ]|
|5|名称|[実行]||
