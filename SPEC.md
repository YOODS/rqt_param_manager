# rqt_param_manager 設計変更

## 1. 起動
~~~
[ROS_NAMESPACE=<prefix>] rqt_param_manager [conf:=<config file>] [dump:=<yaml file>]
~~~
- config fileは項目など設定ファイル。*省略時はカレントの[D|d]efault.pmconfを使う*
- config fileのパラメータ名が **/** 以外で始まるときは、*prefix* をその前に結合します。

## 2. Config file
- テキスト形式のファイルにて、１行が表示項目の１行に対応する。
- 項目分類は以下の３種類
  - タイトル
  - パラメータ(Number,String,Array??)
  - トピック
  - サービス(std_srvs/Triggerのみ)
  
### 2-1. 画面表示  

  rqt_param_managerの画面表示例を示します。
表示項目は、第1列がLabel、第2列がInput、第3列がButton、に分類されます。
<table>
<tr><th>第1列<th>第2列<th>第3列
<tr><td colspan="3">位相シフト用パラメータ
<tr><td align="right">再投影誤差<td>0.222
<tr><td align="right">位置 X<br>Y<br>Z<td>100<br>200<br>300
<tr><td align="right">露光時間(usec)<td>10000
<tr><td align="right">カメラID<td>YOODS
<tr><td align="right">モデルデータファイル<td>model1.ply<td>＜一覧＞
<tr><td align="right">3D撮像<td><td>＜実行＞
</table>

この表示に従いconfig fileの書式を示す。

### 2-2. ConfigFile共通仕様  
Config fileは","で区切られた以下書式のテキストファイルです。
~~~
"<項目>:[<名前>]","表示内容"[,"オプション"]
~~~
"表示内容"が第1列Labelに表示されます。下記のようにこれに改行コード"\n"が含まれていた場合は、
~~~
"Echo:/robot/tf","位置 X\nY\nZ"
~~~
表示例3行目のように、行高さが(\n個数+1)倍されます。  
表示内容は原則的にLabel欄に**右寄せ表示**されます。

### 2-3. タイトルの表記  
表示例の１行目がタイトルです。タイトルはUIをわかりやすくするもので、＜保存＞ボタン操作の対象外です。  
タイトルの表示のみ、**１〜３列に亘り左寄せ表示**されます。  
この表示に対応するConfig fileでの記述は下記となります。
~~~
"Title:","位相シフト用パラメータ"
~~~
Config fileの１行目のタイトルはWindowのタイトルになります。Contentsに$(*XXX*)が含まれるときは、環境変数*XXX*で置換します。以下記述例です。
~~~
"Title:","ロボットグループ$(ROS_NAMESPACE)"
~~~
ROS_NAMESPACE=Robo1であれば、以下のように表示されます。
<table>
<tr><td>ロボットグループRobo1
</table>

### 2-4. トピックの表記  
表示例の2行目がトピックです。トピックも＜保存＞ボタン操作の対象外です。  
この表示に対応するConfig fileでの記述は下記となります。
~~~
"Echo:gridboard/error","再投影誤差"
~~~  
トピックの内容表示は、そのトピックの__str__プロパティの値をInput欄に表示します。複数の値を持つトピックでは、単純に複数行に亘りその値を表示します(表示例3行目)。

**トピックの購読(subscribe)にsubscriberコールバックを用いる場合** 、メッセージの型が必要になる。  
メッセージ型はrostoicを子プロセスで呼べば
~~~
rostopic info <topic名>
~~~
で分かる。例えば
~~~
rostopic info /robot/tf
~~~
だと
~~~
Type: geometry_msgs/Transform
~~~
となります。Type以下をパースして。  
msgclass="geometry_msgs"
msgtype="Transform"
としInterpreterにパースさせます。
~~~
exec("from "+msgclass+"import "+msgtype)
~~~
さらに以下をInterpreterにパースさせる。
~~~
exec("rospy.subscriber("+topic名+","+msgtype+",callback")")  
~~~

### 2-5. 数値パラメータの表記
表示例の4行目が数値パラメータです。  
この表示に対応するConfig fileでの記述は下記となります。
~~~
"Number:camera/ExposureTime","露光時間(usec)"
~~~

第2列を編集中は文字色が**赤**となり、変更イベントで値の書込が完了すると黒に戻ります(＜更新＞ボタンは削除)。

また**Number:**が省略されている時は数値パラメータと解釈されます。よって下記は同じ結果となります。
~~~
"camera/ExposureTime","露光時間(usec)"
~~~
数値パラメータには、将来的には、**表示ゲイン、表示フォーマット、入力上下限値**が追加される予定です。

### 2-6. 文字パラメータの表記  
表示例の5行目が文字パラメータです。  
この表示に対応するConfig fileでの記述は下記となります。
~~~
"Text:camera/ID","カメラID"
~~~
### 2-7. ファイルパラメータの表記(*この機能は2018.12時点では未実装でよい*)  
表示例の6行目がファイルパラメータです。  
ファイルパラメータは文字パラメータの一種ですが、入力手段がキー以外にファイルリストから選べるようになったものです。
この表示に対応するConfig fileでの記述は下記となります。
~~~
"File:solver/model_name","モデルデータファイル","data/*.ply"
~~~

＜一覧＞ボタンを押すと、ダイアログが開き、"data/*.ply"規則に合致するファイル一覧から入力文字を選択できます。

### 2-8. パブリッシャの表記
表示例の7行目がパブリッシャです。  
パブリッシャは、パラメータではなくトピックにパブリッシュ(std_msgs/Bool)します。
この表示に対応するConfig fileでの記述は下記となります。
~~~
"Publish:/rovi/X1","3D撮像"
~~~
＜実行＞ボタンを押すと、std_msgs/Boolをpublishします。  
「確認」のダイアログが出た方が親切かもしれない。


## 3.パラメータの保存  
起動パラメータにdump:=...にてyamlファイルが指定されているときは、＜保存＞ボタンの操作により現在のパラメータを当該ファイルに書き込みます。現行仕様との差異は以下
- 保存先に存在しないパラメータは追加しない
