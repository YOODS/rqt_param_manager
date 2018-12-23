# rqt_param_manager 設計変更

1. 起動
~~~
[ROS_NAMESPACE=test] rqt_param_manager [conf:=<config file>] [dump:=<yaml file>]
~~~
- config fileは項目など設定ファイル。*省略時はカレントの[D|d]efault.pmconfを使う*
- yaml fileは「保存」にて書き込まれるyamlファイル。*保存先に存在しないパラメータは追加しないこと*
- config fileのパラメータ名が**”/”**以外で始まるときは、環境変数ROS_NAMESPACEで与えられた名前をその前に結合します。

2. Config file
- テキスト形式のファイルにて、１行が表示項目の１行に対応する。
- 項目分類は以下の３種類
  - パラメータ(Number,String,Array??)
  - サービス(std_srvs/Triggerのみ)
  - コメント
  
3. 数値パラメータの表記  
Config fileでの数値パラメータの記述を下記に例示します。
~~~
"Number:camera/ExposureTime","露光時間(usec)"
~~~
このときの画面表示は以下のようになります。
<table border>
<tr><th>露光時間(usec)<td>10000
</table>
値を変更した時は表示色が変わります。
<table border>
<tr><th>露光時間(usec)<td><font color="red">8000</font>
</table>
この状態ではパラメータは未だ反映されていません。<button>更新</button>を押すことで変更されたすべてのパラメータが反映されます。
パラメータ名の前の*Number:*が省略されている時は数値パラメータと解釈されます。下記は同じ結果となります。
~~~
"camera/ExposureTime","露光時間(usec)"
~~~
数値パラメータには、将来的には、**表示ゲイン、表示フォーマット、入力上下限値**が追加される予定です。

4. 文字パラメータの表記  
Config fileでの文字パラメータの記述を下記に例示します。
~~~
"Text:camera/ID","カメラID"
~~~
このときの画面表示は以下のようになります。
<table border>
<tr><th>カメラID<td>YOODS
</table>

5. ファイルパラメータの表記(*この機能は2018.12時点では未実装でよい*)  
ファイルパラメータは文字パラメータに分類されますが、入力手段がキー以外にファイルリストから選べるようになったものです。
Config fileでのファイルパラメータの記述を下記に例示します。
~~~
"File:solver/model_name","モデルデータファイル","data/*.ply"
~~~
このときの画面表示は以下のようになります。
<table border>
<tr><th>モデルデータファイル<td>model1.ply<td><button>一覧</button>
</table>
<button>一覧</button>ボタンを押すと、ダイアログが開き、"data/*.ply"規則に合致するファイル一覧から入力文字を選択できます。

6. コメントの表記  
コメントはUIをわかりやすくするもので、<button>更新</button>や<button>保存</button>操作の対象外です。  
Config fileでのコメント行の記述を下記に例示します。
~~~
"Rem:位相シフト用パラメータ"
~~~
このときの画面表示は以下のようになります。
<table border>
<tr><th bgcolor="##00AAAA">位相シフト用パラメータ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
</table>

7. トリガーの表記
トリガーは、パラメータではなくサービスコール(std_srvs/Trigger)を発生させます。
Config fileでのトリガーの記述を下記に例示します。
~~~
"Trigger:pshift_genpc","3D撮像"
~~~
このときの画面表示は以下のようになります。
<table border>
<tr><th><button>3D撮像</button>
</table>
サービスコールの結果はアラートに表示されます。
<table border>
<tr><td><table>
<tr><th>成否<td>OK(True)
<tr><th>メッセージ<td>400000 points scanned
</table>
<br><div align="center"><button>Close</button></div>
</table>
