# -*- coding: utf-8 -*-

# ================ インポート一覧 ================
import sys
import os
import rospy
import rospkg
import string

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import QTimer, QVariant,QPoint
from python_qt_binding.QtWidgets import *
#from python_qt_binding.QtWidgets import (
#    QWidget,
#    QTableWidgetItem,
#    QItemDelegate,
#    QHeaderView,
#    QMessageBox,
#    QPushButton,
#    QLabel
#)
from config_item import *

# ================ 定数一覧 ================
FILE_ENC = "utf-8"
INVALID_VAL = "---"
TBL_COL_LABEL = 0
TBL_COL_INPUT = 1
TBL_COL_ACTION = 2
KEY_CONFFILE_TITLE = "title"
KEY_CONFFILE_GET_INTERVAL = "getInterval"
KEY_CONFFILE_DUMP_YAML = "dumpYaml"
KEY_CONFFILE_PARAMS = "params"
KEY_CONFFILE_PARAM_NM = "paramName"
KEY_CONFFILE_PARAM_DISP = "paramDisp"
FILE_DEFAULT_PM_CONFS = ["default.pmconf","Default.pmconf"]
KEY_ENV_ROS_NAMESPACE = "ROS_NAMESPACE"

# ================ クラス一覧 ================
class NotEditableDelegate(QItemDelegate):
    """特定の列のセルを編集不可にする為に使用するDelegateクラス"""

    def __init__(self, *args):
        """初期化処理"""

        super(NotEditableDelegate, self).__init__(*args)

    def createEditor(self, parent, option, index):
        """エディタ作成処理"""

        return None

    def editorEvent(self, event, model, option, index):
        """エディタイベント処理"""

        return False


class RqtParamManagerPlugin(Plugin):
    """UIのメインクラス"""

    def __init__(self, context):
        """初期化処理"""

        super(RqtParamManagerPlugin, self).__init__(context)

        # クラス変数初期化
        self._title = "rqt_param_manager"
        self._get_interval = 1000
        self._monitor_timer = QTimer()
        self._conf_file_path_list = FILE_DEFAULT_PM_CONFS
        self._dump_yaml_file_path = ""
        self._config_item_list = []
        self._ros_namespace = os.environ.get(KEY_ENV_ROS_NAMESPACE,"")
        
        self.setObjectName('RqtParamManagerPlugin')

        self._parse_args(sys.argv)
        
        ## result_load_conf = self._load_conf_file(sys.argv)
        
        if( len(self._ros_namespace) > 0 and self._ros_namespace[:-1] != "/" ):
            self._ros_namespace = self._ros_namespace + "/"
        
        #print(sys.argv)
        #print(self._conf_file_path_list )
        #print("dump_yaml_file_path="+self._dump_yaml_file_path)
        
        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(
            rospkg.RosPack().get_path('rqt_param_manager'),
            'resource',
            'RqtParamManagerPlugin.ui'
        )

        loadUi(ui_file, self._widget)
        self._widget.setObjectName('RqtParamManagerPluginUi')

        context.add_widget(self._widget)

        self._setup_params_table(self._widget.tblConfigItems)
        
        result_load_conf=self._parse_conf_file(self._conf_file_path_list,self._config_item_list);


        QTimer.singleShot(0, self._update_window_title)
            
        if not result_load_conf:
            #self._widget.btnUpdate.setEnabled(False)
            self._widget.btnSave.setEnabled(False)
        else:

            # bind connections
            #self._widget.btnUpdate.clicked.connect(self._on_exec_update)
            self._widget.btnSave.clicked.connect(self._on_exec_save)
            self._monitor_timer.timeout.connect(self._on_period_monitoring)

            self._load_config_table_item(self._widget.tblConfigItems, self._config_item_list)

            # テーブル行とパラメータ数のチェック
            table_row_num = self._widget.tblConfigItems.rowCount()
            
            #---一時コメント
#            param_num = len(self._params)
#            if table_row_num != param_num or param_num == 0:
#                self._widget.btnUpdate.setEnabled(False)
#                self._widget.btnSave.setEnabled(False)

            # 定期監視処理の開始
            if self._get_interval > 0:
                self._on_period_monitoring()
                rospy.loginfo(
                    "start monitor. interval=%d sec",
                    self._get_interval
                )
                self._monitor_timer.start(self._get_interval)

    def _update_window_title(self):
        """ウィンドウタイトルを変更する処理"""

        # 初期化処理内でsetWindowTitleを呼んでも変更されないので
        self._widget.setWindowTitle(self._title)

        # serial_number = context.serial_number()
        # if serial_number > 1:
        #    self._widget.setWindowTitle(
        #        self._widget.windowTitle() + (' (%d)' % serial_number))

    def _setup_params_table(self, table):
        """パラメータテーブル設定処理"""

        # 列は3列
        table.setColumnCount(3)

        # 列1,2は編集不可
        no_edit_delegate = NotEditableDelegate()
        table.setItemDelegateForColumn(TBL_COL_LABEL, no_edit_delegate)
        table.setItemDelegateForColumn(TBL_COL_ACTION, no_edit_delegate)

        # ヘッダー列の設定
        headerCol1 = QTableWidgetItem()
        headerCol1.setText("ラベル")
        table.setHorizontalHeaderItem(TBL_COL_LABEL, headerCol1)

        headerCol2 = QTableWidgetItem()
        headerCol2.setText("Input")
        table.setHorizontalHeaderItem(TBL_COL_INPUT, headerCol2)

        headerCol3 = QTableWidgetItem()
        headerCol3.setText("ボタン")
        table.setHorizontalHeaderItem(TBL_COL_ACTION, headerCol3)

        header = table.horizontalHeader()
        header.setSectionResizeMode(TBL_COL_LABEL, QHeaderView.Stretch)
        header.setSectionResizeMode(TBL_COL_INPUT, QHeaderView.Fixed)
        header.setSectionResizeMode(TBL_COL_ACTION, QHeaderView.Fixed)
        table.setColumnWidth(TBL_COL_INPUT, 120)
        table.setColumnWidth(TBL_COL_ACTION, 120)

        table.verticalHeader().hide()

    def shutdown_plugin(self):
        """シャットダウン処理"""

        self._monitor_timer.stop()

        """
        # UIが終了してもrosparamに「/rqt_gui_py_node_<no>/conffile」
        # が残っているので削除。この処理が必要なのかどうか不明だが。
        # まぁやっておく。
        self_ros_param_names = [s for s in rospy.get_param_names()
                                if rospy.get_name() in s]
        if len(self_ros_param_names):
            for self_ros_param_name in self_ros_param_names:
                rospy.delete_param(self_ros_param_name)
        """

    def save_settings(self, plugin_settings, instance_settings):
        """設定保存処理"""

        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        """設定復帰処理"""

        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way
        # to configure
        # This will enable a setting button (gear icon)
        # in each dock widget title bar
        # Usually used to open a modal configuration dialog
    
    def _parse_args(self,argv):
        """引数パース処理"""

        for arg in sys.argv:
            tokens = arg.split(":=")
            if len(tokens) == 2 :
                key = tokens[0]
                if( "conf" == key):
                    self._conf_file_path_list= [tokens[1]]
                elif( "dump" == key ):
                    self._dump_yaml_file_path=tokens[1]
                
    def _parse_conf_file(self, conf_file_path_list , items):
        """PM設定ファイル解析処理"""

        result = False

        for conf_file_path in conf_file_path_list:
            rospy.loginfo("load conf file. path=%s", conf_file_path)
            
            try:
                file = open(conf_file_path, 'r')
                for line in file:
                    line = line.strip()
                    if( len(line) == 0 or line[0] == "#" ):
                        #print("invalid or comment line. line=" + line)
                        continue
                    elif ( line.startswith('"Title:",') ):
                        tokens=line.split(",")
                        if( len(tokens) == 2 ):
                            self._title = ConfigItem.trim(tokens[1])
                            try:
                                self._title=string.Template(self._title).substitute(os.environ)
                            except :
                                rospy.logerr("title replace failed. %s", self._title)
                        continue
                        
                    item=ConfigItem()
                    item.prefix=self._ros_namespace
                    if( not item.parse(line) ):
                        rospy.logerr("conf file wrong line. %s", line)
                    else:
                        print("[%02d] %s" % (len(items),item.toString()))
                        items.append(item)

                file.close()
                result = True
            except IOError as e:
                rospy.logerr("conf file load failed. %s", e)
                
            if result :
                break
        
        return result

    def _load_config_table_item(self, table, items):
        """パラメータテーブル項目読込処理"""

        item_num = len(items)
        table.setRowCount(item_num)
        
        model=table.model()
        
        n = 0
        for item in items:
            table.setItem(n, TBL_COL_LABEL, QTableWidgetItem(item.label))

            if ( ITEM_TYPE_ECHO == item.type ):
                if ( len(item.topic) > 0 ):
                    print("TODO:topic")
                    table.setItem(
                        n,
                        TBL_COL_INPUT,
                        QTableWidgetItem(INVALID_VAL)
                    )
                else:
                    table.setSpan(n,TBL_COL_LABEL,1,3)
            elif( ITEM_TYPE_FILE == item.type ):
                lbl = QLabel();
                table.setIndexWidget(model.index(n,TBL_COL_INPUT), lbl);
                
                btn = QPushButton();
                btn.setText("一覧");
                table.setIndexWidget(model.index(n,TBL_COL_ACTION), btn);
                btn.clicked.connect(item._on_action)

            elif( ITEM_TYPE_TRIGGER == item.type ):
                lbl = QLabel();
                table.setIndexWidget(model.index(n,TBL_COL_INPUT), lbl);
                
                btn.clicked.connect(item._on_action)
                btn = QPushButton();
                btn.setText("実行");
                table.setIndexWidget(model.index(n,TBL_COL_ACTION), btn);
                btn.clicked.connect(item._on_action)
                
                item.invoke_trigger.connect(self._on_exec_trigger)
            else:
                table.setItem(
                    n,
                    TBL_COL_INPUT,
                    QTableWidgetItem(INVALID_VAL)
                )
            n += 1

    def _on_period_monitoring(self):
        """定期監視処理"""

        table = self._widget.tblConfigItems
        item_num = len(self._config_item_list)

        for n in range(item_num):
            item = self._config_item_list[n]
            
            if( item.type == ITEM_TYPE_NUMBER or item.type == ITEM_TYPE_TEXT):
                val = INVALID_VAL
                if( len (item.param) > 0 ):
                    try:
                        val = rospy.get_param(item.param)
                    except KeyError as e:
                        # エラーに出すと数がすごいことになりそうなので出さない
                        pass

                table.setItem(
                    n,
                    TBL_COL_INPUT,
                    QTableWidgetItem("%s" % val)
                )

#    def _on_exec_update(self, from_save=False):
#        """パラメータ更新実行処理"""
#        
#        result = False
#        table = self._widget.tblConfigItems
#        row_num = table.rowCount()
#
#        upd_num = 0
#        ok_num = 0
#        for n in range(row_num):
#            cur_val = table.item(n, TBL_COL_INPUT).text()
#            # upd_val = table.item(n, TBL_COL_ACTION).text()
#
#            if cur_val == upd_val or \
#               INVALID_VAL == upd_val or \
#               len(upd_val) <= 0:
#                pass
#            else:
#                param = self._params[n]
#                upd_num += 1
#
#                import yaml
#                try:
#                    param_nm = param[KEY_CONFFILE_PARAM_NM]
#                    param_type = type(rospy.get_param(param_nm))
#                    if (param_type is int):
#                        upd_val = int(upd_val)
#                    elif (param_type is float):
#                        upd_val = float(upd_val)
#                    elif (param_type is list):
#                        upd_val = yaml.load(upd_val)
#                    rospy.set_param(param_nm, upd_val)
#                    rospy.loginfo("param_nm=%s val=%s", param_nm, upd_val)
#                    ok_num += 1
#                except (KeyError, ValueError) as e:
#                    rospy.logerr(
#                        "update failed. paramNo=%d cause=%s",
#                        n,
#                        e
#                    )
#
#        if upd_num != ok_num:
#            if not from_save:
#                QMessageBox.critical(self._widget, "エラー", "パラメータの更新に失敗しました。")
#        else:
#            result = True
#        return result

    def _on_exec_save(self):
        """パラメータ保存実行処理"""

        self._monitor_timer.stop()
        self._widget.setEnabled(False)

        if not self._on_exec_update(True):
            QMessageBox.critical(self._widget, "エラー", "パラメータの更新と保存に失敗しました。")
            self._monitor_timer.start()
            self._widget.setEnabled(True)
            return

        import rosparam
        try:
            rosparam.dump_params(self._dump_yaml_file_path, rospy.get_namespace())
            QMessageBox.information(self._widget, "お知らせ", "設定を保存しました。")
        except IOError as e:
            rospy.logerr("dump failed. %s", e)
            QMessageBox.critical(self._widget, "エラー", "保存に失敗しました。")

        self._monitor_timer.start()
        self._widget.setEnabled(True)
        
    def _on_exec_trigger(self,trigger):
        print("trigger=" + trigger )
        
