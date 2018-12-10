# -*- coding: utf-8 -*-

# ================ インポート一覧 ================
import sys
import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding.QtCore import QTimer, QVariant
from python_qt_binding.QtWidgets import (
    QWidget,
    QTableWidgetItem,
    QItemDelegate,
    QHeaderView,
    QMessageBox,
)

# ================ 定数一覧 ================
FILE_ENC = "utf-8"
INVALID_VAL = "---"
TBL_COL_PARAM_NM = 0
TBL_COL_PARAM_CUR_VAL = 1
TBL_COL_PARAM_UPD_VAL = 2
KEY_CONFFILE_TITLE = "title"
KEY_CONFFILE_GET_INTERVAL = "getInterval"
KEY_CONFFILE_DUMP_YAML = "dumpYaml"
KEY_CONFFILE_PARAMS = "params"
KEY_CONFFILE_PARAM_NM = "paramName"
KEY_CONFFILE_PARAM_DISP = "paramDisp"


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
        self._title = "不明"
        self._get_interval = 0
        self._dump_yaml_file_path = ""
        self._params = {}
        self._monitor_timer = QTimer()

        self.setObjectName('RqtParamManagerPlugin')

        result_load_conf = self._load_conf_file(sys.argv)

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

        self._setup_params_table(self._widget.tblParams)

        if not result_load_conf:
            self._widget.btnUpdate.setEnabled(False)
            self._widget.btnSave.setEnabled(False)
        else:
            QTimer.singleShot(0, self._update_window_title)

            # bind connections
            self._widget.btnUpdate.clicked.connect(self._on_exec_update)
            self._widget.btnSave.clicked.connect(self._on_exec_save)
            self._monitor_timer.timeout.connect(self._on_get_params)

            self._load_params_table_item(self._widget.tblParams, self._params)

            # テーブル行とパラメータ数のチェック
            table_row_num = self._widget.tblParams.rowCount()
            param_num = len(self._params)
            if table_row_num != param_num or param_num == 0:
                self._widget.btnUpdate.setEnabled(False)
                self._widget.btnSave.setEnabled(False)

            # 定期監視処理の開始
            if self._get_interval > 0:
                self._on_get_params()
                rospy.loginfo(
                    "start monitor. interval=%d sec",
                    self._get_interval
                )
                self._monitor_timer.start(self._get_interval * 1000)

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
        table.setItemDelegateForColumn(TBL_COL_PARAM_NM, no_edit_delegate)
        table.setItemDelegateForColumn(TBL_COL_PARAM_CUR_VAL, no_edit_delegate)

        # ヘッダー列の設定
        headerCol1 = QTableWidgetItem()
        headerCol1.setText("パラメータ名")
        table.setHorizontalHeaderItem(TBL_COL_PARAM_NM, headerCol1)

        headerCol2 = QTableWidgetItem()
        headerCol2.setText("現在値")
        table.setHorizontalHeaderItem(TBL_COL_PARAM_CUR_VAL, headerCol2)

        headerCol3 = QTableWidgetItem()
        headerCol3.setText("更新値")
        table.setHorizontalHeaderItem(TBL_COL_PARAM_UPD_VAL, headerCol3)

        header = table.horizontalHeader()
        header.setSectionResizeMode(TBL_COL_PARAM_NM, QHeaderView.Stretch)
        header.setSectionResizeMode(TBL_COL_PARAM_CUR_VAL, QHeaderView.Fixed)
        header.setSectionResizeMode(TBL_COL_PARAM_UPD_VAL, QHeaderView.Fixed)
        table.setColumnWidth(TBL_COL_PARAM_CUR_VAL, 120)
        table.setColumnWidth(TBL_COL_PARAM_UPD_VAL, 120)

        table.verticalHeader().hide()

    def shutdown_plugin(self):
        """シャットダウン処理"""

        self._monitor_timer.stop()

        # UIが終了してもrosparamに「/rqt_gui_py_node_<no>/conffile」
        # が残っているので削除。この処理が必要なのかどうか不明だが。
        # まぁやっておく。
        self_ros_param_names = [s for s in rospy.get_param_names()
                                if rospy.get_name() in s]
        if len(self_ros_param_names):
            for self_ros_param_name in self_ros_param_names:
                rospy.delete_param(self_ros_param_name)

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

    def _load_conf_file(self, argv):
        """PM設定ファイル読込処理"""

        result = False

        if not len(sys.argv) > 1:
            rospy.logerr("argv '_conffile' is not specified.")
        else:
            tokens = sys.argv[1].split(":=")
            if len(tokens) == 2 and tokens[0] == "_conffile":
                conf_file_path = tokens[1]
                result = self._parse_conf_file(conf_file_path)
            else:
                rospy.logerr(
                    "argv '_conffile' is wrong format. %s",
                    sys.argv[1]
                )

        return result

    def _parse_conf_file(self, conf_file_path):
        """PM設定ファイル解析処理"""

        result = False

        rospy.loginfo("load conf file. path=%s", conf_file_path)
        import json
        try:
            f = open(conf_file_path, 'r')
            json_dict = json.load(f)
            self._title = json_dict[KEY_CONFFILE_TITLE]
            self._get_interval = json_dict[KEY_CONFFILE_GET_INTERVAL]
            self._dump_yaml_file_path = json_dict[KEY_CONFFILE_DUMP_YAML]

            rospy.loginfo("title=%s", self._title.encode(FILE_ENC))
            rospy.loginfo("getInterval=%s sec", self._get_interval)
            rospy.loginfo(
                "dumpYaml=%s",
                self._dump_yaml_file_path.encode(FILE_ENC)
            )

            self._params = json_dict[KEY_CONFFILE_PARAMS]

            result = True
        except IOError as e:
            rospy.logerr("json file load failed. %s", e)

        return result

    def _load_params_table_item(self, table, params):
        """パラメータテーブル項目読込処理"""

        param_num = len(params)
        table.setRowCount(param_num)
        n = 0
        for param in params:
            try:
                label = param[KEY_CONFFILE_PARAM_DISP]
                table.setItem(n, TBL_COL_PARAM_NM, QTableWidgetItem(label))
            except KeyError as e:
                table.setItem(n, TBL_COL_PARAM_NM, QTableWidgetItem("不明"))
                rospy.logerr("conf file key error. %s", e)

            table.setItem(
                n,
                TBL_COL_PARAM_CUR_VAL,
                QTableWidgetItem(INVALID_VAL)
            )
            table.setItem(n, TBL_COL_PARAM_UPD_VAL, QTableWidgetItem(""))
            n += 1

    def _on_get_params(self):
        """パラメータ取得処理"""

        param_num = len(self._params)
        for n in range(param_num):
            param = self._params[n]
            val = INVALID_VAL
            try:
                param_nm = param[KEY_CONFFILE_PARAM_NM]
                val = rospy.get_param(param_nm)
            except KeyError as e:
                # エラーに出すと数がすごいことになりそうなので出さない
                pass
            table = self._widget.tblParams
            table.setItem(
                n,
                TBL_COL_PARAM_CUR_VAL,
                QTableWidgetItem("%s" % val)
            )
            upd_val = table.item(n, TBL_COL_PARAM_UPD_VAL).text()
            # 無効データ取得 もしくは 初回データ更新
            if INVALID_VAL == val \
               or len(upd_val) == 0 \
               or (INVALID_VAL != val and upd_val == INVALID_VAL):
                table.setItem(
                    n,
                    TBL_COL_PARAM_UPD_VAL,
                    QTableWidgetItem("%s" % val)
                )

    def _on_exec_update(self):
        """パラメータ更新実行処理"""
        
        result = False
        table = self._widget.tblParams
        row_num = table.rowCount()

        upd_num = 0
        ok_num = 0
        for n in range(row_num):
            cur_val = table.item(n, TBL_COL_PARAM_CUR_VAL).text()
            upd_val = table.item(n, TBL_COL_PARAM_UPD_VAL).text()

            if cur_val == upd_val or \
               INVALID_VAL == upd_val or \
               len(upd_val) <= 0:
                pass
            else:
                param = self._params[n]
                upd_num += 1

                try:
                    param_nm = param[KEY_CONFFILE_PARAM_NM]

                    rospy.set_param(param_nm, upd_val)
                    rospy.loginfo("param_nm=%s val=%s", param_nm, upd_val)
                    ok_num += 1
                except KeyError as e:
                    rospy.logerr(
                        "update faild. paramNo=%d cause=%s",
                        n,
                        e
                    )

        if upd_num != ok_num:
            QMessageBox.warning(self._widget, "警告", "一部パラメータの更新に失敗しました。")
        else:
            result = True
        return result

    def _on_exec_save(self):
        """パラメータ保存実行処理"""

        self._monitor_timer.stop()
        self._widget.setEnabled(False)

        if not self._on_exec_update():
            self._monitor_timer.start()
            QMessageBox.critical(self._widget, "エラー", "パラメータの更新に失敗しました。")
            return

        import rosparam
        try:
            rosparam.dump_params(self._dump_yaml_file_path, "/")
            QMessageBox.information(self._widget, "お知らせ", "設定を保存しました。")
        except IOError as e:
            rospy.logerr("dump failed. %s", e)
            QMessageBox.critical(self._widget, "エラー", "保存に失敗しました。")

        self._monitor_timer.start()
        self._widget.setEnabled(True)
