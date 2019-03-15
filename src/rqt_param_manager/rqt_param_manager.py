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
from python_qt_binding.QtCore import QTimer, QVariant, QPoint
from python_qt_binding.QtWidgets import *
# from python_qt_binding.QtWidgets import (
#    QWidget,
#    QTableWidgetItem,
#    QItemDelegate,
#    QHeaderView,
#    QMessageBox,
#    QPushButton,
#    QLabel
# )
from std_msgs.msg import *

from config_item import *
from ros_topic_listener_thread import *
from ros_param_writer import *
from ui_rqt_param_manager_main import Ui_rqt_param_manager_main

# ================ 定数一覧 ================
INVALID_VAL = "---"
FILE_DEFAULT_PM_CONFS = ["default.pmconf", "Default.pmconf"]
KEY_ENV_ROS_NAMESPACE = "ROS_NAMESPACE"
ARG_DUMP = "dump"
ARG_CONF = "conf"
ARG_COLUMN_WIDTH_LABEL = "col_label_width"
ARG_WIN_WIDTH = "win_width"
ARG_WIN_HEIGHT = "win_height"

WID_PROP_TOPIC_NM = "topic_nm"

# ================ クラス ================


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
        self._config_items = []
        self._ros_namespace = os.environ.get(KEY_ENV_ROS_NAMESPACE, "")
        self._table_input_item_map = {}
        self._topic_listeners = []
        self._topic_data_map = {}
        self._prm_writer = None

        self.setObjectName('RqtParamManagerPlugin')

        args = {}
        self._parse_args(sys.argv, args)

        if(ARG_CONF in args):
            self._conf_file_path_list = args[ARG_CONF]

        if(len(self._ros_namespace) > 0 and self._ros_namespace[:-1] != "/"):
            self._ros_namespace = self._ros_namespace + "/"

        # Create QWidget
        self._widget = QWidget()
        self.ui = Ui_rqt_param_manager_main()
        self.ui.setupUi(self._widget)
        context.add_widget(self._widget)

        self.ui.tblConfigItems.initUI()

        self._initEnv(args)

        result_load_conf = self._parse_conf_file(self._conf_file_path_list,
                                                 self._config_items,
                                                 self._topic_data_map)

        QTimer.singleShot(0, self._update_window_title)

        self.ui.btnClose.clicked.connect(self._app_close)

        if not result_load_conf:
            self.ui.btnSave.setEnabled(False)
        else:
            # bind connections
            self.ui.btnSave.clicked.connect(self._on_exec_save)
            self._monitor_timer.timeout.connect(self._on_period_monitoring)

            self.ui.tblConfigItems.load_items(self._config_items)

            # テーブル行とパラメータ数のチェック
            # table_row_num = self.ui.tblConfigItems.rowCount()

            # ---一時コメント
#            param_num = len(self._params)
#            if table_row_num != param_num or param_num == 0:
#                self.ui.btnUpdate.setEnabled(False)
#                self.ui.btnSave.setEnabled(False)

            # 定期監視処理の開始
            if self._get_interval > 0:
                self._on_period_monitoring()
                rospy.loginfo(
                    "start monitor. interval =%d sec",
                    self._get_interval
                )
                self._monitor_timer.start(self._get_interval)

            self._start_topic_listen(self._config_items)

    def _initEnv(self, args):
        if(ARG_DUMP in args):
            self._dump_yaml_file_path = args[ARG_DUMP]
        else:
            self.ui.btnSave.setVisible(False)

        if(ARG_COLUMN_WIDTH_LABEL in args):
            val = args[ARG_COLUMN_WIDTH_LABEL]
            try:
                if(val.endswith("px")):
                    self.ui.tblConfigItems.colLabelWidthFixed = int(val[:-2])
                elif(val.endswith("%")):
                    self.ui.tblConfigItems.colLabelWidthRatio = int(val[:-1]) / 100.0
                else:
                    self.ui.tblConfigItems.colLabelWidthFixed = int(val)
            except Exception as e:
                rospy.logerr("label column width set failed. val=%s. cause=%s", val, e)

        """
        widSize = self._widget.size()

        if(ARG_WIN_WIDTH in args):
            val = args[ARG_WIN_WIDTH]
            try:
                if(val.endswith("px")):
                    widSize.setWidth(int(val[:-2]))
                else:
                    widSize.setWidth(int(val))
            except Exception as e:
                rospy.logerr("window width set failed. val=%s. cause=%s",val, e)

        if(ARG_WIN_HEIGHT in args):
            val = args[ARG_WIN_HEIGHT]
            try:
                if(val.endswith("px")):
                    widSize.setHeight(int(val[:-2]))
                else:
                    widSize.setHeight(int(val))
            except Exception as e:
                rospy.logerr("window height set failed. val=%s. cause=%s",val, e)

        if(self._widget.size() != widSize):
            print("org %dx%d. win:%dx%d" % (self._widget.size().width(),self._widget.size().height(),widSize.width(),widSize.height()))
            self._widget.setGeometry(0,0,300,300)
        """

    def _update_window_title(self):
        """ウィンドウタイトルを変更する処理"""

        # 初期化処理内でsetWindowTitleを呼んでも変更されないので
        self._widget.setWindowTitle(self._title)

        # serial_number = context.serial_number()
        # if serial_number > 1:
        #    self.ui.setWindowTitle(
        #        self.ui.windowTitle() + (' (%d)' % serial_number))

    def _app_close(self):
        self._monitor_timer.stop()
        QCoreApplication.quit()

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

    def _parse_args(self, argv, args):
        """引数パース処理"""
        for arg in sys.argv:
            tokens = arg.split(":=")
            if len(tokens) == 2:
                key = tokens[0]
                if(ARG_CONF == key):
                    args[ARG_CONF] = [tokens[1]]
                else:
                    args[key] = tokens[1]

    def _parse_conf_file(self, conf_file_path_list, items, topic_data_map):
        """PM設定ファイル解析処理"""

        result = False

        for conf_file_path in conf_file_path_list:
            rospy.loginfo("load conf file. path =%s", conf_file_path)

            try:
                file = open(conf_file_path, 'r')
                for line in file:
                    line = line.strip()
                    if(len(line) == 0 or line[0] == "#"):
                        # print("invalid or comment line. line =" + line)
                        continue

                    item = ConfigItem()
                    item.prefix = self._ros_namespace
                    if(not item.parse(line, topic_data_map)):
                        rospy.logerr("conf file wrong line. %s", line)
                    else:
                        # print("[%02d] %s" % (len(items), item.toString()))
                        if(ITEM_TYPE_TITLE == item.type and len(items) == 0):
                            self._title = item.label

                        items.append(item)

                file.close()
                result = True
            except IOError as e:
                rospy.logerr("conf file load failed. %s", e)

            if result:
                break

        return result

    def _on_period_monitoring(self):
        """定期監視処理"""

        table = self.ui.tblConfigItems
        item_num = len(self._config_items)

        for n in range(item_num):
            item = self._config_items[n]

            key = None
            try:
                key = [k for k,
                       v in self.ui.tblConfigItems._table_input_item_map.items()
                       if v is item]
            except Exception as err:
                print(err)
                key = None

            if(not key or len(key) == 0):
                continue

            key = key[0]

            if(item.type == ITEM_TYPE_NUMBER or item.type == ITEM_TYPE_TEXT):
                txt_edit = key

                val = INVALID_VAL
                if(len(item.param_nm) > 0):
                    try:
                        val = str(rospy.get_param(item.param_nm))
                    except Exception as err:
                        # print(err)
                        pass

                if(item.param_val != val):
                    item.param_val = str(val)
                    txt_edit.setText(item.param_val)
                    txt_edit.setStyleSheet('color: rgb(0, 0, 0);')

    def _start_topic_listen(self, items):
        listened_topics = []

        for item in items:
            if(ITEM_TYPE_ECHO == item.type):
                try:
                    listened_topics.index(item.topic)
                    # もうすでに購読済みなので何もしない
                except ValueError as ve:
                    # 未購読
                    listened_topics.append(item.topic)

                    thread = RosTopicListener()
                    thread._topic = item.topic
                    self._topic_listeners.append(thread)
                    thread.received_topic_values.connect(
                        self.ui.tblConfigItems._on_update_topic_values)

                    thread.start()
                except Except as err:
                    rospy.logerr("conf file load failed. %s", e)

    def _on_exec_save(self):
        """パラメータ保存実行処理"""
        if(self._prm_writer is not None):
            QMessageBox.warning(
                self._widget,
                "警告",
                "パラメータ保存実行中です")
            return

        param_config_items = []
        for item in self._config_items:
            if(ITEM_TYPE_NUMBER == item.type or ITEM_TYPE_TEXT == item.type):
                param_config_items.append(item)

        if(len(self._dump_yaml_file_path) == 0):
            QMessageBox.information(
                self._widget,
                "お知らせ",
                "保存先が指定されていません")
            return

        if(len(param_config_items) == 0):
            QMessageBox.information(
                self._widget,
                "お知らせ",
                "保存対象がありません")
            return

        self._monitor_timer.stop()
        self._widget.setEnabled(False)

        self._prm_writer = RosParamWriter()
        self._prm_writer.work_finished.connect(
            self._on_param_writer_work_finished)
        self._prm_writer.finished.connect(self._prm_writer.deleteLater)
        self._prm_writer.param_config_items = param_config_items
        self._prm_writer.dump_file_path = self._dump_yaml_file_path

        self._prm_writer.start()

    def _on_param_writer_work_finished(self, result):
        self._prm_writer = None
        self._monitor_timer.start()
        self._widget.setEnabled(True)

        if(not result):
            QMessageBox.warning(
                self._widget,
                "警告",
                "パラメータが正常に保存できませんでした")
        else:
            QMessageBox.information(
                self._widget,
                "お知らせ",
                "パラメータを保存しました")

    def _showdialog(self, title, msg):
        mbox = QMessageBox(self._widget)
        mbox.setIcon(QMessageBox.Question)
        mbox.setText(msg)
        mbox.setWindowTitle(title)
        mbox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)

        retval = mbox.exec_()
        return QMessageBox.Ok == retval
