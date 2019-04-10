# -*- coding: utf-8 -*-

# ================ インポート一覧 ================
import sys
import os
import rospy
import rospkg
import string
import re

# ret_codeとmessage送信時にsleepを使う
from time import sleep

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import QTimer, QVariant, QPoint
from python_qt_binding.QtWidgets import *
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
ARG_WINDOW_SIZE = "window_size"
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
        self._monitor_param_nms = []
        self._param_values = {}

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

        tblMon = self.ui.tblMonitor
        tblMon.initUI()
        tblMon.invoke_topic_pub.connect(self._on_topic_publish_invoke)

        self._initEnv(args)

        result_load_conf = self._parse_conf_file(self._conf_file_path_list,
                                                 self._config_items,
                                                 self._topic_data_map)
        QTimer.singleShot(0, self._update_window_title)

        self.ui.btnClose.clicked.connect(self._app_close)

        if not result_load_conf:
            self.ui.btnSave.setEnabled(False)
        else:
            self.ui.btnSave.clicked.connect(self._on_exec_save)
            self._monitor_timer.timeout.connect(self._on_period_monitoring)

            tblMon.load_items(self._config_items)
            self._monitor_param_nms = tblMon.get_monitor_param_nms()

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

        if(ARG_WINDOW_SIZE in args):
            val = args[ARG_WINDOW_SIZE]
            result = re.match("([\d]+)[xX,-:]([\d]+)", val)
            if(result):
                win_width = int(result.group(1))
                win_height = int(result.group(2))

                parentWid = self._widget.parentWidget()
                while True:
                    wkParentWid = parentWid.parentWidget()
                    if (wkParentWid is None):
                        break
                    parentWid = wkParentWid

                parentWid.resize(win_width, win_height)

        if(ARG_COLUMN_WIDTH_LABEL in args):
            val = args[ARG_COLUMN_WIDTH_LABEL]
            try:
                table = self.ui.tblMonitor
                if(val.endswith("px")):
                    table.colLabelWidthFixed = int(val[:-2])
                elif(val.endswith("%")):
                    table.colLabelWidthRatio = int(val[:-1]) / 100.0
                else:
                    table.colLabelWidthFixed = int(val)
            except Exception as e:
                rospy.logerr(
                    "label column width set failed. val=%s. cause=%s",
                    val, e)

    def _update_window_title(self):
        """ウィンドウタイトルを変更する処理"""

        # 初期化処理内でsetWindowTitleを呼んでも変更されないので
        self._widget.setWindowTitle(self._title)

    def _app_close(self):
        self._monitor_timer.stop()
        QCoreApplication.quit()

    def shutdown_plugin(self):
        """シャットダウン処理"""
        for listener in self._topic_listeners:
            listener.doCancel()

        self._monitor_timer.stop()

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
                set_title = False
                for line in file:
                    line = line.strip()
                    if(len(line) == 0 or line[0] == "#"):
                        continue

                    item = ConfigItem()
                    item.prefix = self._ros_namespace
                    if(not item.parse(line, topic_data_map)):
                        rospy.logerr("conf file wrong line. %s", line)
                    else:
                        if(ITEM_TYPE_TITLE == item.type and
                           len(items) == 0 and
                           not set_title):

                            self._title = item.label
                            set_title = True
                            continue
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

        table = self.ui.tblMonitor

        param_values = {}
        for param_nm in self._monitor_param_nms:
            param_values[param_nm] = None
            try:
                val = rospy.get_param(param_nm)
                param_values[param_nm] = val
            except Exception as err:
                pass

        table.update_param_values(param_values)
        self._param_values = param_values

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

                    thread = RosTopicListener(item.topic)
                    self._topic_listeners.append(thread)
                    thread.received_topic_values.connect(
                        self.ui.tblMonitor._on_update_topic_values)

                    thread.start()

                    print "thread start and sleep"
                    sleep(0.001)
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

    def _on_topic_publish_invoke(self, topic_nm, val):
        try:
            confirm_msg = "トピック「{}」のパブリッシュを実行しますか？".format(topic_nm)
            if(not self._showdialog("確認", confirm_msg)):
                return

            # pub = rospy.Publisher(
            #    topic_nm,
            #    std_msgs.msg.Bool,
            #    queue_size=1,
            #    latch=False)
            # pub.publish(val)

            ret = subprocess.call(
                ["rostopic", "pub", "-1", topic_nm, "std_msgs/Bool", str(val)])
            print("ret=%d" % ret)

            # QMessageBox.information(
            #     self._widget,
            #     "お知らせ",
            #     "トピック「{}」のパブリッシュを実行しました。".format(topic_nm))
        except Exception as err:
            print("err=%s" % err)
            rospy.logerr(
                "topic publish failed. topic=%s err=%s",
                topic_nm, err)
            QMessageBox.critical(
                self._widget,
                "エラー",
                "トピック「{}」のパブリッシュに失敗しました。".format(topic_nm))

    def _showdialog(self, title, msg):
        mbox = QMessageBox(self._widget)
        mbox.setIcon(QMessageBox.Question)
        mbox.setText(msg)
        mbox.setWindowTitle(title)
        mbox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)

        retval = mbox.exec_()
        return QMessageBox.Ok == retval
