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
from config_item import *
from ros_topic_listener_thread import *
from std_msgs.msg import *

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
FILE_DEFAULT_PM_CONFS = ["default.pmconf", "Default.pmconf"]
KEY_ENV_ROS_NAMESPACE = "ROS_NAMESPACE"

WID_PROP_TOPIC_NM = "topic_nm"

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
        self._config_items = []
        self._ros_namespace = os.environ.get(KEY_ENV_ROS_NAMESPACE, "")
        self._table_input_item_map = {}
        self._topic_listeners = []
        self._topic_data_map = {}

        self.setObjectName('RqtParamManagerPlugin')

        self._parse_args(sys.argv)

        # result_load_conf = self._load_conf_file(sys.argv)

        if(len(self._ros_namespace) > 0 and self._ros_namespace[:-1] != "/"):
            self._ros_namespace = self._ros_namespace + "/"

        # print(sys.argv)
        # print(self._conf_file_path_list)
        # print("dump_yaml_file_path ="+self._dump_yaml_file_path)

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

        result_load_conf = self._parse_conf_file(self._conf_file_path_list,
                                                 self._config_items,
                                                 self._topic_data_map)

        QTimer.singleShot(0, self._update_window_title)

        if not result_load_conf:
            # self._widget.btnUpdate.setEnabled(False)
            self._widget.btnSave.setEnabled(False)
        else:

            # bind connections
            # self._widget.btnUpdate.clicked.connect(self._on_exec_update)
            self._widget.btnSave.clicked.connect(self._on_exec_save)
            self._monitor_timer.timeout.connect(self._on_period_monitoring)

            self._load_config_table_item(self._widget.tblConfigItems,
                                         self._config_items)

            # テーブル行とパラメータ数のチェック
            table_row_num = self._widget.tblConfigItems.rowCount()

            # ---一時コメント
#            param_num = len(self._params)
#            if table_row_num != param_num or param_num == 0:
#                self._widget.btnUpdate.setEnabled(False)
#                self._widget.btnSave.setEnabled(False)

            # 定期監視処理の開始
            if self._get_interval > 0:
                self._on_period_monitoring()
                rospy.loginfo(
                    "start monitor. interval =%d sec",
                    self._get_interval
                )
                self._monitor_timer.start(self._get_interval)

            self._start_topic_listen(self._config_items)

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
        # This will enable a setting button(gear icon)
        # in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def _parse_args(self, argv):
        """引数パース処理"""

        for arg in sys.argv:
            tokens = arg.split(":=")
            if len(tokens) == 2:
                key = tokens[0]
                if("conf" == key):
                    self._conf_file_path_list = [tokens[1]]
                elif("dump" == key):
                    self._dump_yaml_file_path = tokens[1]

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
                        if(ITEM_TYPE_TITLE == item.type):
                            self._title = item.label

                        items.append(item)

                file.close()
                result = True
            except IOError as e:
                rospy.logerr("conf file load failed. %s", e)

            if result:
                break

        return result

    def _load_config_table_item(self, table, items):
        """パラメータテーブル項目読込処理"""

        item_num = len(items)
        table.setRowCount(item_num)

        model = table.model()

        n = 0
        for item in items:
            if(ITEM_TYPE_ECHO == item.type):
                pnl = QWidget()
                layout = QVBoxLayout()
                layout.setContentsMargins(3, 3, 3, 3)
                pnl.setLayout(layout)

                for topicLbl in item.topic_item_labels:
                    echoLbl = QLabel(topicLbl)
                    echoLbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                    layout.addWidget(echoLbl)

                pnl.adjustSize()
                table.setRowHeight(n, pnl.height())
                table.setIndexWidget(model.index(n, TBL_COL_LABEL), pnl)
            else:
                lbl = QTableWidgetItem(item.label)
                lbl.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                table.setItem(n, TBL_COL_LABEL, lbl)

            if(ITEM_TYPE_TITLE == item.type):
                lbl.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                table.setSpan(n, TBL_COL_LABEL, 1, 3)

            elif(ITEM_TYPE_ECHO == item.type):
                """
                if(len(item.topic) > 0):
                    table.setItem(
                        n,
                        TBL_COL_INPUT,
                        QTableWidgetItem(INVALID_VAL)
                    )
                else:
                    table.setSpan(n, TBL_COL_LABEL, 1, 3)
                """

                if(len(item.topic_items) != 0):
                    pnl = QWidget()
                    layout = QVBoxLayout()
                    layout.setContentsMargins(3, 3, 3, 3)
                    pnl.setLayout(layout)

                    for wIdx in range(len(item.topic_item_labels)):
                        topicItem = None
                        txtTopic = QLineEdit()

                        if(len(item.topic_items) > wIdx):
                            topicItem = item.topic_items[wIdx]

                            txtTopic.setObjectName(str(wIdx))
                            txtTopic.setReadOnly(True)
                            txtTopic.setFocusPolicy(Qt.NoFocus)
                            txtTopic.setProperty(WID_PROP_TOPIC_NM,
                                                 topicItem["name"])
                        else:
                            txtTopic.setEnabled(false)

                        layout.addWidget(txtTopic)

                    pnl.adjustSize()
                    table.setRowHeight(n, pnl.height())
                    table.setIndexWidget(model.index(n, TBL_COL_INPUT), pnl)

            elif(ITEM_TYPE_NUMBER == item.type or ITEM_TYPE_TEXT == item.type):
                txt_edit = QLineEdit()
                txt_edit.setFrame(False)
                txt_edit.setText(INVALID_VAL)
                txt_edit.textEdited.connect(self._on_text_modified)
                txt_edit.editingFinished.connect(self._on_text_changed)

                table.setIndexWidget(model.index(n, TBL_COL_INPUT), txt_edit)
                self._table_input_item_map[txt_edit] = item
                item.param_val = INVALID_VAL

            elif(ITEM_TYPE_FILE == item.type):
                lbl = QLabel()
                table.setIndexWidget(model.index(n, TBL_COL_INPUT), lbl)

                btn = QPushButton()
                btn.setText("一覧")
                table.setIndexWidget(model.index(n, TBL_COL_ACTION), btn)
                # btn.clicked.connect(item._on_action)

            elif(ITEM_TYPE_PUBLISHER == item.type):
                lbl = QLabel()
                table.setIndexWidget(model.index(n, TBL_COL_INPUT), lbl)

                btn = QPushButton()
                btn.setText("実行")
                btn.setProperty(WID_PROP_TOPIC_NM, item.topic)
                btn.clicked.connect(self._on_topic_publish_exec)
                table.setIndexWidget(model.index(n, TBL_COL_ACTION), btn)

                # item.invoke_trigger.connect(self._on_exec_trigger)
            else:
                table.setItem(
                    n,
                    TBL_COL_INPUT,
                    QTableWidgetItem(INVALID_VAL)
                )
            n += 1

    def _on_text_modified(self):
        sender = self.sender()
        try:
            item = self._table_input_item_map[sender]
            if(item.type == ITEM_TYPE_NUMBER or item.type == ITEM_TYPE_TEXT):
                txt_edit = sender
                if(item.param_val != txt_edit.text()):
                    txt_edit.setStyleSheet('color: rgb(255, 0, 0);')
                else:
                    txt_edit.setStyleSheet('color: rgb(0, 0, 0);')
        except Exception as err:
            pass

    def _on_text_changed(self):
        sender = self.sender()
        try:
            item = self._table_input_item_map[sender]
            if(item.type == ITEM_TYPE_NUMBER or item.type == ITEM_TYPE_TEXT):
                txt_edit = sender
                param_val = txt_edit.text().strip()
                if(param_val != item.param_val):
                    if(self._invoke_param_set(item, param_val)):
                        txt_edit.setStyleSheet('color: rgb(0, 0, 0);')

        except Exception as err:
            print(err)
            pass

    def _on_period_monitoring(self):
        """定期監視処理"""

        table = self._widget.tblConfigItems
        item_num = len(self._config_items)

        for n in range(item_num):
            item = self._config_items[n]

            key = None
            try:
                key = [k for k,
                       v in self._table_input_item_map.items()
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
                                          self._on_topic_value_received)
                    thread.start()
                except Except as err:
                    rospy.logerr("conf file load failed. %s", e)

    def _invoke_param_set(self, item, val):
        # print("val =" + val)
        result = False
        try:
            val = item.get_param_value(val)
            rospy.set_param(item.param_nm, val)
            result = True
        except Exception as err:
            rospy.logerr("param value write failed. "
                         "param_nm=%s val=%s err=%s", item.param_nm, val, err)

        return result

#    def _on_exec_update(self, from_save = False):
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
#                    if(param_type is int):
#                        upd_val = int(upd_val)
#                    elif(param_type is float):
#                        upd_val = float(upd_val)
#                    elif(param_type is list):
#                        upd_val = yaml.load(upd_val)
#                    rospy.set_param(param_nm, upd_val)
#                    rospy.loginfo("param_nm =%s val =%s", param_nm, upd_val)
#                    ok_num += 1
#                except(KeyError, ValueError) as e:
#                    rospy.logerr(
#                        "update failed. paramNo =%d cause =%s",
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

        self._monitor_timer.start()
        self._widget.setEnabled(True)

    # def _on_exec_trigger(self, trigger):
    #     print("trigger =" + trigger)

    # @QtCore.pyqtSlot()
    def _on_topic_value_received(self, result, topic, topic_values):
        # print("topic value received."
        #       " result =%d topic = %s" % (result, topic))

        table = self._widget.tblConfigItems
        model = table.model()

        for n in range(len(self._config_items)):
            item = self._config_items[n]
            if(ITEM_TYPE_ECHO == item.type):
                if(item.topic == topic):
                    cell_wid = table.indexWidget(model.index(n, TBL_COL_INPUT))
                    if(cell_wid is not None):
                        pnls = cell_wid.findChildren(QLineEdit)

                        for pIdx in range(len(pnls)):
                            txt_edit = pnls[pIdx]
                            try:
                                key = txt_edit.property(WID_PROP_TOPIC_NM)

                                # print("key=%s val=%s" % (
                                #     key,
                                #     topic_values[key]))
                                txt_edit.setText(topic_values[key])

                                # print("[%d] objNm =%s key =%s" % (
                                #            pIdx,
                                #            txt_edit.objectName(),
                                #            key))
                            except(KeyError) as e:
                                pass
                        # print(pnls)
                        # print("hit topic, n =%d, len =%d" % (n, len(pnls)))

    @QtCore.pyqtSlot()
    def _on_topic_publish_exec(self):
        sender = self.sender()
        topic_nm = ""
        try:
            topic_nm = sender.property(WID_PROP_TOPIC_NM)
            pub = rospy.Publisher(topic_nm, std_msgs.msg.Bool, queue_size=10)
            pub.publish(True)
            QMessageBox.information(
                self._widget,
                "お知らせ",
                "トピック「{}」のパブリッシュを実行しました。".format(topic_nm))
        except Exception as err:
            rospy.logerr(
                "topic publish failed. topic=%s err=%s",
                topic_nm, err)
            QMessageBox.critical(
                self._widget,
                "エラー",
                "トピック「{}」のパブリッシュに失敗しました。".format(topic_nm))
