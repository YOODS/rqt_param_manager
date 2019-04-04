# -*- coding: utf-8 -*-

# ================ インポート一覧 ================
import rospy
import string
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import *
from config_item import *

# ================ 定数一覧 ================
TBL_COL_LABEL = 0
TBL_COL_INPUT = 1
TBL_COL_ACTION = 2
INVALID_VAL = "---"
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


class MonitorTable(QTableWidget):
    """UIのメインクラス"""
    invoke_topic_pub = QtCore.Signal(str)

    colLabelWidthRatio = None
    colLabelWidthFixed = None
    _config_items = []
    _table_input_item_map = {}
    _param_txtedit_map = {}
    _pre_param_values = {}

    def __init__(self, parent):
        """初期化処理"""
        super(MonitorTable, self).__init__(parent)

    def initUI(self):
        """パラメータテーブル設定処理"""

        # 列は3列
        self.setColumnCount(3)

        # 列1,2は編集不可
        no_edit_delegate = NotEditableDelegate()
        self.setItemDelegateForColumn(TBL_COL_LABEL, no_edit_delegate)
        self.setItemDelegateForColumn(TBL_COL_ACTION, no_edit_delegate)

        # ヘッダー列の設定
        headerCol1 = QTableWidgetItem()
        headerCol1.setText("ラベル")
        self.setHorizontalHeaderItem(TBL_COL_LABEL, headerCol1)

        headerCol2 = QTableWidgetItem()
        headerCol2.setText("入力")
        self.setHorizontalHeaderItem(TBL_COL_INPUT, headerCol2)

        headerCol3 = QTableWidgetItem()
        headerCol3.setText("ボタン")
        self.setHorizontalHeaderItem(TBL_COL_ACTION, headerCol3)

        header = self.horizontalHeader()
        header.setSectionResizeMode(TBL_COL_LABEL, QHeaderView.Stretch)
        header.setSectionResizeMode(TBL_COL_INPUT, QHeaderView.Fixed)
        header.setSectionResizeMode(TBL_COL_ACTION, QHeaderView.Fixed)
        self.setColumnWidth(TBL_COL_INPUT, 120)
        self.setColumnWidth(TBL_COL_ACTION, 120)

        self.verticalHeader().hide()

    def load_items(self, items):
        """項目読込処理"""

        self._config_items = items

        item_num = len(items)
        self.setRowCount(item_num)

        model = self.model()

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
                self.setRowHeight(n, pnl.height())
                self.setIndexWidget(model.index(n, TBL_COL_LABEL), pnl)
            else:
                lbl = QTableWidgetItem(item.label)
                lbl.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.setItem(n, TBL_COL_LABEL, lbl)

            if(ITEM_TYPE_TITLE == item.type):
                lbl.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                self.setSpan(n, TBL_COL_LABEL, 1, 3)

            elif(ITEM_TYPE_ECHO == item.type):
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
                    self.setRowHeight(n, pnl.height())
                    self.setIndexWidget(model.index(n, TBL_COL_INPUT), pnl)

            elif(ITEM_TYPE_NUMBER == item.type or ITEM_TYPE_TEXT == item.type):
                txt_edit = QLineEdit()
                txt_edit.setFrame(False)
                txt_edit.setText(INVALID_VAL)
                txt_edit.textEdited.connect(self._on_text_modified)
                txt_edit.editingFinished.connect(self._on_text_changed)

                self.setIndexWidget(model.index(n, TBL_COL_INPUT), txt_edit)
                self._table_input_item_map[txt_edit] = item

                if(item.param_nm in self._param_txtedit_map):
                    self._param_txtedit_map[item.param_nm].append(txt_edit)
                else:
                    self._param_txtedit_map[item.param_nm] = [txt_edit]

                if(ITEM_TYPE_NUMBER == item.type):
                    txt_edit.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

                item.param_val = INVALID_VAL

            elif(ITEM_TYPE_FILE == item.type):
                lbl = QLabel()
                self.setIndexWidget(model.index(n, TBL_COL_INPUT), lbl)

                btn = QPushButton()
                btn.setText("一覧")
                self.setIndexWidget(model.index(n, TBL_COL_ACTION), btn)

            elif(ITEM_TYPE_PUBLISHER == item.type):
                lbl = QLabel()
                self.setIndexWidget(model.index(n, TBL_COL_INPUT), lbl)

                btn = QPushButton()
                btn.setText("実行")
                btn.setProperty(WID_PROP_TOPIC_NM, item.topic)
                btn.clicked.connect(self._on_exec_button_clicked)
                self.setIndexWidget(model.index(n, TBL_COL_ACTION), btn)

            else:
                self.setItem(
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
                pre_param_val_str = ""
                if(item.param_nm in self._pre_param_values):
                    pre_param_val_str = str(
                        self._pre_param_values[item.param_nm])

                if(pre_param_val_str != txt_edit.text()):
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
                cur_param_val_str = txt_edit.text().strip()
                pre_param_val_str = ""
                if(item.param_nm in self._pre_param_values):
                    pre_param_val_str = str(
                        self._pre_param_values[item.param_nm])

                if(cur_param_val_str != pre_param_val_str):
                    if(self._invoke_param_set(item, cur_param_val_str)):
                        txt_edit.setStyleSheet('color: rgb(0, 0, 0);')

        except Exception as err:
            print(err)
            pass

    def _invoke_param_set(self, item, val):
        result = False
        try:
            # if(not rospy.has_param(item.param_nm)):
            #     rospy.logerr("param name is not found. nm=%s", item.param_nm)
            #     return False

            val = item.get_param_value(val)
            rospy.set_param(item.param_nm, val)
            result = True
        except Exception as err:
            rospy.logerr("param value write failed. "
                         "param_nm=%s val=%s err=%s", item.param_nm, val, err)

        return result

    def get_monitor_param_nms(self):
        return self._param_txtedit_map.keys()

    def update_param_values(self, param_values):
        for param_nm, txt_edits in self._param_txtedit_map.items():
            pre_val = ""
            if(param_nm in self._pre_param_values):
                pre_val = self._pre_param_values[param_nm]
            cur_val = ""
            if(param_nm in param_values):
                cur_val = param_values[param_nm]

            if(pre_val != cur_val):
                if(cur_val is None):
                    cur_val_str = INVALID_VAL
                else:
                    cur_val_str = str(cur_val)

                for txt_edit in txt_edits:
                    txt_edit.setText(cur_val_str)
                    txt_edit.setStyleSheet('color: rgb(0, 0, 0);')

        self._pre_param_values = param_values

    @QtCore.pyqtSlot()
    def _on_exec_button_clicked(self):
        sender = self.sender()
        if(sender):
            topic_nm = sender.property(WID_PROP_TOPIC_NM)
            self.invoke_topic_pub.emit(topic_nm)

    def _on_update_topic_values(self, result, topic, topic_values):
        model = self.model()
        for n in range(len(self._config_items)):
            item = self._config_items[n]
            if(ITEM_TYPE_ECHO == item.type):
                if(item.topic == topic):
                    cell_wid = self.indexWidget(model.index(n, TBL_COL_INPUT))
                    if(cell_wid is not None):
                        pnls = cell_wid.findChildren(QLineEdit)

                        for pIdx in range(len(pnls)):
                            txt_edit = pnls[pIdx]
                            try:
                                key = txt_edit.property(WID_PROP_TOPIC_NM)
                                txt_edit.setText(topic_values[key])
                            except(KeyError) as e:
                                pass

    def _showdialog(self, title, msg):
        mbox = QMessageBox(self)
        mbox.setIcon(QMessageBox.Question)
        mbox.setText(msg)
        mbox.setWindowTitle(title)
        mbox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)

        retval = mbox.exec_()
        return QMessageBox.Ok == retval

    def resizeEvent(self, event):
        widWidth = event.size().width()
        widHeight = event.size().height()
        colActWidth = self.columnWidth(TBL_COL_ACTION)

        if(self.colLabelWidthRatio is not None):
            colLabelWidth = widWidth * self.colLabelWidthRatio
            colInputWidth = widWidth - colLabelWidth - colActWidth
        elif(self.colLabelWidthFixed is not None):
            colLabelWidth = self.colLabelWidthFixed
            colInputWidth = widWidth - colLabelWidth - colActWidth
        else:
            super(MonitorTable, self).resizeEvent(event)
            return

        header = self.horizontalHeader()
        header.setSectionResizeMode(TBL_COL_LABEL, QHeaderView.Fixed)
        header.setSectionResizeMode(TBL_COL_INPUT, QHeaderView.Fixed)

        self.setColumnWidth(TBL_COL_LABEL, colLabelWidth)
        self.setColumnWidth(TBL_COL_INPUT, colInputWidth)
