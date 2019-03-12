# -*- coding: utf-8 -*-

import rospy
import string
from python_qt_binding import QtCore
import subprocess
import time
import yaml


class RosParamWriter(QtCore.QThread):
    work_finished = QtCore.Signal(bool)

    param_config_items = []

    dump_file_path = ""

    def append(self, item):
        self.param_config_items.append(item)

    def __init__(self, parent=None):
        QtCore.QThread.__init__(self, parent)

    def _appendLevel(self, data, keys, lv, val):
        if(len(keys)-1 <= lv):
            data[keys[-1]] = val
        else:
            if(keys[lv] in data):
                data = data[keys[lv]]
                self._appendLevel(data, keys, lv+1, val)
            else:
                data[keys[lv]] = {}
                self._appendLevel(data[keys[lv]], keys, lv+1, val)

    def run(self):
        result = True
        param_data_map = {}
        for item in self.param_config_items:
            try:

                val = item.get_param_value(rospy.get_param(item.param_nm))
                # print("param_val=%s type=%s" % (val,type(val)))

                tokens = item.param_nm.split("/")
                if(len(tokens) < 2 or len(tokens[0]) != 0):
                    result = False
                    rospy.logerr(
                        "param name parse failed. param_nm=%s",
                        param_nm)
                    break
                tokens = tokens[1:]

                self._appendLevel(param_data_map, tokens, 0, val)
            except Exception as err:
                result = False

        try:
            with open(self.dump_file_path, "w") as yf:
                yaml.dump(param_data_map, yf, default_flow_style=False)
        except Exception as err:
            rospy.logerr(
                "param data save failed. path=%s",
                self.dump_file_path)
            result = False

        self.work_finished.emit(result)
        self.finished.emit()
