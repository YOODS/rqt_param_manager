# -*- coding: utf-8 -*-

import os
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

    def _get_save_param_names(self, file_path, param_names):
        ret = False
        if(os.path.isfile(file_path)):
            with open(file_path, "r+") as yf:
                data = yaml.load(yf)
                if(data is not None and len(data) > 0):
                    sections = []
                    self._recursive_parse_param_names(
                        data, sections, param_names)
                    ret = True
        return ret

    def _recursive_parse_param_names(self, data, sections, param_names):
        for k, v in data.items():
            if(type(v) is dict):
                sections.append(k)
                self._recursive_parse_param_names(v, sections, param_names)
                sections.pop()
            else:
                key = ""
                if(len(sections) > 0):
                    key = "/"
                key = key + "/".join(sections) + "/" + k
                param_names.append(key)
                # print("key=%s" % (key))

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

        save_param_names = []
        if(not self._get_save_param_names(self.dump_file_path, save_param_names)):
            self.work_finished.emit(False)
            return

        result = True

        param_data_map = {}
        for item in self.param_config_items:
            if(item.param_nm not in save_param_names):
                # print("not save target. param_name=%s" % item.param_nm)
                continue
            try:
                val = item.get_param_value(rospy.get_param(item.param_nm))
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
                "param data save failed. path=%s. cause=%s",
                self.dump_file_path,
                err)
            result = False

        self.work_finished.emit(result)
        self.finished.emit()
