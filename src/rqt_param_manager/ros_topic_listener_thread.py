# -*- coding: utf-8 -*-

import rospy
import string
from python_qt_binding import QtCore
import subprocess
import time

TOPICK_END_TOKEN = "---\n"


class RosTopicListener(QtCore.QThread):
    received_topic_values = QtCore.Signal(bool, str, dict)

    _topic = ""
    _canceled = False
    _interval = 1
    _proc = None

    def __init__(self, topic, parent=None):
        QtCore.QThread.__init__(self, parent)
        self._topic = topic
        print("__init__ topic=%s" % self._topic)

    def run(self):
        # print("run topic=%s" % self._topic)
        while not self._canceled:
            self._proc = subprocess.Popen(
                ["rostopic", "echo", self._topic],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=False)
            lines = []
            while not self._canceled:
                line = self._proc.stdout.readline()
                print("(%s) line=[%s]" % (
                    self._topic, line.replace("\n", "\\n")))
                lines.append(line)
                if(line == TOPICK_END_TOKEN):
                    topic_values = self._on_parse_topic_echo(
                        self._topic, lines)
                    print("")
                    print("")
                    print("topic=%s" % self._topic)
                    print(topic_values)
                    print("------------------------------------------")
                    self.received_topic_values.emit(
                        True,
                        self._topic,
                        topic_values)
                    lines = []

        self.finished.emit()

    def doCancel(self):
        self._canceled = True
        if(self._proc is not None and self._proc.poll()):
            self._proc.kill()

    def _on_parse_topic_echo(self, topic, lines):
        topic_values = {}
        topic_header = ""
        sections = []

        pre_lv = -1
        pre_header_lv = -1
        for line in lines:

            if(len(line) == 0 or line[0] == '\n' or line == TOPICK_END_TOKEN):
                continue

            sidx = line.find(":")
            if(sidx < 0):
                print("unknown format. line =%s" % line)
                continue

            key = line[:sidx]
            val = line[sidx+1:].strip()
            is_header = len(val) == 0
            if(len(val) >= 2 and
                ((val[0] == "\"" and val[-1] == "\"") or
                 (val[0] == "\'" and val[-1] == "\'"))):
                val = val[1:]
                val = val[:-1]

            tokens = key.replace('  ', ' ').split(" ")

            # level count
            curLv = 0
            for token in tokens:
                if(len(token) == 0):
                    curLv = curLv+1

            if(len(tokens) - curLv < 1):
                print("unknown format. line =%s" % line.strip())
                continue

            key = tokens[curLv]

            if(is_header):
                if(pre_header_lv == curLv):
                    sections.pop()
                    sections.append(key)
                elif(pre_header_lv > curLv):
                    sections = sections[:curLv]
                    sections.append(key)
                else:
                    sections.append(key)

                pre_header_lv = curLv
            else:
                topicNm = key

                if(curLv > 0):
                    topicNm = "/".join(sections[0:curLv]) + "/" + topicNm

                topic_values[topicNm] = val

            pre_lv = curLv
        return topic_values
