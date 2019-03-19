# -*- coding: utf-8 -*-

import rospy
import string
from python_qt_binding import QtCore
import subprocess
import time


class RosTopicListener(QtCore.QThread):
    received_topic_values = QtCore.Signal(bool, str, dict)

    _topic = ""
    _canceled = False
    _interval = 1

    def __init__(self, topic, parent=None):
        QtCore.QThread.__init__(self, parent)
        self._topic=topic

    def run(self):
        while not self._canceled:
            p = subprocess.Popen(
                ["rostopic", "echo", "-n", "1", self._topic],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=False)
            ret = p.wait()
            if(ret):
                time.sleep(10)
                err_str = p.stderr.readlines()
                rospy.logerr("topic get failed. topic =%s %s" % (
                    self._topic,
                    err_str))

                self.received_topic_values.emit(false, self._topic, ())
            else:
                lines = p.stdout.readlines()
                topic_values = self._on_parse_topic_echo(self._topic, lines)
                self.received_topic_values.emit(
                    True,
                    self._topic,
                    topic_values)
                if(self._interval > 0):
                    time.sleep(self._interval)

        self.finished.emit()

    def _on_parse_topic_echo(self, topic, lines):
        topic_values = {}
        topic_header = ""
        sections = []

        pre_lv = -1
        pre_header_lv = -1
        for line in lines:
            if(len(line) == 0 or line[0] == '\n'):
                continue

            if(line.find("---") == 0):
                break

            sidx = line.find(":")
            if(sidx < 0):
                print("unknown format. line =%s" % line)
                continue

            key = line[:sidx]
            val = line[sidx+1:].strip()

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

            if(len(val) == 0):
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
