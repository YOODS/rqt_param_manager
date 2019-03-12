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

    def __init__(self, parent=None):
        QtCore.QThread.__init__(self, parent)

    def run(self):
        print("start: topic =%s" % self._topic)

        # call(["rostopic","echo","/turtle1/pose"])
        # call(["rostopic","echo","-n","1","/turtle1/pose"])

        while not self._canceled:
            p = subprocess.Popen(
                ["rostopic", "echo", "-n", "1", "/turtle1/pose"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=False)
            ret = p.wait()
            if(ret):
                # 失敗したら待つ
                time.sleep(10)
                err_str = p.stderr.readlines()
                print("topic get failed. topic =%s %s" % (
                    self._topic,
                    err_str))

                self.received_topic_values.emit(false, self._topic, ())
            else:
                lines = p.stdout.readlines()
                topic_values = self._on_parse_topic_cho(self._topic, lines)

                self.received_topic_values.emit(
                    True,
                    self._topic,
                    topic_values)

                if(self._interval > 0):
                    time.sleep(self._interval)

        # for i in range(100):
        #    self.printLog(self._topic,str(i))
        #    time.sleep(1)

        # print("finished")
        self.finished.emit()

    def _on_parse_topic_cho(self, topic, lines):
        topic_values = {}
        # print("topic_values = %s" % type(topic_values))
        topic_header = ""
        topic_nm_type_sections = []

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

            # print("tokenLen =%d curLv =%d" % (len(tokens),curLv))
            if(len(tokens) - curLv < 1):
                print("unknown format. line =%s" % line.strip())
                continue

            key = tokens[curLv]

            # print("key =%s val =%s" % (key,val))

            if(len(val) == 0):
                # print("lv pre =%s cur =%d %s" % (pre_lv,curLv,line.strip()))

                act = ""
                if(pre_header_lv == curLv):
                    act = "same"
                    topic_nm_type_sections.pop()
                    topic_nm_type_sections.append(key)
                elif(pre_header_lv > curLv):
                    act = "down"
                    topic_nm_type_sections = topic_nm_type_sections[:curLv]
                    topic_nm_type_sections.append(key)
                else:
                    act = "up  "
                    topic_nm_type_sections.append(key)

                # print("H [%d] (%s) name =%s" % (
                #    curLv,
                #    act,
                #    "/".join(topic_nm_type_sections)))
                pre_header_lv = curLv

            else:
                topicNm = key

                if(curLv > 0):
                    topicNm = "/".join(topic_nm_type_sections[0:curLv]) + "/" + topicNm

                # print("V [%d] name =%s val =%s" % (curLv,topicNm,val))
                topic_values[topicNm] = val

            pre_lv = curLv

        return topic_values
