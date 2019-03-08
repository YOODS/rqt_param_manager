# -*- coding: utf-8 -*-

import rospy
import string
from python_qt_binding import QtCore
import subprocess
import time


class RosTopicListener(QtCore.QThread):
    receivedTopicValues = QtCore.Signal(bool, str, dict)

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
            p = subprocess.Popen(["rostopic","echo","-n","1","/turtle1/pose"], stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell = False)
            ret = p.wait()
            if(ret):
                # 失敗したら待つ
                time.sleep(10)
                errStr = p.stderr.readlines()
                print("topic get failed. topic =%s %s" % (self._topic, errStr))

                self.receivedTopicValues.emit(false, self._topic, ())
            else:
                lines = p.stdout.readlines()
                topicValues = self._on_parse_topic_cho(self._topic, lines)

                self.receivedTopicValues.emit(True, self._topic, topicValues)

                if(self._interval > 0):
                    time.sleep(self._interval)

        # for i in range(100):
        #    self.printLog(self._topic,str(i))
        #    time.sleep(1)

        print("finished")
        self.finished.emit()

    def _on_parse_topic_cho(self, topic, lines):
        topicValues = {}
        # print("topicValues = %s" % type(topicValues))
        topicHeader = ""
        topicNmTypeSections = []

        preLv = -1
        preHeaderLv = -1
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
                # print("lv pre =%s cur =%d %s" % (preLv,curLv,line.strip()))

                act = ""
                if(preHeaderLv == curLv):
                    act = "same"
                    topicNmTypeSections.pop()
                    topicNmTypeSections.append(key)
                elif(preHeaderLv > curLv):
                    act = "down"
                    topicNmTypeSections = topicNmTypeSections[:curLv]
                    topicNmTypeSections.append(key)
                else:
                    act = "up  "
                    topicNmTypeSections.append(key)

                # print("H [%d] (%s) name =%s" % (curLv,act,"/".join(topicNmTypeSections)))
                preHeaderLv = curLv

            else:
                topicNm = key

                if(curLv > 0):
                    topicNm = "/".join(topicNmTypeSections[0:curLv]) + "/" + topicNm

                # print("V [%d] name =%s val =%s" % (curLv,topicNm,val))
                topicValues[topicNm] = val

            preLv = curLv

        return topicValues
