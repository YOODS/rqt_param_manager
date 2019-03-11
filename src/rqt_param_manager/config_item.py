# -*- coding: utf-8 -*-

import sys
import os
import rospy
import string
import subprocess
import rostopic
import rosmsg
import ast
from python_qt_binding import QtCore
from python_qt_binding.QtCore import *

ITEM_TYPE_NONE = 0
ITEM_TYPE_TITLE = 1
ITEM_TYPE_ECHO = 2
ITEM_TYPE_NUMBER = 3
ITEM_TYPE_TEXT = 4
ITEM_TYPE_FILE = 5
ITEM_TYPE_TRIGGER = 6


class ConfigItem(QObject):
    # def __init__(self):

    prefix = ""
    type = ITEM_TYPE_NONE
    label = "不明"
    topic = ""
    topic_items = []

    param_nm = ""
    param_type = ""
    param_val = None

    file_option = ""
    trigger = ""
    
    # signals
    invoke_trigger = QtCore.Signal(str)

    @classmethod
    def _trim(self, str):
        str = str.strip()
        str = str.lstrip('"')
        str = str.rstrip('"')
        return str

    def parse(self, line):
        result = False
        # print(os.environ)
        # print("line ="+line)

        line_tokens = line.split(",")
        if(len(line_tokens) > 0):
            type_tokens = self._trim(line_tokens[0]).split(":")
            if(len(type_tokens) == 0):
                return False

            type = type_tokens[0].strip()
            if("Title" == type):
                result = self._parse_line_title(type_tokens, line_tokens)
            elif("Echo" == type):
                result = self._parse_line_echo(type_tokens, line_tokens)
            elif("Number" == type):
                result = self._parse_line_number(type_tokens, line_tokens)
            elif("Text" == type):
                result = self._parse_line_text(type_tokens, line_tokens)
            elif("File" == type):
                result = self._parse_line_file(type_tokens, line_tokens)
            elif("Trigger" == type):
                result = self._parse_line_trigger(type_tokens, line_tokens)
            else:
                result = self._parse_line_number(type_tokens, line_tokens)

            if(result and len(self.prefix) > 0):
                if(len(self.param_nm) > 0 and self.param_nm[0] != '/'):
                    self.param_nm = self.prefix + self.param_nm

        return result


    def _parse_line_title(self, type_tokens, line_tokens):
        self.type = ITEM_TYPE_TITLE
        if(len(line_tokens) > 1):
            self.label = self._trim(line_tokens[1])

            try:
                self.label = string.Template(self.label).substitute(os.environ)
            except:
                rospy.logerr("title replace failed. %s", self.label)
        else:
            return False

        return True

    def _parse_line_echo(self, type_tokens, line_tokens):
        self.type = ITEM_TYPE_ECHO
        
        line_token_num = len(line_tokens)
        if(line_token_num > 1):
            # self.label = self._trim(line_tokens[1])
            self.label = "topic"
            self.topic_item_labels = self._trim(line_tokens[1]).split('\\n')
        else:
            return False

        n = len(type_tokens)
        if(n > 1):
            self.topic = type_tokens[1].strip()
        
        result = False
        try:
            topic_type_info = rostopic.get_topic_type(self.topic)
            # print(topic_type_info[0])
            if( topic_type_info[0] is not None):
                topic_msg = rosmsg.get_msg_text(topic_type_info[0])
                self.topic_items = self._parse_topic_items(self.topic, topic_msg)

                result = True
        except NameError as ne:
            rospy.logerr("topic info get failed. %s", self.topic)
            print(ne)

        return result

    def _parse_line_number(self, type_tokens, line_tokens):
        self.type = ITEM_TYPE_NUMBER

        if(len(line_tokens) > 1):
            self.label = self._trim(line_tokens[1])
        else:
            return False

        n = len(type_tokens)
        if(n == 1):
            self.param_nm = type_tokens[0].strip()
        elif(n == 2):
            self.param_nm = type_tokens[1].strip()
        else:
            return False

        result , self.param_type = self._get_param_type(self.param_nm)

        return True

    def _parse_line_text(self, type_tokens, line_tokens):
        self.type = ITEM_TYPE_TEXT

        if(len(line_tokens) > 1):
            self.label = self._trim(line_tokens[1])
        else:
            return False

        if(len(type_tokens) == 2):
            self.param_nm = type_tokens[1].strip()
        else:
            return False

        result , self.param_type = self._get_param_type(self.param_nm)

        return True

    def _parse_line_file(self, type_tokens, line_tokens):
        self.type = ITEM_TYPE_FILE

        n = len(line_tokens)
        if(n > 1):
            self.label = self._trim(line_tokens[1])
        else:
            return False

        if(n > 2):
            self.file_option = line_tokens[2].strip()

        if(len(type_tokens) == 2):
            self.param_nm = type_tokens[1].strip()
        else:
            return False

        return True

    def _parse_line_trigger(self, type_tokens, line_tokens):
        self.type = ITEM_TYPE_TRIGGER

        if(len(line_tokens) > 1):
            self.label = self._trim(line_tokens[1])
        else:
            return False

        if(len(type_tokens) == 2):
            self.trigger = type_tokens[1].strip()
        else:
            return False
        return True

    # @QtCore.pyqtSlot()
    def _on_action(self):
        print("Acction. type =%d label =%s" % (self.type, self.label))

        if(self.type == ITEM_TYPE_TRIGGER):
            self.invoke_trigger.emit(self.trigger)

    def _parse_topic_items(self, topic, topic_msg):
        topic_items = []

        topic_header = ""
        topic_msg_type = ""
        topic_nm_type_sections = []

        pre_lv = -1
        pre_header_lv = -1

        lines = topic_msg.split('\n')

        for line in lines:
            if(len(line) == 0 or line[0] == '\n'):
                continue

            tokens = line.replace('  ', ' ').split(" ")

            # level count
            curLv = 0
            for token in tokens:
                if(len(token) == 0):
                    curLv = curLv+1

            if(len(tokens) - curLv < 2):
                print("unknown format. line =%s" % line.strip())
                continue

            val1 = tokens[curLv+0].strip()
            val2 = tokens[curLv+1].strip()

            if(val1.find("/") >= 0):
                # print("lv pre =%s cur =%d %s" % (pre_lv,curLv,line.strip()))

                topic_msg_type = val1
                act = ""
                if(pre_header_lv == curLv):
                    act = "same"
                    topic_nm_type_sections.pop()
                    topic_nm_type_sections.append(val2)
                elif(pre_header_lv > curLv):
                    act = "down"
                    topic_nm_type_sections = topic_nm_type_sections[:curLv]
                    topic_nm_type_sections.append(val2)
                else:
                    act = "up  "
                    topic_nm_type_sections.append(val2)

                # print("H [%d] (%s) name =%s" % (curLv,act,"/".join(topic_nm_type_sections)))
                pre_header_lv = curLv

            else:
                values = val2.split("=")
                topicNm = values[0]

                if(len(topic_nm_type_sections) > 0):
                    topicNm = "/".join(topic_nm_type_sections[0:curLv]) + "/" + topicNm

                # print("V [%d] name =%s type =%s" % (curLv,topicNm,val1))

                # data = RosTopicData
                # data.nm = topicNm
                # data.topic = topic
                # data.type = val1
                # topic_items.append(data)
                topic_items.append({"name": topicNm, "type": val1, "topic": topic})

            pre_lv = curLv

        return topic_items

    def _get_param_type(self, param_nm):
        result = False
        param_type = None
        try:
            param_type = type(rospy.get_param(param_nm))
            result = True
        except Exception as err:
            rospy.logerr("param type get failed. param_nm=%s err=%s", param_nm, err)

        return result, param_type

    def get_param_value(self, inputVal):
        val = None
        if(self.param_type is int):
            val = int(inputVal)
        elif(self.param_type is float):
            val = float(inputVal)
        else:
            val = str(inputVal)
        
        return val


    def toString(self):

        str = ", label =%s" % self.label

        if(ITEM_TYPE_TITLE == self.type):
            str = "item = TITLE type =%d" % (self.type) + str

        elif(ITEM_TYPE_ECHO == self.type):
            str = "item = ECHO type =%d" % (self.type) + str
            if(len(self.topic) > 0):
                str = str + ", topic =%s" % self.topic

        elif(ITEM_TYPE_NUMBER == self.type):
            str = "item = NUMBER type =%d" % (self.type) + str
            str = str + ", param_nm =%s" % self.param_nm

        elif(ITEM_TYPE_TEXT == self.type):
            str = "item = TEXT type =%d" % (self.type) + str
            str = str + ", param_nm =%s" % self.param_nm

        elif(ITEM_TYPE_FILE == self.type):
            str = "item = FILE type =%d" % (self.type) + str
            str = str + ", param_nm =%s" % self.param_nm
            if(len(self.file_option) > 0):
                str = str + ", opt =%s" % self.file_option

        elif(ITEM_TYPE_TRIGGER == self.type):
            str = "item = TRIGGER type =%d" % (self.type) + str
            str = str + ", trigger =%s" % self.trigger

        return str
