# -*- coding: utf-8 -*-

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
    #def __init__(self):

    prefix = ""
    type = ITEM_TYPE_NONE
    label = "不明"
    topic = ""
    param_nm = ""
    file_option = ""
    trigger = ""
    param_val = None
    
    # signals 
    invoke_trigger = QtCore.Signal(str)
    
    @classmethod
    def trim(self, str ):
        str = str.strip()
        str = str.lstrip('"')
        str = str.rstrip('"')
        return str
        
    def parse(self,line):
        result=False
        #print("line="+line)

        line_tokens = line.split(",")
        if( len(line_tokens) > 0 ):
            type_tokens = self.trim(line_tokens[0]).split(":")
            if ( len(type_tokens) == 0 ):
                return False
            
            type = type_tokens[0].strip()
            if( "Title" == type ):
                result=self._parseTitleLine(type_tokens,line_tokens)
            if( "Echo" == type ):
                result=self._parseEchoLine(type_tokens,line_tokens)
            elif ( "Number" == type ):
                result=self._parseNumberLine(type_tokens,line_tokens)
            elif ( "Text" == type ):
                result=self._parseTextLine(type_tokens,line_tokens)
            elif ( "File" == type ):
                result=self._parseFileLine(type_tokens,line_tokens)
            elif ( "Trigger" == type ):
                result=self._parseTriggerLine(type_tokens,line_tokens)
            else:
                result=self._parseNumberLine(type_tokens,line_tokens)

            if( result and len(self.prefix) > 0 ):
                if( len(self.param_nm) > 0 and self.param_nm[0] !='/'  ):
                    self.param_nm = self.prefix + self.param_nm


        return result

    def _parseTitleLine(self,type_tokens,line_tokens):
        self.type = ITEM_TYPE_TITLE
        
        if( len(line_tokens) > 1 ):
            self.label = self.trim(line_tokens[1])
            try:
                self.label=string.Template(self.label).substitute(os.environ)
            except :
                print("title replace failed. %s", self.label)
        else:
            return False
        
        return True
        
    def _parseEchoLine(self,type_tokens,line_tokens):
        self.type = ITEM_TYPE_ECHO
        
        if( len(line_tokens) > 1 ):
            self.label = self.trim(line_tokens[1])
        else:
            return False

        n = len(type_tokens)
        if( n > 1 ):
            self.topic = type_tokens[1].strip()
        
        return True

    def _parseNumberLine(self,type_tokens,line_tokens):
        self.type = ITEM_TYPE_NUMBER

        if( len(line_tokens) > 1 ):
            self.label = self.trim(line_tokens[1])
        else:
            return False

        n = len(type_tokens)
        if( n == 1 ):
            self.param_nm = type_tokens[0].strip()
        elif( n == 2 ):
            self.param_nm = type_tokens[1].strip()
        else:
            return False

        return True

    def _parseTextLine(self,type_tokens,line_tokens):
        self.type = ITEM_TYPE_TEXT
         
        if( len(line_tokens) > 1 ):
            self.label = self.trim(line_tokens[1])
        else:
            return False
            
        if( len(type_tokens) == 2 ):
            self.param_nm = type_tokens[1].strip()
        else:
            return False

        return True

    def _parseFileLine(self,type_tokens,line_tokens):
        self.type = ITEM_TYPE_FILE
        
        n = len(line_tokens)
        if( n > 1 ):
            self.label = self.trim(line_tokens[1])
        else:
            return False

        if( n > 2 ):
            self.file_option = line_tokens[2].strip()

        if( len(type_tokens) == 2 ):
            self.param_nm = type_tokens[1].strip()
        else:
            return False
            
        return True
         
    def _parseTriggerLine(self,type_tokens,line_tokens):
         self.type = ITEM_TYPE_TRIGGER
         
         if( len(line_tokens) > 1 ):
             self.label = self.trim(line_tokens[1])
         else:
             return False
            
         if( len(type_tokens) == 2 ):
             self.trigger = type_tokens[1].strip()
         else:
             return False
         return True

    #@QtCore.pyqtSlot()
    def _on_action(self):
        print("Acction. type=%d label=%s" % (self.type,self.label))
        
        if( self.type == ITEM_TYPE_TRIGGER ):
            self.invoke_trigger.emit(self.trigger)

    def toString(self):
        
        str = ", label=%s" % self.label
        if ( ITEM_TYPE_ECHO == self.type ):
            str = "item=ECHO type=%d" % ( self.type ) + str
            if ( len(self.topic) > 0 ):
                str = str + ", topic=%s" % self.topic

        elif ( ITEM_TYPE_NUMBER == self.type ):
            str = "item=NUMBER type=%d" % ( self.type ) + str
            str = str + ", param_nm=%s" % self.param_nm

        elif ( ITEM_TYPE_TEXT == self.type ):
            str = "item=TEXT type=%d" % ( self.type ) + str
            str = str + ", param_nm=%s" % self.param_nm
            
        elif ( ITEM_TYPE_FILE == self.type ):
            str = "item=FILE type=%d" % ( self.type ) + str
            str = str + ", param_nm=%s" % self.param_nm
            if ( len(self.file_option) > 0 ):
                str = str + ", opt=%s" % self.file_option
                
        elif ( ITEM_TYPE_TRIGGER == self.type ):
            str = "item=TRIGGER type=%d" % ( self.type ) + str
            str = str + ", trigger=%s" % self.trigger
            
        return str