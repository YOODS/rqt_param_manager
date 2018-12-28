# -*- coding: utf-8 -*-

from python_qt_binding.QtCore import QObject

ITEM_TYPE_NONE = 0
ITEM_TYPE_ECHO = 1
ITEM_TYPE_NUMBER = 2
ITEM_TYPE_TEXT = 3
ITEM_TYPE_FILE = 4
ITEM_TYPE_TRIGGER = 5


class ConfigItem(QObject):
    #def __init__(self):

    prefix = ""
    type = ITEM_TYPE_NONE
    label = "不明"
    topic = ""
    param = ""
    file_option = ""
    trigger = ""
    
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
            ITEM_TYPE_tokens = self.trim(line_tokens[0]).split(":")
            if ( len(ITEM_TYPE_tokens) == 0 ):
                return False
            
            type = ITEM_TYPE_tokens[0].strip()
            if( "Echo" == type ):
                result=self._parseEchoLine(ITEM_TYPE_tokens,line_tokens)
            elif ( "Number" == type ):
                result=self._parseNumberLine(ITEM_TYPE_tokens,line_tokens)
            elif ( "Text" == type ):
                result=self._parseTextLine(ITEM_TYPE_tokens,line_tokens)
            elif ( "File" == type ):
                result=self._parseFileLine(ITEM_TYPE_tokens,line_tokens)
            elif ( "Trigger" == type ):
                result=self._parseTriggerLine(ITEM_TYPE_tokens,line_tokens)
            else:
                result=self._parseNumberLine(ITEM_TYPE_tokens,line_tokens)

            if( result and len(self.prefix) > 0 ):
                if( len(self.param) > 0 and self.param[0] !='/'  ):
                    self.param = self.prefix + self.param


        return result

    def _parseEchoLine(self,ITEM_TYPE_tokens,line_tokens):
        self.type = ITEM_TYPE_ECHO
        
        if( len(line_tokens) > 1 ):
            self.label = self.trim(line_tokens[1])
        else:
            return False

        n = len(ITEM_TYPE_tokens)
        if( n > 1 ):
            self.topic = ITEM_TYPE_tokens[1].strip()
        
        return True

    def _parseNumberLine(self,ITEM_TYPE_tokens,line_tokens):
        self.type = ITEM_TYPE_NUMBER

        if( len(line_tokens) > 1 ):
            self.label = self.trim(line_tokens[1])
        else:
            return False

        n = len(ITEM_TYPE_tokens)
        if( n == 1 ):
            self.param = ITEM_TYPE_tokens[0].strip()
        elif( n == 2 ):
            self.param = ITEM_TYPE_tokens[1].strip()
        else:
            return False

        return True

    def _parseTextLine(self,ITEM_TYPE_tokens,line_tokens):
        self.type = ITEM_TYPE_TEXT
         
        if( len(line_tokens) > 1 ):
            self.label = self.trim(line_tokens[1])
        else:
            return False
            
        if( len(ITEM_TYPE_tokens) == 2 ):
            self.param = ITEM_TYPE_tokens[1].strip()
        else:
            return False

        return True

    def _parseFileLine(self,ITEM_TYPE_tokens,line_tokens):
        self.type = ITEM_TYPE_FILE
        
        n = len(line_tokens)
        if( n > 1 ):
            self.label = self.trim(line_tokens[1])
        else:
            return False

        if( n > 2 ):
            self.file_option = line_tokens[2].strip()

        if( len(ITEM_TYPE_tokens) == 2 ):
            self.param = ITEM_TYPE_tokens[1].strip()
        else:
            return False
            
        return True
         
    def _parseTriggerLine(self,ITEM_TYPE_tokens,line_tokens):
         self.type = ITEM_TYPE_TRIGGER
         
         if( len(line_tokens) > 1 ):
             self.label = self.trim(line_tokens[1])
         else:
             return False
            
         if( len(ITEM_TYPE_tokens) == 2 ):
             self.trigger = ITEM_TYPE_tokens[1].strip()
         else:
             return False
         return True


    def _on_action(self):
        print("Acction. label=" + self.label)

    def toString(self):
        
        str = ", label=%s" % self.label
        if ( ITEM_TYPE_ECHO == self.type ):
            str = "item=ECHO type=%d" % ( self.type ) + str
            if ( len(self.topic) > 0 ):
                str = str + ", topic=%s" % self.topic

        elif ( ITEM_TYPE_NUMBER == self.type ):
            str = "item=NUMBER type=%d" % ( self.type ) + str
            str = str + ", param=%s" % self.param

        elif ( ITEM_TYPE_TEXT == self.type ):
            str = "item=TEXT type=%d" % ( self.type ) + str
            str = str + ", param=%s" % self.param
            
        elif ( ITEM_TYPE_FILE == self.type ):
            str = "item=FILE type=%d" % ( self.type ) + str
            str = str + ", param=%s" % self.param
            if ( len(self.file_option) > 0 ):
                str = str + ", opt=%s" % self.file_option
                
        elif ( ITEM_TYPE_TRIGGER == self.type ):
            str = "item=TRIGGER type=%d" % ( self.type ) + str
            str = str + ", trigger=%s" % self.trigger
            
        return str