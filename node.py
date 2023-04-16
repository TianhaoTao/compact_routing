class Node():
    def __init__(self,label) -> None:
        self.label = label
        self.ports = []
        self.table = []

    def initLabel(self,label):
        self.label = label
        return
    
    def initTable(self,table):
        self.table = table
        return
    
    def initPort(self,ports):
        self.ports = ports
        return