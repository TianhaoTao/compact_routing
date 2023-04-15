class Packet():
    def __init__(self, init,dest):
        self.init = init
        self.dest = dest
        self.header = None
        self.current = init
        self.path = []

    def initHeader(self):
        return
    
    def extend_path(self,next):
        self.path.append(next)