from map import Map

class TaskManager:
    def __init__(self, map: Map, task_fp=None):
        self.map=map
        self.task_fp=task_fp
        self.tasks=[]
        if task_fp is not None:
            self.load_tasks(task_fp)
    
    def load_tasks(self, task_fp):
        with open(task_fp) as f:
            num_tasks=None
            while True:
                line=f.readline()
                if line.startswith("#"):
                    continue
                else:
                    num_tasks=int(line.strip())
                    break
            for i in range(num_tasks):
                line=f.readline()
                locs=[int(l) for l in line.strip().split(',')]
                self.tasks.append(locs)
    
    @property
    def num_tasks(self):
        return len(self.tasks)