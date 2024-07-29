import time
from tasks import *

from configs import TASK_MAP, TARGET_MAP


class TaskManager(object):
    def __init__(self, max_active=3):
        self.max_active = max_active
        self.active_tasks = {
            # task_id: [task_obj, time]
        }

    def get_task(self, task_id: int):
        task = self.active_tasks.get(task_id, None)
        if task is None:
            if len(self.active_tasks) >= self.max_active:
                key = sorted(self.active_tasks.items(), key=lambda x: x[1][1])[0][0]
                self.release(key)

            target_map = TARGET_MAP[task_id]
            kwargs = TASK_MAP[task_id][1]
            kwargs["target_map"] = target_map
            try:
                task = eval(TASK_MAP[task_id][0])(**kwargs)
            except RuntimeError as e:
                if "CUDA out of memory" in e:
                    # cuda memory out, release one task and try again
                    key = sorted(self.active_tasks.items(), key=lambda x: x[1][1])[0][0]
                    self.release(key)
                    task = eval(TASK_MAP[task_id][0])(**kwargs)
                else:
                    raise RuntimeError(e)
            self.active_tasks[task_id] = [task, time.time()]
        else:
            self.active_tasks[task_id][1] = time.time()     # update last call time
            task = task[0]

        return task

    def set_max_active(self, n: int):
        if n > 0:
            self.max_active = n

    def clear(self):
        """
        clear all task
        """
        for key in self.active_tasks.keys():
            task, t = self.active_tasks.pop(key)
            task.release()
            del task

    def release(self, task_id):
        """
        release one task
        """
        task = self.active_tasks.get(task_id)
        if task is not None:
            task, t = self.active_tasks.pop(task_id)
            task.release()
            del task
