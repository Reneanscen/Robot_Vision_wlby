# graspCV



## custom ROS2 msg

before using the code, it is necessary to first clone the code for `msg`

```bash
git clone https://gitlab.weilaibuyuan.com/advanced_robot_algorithm/robot_cv_msg.git
```



## add new task

create a folder under the `tasks` directory named your task name, and inherit `TaskInterface` class from `core/taskInterface.py` to define your related processing code.

for example:

```python
from core import TaskInterface

class ExampleTask(TaskInterface):
	def __init__(self, self, model_path: str,
                 input_shape: Union[Tuple, List] = (640, 640),  # [height, width]
                 backend: str = "tensorrt",
                 target_map: Dict = None,
                 debug: bool = False)
    	super(ExampleTask, self).__init__(model_path, input_shape, backend, 
                                          target_map, debug)
        # your code...
       
    def _preprocess(self, image: ndarray) -> ndarray:
        # your code...
        return image
    
    def _post_process(self, outputs, **kwargs) -> Union[List, bool]:
        # your code...
        # if detect fail, you can directly return False
        # if nothing is detected, you can directly return []
        # else return a list with dict elements:
        #     [{"target_id": cls, "grasp_scores": [score], "grasp_rt": [rt]}, ...]
        # the dict must contain keys 'target_id', 'grasp_scores' and 'grasp_rt'
        # 'target_id' is int, 'grasp_scores' and 'grasp_rt' are list
        return []
```





