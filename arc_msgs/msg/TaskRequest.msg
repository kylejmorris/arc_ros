#A task is a small piece of behaviour defined in ARC, prompting a robot to do some specific piece of work.

#Unique id created for this task instance. 
#All requests and responses about this instance must have the same task_id, otherwise robot will think it’s a different task.
uint32 task_id

time created #The time that this task instance was created.

#The name of the task to perform
string task_name

#Configuration parameters for this task. 
dynamic_reconfigure/Config parameters

#The intend of the request. some task requests are requesting to complete a task.
#Another request may be asking to cancel the execution of a task.
uint8 request_type

#Request types
uint8 TYPE_COMPLETION=0   #Requesting robot to complete the task.
uint8 TYPE_CANCELLATION=1 #Requesting robot to cancel the task.
