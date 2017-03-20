#A task is a small piece of behaviour defined in ARC, prompting a robot to do some specific piece of work.

#Unique id created for this task instance. 
#All requests and responses about this instance must have the same task_id, otherwise robot will think it’s a different task.
uint32 task_id
