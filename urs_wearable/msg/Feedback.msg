# Intermediate States
uint8 STATUS_PENDING=0    # The plan has yet to be executed
uint8 STATUS_ACTIVE=1     # The plan is being executed
uint8 STATUS_REPLANNED=2  # The plan has been changed

# Terminal States
uint8 STATUS_REJECTED=3   # The plan for the given goal is empty
uint8 STATUS_PREEMPTED=4  # The plan was being executed, but was interrupted by other plan executions
uint8 STATUS_ABORTED=5    # The plan was being executed, but some errors have occurred
uint8 STATUS_SUCCEEDED=6  # The plan was executed successfully

uint32 executor_id
uint8 status
Action current_action
string message
